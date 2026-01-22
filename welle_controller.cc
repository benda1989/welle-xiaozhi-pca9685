/*
    Welle机器人控制器 - MCP协议版本
*/

#include <cJSON.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/i2c.h>

#include <cstring>
#include <algorithm>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include <pca9685.h>
#include <i2cdev.h>
#include "sdkconfig.h"
#include "settings.h"
#include "esp_http_server.h"
#include "esp_netif.h"

#define TAG "WelleController"

// 舵机通道映射
#define LEFT_WHEEL_CHANNEL     0  // 左轮 (前后移动)
#define RIGHT_WHEEL_CHANNEL    1  // 右轮
#define LEFT_HAND_CHANNEL      2  // 左手 (上下移动)
#define RIGHT_HAND_CHANNEL     3  // 右手
#define LEFT_EYE_CHANNEL       4  // 左眼 (上下移动)
#define RIGHT_EYE_CHANNEL      5  // 右眼
#define NECK_CHANNEL           6  // 脖子 (左右移动)
#define HEAD_CHANNEL           7  // 头 (上下移动)

class WelleController {
private:
    i2c_dev_t pca9685_dev_;
    gpio_num_t scl_pin_;
    gpio_num_t sda_pin_;
    bool pca9685_initialized_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool is_action_in_progress_ = false;
    
    // Web控制相关
    httpd_handle_t web_server_ = nullptr;
    bool web_control_enabled_ = false;

    // 舵机角度范围和中心位置
    struct ServoConfig {
        bool is_mirrored;       
        uint16_t min_angle;
        uint16_t max_angle;
        uint16_t center_angle;
        int trim;
    };
    
    ServoConfig servo_configs_[8];

    enum class ActionType {
        HOME ,        // 回中心位置
        SERVO,           // 舵机控制
        MOVE_FORWARD,    // 前进
        MOVE_BACKWARD,   // 后退
        TURN_LEFT,       // 左转
        TURN_RIGHT,      // 右转
        STOP_MOVEMENT,   // 停止移动
        // 其他动作
    };

    struct ActionParams {
        ActionType action;
        int channel;
        int angle;
    };
    

 
    uint16_t ProcessServoAngle(uint8_t channel, uint16_t angle) {
        ServoConfig& config = servo_configs_[channel];
        int processed_angle = angle;
        
        // 应用微调
        processed_angle += config.trim;
        if (config.is_mirrored) {
            processed_angle = 180 - processed_angle;
        }
        processed_angle = std::max((int)config.min_angle, 
                                    std::min((int)config.max_angle, processed_angle));
        return (uint16_t)processed_angle;
    }

    esp_err_t SetServoAngle(uint8_t channel, uint16_t angle) {
        if (!pca9685_initialized_) {
            ESP_LOGE(TAG, "PCA9685未初始化");
            return ESP_FAIL;
        }

        uint16_t processed_angle = ProcessServoAngle(channel, angle);
        // 1ms-2ms脉宽对应205-409 PWM值 (50Hz, 12位分辨率)
        uint16_t pwm_value = 102 + (processed_angle * 410 / 180);
        esp_err_t ret = pca9685_set_pwm_value(&pca9685_dev_, channel, pwm_value);
        
        if (ret == ESP_OK) {
            const char* servo_names[] = {"左轮", "右轮", "左手", "右手", "左眼", "右眼", "脖子", "头"};
            ESP_LOGI(TAG, "设置%s(通道%d) 输入角度:%d° 处理后:%d° PWM:%d", 
                    servo_names[channel], channel, angle, processed_angle, pwm_value);
        } else {
            ESP_LOGE(TAG, "设置舵机 %d 失败", channel);
        }
        
        return ret;
    }

    static void ActionTask(void* arg) {
        WelleController* controller = static_cast<WelleController*>(arg);
        ActionParams params;

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", (int)params.action);
                controller->is_action_in_progress_ = true;
                if (params.action == ActionType::HOME) {
                    for (int i = 0; i < 8; i++) {
                            controller->SetServoAngle(i, controller->servo_configs_[i].center_angle);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                } else {
                    controller->SetServoAngle(params.channel, params.angle);
                }
                controller->is_action_in_progress_ = false;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    void StartActionTask() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "welle_action", 1024 * 3, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    void QueueServoAction(int channel, int angle ) {
        if (channel < 0 || channel >= 8) {
            ESP_LOGW(TAG, "无效的舵机通道: %d", channel);
            return;
        }
        ActionParams params = {ActionType::SERVO, channel, angle};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
    }
    
    void QueueHomeAction() {
        ESP_LOGI(TAG, "回到中心位置");
        ActionParams params = {ActionType::HOME, 0, 0};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
    }

    void InitializeServos() {
        // 左轮 - 镜像安装（物理安装方向相反）
        servo_configs_[LEFT_WHEEL_CHANNEL] = {true, 0, 180, 90, 0};
        // 右轮 - 标准安装
        servo_configs_[RIGHT_WHEEL_CHANNEL] = {false, 0, 180, 90, 0};
        // 左手 - 标准安装
        servo_configs_[LEFT_HAND_CHANNEL] = {false, 0, 180, 90, 90};
        // 右手 - 镜像安装
        servo_configs_[RIGHT_HAND_CHANNEL] = {true, 0, 180, 90, 90};
        // 左眼 - 镜像安装
        servo_configs_[LEFT_EYE_CHANNEL] = {true, 50, 130, 90, 90};
        // 右眼 - 标准安装
        servo_configs_[RIGHT_EYE_CHANNEL] = {false, 50, 130, 90, 90};
        // 脖子 - 镜像安装
        servo_configs_[NECK_CHANNEL] = {true, 50, 130, 90, 90};
        // 头 - 标准安装
        servo_configs_[HEAD_CHANNEL] = {false, 50, 130, 90, 90};
    }
    
    void LoadTrimsFromNVS() {
        Settings settings("welle_trims", false);

        const char* servo_names[] = {"left_wheel", "right_wheel", "left_hand", "right_hand",
                                    "left_eye", "right_eye", "neck", "head"};
        for (int i = 0; i < 8; i++) {
            servo_configs_[i].trim = settings.GetInt(servo_names[i], 0);
            ESP_LOGI(TAG, "舵机 %s (通道%d) 微调: %d°", servo_names[i], i, servo_configs_[i].trim);
        }
    }
    
    esp_err_t InitializePCA9685() {
        memset(&pca9685_dev_, 0, sizeof(i2c_dev_t));
        
        // 初始化I2C设备库
        esp_err_t ret = i2cdev_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C设备库初始化失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = pca9685_init_desc(&pca9685_dev_, PCA9685_ADDR_BASE, I2C_NUM_1, sda_pin_, scl_pin_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PCA9685初始化失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = pca9685_init(&pca9685_dev_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PCA9685设备初始化失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = pca9685_restart(&pca9685_dev_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PCA9685重启失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = pca9685_set_pwm_frequency(&pca9685_dev_, 50); // 舵机标准频率50Hz
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "设置PWM频率失败: %s", esp_err_to_name(ret));
            return ret;
        }
        
        uint16_t actual_freq;
        pca9685_get_pwm_frequency(&pca9685_dev_, &actual_freq);
        ESP_LOGI(TAG, "PCA9685初始化成功，PWM频率: %dHz", actual_freq);
        
        pca9685_initialized_ = true;
        return ESP_OK;
    }

    // 统一动作控制器 - 核心控制抽象层
    class UnifiedActionController {
    private:
        WelleController* welle_;
        esp_timer_handle_t movement_timer_ = nullptr;
        
        // 执行移动组合动作 (私有方法) - 处理逻辑层运动控制
        bool ExecuteMoveAction(ActionType action, int speed) {
            int left_wheel_angle, right_wheel_angle;
            switch (action) {
                case ActionType::MOVE_FORWARD:
                    left_wheel_angle = 90 + std::min(speed, 90);
                    right_wheel_angle = left_wheel_angle;
                    break;
                    
                case ActionType::MOVE_BACKWARD:
                    left_wheel_angle = 90 - std::min(speed, 90);
                    right_wheel_angle = left_wheel_angle;
                    break;
                    
                case ActionType::TURN_RIGHT:
                    left_wheel_angle = 90 ;
                    right_wheel_angle = 90 + std::min(speed, 90);
                    break;
                    
                case ActionType::TURN_LEFT:
                    left_wheel_angle = 90 + std::min(speed, 90);
                    right_wheel_angle = 90;
                    break;
                default:
                    left_wheel_angle = 90;
                    right_wheel_angle = 90;
            }
            welle_->QueueServoAction(LEFT_WHEEL_CHANNEL, left_wheel_angle);
            welle_->QueueServoAction(RIGHT_WHEEL_CHANNEL, right_wheel_angle);
            return true;
        }

    public:
        UnifiedActionController(WelleController* welle) : welle_(welle) {}
        
        // 统一的字符串命令解析和执行接口
        bool ExecuteAction(const std::string& command) {
            ESP_LOGI(TAG, "处理统一动作命令: %s", command.c_str());
            // 解析格式: action:param 或 channel:angle
            char first_part[30];
            int param = 0;
            int parsed = sscanf(command.c_str(), "%[^:]:%d", first_part, &param);
            
            if (parsed >= 1) {
                ActionParams params;
                params.angle = (parsed >= 2) ? param : 50;  
                // 检查是否为动作命令
                if (strcmp(first_part, "move_forward") == 0) {
                    params.action = ActionType::MOVE_FORWARD;
                } else if (strcmp(first_part, "move_backward") == 0) {
                    params.action = ActionType::MOVE_BACKWARD;
                } else if (strcmp(first_part, "turn_left") == 0) {
                    params.action = ActionType::TURN_LEFT;
                } else if (strcmp(first_part, "turn_right") == 0) {
                    params.action = ActionType::TURN_RIGHT;
                } else if (strcmp(first_part, "stop") == 0) {
                    params.action = ActionType::STOP_MOVEMENT;
                } else if (strcmp(first_part, "home") == 0) {
                    params.action = ActionType::HOME;
                } else {
                    int channel = atoi(first_part);
                    if (channel >= 0 && channel <= 7) {
                        params.action = ActionType::SERVO;
                        params.channel = channel;
                    } else {
                        return false;
                    }
                }
                return ExecuteAction(params);
            }
            return false;
        }
        
        // 执行参数结构体的接口
        bool ExecuteAction(const ActionParams& params) {
            switch (params.action) {
                case ActionType::SERVO:
                    welle_->QueueServoAction(params.channel, params.angle);
                    return true;
                case ActionType::MOVE_FORWARD:
                case ActionType::MOVE_BACKWARD:
                case ActionType::TURN_LEFT:
                case ActionType::TURN_RIGHT:
                case ActionType::STOP_MOVEMENT:
                    return ExecuteMoveAction(params.action, params.angle);
                case ActionType::HOME:
                    welle_->QueueHomeAction();
                    return true;
                default:
                    return false;
            }
        }
    };
    
    UnifiedActionController* action_controller_ = nullptr;
    
    // 简化的消息处理函数
    void ProcessWifiMessage(const char* message) {
        if (!message || !action_controller_) {
            ESP_LOGE(TAG, "无效的消息参数或控制器未初始化");
            return;
        }
        ESP_LOGI(TAG, "处理WiFi消息: %s", message);
        action_controller_->ExecuteAction(std::string(message));
    }

    // HTTP请求处理器 - 主页面(返回嵌入的HTML)
    static esp_err_t WebRootHandler(httpd_req_t *req) {
        // 声明嵌入的HTML文件
        extern const char gamepad_min_html_start[] asm("_binary_gamepad_min_html_start");
        extern const char gamepad_min_html_end[] asm("_binary_gamepad_min_html_end");
        
        const size_t gamepad_html_size = gamepad_min_html_end - gamepad_min_html_start;
        
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, gamepad_min_html_start, gamepad_html_size);
        return ESP_OK;
    }

    // HTTP请求处理器 - 控制命令
    static esp_err_t WebControlHandler(httpd_req_t *req) {
        WelleController* controller = static_cast<WelleController*>(req->user_ctx);
        
        char content[256];
        size_t recv_size = std::min(req->content_len, sizeof(content) - 1);
        int ret = httpd_req_recv(req, content, recv_size);
        
        if (ret <= 0) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to receive data");
            return ESP_FAIL;
        }
        
        content[ret] = '\0';
        ESP_LOGI(TAG, "收到web控制命令: %s", content);
        
        // 处理WiFi消息
        controller->ProcessWifiMessage(content);
        
        // 发送成功响应
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    // 启动Web服务器
    esp_err_t StartWebServer() {
        if (web_server_ != nullptr) {
            return ESP_OK;
        }
        
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = 8080;
        config.ctrl_port = 32768;
        config.max_open_sockets = 7;
        config.stack_size = 8192;
        
        esp_err_t ret = httpd_start(&web_server_, &config);
        if (ret != ESP_OK) {
            return ret;
        }
        
        // 注册根路径处理器
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = WebRootHandler,
            .user_ctx = this
        };
        httpd_register_uri_handler(web_server_, &root_uri);
        
        // 注册控制命令处理器
        httpd_uri_t control_uri = {
            .uri = "/control",
            .method = HTTP_POST,
            .handler = WebControlHandler,
            .user_ctx = this
        };
        httpd_register_uri_handler(web_server_, &control_uri);
        
        web_control_enabled_ = true;
        return ESP_OK;
    }

    // 停止Web服务器
    esp_err_t StopWebServer() {
        if (web_server_ == nullptr) {
            return ESP_OK;
        }
        esp_err_t ret = httpd_stop(web_server_);
        if (ret == ESP_OK) {
            web_server_ = nullptr;
            web_control_enabled_ = false;
        }  
        return ret;
    }

public:
    WelleController(gpio_num_t scl, gpio_num_t sda) : scl_pin_(scl), sda_pin_(sda), pca9685_initialized_(false) {
        InitializeServos();
        LoadTrimsFromNVS();
        action_controller_ = new UnifiedActionController(this);
        
        if (InitializePCA9685() == ESP_OK) {
            action_queue_ = xQueueCreate(10, sizeof(ActionParams));
            QueueHomeAction();
            RegisterMcpTools();
            StartActionTask();
        }  
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        ESP_LOGI(TAG, "开始注册MCP工具...");

        // 协同轮子控制
        mcp_server.AddTool("self.welle.action",
                           "统一运动控制。格式: action_type:param。"
                           "移动: move_forward/move_backward:speed(0-90); "
                           "转向: turn_left/turn_right:angle(0-90);"
                           "特殊: stop, home",
                           PropertyList({Property("command", kPropertyTypeString, "home")}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string command = properties["command"].value<std::string>();
                               return action_controller_->ExecuteAction(command) ? "OK" : "FAILED";
                           });

        mcp_server.AddTool(
            "self.welle.set_trim",
            "校准单个舵机位置。设置指定舵机的微调参数，设置将永久保存。"
            "servo_type: 舵机类型(left_wheel/right_wheel/left_hand/right_hand/left_eye/right_eye/neck/head); "
            "trim_value: 微调值(-50到50度)",
            PropertyList({Property("servo_type", kPropertyTypeString, "left_wheel"),
                          Property("trim_value", kPropertyTypeInteger, 0, -50, 50)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string servo_type = properties["servo_type"].value<std::string>();
                int trim_value = properties["trim_value"].value<int>();

                ESP_LOGI(TAG, "设置舵机微调: %s = %d度", servo_type.c_str(), trim_value);

                const char* servo_names[] = {"left_wheel", "right_wheel", "left_hand", "right_hand",
                                            "left_eye", "right_eye", "neck", "head"};
                
                int channel = -1;
                for (int i = 0; i < 8; i++) {
                    if (servo_type == servo_names[i]) {
                        channel = i;
                        break;
                    }
                }
                
                if (channel == -1) {
                    return "错误：无效的舵机类型，请使用: left_wheel, right_wheel, left_hand, right_hand, left_eye, right_eye, neck, head";
                }
                
                // 更新微调值
                Settings settings("welle_trims", true);
                settings.SetInt(servo_type.c_str(), trim_value);
                servo_configs_[channel].trim = trim_value;
                
                // 测试舵机动作
                QueueServoAction(channel, servo_configs_[channel].center_angle);

                return "舵机 " + servo_type + " 微调设置为 " + std::to_string(trim_value) +
                       " 度，已永久保存";
            });

        mcp_server.AddTool("self.welle.get_trims", "获取当前的舵机微调设置", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("welle_trims", false);

                               const char* servo_names[] = {"left_wheel", "right_wheel", "left_hand", "right_hand",
                                                            "left_eye", "right_eye", "neck", "head"};
                               
                               std::string result = "{";
                               for (int i = 0; i < 8; i++) {
                                   if (i > 0) result += ",";
                                   int trim_value = settings.GetInt(servo_names[i], 0);
                                   result += "\"" + std::string(servo_names[i]) + "\":" + std::to_string(trim_value);
                               }
                               result += "}";

                               ESP_LOGI(TAG, "获取微调设置: %s", result.c_str());
                               return result;
                           });

        mcp_server.AddTool("self.welle.get_status", "获取机器人状态，返回 moving 或 idle",
                           PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                               return is_action_in_progress_ ? "moving" : "idle";
                           });

        // Web控制服务器管理
        mcp_server.AddTool("self.welle.start_web",
                           "启动Web控制服务器",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               esp_err_t ret = StartWebServer();
                               if (ret == ESP_OK) {
                                   esp_netif_ip_info_t ip_info;
                                   esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                                   if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                                       char ip_str[16];
                                       esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
                                       return std::string("Web控制服务器已启动，请访问IP: ") + ip_str + "端口: 8080";
                                   } else {
                                       return "Web控制服务器已启动，端口: 8080";
                                   }
                               } else {
                                   return "启动Web控制服务器失败";
                               }
                           });

        mcp_server.AddTool("self.welle.stop_web",
                           "停止Web控制服务器",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               esp_err_t ret = StopWebServer();
                               return ret == ESP_OK ? "Web控制服务器已停止" : "停止Web控制服务器失败";
                           });

        mcp_server.AddTool("self.welle.web_status",
                           "获取Web控制服务器运行状态",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (web_control_enabled_) {
                                   esp_netif_ip_info_t ip_info;
                                   esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                                   if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                                       char ip_str[16];
                                       esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
                                       return std::string("Web控制服务器运行中，地址：") + ip_str + "端口：8080";
                                   } else {
                                       return "Web控制服务器运行中，端口: 8080";
                                   }
                               } else {
                                   return "Web控制服务器未启动";
                               }
                           });
        ESP_LOGI(TAG, "MCP工具注册完成");
    }

    ~WelleController() {
        // 停止Web服务器
        if (web_server_ != nullptr) {
            StopWebServer();
        }
        
        // 清理任务和队列
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
        
        if (action_controller_ != nullptr) {
            delete action_controller_;
            action_controller_ = nullptr;
        }
    }
};

static WelleController* g_welle_controller = nullptr;

void InitializeWelleController(gpio_num_t scl, gpio_num_t sda) {
    if (g_welle_controller == nullptr) {
        g_welle_controller = new WelleController(scl, sda);
        ESP_LOGI(TAG, "Welle控制器已初始化并注册MCP工具");
    }
}
