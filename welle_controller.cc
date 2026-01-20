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
    enum ServoType {

        SERVO_360_DEGREE = 0,  // 360度连续旋转舵机
        SERVO_180_DEGREE = 1,  // 180度标准舵机
        SERVO_180_MIRRORED = 2, // 180度镜像安装舵机
        SERVO_360_MIRRORED = 3 // 360度镜像连续旋转舵机
    };

    struct ServoConfig {
        ServoType type;
        uint16_t min_angle;
        uint16_t max_angle;
        uint16_t center_angle;
        int trim;
    };
    
    ServoConfig servo_configs_[8];

    struct WelleActionParams {
        int action_type;
        int channel;
        int angle;
        int speed;
    };

    enum ActionType {
        ACTION_SET_SERVO_ANGLE = 1,
        ACTION_HOME = 2
    };

    uint16_t AngleToLusPulse(uint16_t angle) {
        // 1ms-2ms脉宽对应205-409 PWM值 (50Hz, 12位分辨率)
        return 102 + (angle * 410 / 180);
    }

    uint16_t ProcessServoAngle(uint8_t channel, uint16_t angle) {
        ServoConfig& config = servo_configs_[channel];
        int processed_angle = angle;
        
        // 应用微调
        processed_angle += config.trim;
        
        // 根据舵机类型处理角度
        switch (config.type) {
            case SERVO_360_DEGREE:
                // 360度舵机：90度停止，小于90度正转，大于90度反转
                break;
            case SERVO_360_MIRRORED:
                // 镜像安装舵机：角度反向
                processed_angle = 180 - processed_angle;
                break;
            case SERVO_180_DEGREE:
                // 标准180度舵机
                break;
            case SERVO_180_MIRRORED:
                // 镜像安装舵机：角度反向
                processed_angle = 180 - processed_angle;
                break;
        }
        
        // 角度限制
        processed_angle = std::max((int)config.min_angle, 
                                  std::min((int)config.max_angle, processed_angle));
        
        return (uint16_t)processed_angle;
    }

    esp_err_t SetServoAngle(uint8_t channel, uint16_t angle) {
        if (channel >= 8) {
            ESP_LOGE(TAG, "无效的舵机通道: %d", channel);
            return ESP_ERR_INVALID_ARG;
        }
        
        if (!pca9685_initialized_) {
            ESP_LOGE(TAG, "PCA9685未初始化");
            return ESP_FAIL;
        }

        uint16_t processed_angle = ProcessServoAngle(channel, angle);
        uint16_t pwm_value = AngleToLusPulse(processed_angle);
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
        WelleActionParams params;

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", params.action_type);
                controller->is_action_in_progress_ = true;

                switch (params.action_type) {
                    case ACTION_SET_SERVO_ANGLE:
                        controller->SetServoAngle(params.channel, params.angle);
                        if (params.speed > 0) {
                            vTaskDelay(pdMS_TO_TICKS(params.speed));
                        }
                        break;
                    case ACTION_HOME:
                        // 回到中心位置
                        for (int i = 0; i < 8; i++) {
                            controller->SetServoAngle(i, controller->servo_configs_[i].center_angle);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                        break;
                }
                
                controller->is_action_in_progress_ = false;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "welle_action", 1024 * 3, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    void QueueServoAction(int channel, int angle, int delay_ms = 0) {
        if (channel < 0 || channel >= 8) {
            ESP_LOGW(TAG, "无效的舵机通道: %d", channel);
            return;
        }
        ESP_LOGI(TAG, "舵机控制: 通道=%d, 角度=%d°, 延时=%dms", channel, angle, delay_ms);
        WelleActionParams params = {ACTION_SET_SERVO_ANGLE, channel, angle, delay_ms};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }
    
    void QueueHomeAction() {
        ESP_LOGI(TAG, "回到中心位置");
        WelleActionParams params = {ACTION_HOME, 0, 0, 0};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void InitializeServos() {
        // 左轮 - 360度连续旋转
        servo_configs_[LEFT_WHEEL_CHANNEL] = {SERVO_360_MIRRORED, 0, 180, 90, 0};
        // 右轮 - 360度连续旋转
        servo_configs_[RIGHT_WHEEL_CHANNEL] = {SERVO_360_DEGREE, 0, 180, 90, 0};
        // 左手 - 180度镜像安装
        servo_configs_[LEFT_HAND_CHANNEL] = {SERVO_180_DEGREE, 0, 180, 90, 0};
        // 右手 - 180度标准
        servo_configs_[RIGHT_HAND_CHANNEL] = {SERVO_180_MIRRORED, 0, 180, 90, 0};
        // 左眼 - 180度镜像安装
        servo_configs_[LEFT_EYE_CHANNEL] = {SERVO_180_MIRRORED, 0, 180, 90, 0};
        // 右眼 - 180度标准
        servo_configs_[RIGHT_EYE_CHANNEL] = {SERVO_180_DEGREE, 0, 180, 90, 0};
        // 脖子 - 180度标准
        servo_configs_[NECK_CHANNEL] = {SERVO_180_MIRRORED, 0, 180, 90, 90};
        // 头 - 180度标准
        servo_configs_[HEAD_CHANNEL] = {SERVO_180_DEGREE, 0, 180, 90, 90};
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

    // WiFi消息解析函数，参考wall-e-code项目
    void ProcessWifiMessage(const char* message) {
        if (!message) {
            ESP_LOGE(TAG, "无效的消息参数");
            return;
        }
        
        ESP_LOGI(TAG, "处理WiFi消息: %s", message);
        
        // 处理预设动作
        if (strcmp(message, "preset1") == 0) {
            // 问候姿势: 左右手挥动，眼睛看向前方
            QueueServoAction(LEFT_HAND_CHANNEL, 45, 0);
            QueueServoAction(RIGHT_HAND_CHANNEL, 135, 100);
            QueueServoAction(LEFT_EYE_CHANNEL, 60, 0);
            QueueServoAction(RIGHT_EYE_CHANNEL, 120, 100);
            QueueServoAction(NECK_CHANNEL, 90, 0);
            QueueServoAction(HEAD_CHANNEL, 75, 0);
            return;
        } else if (strcmp(message, "preset2") == 0) {
            // 思考姿势: 一只手托下巴
            QueueServoAction(LEFT_HAND_CHANNEL, 120, 0);
            QueueServoAction(RIGHT_HAND_CHANNEL, 60, 100);
            QueueServoAction(LEFT_EYE_CHANNEL, 45, 0);
            QueueServoAction(RIGHT_EYE_CHANNEL, 135, 100);
            QueueServoAction(NECK_CHANNEL, 75, 0);
            QueueServoAction(HEAD_CHANNEL, 105, 0);
            return;
        } else if (strcmp(message, "preset3") == 0) {
            // 开心姿势: 双手举高
            QueueServoAction(LEFT_HAND_CHANNEL, 30, 0);
            QueueServoAction(RIGHT_HAND_CHANNEL, 150, 100);
            QueueServoAction(LEFT_EYE_CHANNEL, 75, 0);
            QueueServoAction(RIGHT_EYE_CHANNEL, 105, 100);
            QueueServoAction(NECK_CHANNEL, 90, 0);
            QueueServoAction(HEAD_CHANNEL, 60, 0);
            return;
        } else if (strcmp(message, "center") == 0) {
            // 回中位
            QueueHomeAction();
            return;
        }
        
        // 解析单个舵机控制指令，格式: "s1:180,s2:90,s3:45,..."
        char* msg_copy = strdup(message);
        if (!msg_copy) {
            ESP_LOGE(TAG, "内存分配失败");
            return;
        }
        
        // 按逗号分割字符串
        char* token = strtok(msg_copy, ",");
        int processed_count = 0;
        
        while (token && processed_count < 8) {
            // 解析格式 "sX:YYY"
            if (token[0] != 's') {
                ESP_LOGE(TAG, "无效格式: %s (缺少's')", token);
                token = strtok(NULL, ",");
                continue;
            }
            
            // 查找冒号分隔符
            char* colon_ptr = strchr(token, ':');
            if (!colon_ptr) {
                ESP_LOGE(TAG, "无效格式: %s (缺少':')", token);
                token = strtok(NULL, ",");
                continue;
            }
            
            // 分割字符串并提取舵机编号
            *colon_ptr = '\0';
            int servo_num = atoi(token + 1);  // 跳过's'
            if (servo_num < 1 || servo_num > 8) {
                ESP_LOGE(TAG, "舵机编号超出范围: %d", servo_num);
                token = strtok(NULL, ",");
                continue;
            }
            
            // 提取角度值
            int angle = atoi(colon_ptr + 1);
            if (angle < 0 || angle > 180) {
                ESP_LOGE(TAG, "角度超出范围: %d", angle);
                token = strtok(NULL, ",");
                continue;
            }
            
            // 执行舵机控制 (转换为0-7的通道号)
            QueueServoAction(servo_num - 1, angle, 0);
            ESP_LOGI(TAG, "设置舵机S%d角度为%d°", servo_num, angle);
            processed_count++;
            
            token = strtok(NULL, ",");
        }
        
        free(msg_copy);
        ESP_LOGI(TAG, "WiFi消息处理完成，共处理%d个舵机指令", processed_count);
    }

    // HTTP请求处理器 - 主页面
    static esp_err_t WebRootHandler(httpd_req_t *req) {
        WelleController* controller = static_cast<WelleController*>(req->user_ctx);
        
        const char* html_response = 
            "<!DOCTYPE html><html><head><meta charset='utf-8'><title>WALL-E 控制</title>"
            "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
            "<style>"
            "body { font-family: 'Segoe UI', Arial, sans-serif; text-align: center; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); margin: 0; padding: 20px; color: #333; }"
            ".container { max-width: 900px; margin: 0 auto; background: rgba(255,255,255,0.95); padding: 30px; border-radius: 15px; box-shadow: 0 8px 25px rgba(0,0,0,0.3); }"
            "h1 { color: #4a5568; margin-bottom: 30px; font-size: 2.5em; text-shadow: 2px 2px 4px rgba(0,0,0,0.1); }"
            ".servo-group { margin: 25px 0; padding: 20px; background: #f8f9fa; border-radius: 10px; border-left: 4px solid #4CAF50; }"
            ".servo-group h3 { margin-top: 0; color: #2d3748; font-size: 1.4em; }"
            ".servo-control { margin: 15px 0; display: flex; align-items: center; justify-content: space-between; }"
            ".servo-label { font-weight: bold; color: #4a5568; min-width: 120px; text-align: left; }"
            "input[type=range] { flex: 1; margin: 0 15px; height: 8px; background: #ddd; border-radius: 5px; outline: none; }"
            "input[type=range]::-webkit-slider-thumb { appearance: none; width: 20px; height: 20px; background: #4CAF50; cursor: pointer; border-radius: 50%; }"
            ".angle-display { font-weight: bold; color: #2d3748; min-width: 50px; text-align: right; font-size: 1.1em; }"
            ".preset-buttons { margin: 30px 0; }"
            "button { padding: 12px 24px; margin: 8px; background: linear-gradient(145deg, #4CAF50, #45a049); color: white; border: none; border-radius: 8px; cursor: pointer; font-size: 16px; font-weight: bold; box-shadow: 0 4px 10px rgba(76, 175, 80, 0.3); transition: all 0.3s ease; }"
            "button:hover { transform: translateY(-2px); box-shadow: 0 6px 15px rgba(76, 175, 80, 0.4); }"
            "button:active { transform: translateY(0); }"
            ".reset-btn { background: linear-gradient(145deg, #ff6b6b, #ee5a52); box-shadow: 0 4px 10px rgba(255, 107, 107, 0.3); }"
            ".reset-btn:hover { box-shadow: 0 6px 15px rgba(255, 107, 107, 0.4); }"
            "#status { margin: 25px 0; padding: 15px; background: #e3f2fd; border-radius: 8px; color: #1976d2; font-weight: bold; border: 1px solid #bbdefb; }"
            ".emoji { font-size: 1.2em; margin-right: 8px; }"
            "@media (max-width: 600px) { .servo-control { flex-direction: column; } .servo-label, .angle-display { margin: 5px 0; } }"
            "</style></head><body>"
            "<div class='container'>"
            "<h1><span class='emoji'>🤖</span>WALL-E 控制面板</h1>"
            
            "<div class='servo-group'>"
            "<h3><span class='emoji'>🚗</span>移动控制</h3>"
            "<div class='servo-control'>"
            "<span class='servo-label'>左轮</span>"
            "<input type='range' id='s1' min='0' max='180' value='90' oninput='updateServo(1, this.value)'>"
            "<span class='angle-display' id='s1_val'>90°</span>"
            "</div>"
            "<div class='servo-control'>"
            "<span class='servo-label'>右轮</span>"
            "<input type='range' id='s2' min='0' max='180' value='90' oninput='updateServo(2, this.value)'>"
            "<span class='angle-display' id='s2_val'>90°</span>"
            "</div></div>"
            
            "<div class='servo-group'>"
            "<h3><span class='emoji'>🙌</span>手部控制</h3>"
            "<div class='servo-control'>"
            "<span class='servo-label'>左手</span>"
            "<input type='range' id='s3' min='0' max='180' value='90' oninput='updateServo(3, this.value)'>"
            "<span class='angle-display' id='s3_val'>90°</span>"
            "</div>"
            "<div class='servo-control'>"
            "<span class='servo-label'>右手</span>"
            "<input type='range' id='s4' min='0' max='180' value='90' oninput='updateServo(4, this.value)'>"
            "<span class='angle-display' id='s4_val'>90°</span>"
            "</div></div>"
            
            "<div class='servo-group'>"
            "<h3><span class='emoji'>👀</span>眼部控制</h3>"
            "<div class='servo-control'>"
            "<span class='servo-label'>左眼</span>"
            "<input type='range' id='s5' min='0' max='180' value='90' oninput='updateServo(5, this.value)'>"
            "<span class='angle-display' id='s5_val'>90°</span>"
            "</div>"
            "<div class='servo-control'>"
            "<span class='servo-label'>右眼</span>"
            "<input type='range' id='s6' min='0' max='180' value='90' oninput='updateServo(6, this.value)'>"
            "<span class='angle-display' id='s6_val'>90°</span>"
            "</div></div>"
            
            "<div class='servo-group'>"
            "<h3><span class='emoji'>🗣️</span>头部控制</h3>"
            "<div class='servo-control'>"
            "<span class='servo-label'>脖子</span>"
            "<input type='range' id='s7' min='0' max='180' value='90' oninput='updateServo(7, this.value)'>"
            "<span class='angle-display' id='s7_val'>90°</span>"
            "</div>"
            "<div class='servo-control'>"
            "<span class='servo-label'>头部</span>"
            "<input type='range' id='s8' min='0' max='180' value='90' oninput='updateServo(8, this.value)'>"
            "<span class='angle-display' id='s8_val'>90°</span>"
            "</div></div>"
            
            "<div class='preset-buttons'>"
            "<button onclick='sendCommand(\"preset1\")'>🤝 预设1: 问候</button>"
            "<button onclick='sendCommand(\"preset2\")'>🤔 预设2: 思考</button>"
            "<button onclick='sendCommand(\"preset3\")'>🎉 预设3: 开心</button>"
            "<button class='reset-btn' onclick='sendCommand(\"center\")'>🏠 回中位</button>"
            "</div>"
            
            "<div id='status'>🟢 就绪 - 请拖动滑块或点击预设动作</div>"
            "</div>"
            
            "<script>"
            "function updateServo(servo, angle) {"
            "  document.getElementById('s' + servo + '_val').textContent = angle + '°';"
            "  sendServoCommand(servo, angle);"
            "}"
            "function sendServoCommand(servo, angle) {"
            "  var cmd = 's' + servo + ':' + angle;"
            "  sendCommand(cmd);"
            "}"
            "function sendCommand(cmd) {"
            "  document.getElementById('status').textContent = '📡 发送中: ' + cmd;"
            "  document.getElementById('status').style.background = '#fff3e0';"
            "  document.getElementById('status').style.color = '#f57c00';"
            "  var xhr = new XMLHttpRequest();"
            "  xhr.open('POST', '/control', true);"
            "  xhr.setRequestHeader('Content-Type', 'text/plain');"
            "  xhr.onreadystatechange = function() {"
            "    if (xhr.readyState == 4) {"
            "      if (xhr.status == 200) {"
            "        document.getElementById('status').textContent = '✅ 命令已执行: ' + cmd;"
            "        document.getElementById('status').style.background = '#e8f5e8';"
            "        document.getElementById('status').style.color = '#2e7d32';"
            "      } else {"
            "        document.getElementById('status').textContent = '❌ 发送失败: ' + cmd;"
            "        document.getElementById('status').style.background = '#ffebee';"
            "        document.getElementById('status').style.color = '#c62828';"
            "      }"
            "    }"
            "  };"
            "  xhr.send(cmd);"
            "}"
            "setTimeout(function() {"
            "  document.getElementById('status').textContent = '🟢 就绪 - 请拖动滑块或点击预设动作';"
            "  document.getElementById('status').style.background = '#e3f2fd';"
            "  document.getElementById('status').style.color = '#1976d2';"
            "}, 3000);"
            "</script></body></html>";
        
        httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);
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
        
        if (InitializePCA9685() == ESP_OK) {
            action_queue_ = xQueueCreate(10, sizeof(WelleActionParams));
            QueueHomeAction();
            RegisterMcpTools();
        }  
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        ESP_LOGI(TAG, "开始注册MCP工具...");

        // 协同轮子控制
        mcp_server.AddTool("self.welle.move_forward",
                           "前进。speed: 速度(1-100); duration: 持续时间(毫秒)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 50, 1, 100),
                                         Property("duration", kPropertyTypeInteger, 1000, 100, 5000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int duration = properties["duration"].value<int>();
                               // 双轮同向前进
                               int wheel_angle = 90 + speed;
                               QueueServoAction(LEFT_WHEEL_CHANNEL, wheel_angle, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, wheel_angle, duration);
                               // 停止
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90, 0);
                               return true;
                           });

        mcp_server.AddTool("self.welle.move_backward",
                           "后退。speed: 速度(1-100); duration: 持续时间(毫秒)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 50, 1, 100),
                                         Property("duration", kPropertyTypeInteger, 1000, 100, 5000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int duration = properties["duration"].value<int>();
                               // 双轮同向后退
                               int wheel_angle = 90 - speed;
                               QueueServoAction(LEFT_WHEEL_CHANNEL, wheel_angle, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, wheel_angle, duration);
                               // 停止
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90, 0);
                               return true;
                           });

        mcp_server.AddTool("self.welle.turn_left",
                           "左转。speed: 速度(1-100); duration: 持续时间(毫秒)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 50, 1, 100),
                                         Property("duration", kPropertyTypeInteger, 1000, 100, 5000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int duration = properties["duration"].value<int>();
                               // 左轮反转，右轮正转
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90 - speed, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90 + speed, duration);
                               // 停止
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90, 0);
                               return true;
                           });

        mcp_server.AddTool("self.welle.turn_right",
                           "右转。speed: 速度(1-100); duration: 持续时间(毫秒)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 50, 1, 100),
                                         Property("duration", kPropertyTypeInteger, 1000, 100, 5000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int duration = properties["duration"].value<int>();
                               // 左轮正转，右轮反转
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90 + speed, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90 - speed, duration);
                               // 停止
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90, 0);
                               return true;
                           });

        mcp_server.AddTool("self.welle.stop_wheels",
                           "停止车轮",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               QueueServoAction(LEFT_WHEEL_CHANNEL, 90, 0);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, 90, 0);
                               return true;
                           });

        // 单个轮子控制 (高级用户)
        mcp_server.AddTool("self.welle.left_wheel",
                           "左轮独立控制。speed: 速度(-100到100，负值反转，0停止，正值正转); delay: 延时(毫秒)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 0, -100, 100),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int delay = properties["delay"].value<int>();
                               int angle = 90 + (speed * 90 / 100);
                               QueueServoAction(LEFT_WHEEL_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.right_wheel",
                           "右轮独立控制。speed: 速度(-100到100，负值反转，0停止，正值正转); delay: 延时(毫秒)",
                           PropertyList({Property("speed", kPropertyTypeInteger, 0, -100, 100),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int speed = properties["speed"].value<int>();
                               int delay = properties["delay"].value<int>();
                               int angle = 90 + (speed * 90 / 100);
                               QueueServoAction(RIGHT_WHEEL_CHANNEL, angle, delay);
                               return true;
                           });

        // 手部控制
        mcp_server.AddTool("self.welle.left_hand",
                           "左手上下移动。angle: 角度(0-180度); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 180),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(LEFT_HAND_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.right_hand",
                           "右手上下移动。angle: 角度(0-180度); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 180),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(RIGHT_HAND_CHANNEL, angle, delay);
                               return true;
                           });

        // 眼部控制
        mcp_server.AddTool("self.welle.left_eye",
                           "左眼上下移动。angle: 角度(0-180度); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 180),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(LEFT_EYE_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.right_eye",
                           "右眼上下移动。angle: 角度(0-180度); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 180),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(RIGHT_EYE_CHANNEL, angle, delay);
                               return true;
                           });

        // 头颈控制
        mcp_server.AddTool("self.welle.neck",
                           "脖子左右转动。angle: 角度(0-180度); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 180),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(NECK_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.head",
                           "头部上下移动。angle: 角度(0-180度); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 180),
                                         Property("delay", kPropertyTypeInteger, 0, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(HEAD_CHANNEL, angle, delay);
                               return true;
                           });

        // 姿态控制动作

        // 姿态控制
        mcp_server.AddTool("self.welle.look_up",
                           "抬头。angle: 角度(90-180); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 120, 90, 180),
                                         Property("delay", kPropertyTypeInteger, 500, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(HEAD_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.look_down",
                           "低头。angle: 角度(0-90); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 60, 0, 90),
                                         Property("delay", kPropertyTypeInteger, 500, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(HEAD_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.turn_head_left",
                           "左转头。angle: 角度(90-180); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 135, 90, 180),
                                         Property("delay", kPropertyTypeInteger, 500, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(NECK_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.turn_head_right",
                           "右转头。angle: 角度(0-90); delay: 延时(毫秒)",
                           PropertyList({Property("angle", kPropertyTypeInteger, 45, 0, 90),
                                         Property("delay", kPropertyTypeInteger, 500, 0, 2000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int angle = properties["angle"].value<int>();
                               int delay = properties["delay"].value<int>();
                               QueueServoAction(NECK_CHANNEL, angle, delay);
                               return true;
                           });

        mcp_server.AddTool("self.welle.wave_hands",
                           "挥手。cycles: 挥手次数(1-10); speed: 速度(毫秒)",
                           PropertyList({Property("cycles", kPropertyTypeInteger, 3, 1, 10),
                                         Property("speed", kPropertyTypeInteger, 300, 100, 1000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int cycles = properties["cycles"].value<int>();
                               int speed = properties["speed"].value<int>();
                               for (int i = 0; i < cycles; i++) {
                                   // 左右手交替上下挥动
                                   QueueServoAction(LEFT_HAND_CHANNEL, 45, speed);
                                   QueueServoAction(RIGHT_HAND_CHANNEL, 135, speed);
                                   QueueServoAction(LEFT_HAND_CHANNEL, 135, speed);
                                   QueueServoAction(RIGHT_HAND_CHANNEL, 45, speed);
                               }
                               // 回到中心位置
                               QueueServoAction(LEFT_HAND_CHANNEL, 90, speed);
                               QueueServoAction(RIGHT_HAND_CHANNEL, 90, 0);
                               return true;
                           });

        mcp_server.AddTool("self.welle.blink_eyes",
                           "眨眼。cycles: 眨眼次数(1-10); speed: 速度(毫秒)",
                           PropertyList({Property("cycles", kPropertyTypeInteger, 3, 1, 10),
                                         Property("speed", kPropertyTypeInteger, 200, 50, 1000)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               int cycles = properties["cycles"].value<int>();
                               int speed = properties["speed"].value<int>();
                               for (int i = 0; i < cycles; i++) {
                                   // 闭眼
                                   QueueServoAction(LEFT_EYE_CHANNEL, 45, speed);
                                   QueueServoAction(RIGHT_EYE_CHANNEL, 45, speed);
                                   // 张眼
                                   QueueServoAction(LEFT_EYE_CHANNEL, 135, speed);
                                   QueueServoAction(RIGHT_EYE_CHANNEL, 135, speed);
                               }
                               // 回到中心位置
                               QueueServoAction(LEFT_EYE_CHANNEL, 90, speed);
                               QueueServoAction(RIGHT_EYE_CHANNEL, 90, 0);
                               return true;
                           });
    
        mcp_server.AddTool("self.welle.home", "回到中心位置", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               QueueHomeAction();
                               return true;
                           });

        mcp_server.AddTool("self.welle.stop", "立即停止", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (action_task_handle_ != nullptr) {
                                   vTaskDelete(action_task_handle_);
                                   action_task_handle_ = nullptr;
                               }
                               is_action_in_progress_ = false;
                               xQueueReset(action_queue_);
                               QueueHomeAction();
                               return true;
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
                QueueServoAction(channel, servo_configs_[channel].center_angle, 100);

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
        mcp_server.AddTool("self.welle.start_web_control",
                           "启动Web控制服务器，用户可通过浏览器访问IP地址控制机器人",
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

        mcp_server.AddTool("self.welle.stop_web_control",
                           "停止Web控制服务器",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               esp_err_t ret = StopWebServer();
                               return ret == ESP_OK ? "Web控制服务器已停止" : "停止Web控制服务器失败";
                           });

        mcp_server.AddTool("self.welle.web_control_status",
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

        // 处理WiFi消息的MCP工具 (用于调试和高级用户)
        mcp_server.AddTool("self.welle.process_wifi_message",
                           "处理WiFi控制消息进行舵机控制。message: 消息内容(支持格式: s1:180,s2:90 或 preset1/preset2/preset3/center)",
                           PropertyList({Property("message", kPropertyTypeString, "center")}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string message = properties["message"].value<std::string>();
                               ProcessWifiMessage(message.c_str());
                               return "已处理WiFi消息: " + message;
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
    }
};

static WelleController* g_welle_controller = nullptr;

void InitializeWelleController(gpio_num_t scl, gpio_num_t sda) {
    if (g_welle_controller == nullptr) {
        g_welle_controller = new WelleController(scl, sda);
        ESP_LOGI(TAG, "Welle控制器已初始化并注册MCP工具");
    }
}
