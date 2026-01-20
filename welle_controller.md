# Welle机器人控制器 - MCP协议版本

## 概述

`welle_controller.cc` 是一个基于ESP32的Welle（Wall-E）机器人控制器，使用PCA9685 PWM控制器来驱动8个舵机，实现机器人的各种动作。该控制器支持MCP（Model Control Protocol）协议，并提供了Web控制界面。

## 硬件架构

### 舵机通道映射

| 通道 | 部位 | 功能 |
|------|------|------|
| 0 | 左轮 | 前后移动 |
| 1 | 右轮 | 前后移动 |
| 2 | 左手 | 上下移动 |
| 3 | 右手 | 上下移动 |
| 4 | 左眼 | 上下移动 |
| 5 | 右眼 | 上下移动 |
| 6 | 脖子 | 左右转动 |
| 7 | 头部 | 上下移动 |

### 硬件组件

- **主控**: ESP32
- **PWM控制器**: PCA9685（I2C接口）
- **通信**: I2C（SCL/SDA引脚可配置）
- **舵机**: 8个舵机（180度标准舵机和360度连续旋转舵机）

## 核心功能

### 1. 舵机控制系统

#### 舵机类型支持
- `SERVO_360_DEGREE`: 360度连续旋转舵机
- `SERVO_180_DEGREE`: 180度标准舵机  
- `SERVO_180_MIRRORED`: 180度镜像安装舵机
- `SERVO_360_MIRRORED`: 360度镜像连续旋转舵机

#### 角度处理
- PWM脉宽范围：1ms-2ms (对应205-409 PWM值)
- 角度限制：0-180度
- 支持舵机微调（trim）功能
- 角度映射到PWM值：`102 + (angle * 410 / 180)`

### 2. 动作队列系统

使用FreeRTOS任务和队列实现异步动作控制：

```cpp
struct WelleActionParams {
    int action_type;  // 动作类型
    int channel;      // 舵机通道
    int angle;        // 目标角度
    int speed;        // 延时时间
};
```

支持的动作类型：
- `ACTION_SET_SERVO_ANGLE`: 设置舵机角度
- `ACTION_HOME`: 回到中心位置

### 3. Web控制界面

提供完整的Web控制界面，包括：
- 实时滑块控制8个舵机
- 预设动作按钮（问候、思考、开心、回中位）
- 状态显示和反馈
- 响应式设计，支持移动设备

#### Web API端点
- `GET /`: 主控制页面
- `POST /control`: 接收控制命令

### 4. WiFi消息处理

支持多种消息格式：

#### 预设动作
- `"preset1"`: 问候姿势（挥手、眼睛看前方）
- `"preset2"`: 思考姿势（托下巴）
- `"preset3"`: 开心姿势（双手举高）
- `"center"`: 回到中心位置

#### 自定义控制
格式：`"s1:180,s2:90,s3:45,..."`
- s1-s8对应8个舵机
- 角度范围：0-180度

### 5. MCP工具集成

注册了丰富的MCP工具，支持：

#### 移动控制
- `self.welle.move_forward`: 前进
- `self.welle.move_backward`: 后退
- `self.welle.turn_left`: 左转
- `self.welle.turn_right`: 右转
- `self.welle.stop_wheels`: 停止车轮

#### 部位控制
- `self.welle.left_hand` / `self.welle.right_hand`: 手部控制
- `self.welle.left_eye` / `self.welle.right_eye`: 眼部控制
- `self.welle.neck` / `self.welle.head`: 头颈控制

#### 动作控制
- `self.welle.wave_hands`: 挥手
- `self.welle.blink_eyes`: 眨眼
- `self.welle.look_up` / `self.welle.look_down`: 抬头/低头
- `self.welle.turn_head_left` / `self.welle.turn_head_right`: 转头

#### 系统管理
- `self.welle.home`: 回到中心位置
- `self.welle.stop`: 紧急停止
- `self.welle.get_status`: 获取状态
- `self.welle.set_trim` / `self.welle.get_trims`: 微调管理

#### Web服务器管理
- `self.welle.start_web_control`: 启动Web服务器
- `self.welle.stop_web_control`: 停止Web服务器
- `self.welle.web_control_status`: 获取Web服务器状态

## 关键类和方法

### WelleController类

#### 私有成员
- `i2c_dev_t pca9685_dev_`: PCA9685设备句柄
- `ServoConfig servo_configs_[8]`: 舵机配置数组
- `QueueHandle_t action_queue_`: 动作队列
- `TaskHandle_t action_task_handle_`: 动作任务句柄
- `httpd_handle_t web_server_`: Web服务器句柄

#### 核心方法

##### 初始化方法
- `InitializePCA9685()`: 初始化PCA9685控制器
- `InitializeServos()`: 初始化舵机配置
- `LoadTrimsFromNVS()`: 从NVS加载微调设置

##### 控制方法
- `SetServoAngle(uint8_t channel, uint16_t angle)`: 设置舵机角度
- `ProcessServoAngle(uint8_t channel, uint16_t angle)`: 处理舵机角度
- `AngleToLusPulse(uint16_t angle)`: 角度转换为PWM脉宽

##### 动作管理
- `QueueServoAction()`: 将动作加入队列
- `ActionTask()`: 动作执行任务（静态方法）
- `StartActionTaskIfNeeded()`: 按需启动动作任务

##### 消息处理
- `ProcessWifiMessage()`: 处理WiFi控制消息

##### Web服务器
- `StartWebServer()` / `StopWebServer()`: Web服务器管理
- `WebRootHandler()` / `WebControlHandler()`: HTTP请求处理器

## 配置和存储

### NVS存储
使用Settings类管理舵机微调值的持久化存储：
```cpp
Settings settings("welle_trims", false);
servo_configs_[i].trim = settings.GetInt(servo_names[i], 0);
```

### 舵机配置示例
```cpp
// 左轮 - 360度镜像连续旋转
servo_configs_[LEFT_WHEEL_CHANNEL] = {SERVO_360_MIRRORED, 0, 180, 90, 0};
// 右眼 - 180度标准舵机
servo_configs_[RIGHT_EYE_CHANNEL] = {SERVO_180_DEGREE, 0, 180, 90, 0};
```

## 安全特性

1. **参数验证**: 所有角度和通道参数都进行范围检查
2. **错误处理**: 完整的ESP32错误处理机制
3. **资源管理**: 正确的任务和队列生命周期管理
4. **防护机制**: 舵机角度限制和类型检查

## 依赖库

- **ESP-IDF**: ESP32开发框架
- **FreeRTOS**: 实时操作系统
- **PCA9685驱动**: I2C PWM控制器驱动
- **i2cdev**: I2C设备抽象层
- **ESP HTTP Server**: Web服务器组件
- **cJSON**: JSON解析库（用于MCP）

## 使用示例

### 初始化
```cpp
InitializeWelleController(GPIO_NUM_22, GPIO_NUM_21);  // SCL=22, SDA=21
```

### 直接控制
```cpp
// 通过MCP工具控制
self.welle.wave_hands(cycles=3, speed=300);
self.welle.move_forward(speed=70, duration=2000);
```

### Web控制
访问 `http://<ESP32_IP>:8080` 使用浏览器控制界面

### WiFi消息控制
发送控制消息：`"s1:120,s3:60,s7:135"` 或 `"preset1"`

## 特点总结

1. **模块化设计**: 清晰的类结构和方法分离
2. **多控制方式**: MCP工具、Web界面、WiFi消息
3. **异步执行**: 基于队列的非阻塞动作控制
4. **配置灵活**: 支持不同类型舵机和微调
5. **用户友好**: 直观的Web界面和丰富的预设动作
6. **扩展性强**: 易于添加新的控制命令和动作序列

该控制器为Welle机器人提供了完整的控制解决方案，适合教育、娱乐和机器人学习项目使用。