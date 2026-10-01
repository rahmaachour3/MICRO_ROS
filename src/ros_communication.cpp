#include "ros_communication.h"
#include "robot_protocole.h"
#include <string.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
namespace
{
constexpr UBaseType_t COMMAND_QUEUE_LENGTH = 5;
constexpr UBaseType_t SCREEN_QUEUE_LENGTH = 3;
constexpr UBaseType_t PUBLISH_QUEUE_LENGTH = 20;
constexpr UBaseType_t MOTOR_TASK_PRIORITY = 3;
constexpr UBaseType_t ODOMETRY_TASK_PRIORITY = 2;
constexpr UBaseType_t MICRO_ROS_TASK_PRIORITY = 2;
constexpr UBaseType_t SCREEN_TASK_PRIORITY = 1;
constexpr TickType_t ODOMETRY_PERIOD = pdMS_TO_TICKS(50);

volatile IdleState robot_idle_state = IdleState::READY;

struct ScreenMessage
{
    size_t length;
    char data[128];
};

struct OdometrySnapshot
{
    float x;
    float y;
    float theta;
    float linear_velocity;
    float angular_velocity;
};

enum class PublishEventType : uint8_t
{
    IDLE,
    COMMAND_STATE
};

struct PublishEvent
{
    PublishEventType type;
    int32_t command_id;
    int32_t value;
};

QueueHandle_t command_queue = nullptr;
QueueHandle_t screen_queue = nullptr;
QueueHandle_t odometry_queue = nullptr;
QueueHandle_t publish_queue = nullptr;

void queue_idle_state(IdleState state)
{
    if (publish_queue == nullptr)
    {
        return;
    }

    const PublishEvent event = {
        PublishEventType::IDLE,
        0,
        static_cast<int32_t>(state)
    };
    xQueueSend(publish_queue, &event, portMAX_DELAY);
}

void queue_command_state(int32_t command_id, CommandState state)
{
    if (publish_queue == nullptr)
    {
        return;
    }

    const PublishEvent event = {
        PublishEventType::COMMAND_STATE,
        command_id,
        static_cast<int32_t>(state)
    };
    xQueueSend(publish_queue, &event, portMAX_DELAY);
}

void motor_command_task(void *argument)
{
    (void)argument;
    RobotCommand command;

    for (;;)
    {
        if (xQueueReceive(command_queue, &command, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }


        

        switch (command.type)
        {
            case CommandType::FORWARD:
                queue_command_state(
                    command.id,
                CommandState::ACCEPTED
             );
                robot_idle_state = IdleState::BUSY;
                queue_idle_state(robot_idle_state);

            //queue_command_state(command.id, CommandState::COMPLETED);

            break;
            case CommandType::BACKWARD:
                queue_command_state(command.id, CommandState::ACCEPTED);
                robot_idle_state = IdleState::BUSY;
                queue_idle_state(robot_idle_state);

            queue_command_state(command.id,CommandState::COMPLETED);
                break;


            case CommandType::ROTATE:
                queue_command_state(command.id,CommandState::ACCEPTED );
                robot_idle_state = IdleState::BUSY;
                queue_idle_state(robot_idle_state);

                queue_command_state(command.id,CommandState::COMPLETED);
                break;

            case CommandType::PIGNON:
                queue_command_state(command.id,CommandState::ACCEPTED );
                robot_idle_state = IdleState::BUSY;
                queue_idle_state(robot_idle_state);

                queue_command_state(command.id,CommandState::COMPLETED);
                break;

            case CommandType::STOP:
                queue_command_state(command.id,CommandState::ACCEPTED);
                robot_idle_state = IdleState::BUSY;
                queue_idle_state(robot_idle_state);

                // Connect the motor driver here; command arrived off the ROS callback.
                break;

            default:

                robot_idle_state = IdleState::FAULT;
                queue_idle_state(robot_idle_state);

                queue_command_state(command.id,CommandState::FAILED);

                continue;
        }


    }
}

void odometry_task(void *argument)
{
    (void)argument;

    for (;;)
    {
        updatePosition();

        const OdometrySnapshot snapshot = {
            static_cast<float>(current_pose.x / 1000.0),
            static_cast<float>(current_pose.y / 1000.0),
            static_cast<float>(current_pose.theta),
            static_cast<float>(current_velocity.linear / 1000.0),
            static_cast<float>(current_velocity.angular)
        };

        xQueueOverwrite(odometry_queue, &snapshot);
        vTaskDelay(ODOMETRY_PERIOD);
    }
}

void micro_ros_task(void *argument)
{
    auto *ros = static_cast<RosCommunication *>(argument);

    for (;;)
    {
        ros->start_receiving_msgs();
        ros->publish_pending_odom();
        ros->publish_idle(
        static_cast<int32_t>(IdleState::READY)
    );

        PublishEvent event;
        while (publish_queue != nullptr &&
               xQueueReceive(publish_queue, &event, 0) == pdTRUE)
        {
            if (event.type == PublishEventType::IDLE)
            {
                ros->publish_idle(event.value);
            }
            else
            {
                ros->publish_command_state(event.command_id, event.value);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void screen_task(void *argument)
{
    (void)argument;
    ScreenMessage message;

    for (;;)
    {
        if (xQueueReceive(screen_queue, &message, portMAX_DELAY) == pdTRUE)
        {
            message.data[message.length] = '\0';
            Wire.begin(21, 22);  // ESP32 SDA = GPIO21, SCL = GPIO22

            lcd.init();
            lcd.backlight();

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.printf(message.data);
 

        }
    }
}
}



void status_service_callback(const void *req_msg, void *res_msg)
{
    (void)req_msg;

    std_srvs__srv__Trigger_Response *response =
        (std_srvs__srv__Trigger_Response *)res_msg;

    response->success = true;

    const char *text = "ESP32 online";
    response->message.data = (char *)text;
    response->message.size = strlen(text);
    response->message.capacity = strlen(text) + 1;
}


void command_callback(const void *msgin)
{
    const auto *msg =
        static_cast<const std_msgs__msg__Int32MultiArray *>(msgin);

    if (msg == nullptr)
        return;

    if (msg->data.size != 3)
    {
        return;
    }

    const int32_t command_id = msg->data.data[0];
    const int32_t command_type = msg->data.data[1];
    const int32_t amount = msg->data.data[2];

    CommandType command =
        static_cast<CommandType>(command_type);

    if (command != CommandType::FORWARD &&
        command != CommandType::BACKWARD &&
        command != CommandType::ROTATE &&
        command != CommandType::STOP)
    {
        return;
    }

    if (command_queue == nullptr)
    {
        return;
    }

    RobotCommand queued_command = {
        command_id,
        command,
        amount
    };

    if (xQueueSend(command_queue, &queued_command, 0) != pdTRUE)
    {
        return;
    }

}

void screen_callback(const void *msgin)
{
    const auto *msg =
        static_cast<const std_msgs__msg__String *>(msgin);

    if (msg == nullptr || msg->data.data == nullptr)
    {
        return;
    }

    if (screen_queue == nullptr)
    {
        return;
    }

    ScreenMessage screen_message = {};
    screen_message.length = msg->data.size;
    if (screen_message.length >= sizeof(screen_message.data))
    {
        screen_message.length = sizeof(screen_message.data) - 1;
    }

    memcpy(screen_message.data, msg->data.data, screen_message.length);
    screen_message.data[screen_message.length] = '\0';
    xQueueSend(screen_queue, &screen_message, 0);
}



RosCommunication::RosCommunication()
{
}

bool RosCommunication::createRosEntities()
{
    delay(2000);
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(
            &support,
            0,
            nullptr,
            &allocator) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_node_init_default(
            &node,
            "esp32_robot",
            "",
            &support) != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_publisher_init_default(
            &idle_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "/idle") != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_publisher_init_default(
            &command_state_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/command_state") != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_publisher_init_default(
            &odom_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/odometry") != RCL_RET_OK)
    {
        return false;
    }

    command_msg.data.data = command_msg_buffer;
    command_msg.data.size = 0;
    command_msg.data.capacity = sizeof(command_msg_buffer) / sizeof(command_msg_buffer[0]);
    if (rclc_subscription_init_default(
            &command_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/command") != RCL_RET_OK)
    {
        return false;
    }

    screen_msg.data.data = screen_msg_buffer;
    screen_msg.data.size = 0;
    screen_msg.data.capacity = sizeof(screen_msg_buffer);
    if (rclc_subscription_init_default(
            &screen_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/screen") != RCL_RET_OK)
    {
        return false;
    }

    if (rclc_service_init_default(
            &status_service,
            &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
            "/get_status") != RCL_RET_OK)
    {
        return false;
    }

    return true;
}

void RosCommunication::initialize()
{

    // Allocator
    allocator = rcl_get_default_allocator();

    // Initialisation du support micro-ROS
    rclc_support_init(
        &support,
        0,
        NULL,
        &allocator
    );

    // Création du node ROS 2
    rclc_node_init_default(
        &node,
        "esp32_node",
        "",
        &support
    );

}


// ============================================================
// ODOMETRY PUBLISHER
// ============================================================

void RosCommunication::odom_define()
{
    rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            nav_msgs,
            msg,
            Odometry
        ),
        "/odometry"
    );
}

void RosCommunication::command_define()
{
    command_msg.data.data     = command_msg_buffer;
    command_msg.data.size     = 0;
    command_msg.data.capacity = sizeof(command_msg_buffer) / sizeof(command_msg_buffer[0]);
    rclc_subscription_init_default(
        &command_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Int32MultiArray
        ),
        "/command"
    );
}

void RosCommunication::screen_define()
{
    screen_msg.data.data = screen_msg_buffer;
    screen_msg.data.size = 0;
    screen_msg.data.capacity = sizeof(screen_msg_buffer);

    rclc_subscription_init_default(
        &screen_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            String
        ),
        "/screen"
    );
}


void RosCommunication::command_state_define()
{
    rclc_publisher_init_default(
        &command_state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Int32MultiArray
        ),
        "/command_state"
    );
}

void RosCommunication::idle_define()
{
    rclc_publisher_init_default(
        &idle_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Int32
        ),
        "/idle"
    );
}

void RosCommunication::publish_idle(int32_t state)
{
    idle_msg.data =
        static_cast<int32_t>(state);

    rcl_publish(
        &idle_pub,
        &idle_msg,
        nullptr
    );
}

void RosCommunication::service_define()
{
    rclc_service_init_default(
        &status_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(
            std_srvs,
            srv,
            Trigger
        ),
        "/get_status"
    );
}

void RosCommunication::publish_command_state(
    int32_t command_id,
    int32_t command_state)
{
    int32_t data[2];

    data[0] = command_id;
    data[1] = command_state;

    command_state_msg.data.data = data;
    command_state_msg.data.size = 2;
    command_state_msg.data.capacity = 2;

    rcl_publish(
        &command_state_pub,
        &command_state_msg,
        nullptr
    );
}


void RosCommunication::publish_odom(
    float x,
    float y,
    float theta,
    float linear_vel,
    float angular_vel)
{
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;

    // TODO:
    // Convert theta to quaternion.

    odom_msg.twist.twist.linear.x =
        linear_vel;

    odom_msg.twist.twist.angular.z =
        angular_vel;

    rcl_publish(
        &odom_pub,
        &odom_msg,
        nullptr
    );
}


void RosCommunication::executors_start()
{
    rcl_ret_t ret = rclc_executor_init(
        &executor,
        &support.context,
        3,
        &allocator
    );

    if (ret != RCL_RET_OK)
    {
        return;
    }


    // --------------------------------------------------------
    // /get_status
    // --------------------------------------------------------

    (void)rclc_executor_add_service(
        &executor,
        &status_service,
        &status_req,
        &status_res,
        &status_service_callback
    );


    // --------------------------------------------------------
    // /command
    // --------------------------------------------------------

    (void)rclc_executor_add_subscription(
        &executor,
        &command_sub,
        &command_msg,
        &command_callback,
        ON_NEW_DATA
    );

    (void)rclc_executor_add_subscription(
        &executor,
        &screen_sub,
        &screen_msg,
        &screen_callback,
        ON_NEW_DATA
    );
}


// ============================================================
// RECEIVE ROS 2 EVENTS
// ============================================================

void RosCommunication::start_receiving_msgs()
{
    rclc_executor_spin_some(
        &executor,
        RCL_MS_TO_NS(10)
    );
}

void RosCommunication::publish_pending_odom()
{
    OdometrySnapshot snapshot;

    if (odometry_queue != nullptr &&
        xQueueReceive(odometry_queue, &snapshot, 0) == pdTRUE)
    {
        publish_odom(
            snapshot.x,
            snapshot.y,
            snapshot.theta,
            snapshot.linear_velocity,
            snapshot.angular_velocity
        );
    }
}

bool RosCommunication::start_tasks()
{
    TaskHandle_t task_handles[4] = {};
    command_queue = xQueueCreate(
        COMMAND_QUEUE_LENGTH,
        sizeof(RobotCommand)
    );
    screen_queue = xQueueCreate(
        SCREEN_QUEUE_LENGTH,
        sizeof(ScreenMessage)
    );
    odometry_queue = xQueueCreate(1, sizeof(OdometrySnapshot));
    publish_queue = xQueueCreate(
        PUBLISH_QUEUE_LENGTH,
        sizeof(PublishEvent)
    );

    if (command_queue == nullptr || screen_queue == nullptr ||
        odometry_queue == nullptr || publish_queue == nullptr)
    {
        goto task_start_failed;
    }

    if (xTaskCreatePinnedToCore(
            motor_command_task,
            "motor_command",
            4096,
            nullptr,
            MOTOR_TASK_PRIORITY,
            &task_handles[0],
            0) != pdPASS)
    {
        goto task_start_failed;
    }

    if (xTaskCreatePinnedToCore(
            odometry_task,
            "odometry",
            4096,
            nullptr,
            ODOMETRY_TASK_PRIORITY,
            &task_handles[1],
            0) != pdPASS)
    {
        goto task_start_failed;
    }

    if (xTaskCreatePinnedToCore(
            screen_task,
            "screen",
            4096,
            nullptr,
            SCREEN_TASK_PRIORITY,
            &task_handles[2],
            1) != pdPASS)
    {
        goto task_start_failed;
    }

    if (xTaskCreatePinnedToCore(
            micro_ros_task,
            "micro_ros",
            8192,
            this,
            MICRO_ROS_TASK_PRIORITY,
            &task_handles[3],
            1) != pdPASS)
    {
        goto task_start_failed;
    }

    return true;

task_start_failed:
    for (TaskHandle_t task_handle : task_handles)
    {
        if (task_handle != nullptr)
        {
            vTaskDelete(task_handle);
        }
    }

    if (command_queue != nullptr)
    {
        vQueueDelete(command_queue);
        command_queue = nullptr;
    }
    if (screen_queue != nullptr)
    {
        vQueueDelete(screen_queue);
        screen_queue = nullptr;
    }
    if (odometry_queue != nullptr)
    {
        vQueueDelete(odometry_queue);
        odometry_queue = nullptr;
    }
    if (publish_queue != nullptr)
    {
        vQueueDelete(publish_queue);
        publish_queue = nullptr;
    }

    return false;
}