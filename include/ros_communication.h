#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include <micro_ros_arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

#include "odometry.h"





class RosCommunication {
public:
    RosCommunication();
    void initialize();
    bool createRosEntities();

    void odom_define();
    void idle_define();
    void command_state_define();

    void command_define();
    void screen_define();

    void service_define();

    void publish_odom(
        float x,
        float y,
        float theta,
        float linear_vel,
        float angular_vel
    );
    void publish_idle(int32_t state);
    void publish_command_state(
        int32_t command_id,
        int32_t command_state
    );

    void executors_start();
    void start_receiving_msgs();
    bool start_tasks();
    void publish_pending_odom();

private:
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_publisher_t odom_pub;
    nav_msgs__msg__Odometry odom_msg;

    rcl_service_t status_service;
    std_srvs__srv__Trigger_Request status_req;
    std_srvs__srv__Trigger_Response status_res;

    rcl_subscription_t command_sub;
    std_msgs__msg__Int32MultiArray command_msg;
    int32_t command_msg_buffer[3];

    rcl_publisher_t command_state_pub;
    std_msgs__msg__Int32MultiArray command_state_msg;
    int32_t command_state_data[2];

    rcl_publisher_t idle_pub;
    std_msgs__msg__Int32 idle_msg;

    rcl_subscription_t screen_sub;
    std_msgs__msg__String screen_msg;
    char screen_msg_buffer[128];

    char status_msg_buf[50];
};

// /command subscriber callback
void command_callback(
    const void * msgin
);


// /screen subscriber callback
void screen_callback(
    const void * msgin
);


// /get_status service callback
void status_service_callback(
    const void * req_msg,
    void * res_msg
);

#endif