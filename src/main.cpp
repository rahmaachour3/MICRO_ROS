#include <Arduino.h>
#include "ros_communication.h"
#include <micro_ros_arduino.h>
#include "robot_protocole.h"

RosCommunication ros;
bool freertos_started = false;
bool ros_entities_created = false;

void setup() {
    set_microros_transports();


    ros_entities_created = ros.createRosEntities();
    if (!ros_entities_created)
    {
        return;
    }

    ros.executors_start();
    freertos_started = ros.start_tasks();
}

void loop() {
    if (ros_entities_created && !freertos_started)
    {
        ros.start_receiving_msgs();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}