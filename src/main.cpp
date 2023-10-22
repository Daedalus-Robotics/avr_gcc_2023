#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "nodes/gimbal_motor_node.hpp"

#define EXECUTOR_HANDLES ((SYSTEM_EXECUTOR_HANDLES + GIMBAL_MOTOR_NODE_EXECUTOR_HANDLES * 1) + 5)

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

GimbalMotorNode *xAxisMotorNode;
//GimbalMotorNode *yAxisMotorNode;

FLASHMEM void cleanup()
{
//    yAxisMotorNode->cleanup();
    xAxisMotorNode->cleanup();
    cleanupSystem();
}

FASTRUN void eStop()
{
    xAxisMotorNode->eStop();
//    yAxisMotorNode->eStop();
}

FLASHMEM void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    initSystem(cleanup, eStop);

    xAxisMotorNode = new GimbalMotorNode("gimbal/x", 1);
//    yAxisMotorNode = new GimbalMotorNode("gimbal/y", 1);

    allocator = rcl_get_default_allocator();
    HANDLE_ROS_ERROR(rclc_support_init(&support, 0, nullptr, &allocator), true);
    HANDLE_ROS_ERROR(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator), true);

    setupSystem(&support, &executor);
    xAxisMotorNode->setup(&support, &executor);
//    yAxisMotorNode->setup(&support, &executor);
}

void loop()
{
    HANDLE_ROS_ERROR(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)), false);
}