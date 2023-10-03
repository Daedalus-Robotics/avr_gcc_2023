#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define EXECUTOR_HANDLES 0

rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, nullptr, &allocator);
    rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator);
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}