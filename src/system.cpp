#include "system.hpp"

#include <rcl/error_handling.h>
#include <rcl_interfaces/msg/log.h>
#include <rmw_microros/rmw_microros.h>
#include <std_srvs/srv/trigger.h>
#include <std_msgs/msg/empty.h>

#define ENABLE_BLINK_ERROR 0

#define DEBUG_LOG 0

bool setupDone = false;
bool resetScheduled = false;

void (*cleanupFunc)();

void (*eStopFunc)();

rcl_timer_t pingTimer;
rcl_node_t systemNode;
rcl_publisher_t loggerPublisher;
rcl_subscription_t eStopSubscriber;
std_msgs__msg__Empty eStopMessage;
rcl_service_t resetService;
std_srvs__srv__Trigger_Request resetServiceRequest;
std_srvs__srv__Trigger_Response resetServiceResponse;


[[noreturn]] void reset()
{
    delay(1000);
    SCB_AIRCR = 0x05FA0004;
    while (true)
    {
    }
}

bool log(const LogLevel level, const char msg[], const char file[], const char function[], uint32_t line)
{
#if !DEBUG_LOG
    if (level <= LOGLEVEL_DEBUG)
    {
        return true;
    }
#endif
    if (setupDone)
    {
        rcl_interfaces__msg__Log logger_msg;

        logger_msg.name.data = const_cast<char *>("GCC");
        logger_msg.name.size = 3;

        logger_msg.stamp.sec = millis() / 1000;
        logger_msg.stamp.nanosec = 0;

        logger_msg.level = level;
        logger_msg.msg.data = const_cast<char *>(msg);
        logger_msg.msg.size = strlen(msg);
        logger_msg.file.data = const_cast<char *>(file);
        logger_msg.file.size = strlen(file);
        logger_msg.function.data = const_cast<char *>(function);
        logger_msg.function.size = strlen(function);
        logger_msg.line = line;

        rcl_ret_t rc = rcl_publish(&loggerPublisher, &logger_msg, nullptr);
        return rc == RCL_RET_OK;
    }
    return false;
}

void blinkError(rcl_ret_t error)
{
    int digit;
    while (error > 0)
    {
        digit = error % 10;
        error /= 10;

        digitalWrite(LED_BUILTIN, 0);
        rclc_sleep_ms(2500);
        while (digit > 0)
        {
            rclc_sleep_ms(1000);
            digitalWrite(LED_BUILTIN, 1);
            rclc_sleep_ms(500);
            digitalWrite(LED_BUILTIN, 0);
            digit--;
        }
    }
    rclc_sleep_ms(2500);
}

bool handleError(const int32_t rc,
                 const bool do_reset,
                 const char file[], const char function[], uint32_t line)
{
    if (rc != RCL_RET_OK)
    {
        char message[32];

        snprintf(message, sizeof(message), "ROS ERROR: %li", rc);

        if (do_reset)
        {
            log(LOGLEVEL_FATAL, message, file, function, line);
#if (ENABLE_BLINK_ERROR == 1)
            blinkError(rc);
#endif
            reset();
        }
        else
        {
#if (ENABLE_BLINK_ERROR == 1)
            bool can_log = log(LOGLEVEL_ERROR, message, file, function, line);
            if (!can_log)
            {
                blinkError(rc);
            }
#else
            log(LOGLEVEL_ERROR, message, file, function, line);
#endif
        }
    }
    return rc == RCL_RET_OK;
}

void pingTimerCallback(__attribute__((unused)) rcl_timer_t *timer, __attribute__((unused)) int64_t last_call_time)
{
    if (resetScheduled)
    {
        LOG(LOGLEVEL_INFO, "Running scheduled reset");
        setupDone = false;
        cleanupFunc();
        reset();
    }

    rmw_ret_t ping_result = rmw_uros_ping_agent(100, 5);
    if (ping_result == RMW_RET_ERROR)
    {
        reset();
    }
}

void eStopCallback(__attribute__((unused)) const void *msgin)
{
    eStopFunc();
}

void resetCallback(__attribute__((unused)) const void *request, void *response)
{
    auto response_msg = (std_srvs__srv__Trigger_Response *) response;
    response_msg->success = true;
    response_msg->message.data = const_cast<char *>("Reset scheduled");
    response_msg->message.size = 15;

    resetScheduled = true;
    LOG(LOGLEVEL_INFO, "Reset scheduled");
}

void setupSystem(rclc_support_t *support, rclc_executor_t *executor)
{

    HANDLE_ROS_ERROR(rclc_timer_init_default(
            &pingTimer,
            support,
            RCL_MS_TO_NS(1000),
            &pingTimerCallback), true);
    HANDLE_ROS_ERROR(rclc_executor_add_timer(executor, &pingTimer), true);

    HANDLE_ROS_ERROR(rclc_node_init_default(&systemNode, "gcc_system", "gcc", support), true);
    HANDLE_ROS_ERROR(rclc_publisher_init_default(&loggerPublisher,
                                                 &systemNode,
                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
                                                 "/rosout"), true);
    setupDone = true;
    LOG(LOGLEVEL_INFO, "Setting up System");
    LOG(LOGLEVEL_INFO, "Logger started");

    HANDLE_ROS_ERROR(rclc_subscription_init_default(&eStopSubscriber, &systemNode,
                                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
                                                    "/e_stop"), true);

    HANDLE_ROS_ERROR(rclc_executor_add_subscription(executor,
                                                    &eStopSubscriber,
                                                    &eStopMessage,
                                                    eStopCallback,
                                                    ON_NEW_DATA), true);
    LOG(LOGLEVEL_INFO, "Set up eStop subscriber");

    HANDLE_ROS_ERROR(rclc_service_init_best_effort(&resetService,
                                                   &systemNode,
                                                   ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                                   "reset"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service(executor, &resetService,
                                               &resetServiceRequest, &resetServiceResponse,
                                               resetCallback), true);
    LOG(LOGLEVEL_DEBUG, "Set up reset service");
}

void cleanupSystem()
{
    HANDLE_ROS_ERROR(rcl_service_fini(&resetService, &systemNode), false);
    HANDLE_ROS_ERROR(rcl_subscription_fini(&eStopSubscriber, &systemNode), false);
    HANDLE_ROS_ERROR(rcl_publisher_fini(&loggerPublisher, &systemNode), false);
    HANDLE_ROS_ERROR(rcl_node_fini(&systemNode), false);
}

void initSystem(void (*cleanup_func)(), void (*e_stop_func)())
{
//    setupFunc = setup_func;
    cleanupFunc = cleanup_func;
    eStopFunc = e_stop_func;

}
