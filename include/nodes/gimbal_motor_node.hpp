#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <SimpleFOC.h>
#include <std_srvs/srv/trigger.h>
#include <std_srvs/srv/set_bool.h>
#include <std_msgs/msg/empty.h>
#include <sensor_msgs/msg/joint_state.h>
#include <avr_gcc_2023_interfaces/srv/set_motor_angle.h>

#include "context_timer.hpp"
#include "node.hpp"
#include "system.hpp"

#define GIMBAL_MOTOR_NODE_EXECUTOR_HANDLES (4 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES)

#ifndef AVR_GCC_2023_GIMBAL_MOTOR_NODE_HPP
#define AVR_GCC_2023_GIMBAL_MOTOR_NODE_HPP

class GimbalMotorNode : Node
{
public:
    GimbalMotorNode(const char *ns, int cs);

    void setup(rclc_support_t *support, rclc_executor_t *executor) override;

    void cleanup() override;

    void eStop();

private:

    MagneticSensorSPI sensor;
    BLDCDriver3PWM driver;
    BLDCMotor motor;

    TimerWithContext focUpdateTimer;
    TimerWithContext jointUpdateTimer;

    rcl_publisher_t currAnglePublisher;
    sensor_msgs__msg__JointState currAngleMessage;

    rclc_parameter_server_t updatePid;

    rcl_service_t setAngleService;
    rcl_service_t enableService;

    // ToDo: create custom message type for setAngle; request: angle, response: bool
    avr_gcc_2023_interfaces__srv__SetMotorAngle_Request setAngleRequest;
    std_srvs__srv__SetBool_Request enableRequest;
    avr_gcc_2023_interfaces__srv__SetMotorAngle_Response setAngleResponse;
    std_srvs__srv__SetBool_Response enableResponse;

    bool isKilled;

    void focUpdateTimerCallback(rcl_timer_t *timer, int64_t n);

    void jointUpdateTimerCallback(rcl_timer_t *timer, int64_t n);

    bool updatePidCallback(const Parameter *old_param, const Parameter *new_param);

    void setAngleCallback(const void *request, void *response);

    void enableCallback(const void *request, void *response);
};


#endif //AVR_GCC_2023_GIMBAL_MOTOR_NODE_HPP
