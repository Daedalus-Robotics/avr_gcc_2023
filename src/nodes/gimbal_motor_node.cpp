#include "nodes/gimbal_motor_node.hpp"

GimbalMotorNode::GimbalMotorNode(const char *ns, int cs) : Node("gcc_gimbal_motor_node", ns),
                                                           sensor(cs, 14, 0x3FFF),
                                                           driver(9, 5, 6, 8),
                                                           motor(11, 9.75),
                                                           focUpdateTimer(), jointUpdateTimer(),
                                                           currAnglePublisher(),
                                                           currAngleMessage(),
                                                           updatePid(),
                                                           setAngleService(), enableService(),
                                                           setAngleRequest(), enableRequest(),
                                                           setAngleResponse(), enableResponse(),
                                                           isKilled(false)
{
    focUpdateTimer.context = this;
    jointUpdateTimer.context = this;

    sensor.init();
    motor.linkSensor(&sensor);

    driver.pwm_frequency = 20000;
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    // Max DC voltage allowed - default voltage_power_supply
    driver.voltage_limit = 3;
    // driver init
    driver.init();
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;

    motor.PID_velocity.P = 0;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0;

    motor.P_angle.P = 0;
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;
    motor.LPF_angle.Tf = 0;


    // initialize motor
    motor.init();

    motor.initFOC();

    motor.disable();

    // I think we need to set some of the currAngleMessage data here

}

void GimbalMotorNode::setup(rclc_support_t *support, rclc_executor_t *executor)
{
    LOG(LOGLEVEL_INFO, "Setting up GimbalMotorNode");

    Node::setup(support, executor);

    HANDLE_ROS_ERROR(rclc_timer_init_default(&focUpdateTimer.timer,
                                             support,
                                             RCL_MS_TO_NS(10),
                                             CONTEXT_TIMER_CALLBACK(GimbalMotorNode, focUpdateTimerCallback)), true);
    HANDLE_ROS_ERROR(rclc_executor_add_timer(executor, &focUpdateTimer.timer), true);

    HANDLE_ROS_ERROR(rclc_timer_init_default(&jointUpdateTimer.timer,
                                             support,
                                             RCL_MS_TO_NS(200),
                                             CONTEXT_TIMER_CALLBACK(GimbalMotorNode, jointUpdateTimerCallback)), true);
    HANDLE_ROS_ERROR(rclc_executor_add_timer(executor, &focUpdateTimer.timer), true);

    LOG(LOGLEVEL_DEBUG, "Setting up GimbalMotorNode: parameter server");

    HANDLE_ROS_ERROR(rclc_parameter_server_init_default(&updatePid, &node), true);

    HANDLE_ROS_ERROR(rclc_executor_add_parameter_server_with_context(executor,
                                                                     &updatePid,
                                                                     CONTEXT_PARAMETER_CALLBACK(GimbalMotorNode,
                                                                                                updatePidCallback),
                                                                     this), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "velocity/p", RCLC_PARAMETER_DOUBLE), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "velocity/i", RCLC_PARAMETER_DOUBLE), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "velocity/d", RCLC_PARAMETER_DOUBLE), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "velocity/l", RCLC_PARAMETER_DOUBLE), true);

    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "angle/p", RCLC_PARAMETER_DOUBLE), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "angle/i", RCLC_PARAMETER_DOUBLE), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "angle/d", RCLC_PARAMETER_DOUBLE), true);
    HANDLE_ROS_ERROR(rclc_add_parameter(&updatePid, "angle/l", RCLC_PARAMETER_DOUBLE), true);

    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "velocity/p", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "velocity/i", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "velocity/d", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "velocity/l", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "angle/p", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "angle/i", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "angle/d", 0), true);
    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, "angle/l", 0), true);

    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "velocity/p", (double *) &motor.PID_velocity.P), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "velocity/i", (double *) &motor.PID_velocity.I), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "velocity/d", (double *) &motor.PID_velocity.D), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "velocity/l", (double *) &motor.LPF_velocity.Tf), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "angle/p", (double *) &motor.P_angle.P), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "angle/i", (double *) &motor.P_angle.I), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "angle/d", (double *) &motor.P_angle.D), true);
    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid, "angle/l", (double *) &motor.LPF_angle.Tf), true);

    LOG(LOGLEVEL_DEBUG, "Setting up GimbalMotorNode: current angle publisher");
    HANDLE_ROS_ERROR(rclc_publisher_init_default(&currAnglePublisher, &node,
                                                 ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                                 "current_angle"),
                     true);

    LOG(LOGLEVEL_DEBUG, "Setting up GimbalMotorNode: set angle service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&setAngleService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
                                               "set_angle"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &setAngleService,
                                                            &setAngleRequest,
                                                            &setAngleResponse,
                                                            CONTEXT_SERVICE_CALLBACK(GimbalMotorNode, setAngleCallback),
                                                            this), true);

    LOG(LOGLEVEL_DEBUG, "Setting up GimbalMotorNode: enable service");
    HANDLE_ROS_ERROR(rclc_service_init_default(&enableService,
                                               &node,
                                               ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                               "enable"), true);
    HANDLE_ROS_ERROR(rclc_executor_add_service_with_context(executor,
                                                            &enableService,
                                                            &enableRequest,
                                                            &enableResponse,
                                                            CONTEXT_SERVICE_CALLBACK(GimbalMotorNode, enableCallback),
                                                            this), true);

}

void GimbalMotorNode::cleanup()
{
    LOG(LOGLEVEL_DEBUG, "Cleaning up GimbalMotorNode");

    HANDLE_ROS_ERROR(rcl_service_fini(&enableService, &node), false);
    HANDLE_ROS_ERROR(rcl_service_fini(&setAngleService, &node), false);
    HANDLE_ROS_ERROR(rcl_publisher_fini(&currAnglePublisher, &node), false);
    HANDLE_ROS_ERROR(rclc_parameter_server_fini(&updatePid, &node), false);

    Node::cleanup();
}

void GimbalMotorNode::eStop()
{
    isKilled = true;
    motor.disable();

    HANDLE_ROS_ERROR(rcl_timer_cancel(&focUpdateTimer.timer), false);
    HANDLE_ROS_ERROR(rcl_timer_cancel(&jointUpdateTimer.timer), false);
}


void GimbalMotorNode::focUpdateTimerCallback(__attribute__((unused)) rcl_timer_t *timer,
                                             __attribute__((unused)) int64_t n)
{

    motor.loopFOC();

}

void GimbalMotorNode::jointUpdateTimerCallback(__attribute__((unused)) rcl_timer_t *timer,
                                               __attribute__((unused)) int64_t n)
{
    double position[] = {sensor.getAngle(), 0, 0};
    currAngleMessage.position.data = position;
    currAngleMessage.position.size = 3;

    double velocity[] = {sensor.getVelocity(), 0, 0};
    currAngleMessage.velocity.data = velocity;
    currAngleMessage.velocity.size = 3;

    HANDLE_ROS_ERROR(rcl_publish(&currAnglePublisher, &currAngleMessage, nullptr), false);
}

bool GimbalMotorNode::updatePidCallback(const Parameter * old_param, const Parameter * new_param)
{
    static char message[32];

    HANDLE_ROS_ERROR(rclc_parameter_set_double(&updatePid, old_param->name.data, new_param->value.double_value), false);

    switch (old_param->name.data[0])
    {
        case 'v':
            switch (old_param->name.data[strlen(old_param->name.data) - 1])
            {
                case 'p':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.PID_velocity.P), false);
                    break;
                case 'i':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.PID_velocity.I), false);
                    break;
                case 'd':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.PID_velocity.D), false);
                    break;
                case 'l':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.LPF_velocity.Tf), false);
                    break;
            }
            break;
        case 'a':
            switch (old_param->name.data[strlen(old_param->name.data) - 1])
            {
                case 'p':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.P_angle.P), false);
                    break;
                case 'i':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.P_angle.I), false);
                    break;
                case 'd':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.P_angle.D), false);
                    break;
                case 'l':
                    HANDLE_ROS_ERROR(rclc_parameter_get_double(&updatePid,
                                                               old_param->name.data,
                                                               (double *) &motor.LPF_angle.Tf), false);
                    break;
            }
            break;
    }

    snprintf(message, sizeof(message), "Parameter %s modified.", old_param->name.data);
    LOG(LOGLEVEL_INFO, message);

    return true;
}

void GimbalMotorNode::setAngleCallback(const void *request, void *response)
{
    auto request_msg = (avr_gcc_2023_interfaces__srv__SetMotorAngle_Request *) request;
    auto response_msg = (avr_gcc_2023_interfaces__srv__SetMotorAngle_Response *) response;

    if (!isKilled)
    {
        motor.move((float) request_msg->angle);

        response_msg->success = true;
        response_msg->message.data = const_cast<char *>("Success");
        response_msg->message.size = 7;
    }
    else
    {
        response_msg->success = false;
        response_msg->message.data = const_cast<char *>("E-Stop was activated");
        response_msg->message.size = 20;
    }

}

void GimbalMotorNode::enableCallback(const void *request, void *response)
{
    auto request_msg = (std_srvs__srv__SetBool_Request *) request;
    auto response_msg = (std_srvs__srv__SetBool_Response *) response;

    if(!isKilled)
    {
        if (request_msg->data)
        {
            if (motor.enabled != 0)
            {
                response_msg->success = false;
                response_msg->message.data = const_cast<char *>("Motor is already enabled");
                response_msg->message.size = 24;
            }
            else
            {
                motor.enable();
                response_msg->success = true;
                response_msg->message.data = const_cast<char *>("Success");
                response_msg->message.size = 7;
            }
        }
        else
        {

            if (motor.enabled != 0)
            {
                motor.disable();
                response_msg->success = true;
                response_msg->message.data = const_cast<char *>("Success");
                response_msg->message.size = 7;
            }
            else
            {
                response_msg->success = false;
                response_msg->message.data = const_cast<char *>("Motor is already disabled");
                response_msg->message.size = 25;
            }
        }
    }
    else
    {
        response_msg->success = false;
        response_msg->message.data = const_cast<char *>("eStop is active, will not enable");
        response_msg->message.size = 32;
    }

}



