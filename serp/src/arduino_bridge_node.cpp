#include "arduino_bridge_node.h"

bool VelocitySetPoint(serp::VelocitySetPointRequest &req, serp::VelocitySetPointResponse &res)
{
    if(req.state && (operation_mode == Stopped || operation_mode == FixedVelocity)) {
        if(operation_mode == Stopped) operation_mode = FixedVelocity;
        // Save requested velocities
        robot.motor_left_velocity = req.vel_motor_left;
        robot.motor_right_velocity = req.vel_motor_right;
        res.success = true;
    }
    else if(req.state == false && operation_mode == FixedVelocity) {
        // Exit FixedVelocity mode and stop the robot
        operation_mode = Stopped;
        robot.motor_left_velocity = 0;
        robot.motor_right_velocity = 0;
        res.success = true;
    }
    else {
        res.success = false;
    }
    ROS_INFO("Received request to set the following velocity values: ML=%d%%\tMR=%d%%", robot.motor_left_velocity, robot.motor_right_velocity);
    return res.success;
}

bool sendBatteryLevel(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    res.success = true;
    res.message = std::to_string(robot.battery_level);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduino_bridge_node");
    ros::NodeHandle n_public;

    ros::ServiceServer service_velocity = n_public.advertiseService("velocity_setpoint", VelocitySetPoint);
    ros::ServiceServer service_battery = n_public.advertiseService("srv_battery_level", sendBatteryLevel);

    robot.battery_level = 64;

    ros::spin();
    return 0;
}