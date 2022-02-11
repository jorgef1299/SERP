#include "arduino_bridge_node.h"

bool VelocitySetPoint(serp::VelocitySetPointRequest &req, serp::VelocitySetPointResponse &res)
{
    if(req.state) {
        robot_state = ManualControl;
        // Save requested velocities
        robot.motor_left_velocity = req.vel_motor_left;
        robot.motor_right_velocity = req.vel_motor_right;
        res.success = true;
    }
    else if(req.state == false) {
        // Exit FixedVelocity mode and stop the robot
        robot_state = Stopped;
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

void cb_robot_state(const std_msgs::StringConstPtr &state) {
    if(state->data == "Stopped" || state->data == "ReadingProgrammingSheet") robot_state = Stopped;
    else if(state->data == "ManualControl") robot_state = ManualControl;
    else if(state->data == "Executing") robot_state = Executing;
}

int main(int argc, char** argv)
{
    // Initialize robot state
    robot_state = Stopped;
    ros::init(argc, argv, "arduino_bridge_node");
    ros::NodeHandle n_public;

    // ROS Servers
    ros::ServiceServer service_velocity = n_public.advertiseService("velocity_setpoint", VelocitySetPoint);
    ros::ServiceServer service_battery = n_public.advertiseService("srv_battery_level", sendBatteryLevel);

    // Subscriber for Robot State
    ros::Subscriber sub_robot_state = n_public.subscribe("robot_state", 2, cb_robot_state);

    robot.battery_level = 64;

    ros::spin();
    return 0;
}