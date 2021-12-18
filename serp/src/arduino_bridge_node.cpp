#include "arduino_bridge_node.h"

bool VelocitySetPoint(serp::VelocitySetPointRequest &req, serp::VelocitySetPointResponse &res)
{
    if(req.state && (operation_mode == Stopped || operation_mode == FixedVelocity)) {
        if(operation_mode == Stopped) operation_mode = FixedVelocity;
        // Save requested velocities
        motors.motor_left = req.vel_motor_left;
        motors.motor_right = req.vel_motor_right;
        res.success = true;
    }
    else if(req.state == false && operation_mode == FixedVelocity) {
        // Exit FixedVelocity mode and stop the robot
        operation_mode = Stopped;
        motors.motor_left = 0;
        motors.motor_right = 0;
        res.success = true;
    }
    else {
        res.success = false;
    }
    ROS_INFO("Received request to set the following velocity values: ML=%d%%\tMR=%d%%", motors.motor_left, motors.motor_right);
    return res.success;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduino_bridge_node");
    ros::NodeHandle n_public;

    ros::ServiceServer service = n_public.advertiseService("velocity_setpoint", VelocitySetPoint);

    ros::spin();
    return 0;
}