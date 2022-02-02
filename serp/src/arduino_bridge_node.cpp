#include "arduino_bridge_node.h"
#include <serp/RobotInfo.h>
#include <serp/Velocity.h>
#include "logica.h"
serp::RobotInfo robotget;



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
    //ROS_INFO("Received request to set the following velocity values: ML=%d%%\tMR=%d%%", robot.motor_left_velocity, robot.motor_right_velocity);
    return res.success;
}

bool sendBatteryLevel(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    res.success = true;
    res.message = std::to_string(robotget.battery_level);
    return true;
}

void getInfo(const serp::RobotInfo &msg)
{
    robotget.battery_level=msg.battery_level;
    robotget.vel_linear=msg.vel_linear;
    robotget.vel_angular=msg.vel_angular;		
    ROS_INFO("batt: %d%%", robotget.battery_level);
    ROS_INFO("left speed: %0.1f right speed: %0.1f", robotget.vel_linear, robotget.vel_angular);
}

int main(int argc, char** argv)
{
    float velocidades[2]={0};
    float matrixValores[61][61]={0};
    int ligacoes[61][61]={0};
    float velocidadesblocos[2]={0};
    //exemplo de teste
    ligacoes[4][8] = 1;
    ligacoes[8][4] = 1;
    ligacoes[9][2] = 1;
    ligacoes[2][9] = 1; //inicializa valor sensor esq

    matrixValores[4][8] = 2;
    matrixValores[8][4] = 2;
    matrixValores[9][2] = 2;
    matrixValores[2][9] = 2; //inicializa valor sensor esq

    ligacoes[59][3] = 1;
    ligacoes[3][59] = 1;
    ligacoes[60][10] = 1;
    ligacoes[10][60] = 1;
    matrixValores[59][3] = 4;
    matrixValores[3][59] = 4;

    ligacoes[61][24] = 1;
    ligacoes[24][61] = 1;
    //fim de exemplo de teste
    verificarBlocos(ligacoes, matrixValores, velocidades);
    ros::init(argc, argv, "arduino_bridge_node");
    ros::NodeHandle n_public;
    serp::Velocity vel_cmd;
    ros::ServiceServer service_velocity = n_public.advertiseService("velocity_setpoint", VelocitySetPoint);
    ros::ServiceServer service_battery = n_public.advertiseService("srv_battery_level", sendBatteryLevel);
    int8_t pvel_l;
    int8_t pvel_r;

    //robot.battery_level = 64;

    ros::Publisher send_velocities = n_public.advertise<serp::Velocity>("motors_vel", 2);
    ros::Subscriber receive_info = n_public.subscribe("hardware_info", 2, getInfo);
    ROS_INFO("vel: %f%%", velocidades[2]);
    //ros::Rate rate(100); //100hz update frequency
    while (ros::ok())
    {
	ros::Duration(0.1).sleep();
        pvel_l=vel_cmd.vel_motor_left;
        pvel_r=vel_cmd.vel_motor_right;
        vel_cmd.vel_motor_left=robot.motor_left_velocity;
        vel_cmd.vel_motor_right=robot.motor_right_velocity;
        if(pvel_l!=vel_cmd.vel_motor_left || pvel_r!=vel_cmd.vel_motor_right)
          {
            send_velocities.publish(vel_cmd);
          }
        ros::spinOnce();
    }
    return 0;
}
