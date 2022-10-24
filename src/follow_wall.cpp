#include <iostream>
#include <math.h>
#include <bits/stdc++.h>

#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h" 
#include "sensor_msgs/LaserScan.h"

#define LEFT 0
#define FRONT 1
#define RIGHT 2

#define FIND_WALL 0
#define GO_TO_WALL 1
#define FOLLOW_INNER_WALL 2
#define SWITCH_TO_OUTER_WALL 3
#define FOLLOW_OUTER_WALL 4
#define FINISH 5

int state = FIND_WALL;


void printState(){
    switch (state)
    {
    case FIND_WALL:
        std::cout << "STATE = FIND_WALL\n";
        break;
    case GO_TO_WALL:
        std::cout << "STATE = GO_TO_WALL\n";
        break;
    case FOLLOW_INNER_WALL:
        std::cout << "STATE = FOLLOW_INNER_WALL\n";
        break;
    case SWITCH_TO_OUTER_WALL:
        std::cout << "STATE = SWITCH_TO_OUTER_WALL\n";
        break;
    case FOLLOW_OUTER_WALL:
        std::cout << "STATE = FOLLOW_OUTER_WALL\n";
        break;
    case FINISH:
        std::cout << "STATE = FINISH\n";
        break;
    
    default:
        std::cout << state <<"\n";
        break;
    }
}

class ReactiveAgent
{
    private:
        float linear_s;
        float angular_s;
        float dir_distance[3] = {0.0};
        //float distances[180] = {0.0};

        ros::NodeHandle node;

        ros::Subscriber sub;
        ros::Publisher pub;
    public:
        ReactiveAgent(float linear_s, float angular_s);
        void run();
        geometry_msgs::Twist move_Forwards();
        geometry_msgs::Twist move_Backwards();
        geometry_msgs::Twist rotate_Left();
        geometry_msgs::Twist rotate_right(); 
        geometry_msgs::Twist stop();
        geometry_msgs::Twist selectMove();
        void updateSensorData(const sensor_msgs::LaserScan::ConstPtr& scan);
};

ReactiveAgent::ReactiveAgent(float linear_s, float angular_s){
    this->linear_s = linear_s;
    this->angular_s = angular_s;
}

geometry_msgs::Twist ReactiveAgent::move_Forwards(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = this->linear_s;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = 0.0;

    return velCommand;
}

geometry_msgs::Twist ReactiveAgent::move_Backwards(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = - this->linear_s;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = 0.0;

    return velCommand;
}

geometry_msgs::Twist ReactiveAgent::rotate_Left(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = 0.0;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = -this->angular_s;

    return velCommand;
}

geometry_msgs::Twist ReactiveAgent::rotate_right(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = 0.0;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = this->angular_s;

    return velCommand;
} 

geometry_msgs::Twist ReactiveAgent::stop(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = 0.0;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = 0.0;

    return velCommand;
}



geometry_msgs::Twist ReactiveAgent::selectMove(){
    geometry_msgs::Twist velCommand;
    
    switch (state)
    {
    case FIND_WALL:
        if(this->dir_distance[FRONT] == 0){
            std::cout << "[FIND_WALL] : Just Started\n";
            velCommand = this->stop();
            break;
        }
        
        if(isinf(this->dir_distance[FRONT])){
            std::cout << "[FIND_WALL] : Rotating to find wall\n";
            velCommand = this->rotate_Left();
            break;
        }
        
        std::cout << "[FIND_WALL] : Found Wall\n";
        state = GO_TO_WALL;
        velCommand = this->stop();
        
        break;

    case GO_TO_WALL:

        if(isinf(this->dir_distance[FRONT])){
            std::cout << "[GO_TO_WALL] : Rotating to compensate deviation\n";
            velCommand = this->rotate_Left();
            break;
        }

        if(this->dir_distance[FRONT] > 0.5){
            std::cout << "[GO_TO_WALL] : Moving Towards Wal\n";
            velCommand = this->move_Forwards();
            break;
        }

        std::cout << "[FIND_WALL] : Reached Wall\n";
        state = FOLLOW_INNER_WALL;
        velCommand = this->stop();
        break;

    
    case FOLLOW_INNER_WALL:
        return this->stop();
        break;

    default:
        std::cout << "[ERROR] : Shouldn't be here\n";
        velCommand = this->stop();
        break;
    }

    return velCommand;
}

void ReactiveAgent::updateSensorData(const sensor_msgs::LaserScan::ConstPtr& scan){
    
    this->dir_distance[LEFT] = scan->ranges[0];
    this->dir_distance[FRONT] = scan->ranges[360];
    this->dir_distance[RIGHT] = scan->ranges[719];

    std::cout << "[RANGE] left = " << this->dir_distance[LEFT] << "\n";
    std::cout << "[RANGE] front = " << this->dir_distance[FRONT] << "\n";
    std::cout << "[RANGE] right = " << this->dir_distance[RIGHT] << "\n";
}

void ReactiveAgent::run(){
    
    this->pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 0);
    this->sub = node.subscribe<sensor_msgs::LaserScan>("m2wr/laser/scan",10,&ReactiveAgent::updateSensorData,this);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        ros::spinOnce();
        
        geometry_msgs::Twist velCommand = this->selectMove();
        
        std::cout << "[CURRENT_VELOCITY] linear =" << velCommand.linear.x << "\n";
        std::cout << "[CURRENT_VELOCITY] angular =" << velCommand.angular.z << "\n";

        
        this->pub.publish(velCommand);


        loop_rate.sleep();

        if(state == FOLLOW_INNER_WALL)
            break;
    }
    
}



int main(int argc, char **argv)
{
    //Init ROS
    ros::init(argc, argv, "follow_wall");

    ReactiveAgent agent = ReactiveAgent(0.3,0.3);

    agent.run();

    return 0;
}
