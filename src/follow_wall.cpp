#include <iostream>
#include <math.h>
#include <bits/stdc++.h>
#include <chrono>


#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h" 
#include "sensor_msgs/LaserScan.h"

#define DELTA_TIME 0.1

#define LEFT 0
#define DIAG_LEFT 1
#define FRONT 2
#define DIAG_RIGHT 3
#define RIGHT 4

#define FIND_WALL 0
#define GO_TO_WALL 1
#define FOLLOW_WALL 2
#define CORNERING 3
#define FINISH 4

int num_edge = 0;
int isFinished = 0;

//Stats
int num_iter = 0;
float distance = 0.0;
std::chrono::duration<double> elapsed_seconds;


void printStatistics(){
    std::cout << "*************************************************" << std::endl;
    std::cout << "Aproximate Distance Traveled (m) = " << distance << std::endl;
    std::cout << "Number of Loop Iterations = " << num_iter << std::endl;
    std::cout << "Duration Time (s) = " << elapsed_seconds.count() << std::endl;
    std::cout << "*************************************************" << std::endl;
}


void calculateDistance(float linear_s, float angular_s){
    if(angular_s != 0 && linear_s != 0){
        float theta = angular_s * DELTA_TIME;
        float radius = (linear_s * DELTA_TIME)/theta;
        distance +=  theta * radius;
    } else {
        distance += fabs(DELTA_TIME * linear_s);
    }
}

void printState(int state){
    switch (state)
    {
    case FIND_WALL:
        std::cout << "STATE = FIND_WALL\n";
        break;
    case GO_TO_WALL:
        std::cout << "STATE = GO_TO_WALL\n";
        break;
    case FOLLOW_WALL:
        std::cout << "STATE = FOLLOW_WALL\n";
        break;
    case CORNERING:
        std::cout << "STATE = CORNERING\n";
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
        float dir_distance[5] = {10.0};
        //float distances[180] = {0.0};

        int rotate_along = 0;
        ros::NodeHandle node;

        ros::Subscriber sub;
        ros::Publisher pub;
    public:
        ReactiveAgent(float linear_s, float angular_s);
        void run();
        geometry_msgs::Twist move_Forwards();
        geometry_msgs::Twist move_Backwards();
        geometry_msgs::Twist rotate_Left();
        geometry_msgs::Twist rotate_Right(); 
        geometry_msgs::Twist stop();
        geometry_msgs::Twist selectMove();
        geometry_msgs::Twist move_diagonalLeft();
        geometry_msgs::Twist move_diagonalRight();
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

geometry_msgs::Twist ReactiveAgent::rotate_Right(){
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

geometry_msgs::Twist ReactiveAgent::move_diagonalLeft(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = this->linear_s;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = -this->angular_s;

    return velCommand;
}
        
geometry_msgs::Twist ReactiveAgent::move_diagonalRight(){
    geometry_msgs::Twist velCommand;
    velCommand.linear.x = this->linear_s;
    velCommand.linear.y = 0.0;
    velCommand.linear.z = 0.0;
    velCommand.angular.x = 0.0;
    velCommand.angular.y = 0.0;
    velCommand.angular.z = this->angular_s;

    return velCommand;
}



geometry_msgs::Twist ReactiveAgent::selectMove(){
    
    geometry_msgs::Twist velCommand;
    int state = -1;

    if((this->dir_distance[RIGHT] < 1.0) || (this->dir_distance[LEFT] < 1.0 )){
        state = FOLLOW_WALL;
    } else if(isinf(this->dir_distance[FRONT]) && 
              isinf(this->dir_distance[RIGHT]) && num_edge == 0) {
        state = CORNERING;
    } else if(isinf(this->dir_distance[FRONT]) && 
              isinf(this->dir_distance[RIGHT]) && num_edge >= 2) {
        state = FINISH;
    } else if(isinf(this->dir_distance[FRONT])){
        state = FIND_WALL;
    } else {
        if(this->dir_distance[FRONT] > 0.4)
            state = GO_TO_WALL;
        else{
            state = FOLLOW_WALL;
        }
    }
    std::cout << "------------------Number of edges = " << num_edge << std::endl;
    printState(state);
    
    switch (state)
    {
    case FIND_WALL:
        std::cout << "[FIND_WALL] : Readjusting to find wall\n";
        velCommand = this->rotate_Left();
        break;

    case GO_TO_WALL:
        std::cout << "[GO_TO_WALL] : Moving towards wall\n";
        velCommand = this->move_Forwards();
        break;

    case FOLLOW_WALL:
        if(this->dir_distance[RIGHT] > 0.6 && this->dir_distance[DIAG_RIGHT] > 0.6){
            std::cout << "[FOLLOW_WALL] : Readjusting to get closer to the wall\n";
            velCommand = this->rotate_Left();
            break;
        } else if ((this->dir_distance[RIGHT] < 0.35 && this->dir_distance[RIGHT] > 0.2) || 
                   (this->dir_distance[FRONT] < 0.4 && this->dir_distance[FRONT] > 0.3)){

            std::cout << "[FOLLOW_WALL] : Readjusting to get away from the wall\n";
            velCommand = this->rotate_Right();
            break;
        }
        else{
            std::cout << "[FOLLOW_WALL] : Following wall\n";
            velCommand = this->move_Forwards();
            break;
        }
        break;

        case CORNERING:
        std::cout << "[CORNERING] : Driving over the corner\n";
        num_edge++;
        velCommand = this->move_diagonalLeft();
        break;

        case FINISH:
        std::cout << "[FINISHED] : Reached Destination\n";
        isFinished = 1;
        velCommand = this->stop();
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
    this->dir_distance[DIAG_LEFT] = scan->ranges[180];
    this->dir_distance[FRONT] = scan->ranges[360];
    this->dir_distance[DIAG_RIGHT] = scan->ranges[540];
    this->dir_distance[RIGHT] = scan->ranges[719];

    std::cout << "[RANGE] left = " << this->dir_distance[LEFT] << "\n";
    std::cout << "[RANGE] front = " << this->dir_distance[FRONT] << "\n";
    std::cout << "[RANGE] right = " << this->dir_distance[RIGHT] << "\n";
    std::cout << "[RANGE] diag_left = " << this->dir_distance[DIAG_LEFT] << "\n";
    std::cout << "[RANGE] diag_right = " << this->dir_distance[DIAG_RIGHT] << "\n";

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
        calculateDistance(velCommand.linear.x,velCommand.angular.z);
        num_iter++;
        loop_rate.sleep();

        if(isFinished == 1)
            break;
    }
    
}



int main(int argc, char **argv)
{
    //Init ROS
    ros::init(argc, argv, "follow_wall");

    ReactiveAgent agent = ReactiveAgent(0.3,0.3);

    auto start = std::chrono::system_clock::now();
    agent.run();
    auto end = std::chrono::system_clock::now();

    elapsed_seconds = end-start;

    printStatistics();
    return 0;
}

