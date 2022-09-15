#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <motion_planning/Direction.h>
#include <array>

// Task subscribe to /number, process received data, publish data to /number_count
// Reset number count via srv /reset_number_count

// callbackprototypes
void callback_laser_data(const sensor_msgs::LaserScan &msg);
bool give_direction(motion_planning::Direction::Request &req, motion_planning::Direction::Response &res);

// declare Pub, Sub & Srv
// ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceServer ser;

// obstacle positions & array
enum struct Position{
    NONE, FRONT, LEFT, RIGHT, BACK
};
namespace Obstacle{
    std::array<Position, 2> arr;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_planning_main"); // make Anonymous with ros::init_options::AnonymousName
    ros::NodeHandle nh;

    // init
    Obstacle::arr.at(0) = Position::NONE;
    Obstacle::arr.at(1) = Position::NONE;

    // get laserdata every loop
    sub = nh.subscribe("/scan", // topicname 
                        1000,   // buffer
                        callback_laser_data); // callbackfunction 

    // publish
    //pub = nh.advertise<std_msgs::Int64>("/number_count", // topicname 
    //                                    10);             // buffer 

    // advertise service when called
    ser = nh.advertiseService("/direction",    // service name
                              give_direction); // callback function

    ros::spin(); // keeps callbacks in loop
}

void callback_laser_data(const sensor_msgs::LaserScan &msg){

    int numElem = 1147; // number of elements in ranges[] found with py script (motion_planning_main.py)
    int n = numElem/8;
    float radius = 0.5;

    //angle increment of measurement
    int alpha = 15; // can be higher resolution than python file because faster proccessed
    int step = (n*alpha)/360;
    
    // back
    for (int i=(3*n); i<(5*n); i+=step){
        if ((msg.ranges[i] < radius) && (Obstacle::arr.at(0) != Position::BACK)){
            // update array
            Obstacle::arr.at(1)= Obstacle::arr.at(0);
            Obstacle::arr.at(0) = Position::BACK;
        }
    }
    // right
    for (int i=(5*n); i<(7*n); i+=step){
        if ((msg.ranges[i] < radius) && (Obstacle::arr.at(0) != Position::RIGHT)){
            Obstacle::arr.at(1)= Obstacle::arr.at(0);
            Obstacle::arr.at(0) = Position::RIGHT;
        }
    }
    // left
    for (int i=n; i<(3*n); i+=step){
        if ((msg.ranges[i] < radius) && (Obstacle::arr.at(0) != Position::LEFT)){
            Obstacle::arr.at(1)= Obstacle::arr.at(0);
            Obstacle::arr.at(0) = Position::LEFT;
        }
    }
    // front
    for (int i=0; i<n; i+=step){
        if ((msg.ranges[i] < radius) && (Obstacle::arr.at(0) != Position::FRONT)){
            Obstacle::arr.at(1)= Obstacle::arr.at(0);
            Obstacle::arr.at(0) = Position::FRONT;
        }
    }
    for (int i=(7*n); i<(8*n); i+=step){
        if ((msg.ranges[i] < radius) && (Obstacle::arr.at(0) != Position::FRONT)){
            Obstacle::arr.at(1)= Obstacle::arr.at(0);
            Obstacle::arr.at(0) = Position::FRONT;
        }
    }
    ROS_INFO("obstacle: %i, %i", static_cast<int> (Obstacle::arr.at(0)), static_cast<int> (Obstacle::arr.at(1)));
}

// callback function returns True if service was called, and assignes values to response
bool give_direction(motion_planning::Direction::Request &req, motion_planning::Direction::Response &res){

    // full front
    /*
    if (obstacle == Obstacle::FRONT{

    }*/
    
    return true;
}