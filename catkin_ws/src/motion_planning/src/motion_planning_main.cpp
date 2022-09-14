#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <motion_planning/Direction.h>

// Task subscribe to /number, process received data, publish data to /number_count
// Reset number count via srv /reset_number_count

// callbackprototypes
void callback_laser_data(const sensor_msgs::LaserScan &msg);
bool give_direction(motion_planning::Direction::Request &req, 
                    motion_planning::Direction::Response &res);

// declare Pub, Sub & Srv
// ros::Publisher pub;
ros::Subscriber sub;
// ros::ServiceServer ser;

// Obstacle positions
enum struct Obstacle{
    NONE,
    FRONT,
    LEFT,
    RIGHT,
    BACK
};

Obstacle obstacle {Obstacle::NONE};

int main(int argc, char **argv){
    ros::init(argc, argv, "motion_planning_main"); // make Anonymous with ros::init_options::AnonymousName
    ros::NodeHandle nh;

    // subscribe
    sub = nh.subscribe("/scan", // topicname 
                        1000,     // buffer
                        callback_laser_data); // callbackfunction 

    // publish
    //pub = nh.advertise<std_msgs::Int64>("/number_count", // topicname 
    //                                    10);             // buffer 

    // server
    // ser = nh.advertiseService("/reset_number_count",        // service name
    //                           callback_reset_number_count); // callback function

    ros::spin();
}

void callback_laser_data(const sensor_msgs::LaserScan &msg){

    obstacle = Obstacle::NONE; // reset

    int numElem = 1147; // found with py script (motion_planning_main.py)
    int n = numElem/8;
    float radius = 0.5;

    //angle increment of measurement
    int alpha = 15; // can be higher resolution than python file because faster
    int step = (n*alpha)/360;
    
    // ROS_INFO("...");
    // ROS_INFO("\nnumElem: %i\nn: %i\nsize ranges: %ld\nsize range: %ld\nstep: %i\n\n", numElem, n, msg.ranges.size(), msg.ranges[0].size(), step);

    // back
    for (int i=(3*n); i<(5*n); i+=step){
        if (msg.ranges[i] < radius){
            obstacle = Obstacle::BACK;
        }
    }
    // right
    for (int i=(5*n); i<(7*n); i+=step){
        if (msg.ranges[i] < radius){
            obstacle = Obstacle::RIGHT;
        }
    }
    // left
    for (int i=n; i<(3*n); i+=step){
        if (msg.ranges[i] < radius){
            obstacle = Obstacle::LEFT;
        }
    }
    // front
    for (int i=0; i<n; i+=step){
        if (msg.ranges[i] < radius){
            obstacle = Obstacle::FRONT;
        }
    }
    for (int i=(7*n); i<(8*n); i+=step){
        if (msg.ranges[i] < radius){
            obstacle = Obstacle::FRONT;
        }
    }

    if (static_cast<int> (obstacle)){
        ROS_INFO("obstacle: %i", static_cast<int> (obstacle));
    }
}

// callback function returns True if service was called, and assignes value to response
bool give_direction(motion_planning::Direction::Request &req, motion_planning::Direction::Response &res){

    //if obstacle this & that, then send some Direction
    
    return true;
}