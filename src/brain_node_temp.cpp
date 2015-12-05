#include <sstream>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <cmath> 
#include "ros/ros.h"
#include "ras_msgs/Object_id.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "motors/odometry.h"
#include "nav_msgs/GridCells.h"
#include "ras_arduino_msgs/PWM.h"
#include "ras_arduino_msgs/Encoders.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "grid_generator/Grid_map.h"
#include "grid_generator/Grid_map_struct.h"
#include "localization/Position.h"
#include "localization/Map_message.h"
#include "localization/Distance_message.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_datatypes.h"
#include "tf/exceptions.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

//#define ROT_DURATION  0.2
#define FORWARD_DURATION  0.3


const int times_to_save = 50;

//Memory for PID controller for rotation 
double errorOld_rot=0.0;
double integral_rot_Old=0.0;

//Memory for PID controller for going forward
double errorOld_lin=0.0;
double integral_lin_Old=0.0;
double alpha;
double lastTime;

bool final_node = false;

bool flag_forward=false;
bool flag_rotation=false;

struct timestamp{
    double time;
    double delta_trans;
    double delta_rot;
};


double sign(double val) {
    return double((0.0 < val) - (val < 0.0));
}


class BrainNode
{
public:

    std::vector< std::vector<grid_generator::Grid_map_struct> > matrix_a;
    std::vector<grid_generator::Grid_map_struct> grid_map_send;
    std::vector<geometry_msgs::Point> paths;
    std::vector<timestamp> timestamp_vec;

    std_msgs::Float32MultiArray vec_data;

    nav_msgs::OccupancyGrid grid_cost;
    nav_msgs::OccupancyGrid grid_obs;

    ras_msgs::Object_id current_object_id;

    geometry_msgs::Point Path[];
    geometry_msgs::PointStamped object_pos_to_cost_map;

    visualization_msgs::Marker marker_object;
    visualization_msgs::MarkerArray marker_object_vec;


    bool got_path;
    bool pos_received ;

    ros::NodeHandle n;

    ros::Subscriber object_pos_sub_;
    ros::Subscriber robot_pos_sub_;
    ros::Subscriber path_sub;
    ros::Subscriber odom_sub;

    ros::Publisher marker_object_pub;
    ros::Publisher object_pos_pub;
    ros::Publisher requst_pub_;
    ros::Publisher twist_pub;

    BrainNode()
    {
        n = ros::NodeHandle("~");
        object_seen = false;
        got_path=false;
        pos_received = false;
        path_node_index =0;

        fill_timestamps();

        //The rows and cols needs to be one bigger than the length of the outer walls
        //grid_cost_map_pub = n.advertise<std::vector< std::vector<struct position_node> >("test",1);
        path_sub = n.subscribe<nav_msgs::GridCells>( "/nodes_generator/path", 1,&BrainNode::path_vector_function,this);
        object_pos_sub_ = n.subscribe<ras_msgs::Object_id>("/object/object",1,&BrainNode::object_detected_function,this);
        robot_pos_sub_ = n.subscribe<localization::Position>("/position",1,&BrainNode::localization_callback,this);
        odom_sub = n.subscribe<motors::odometry>("/odometry",10,&BrainNode::odom_callback,this);


        
        requst_pub_ = n.advertise<std_msgs::Int32>("/grid_generator/update_query", 100);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",100);
        marker_object_pub = n.advertise<visualization_msgs::MarkerArray>( "/object_to_rvis_grid_map", 1);
        object_pos_pub= n.advertise<geometry_msgs::PointStamped>( "/object_pos", 1);
    }
    ~BrainNode()
    {
        //delete motor_controller_;
    }



void path_vector_function(nav_msgs::GridCells msg)
{
        paths = msg.cells;
        newx = paths[path_node_index].x;
        newy = paths[path_node_index].y;
        for (int i = 0; i< paths.size(); i++) std::cout << "node(" << i << "): x =" << paths[i].x << "  y =" << paths[i].y << std::endl;
        got_path=true;
}

void odom_callback(motors::odometry msg){

    struct timestamp ts1;
    ts1.time = ros::Time::now().toSec();
    ts1.delta_trans = msg.v*msg.dt;
    ts1.delta_rot = msg.w*msg.dt;

    timestamp_vec.push_back(ts1);
    timestamp_vec.erase(timestamp_vec.begin()); //TODO: does this really work??

    robot_theta += msg.w*msg.dt;
    robot_x += msg.v*msg.dt*cos(robot_theta);
    robot_y += msg.v*msg.dt*sin(robot_theta);

    //publish_position();
}

void fill_timestamps(){
    for (int i=0;i<times_to_save;i++){
        struct timestamp ts;
        ts.time = ros::Time::now().toSec();
        ts.delta_rot = 0.0;
        ts.delta_trans = 0.0;
        timestamp_vec.push_back(ts);
    }
}

void get_next_path()
{
    path_node_index++;
    if(path_node_index == paths.size()) 
    {
        path_node_index = 0;
        final_node = true;
        return;
    }
    newx = paths[path_node_index].x;
    newy = paths[path_node_index].y;
}

bool check_at_correct_place()
{
    double threshold = 0.05;
    // for(int i = -5; i <= 5;i++)
    // {
    //     for(int j = -5; j <= 5;j++)
    //     {
    //         if((floor(robot_x) == floor(newx+(i/100))) && (floor(robot_y)==floor(newy+(j/100))))
    //         {
    //             get_next_path();
    //         }
    //     }
    // }

    // std::cout << "robot: x= " << robot_x << "  y = " << robot_y << std::endl;
    // std::cout << "nó: x= " << newx << "  y = " << newy << std::endl;
    // std::cout << "ABS diff: x= " << fabs(robot_x - newx) << "  y = " << fabs(robot_y - newy) << std::endl;


    if((fabs(robot_x - newx) < threshold) && (fabs(robot_y - newy) < threshold))  return true;
    else return false;


}

void move_function()
{
    alpha = atan2(newy-robot_y,newx-robot_x);
    double dist_to_goal = std::sqrt((newx-robot_x)*(newx-robot_x) + (newy-robot_y)*(newy-robot_y));
    double theta_temp = robot_theta;
    //beta = (robot_theta-alpha);
    // if(std::fabs(beta)>=M_PI){
    //     beta=2*M_PI-beta;
    // }



    //if(theta_temp < 0) theta_temp = 2*M_PI + theta_temp;
    //if(alpha < 0) alpha = 2*M_PI + alpha;
    if(std::fabs(alpha-robot_theta) < std::fabs(-2.0*M_PI + alpha - robot_theta)){
        if (std::fabs(alpha-robot_theta) < std::fabs(2.0*M_PI + alpha - robot_theta)){
        beta = alpha-robot_theta;
        }
        else{
            beta = 2.0*M_PI + alpha - robot_theta;
        }
    }
    else{
        beta = -2.0*M_PI + alpha - robot_theta;
    }

    //beta = -(alpha-theta_temp);

    //beta_2 = (robot_theta-alpha);
    double threshold = 0.3;



    // std::cout << "Beta:"<< beta <<  std::endl;
    // std::cout << "ABS(Beta):"<< fabs(beta) <<  std::endl;


    if (fabs(beta)>threshold)
    {
        if(flag_forward)
        {
            errorOld_rot=0.0;
            integral_rot_Old=0.0;
            errorOld_lin=0.0;
            integral_lin_Old=0.0;
            flag_forward=false;
            std::cout << "flag_forward:" <<  std::endl;
        } 
        rotate();    // Stops and aligns 
        flag_rotation = true;
    } 
    else
    {
        if(flag_rotation)
        {
            errorOld_rot=0.0;
            integral_rot_Old=0.0;
            errorOld_lin=0.0;
            integral_lin_Old=0.0;
            flag_rotation = false;
            std::cout << "flag_rotate:"<<  std::endl;
            stop_robot_and_wait(3);
        }
        go_forward(dist_to_goal);  
        flag_forward = true;
        //lastTime = ros::Time::now().toSec();
    }     // Go forward with small corrections on the angle       




}



void move_to_object()

{

    //rotate(angle_to_object);

    //perhaps move a little close

    //ask for determination - 
}

void rotate(){


    double now = ros::Time::now().toSec();
    double dt = now - lastTime;
    lastTime = now;

    geometry_msgs::Twist twist_msg;

    double error_rot;

    double Kp_rot = 4.0;
    double Ki_rot = 0.3; 
    double Kd_rot = 0.00;

    double proportional_rot=0.0;
    double integral_rot=0.0;
    double derivative_rot=0.0;

    error_rot = beta;

    //PID controller to turn
    proportional_rot = error_rot;
    integral_rot = integral_rot_Old + dt*error_rot;
    derivative_rot = (error_rot - errorOld_rot)/dt;

    //Anti windup:
    if(std::fabs(integral_rot)>4.0){
        integral_rot = sign(integral_rot)*5.0;
    }

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = Kp_rot*proportional_rot + Ki_rot*integral_rot + Kd_rot*derivative_rot;
    if(twist_msg.angular.z > 3.0) twist_msg.angular.z = 3.0;
    if(twist_msg.angular.z < -3.0) twist_msg.angular.z = -3.0;

   if(twist_msg.angular.z > 0.0) std::cout << "TURNING LEFT....  beta: "<< beta << ", alpha: "<< alpha << ", theta: " << robot_theta << std::endl;
   if(twist_msg.angular.z < 0.0) std::cout << "TURNING RIGHT...  beta: "<< beta << ", alpha: "<< alpha << ", theta: " << robot_theta << std::endl;
    // std::cout << "integral " << integral_rot  << std::endl;

    errorOld_rot = error_rot;
    integral_rot_Old = integral_rot;

    twist_pub.publish(twist_msg);


}




void go_forward(double distance)
{
    geometry_msgs::Twist twist_msg;
    double now = ros::Time::now().toSec();
    double dt = now - lastTime;
    lastTime = now;

    double error_lin;
    double error_rot;

    double Kp_l = 1.0;
    double Kd_l = 0.08;
    double Ki_l = 0.05;
    double Kp_rot = 3.7;
    double Kd_rot = 0.7;
    double Ki_rot = 0.0;

    double proportional_lin = 0.0;
    double integral_lin=0.0;
    double derivative_lin=0.0;
    double proportional_rot=0.0;
    double integral_rot=0.0;
    double derivative_rot=0.0;

    error_lin= distance;
    error_rot = beta;

    //PID controller to go forward
    proportional_lin = error_lin;
    integral_lin = integral_lin_Old + error_lin;
    derivative_lin = (error_lin - errorOld_lin);
    //PID controller to align
    proportional_rot = error_rot;
    integral_rot = integral_rot_Old + error_rot;
    derivative_rot = (error_rot - errorOld_rot);

    twist_msg.linear.x = Kp_l*proportional_lin + Ki_l*integral_lin + Kd_l*derivative_lin;
    if (twist_msg.linear.x>0.35) twist_msg.linear.x = 0.35;

    twist_msg.angular.z = Kp_rot*proportional_rot + Ki_rot*integral_rot + Kd_rot*derivative_rot;
    if(twist_msg.angular.z > 4.0) twist_msg.angular.z = 4.0;
    if(twist_msg.angular.z < -4.0) twist_msg.angular.z = -4.0;

    errorOld_lin = error_lin;
    integral_lin_Old = integral_lin;
    errorOld_rot = error_rot;
    integral_rot_Old = integral_rot;

    std::cout << "GOING FORWARD....  beta: "<< beta << ", alpha: "<< alpha << ", theta: " << robot_theta << std::endl;

    twist_pub.publish(twist_msg);




    // if(distance >0.4){
    //     twist_msg.linear.x = 0.30;
    // }else if(distance >0.3){
    //     twist_msg.linear.x = 0.25;
    // }else if(distance >0.2){
    //     twist_msg.linear.x = 0.22;
    // }else if(distance >0.15){
    //     twist_msg.linear.x = 0.20;
    // }else if(distance >0.10){
    //     twist_msg.linear.x = 0.18;
    // }else if(distance >0.08){
    //     twist_msg.linear.x = 0.16;
    // }else if(distance >0.05){
    //     twist_msg.linear.x = 0.15;
    // }else if(distance >0.0){
    //     twist_msg.linear.x = 0.10;
    // }
}

    //std::cout << "Forward: linear x: "<< twist_msg.linear.x << ", angular z: "<<twist_msg.angular.z<< std::endl;
    
   // double start_time =ros::Time::now().toSec();
    //while (ros::Time::now().toSec()-start_time<FORWARD_DURATION){
    
     //}

    //twist_msg.linear.x = 0.0;
    //twist_pub.publish(twist_msg);




void object_detected_function(ras_msgs::Object_id msg)
{
    current_object_id = msg;

    x = (robot_x + current_object_id.x*cos(robot_theta));
    y = (robot_y + current_object_id.y*sin(robot_theta));

    if(current_object_id.object==200){
      // Here it is debris


    }
    else if(current_object_id.object==100){
        //If not already recognized at this position:
            object_seen = true;
            angle_to_object = atan2(current_object_id.y,current_object_id.x);

    }
    else{
        marker_object.color.a = 1.0; // Don't forget to set the alpha!
        marker_object.header.frame_id = "/map";
        marker_object.ns = "object";
        marker_object.scale.x = 0.05;
        marker_object.scale.y = 0.05;
        marker_object.scale.z = 0.0;
        marker_object.pose.orientation.x = 0;
        marker_object.pose.orientation.y = 0;
        marker_object.pose.orientation.z = 0;
        marker_object.pose.orientation.w = 1.0;
        if(current_object_id.shape ==2){// Blue Cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 1.0;
        }
        if(current_object_id.shape ==3){// Green Cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
        }
        if(current_object_id.shape ==4){// Yellow cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
        }
        if(current_object_id.shape ==5){// yellow Ball
            marker_object.type = visualization_msgs::Marker::SPHERE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
        }
        if(current_object_id.shape ==6){// red ball
            marker_object.type = visualization_msgs::Marker::SPHERE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 0.0;
        }
        if(current_object_id.shape ==7){// green cylinder
            marker_object.type = visualization_msgs::Marker::CYLINDER;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
        }
        if(current_object_id.shape ==8){// blue triangle
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 1.0;
        }
        if(current_object_id.shape ==9){//purple cross
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.5;
            marker_object.color.g = 0.2;
            marker_object.color.b = 1.0;
        }
        if(current_object_id.shape ==10){// orange star aka patric
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.4;
            marker_object.color.b = 0.0;
        }



    //marker_object.header.stamp = current_object_id.time;
    marker_object.header.stamp = ros::Time::now();
    marker_object.id = object_counter;
    marker_object.pose.position.x = y;
    marker_object.pose.position.y = x;
    

    object_pos_to_cost_map.header.stamp = ros::Time::now();  
    object_pos_to_cost_map.header.frame_id = object_counter;
    object_pos_to_cost_map.point.x=x;
    object_pos_to_cost_map.point.y=y;
    object_pos_to_cost_map.point.z=0;

    marker_object_vec.markers.push_back(marker_object);

    marker_object_pub.publish(marker_object_vec);
    object_pos_pub.publish(object_pos_to_cost_map);

    object_counter++;
    }


    }

    void stop_robot_and_wait(int time_wait){
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    twist_pub.publish(twist_msg);
    //wait a little bit (1 second right now)
    double control_frequency = 10.0;
    ros::Rate loop_rate(control_frequency);
    int counter = 0;
    while(counter<time_wait)
    {
        loop_rate.sleep();
        counter++;
        ros::spinOnce();
    }
    lastTime = ros::Time::now().toSec();

}

void localization_callback(localization::Position msg)    //meters
{
    robot_x=msg.x;
    robot_y=msg.y;
    robot_theta=msg.theta;

    double time_to_consider = msg.time;
    for (int i=0; i<times_to_save;i++){
        if (timestamp_vec[i].time>time_to_consider){
            robot_theta += timestamp_vec[i].delta_rot;
            robot_x += timestamp_vec[i].delta_trans*cos(robot_theta);
            robot_y += timestamp_vec[i].delta_trans*sin(robot_theta);
        }
    }
    pos_received = true;
}


private:

 bool object_seen;

 int rows;
 int col; 
 int object_counter;
 int path_node_index;
 
 double beta;
 double xEnd;
 double xStep;
 double x;
 double y;
 double angle_to_object;
 double newx, newy;
 double robot_x;
 double robot_y;
 double robot_theta;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "brain_node");
    BrainNode brain_node;
   

    // Control @ 10 Hz
    double control_frequency = 10.0;  
    ros::Rate loop_rate(control_frequency);

    
    while(brain_node.n.ok())
    {   
        if(brain_node.got_path==true && brain_node.pos_received)
        {
             // std::cout << "-----------here 1 -----------"<< std::endl;

            if (brain_node.check_at_correct_place())  
            {
                std::cout << "----------GO TO NEXT NODE-----------"<< std::endl;
                brain_node.stop_robot_and_wait(5);
                brain_node.get_next_path();

            }

            if(!final_node)
            {
                // std::cout << "-----------here 3 -----------"<< std::endl;
                brain_node.move_function();
            }else
            {
                brain_node.got_path = false;
                // Ask for new path
            }  

            if(object_seen)
            {
                
            } 
        }


        ros::spinOnce();
        loop_rate.sleep();
     }

    return 0;
}