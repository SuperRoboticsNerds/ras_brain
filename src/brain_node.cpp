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
#include "nav_msgs/Odometry.h"
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



class BrainNode
{
public:

    std::vector< std::vector<grid_generator::Grid_map_struct> > matrix_a;
    std::vector<grid_generator::Grid_map_struct> grid_map_send;
    std::vector<geometry_msgs::Point> paths;
    std_msgs::Float32MultiArray vec_data;
    nav_msgs::OccupancyGrid grid_cost;
    nav_msgs::OccupancyGrid grid_obs;
    ras_msgs::Object_id current_object_id;
    geometry_msgs::Point Path[];
    geometry_msgs::PointStamped object_pos_to_cost_map;

    visualization_msgs::Marker marker_object;
    visualization_msgs::MarkerArray marker_object_vec;




    ros::NodeHandle n;

    ros::Publisher marker_object_pub;
    ros::Publisher object_pos_pub;
    ros::Publisher requst_pub_;
    ros::Publisher twist_pub;

    ros::Subscriber object_pos_sub_;
    ros::Subscriber robot_pos_sub_;
    ros::Subscriber path_sub;


    BrainNode()
    {
        n = ros::NodeHandle("~");
        object_seen = false;
        //The rows and cols needs to be one bigger than the length of the outer walls
        //grid_cost_map_pub = n.advertise<std::vector< std::vector<struct position_node> >("test",1);
        path_sub = n.subscribe<nav_msgs::GridCells>( "/nodes_generator/path", 1,&BrainNode::path_vector_function,this);
        object_pos_sub_ = n.subscribe<ras_msgs::Object_id>("/objects/object",1,&BrainNode::object_detected_function,this);
        robot_pos_sub_ = n.subscribe<localization::Position>("/position",1,&BrainNode::current_robot_position_function,this);

        
        requst_pub_ = n.advertise<std_msgs::Int32>("/grid_generator/update_query", 100);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",100);
        marker_object_pub = n.advertise<visualization_msgs::MarkerArray>( "/object_to_rvis_grid_map", 1);
        object_pos_pub= n.advertise<geometry_msgs::PointStamped>( "/object_pos", 1);
        
    }
    ~BrainNode()
    {
        //delete motor_controller_;
    }



void path_vector_function(nav_msgs::GridCells msg){
        paths = msg.cells;
   }

void get_next_path(){
    geometry_msgs::Point pt = paths[0];
    newx = pt.x;
    newy = pt.y;
    paths.erase(paths.begin());
}



void move_function(){
    //while not close
    double alpha = atan2(newy-robot_y,newx-robot_x);
    if(!(std::abs(alpha-robot_theta) < 0.2 || std::abs(2*M_PI-robot_theta+alpha) < 0.2)){
        if(std::abs(alpha-robot_theta) < std::abs(2*M_PI-robot_theta+alpha)){
            rotate(alpha-robot_theta);
        }
        else{
            rotate(2*M_PI-robot_theta+alpha);
        }
    }
    double dist_to_goal = std::sqrt((newx-robot_x)*(newx-robot_x) + (newy-robot_y)*(newy-robot_y));
    if (!dist_to_goal<0.05){
        go_forward(std::max(0.2,dist_to_goal));
    }
    if(object_seen){
        move_to_object();
    }
}

void move_to_object(){

    rotate(angle_to_object);

    //perhaps move a little close

    //ask for determination - 




}

void rotate(double angle){
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = 0.2; // [m/s]

    twist_pub.publish(twist_msg);
}

void go_forward(double distance){
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = M_PI/4; // [rad/s]

    twist_pub.publish(twist_msg);

}




void object_detected_function(ras_msgs::Object_id msg){
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
void current_robot_position_function(localization::Position msg){
    robot_x=msg.x;
    robot_y=msg.y;
    robot_theta=msg.theta;

    }


private:

    bool object_seen;
 int rows;
 int col; 
 int object_counter;
 
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
    int counter;
    counter=0;
    ros::init(argc, argv, "brain_node");
    BrainNode brain_node;
   

    // Control @ 10 Hz
    double control_frequency = 10.0;
  
    ros::Rate loop_rate(control_frequency);
    
    while(brain_node.n.ok())
    {   




        counter++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}