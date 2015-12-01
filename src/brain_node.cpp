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

#define ROT_DURATION  0.2
#define FORWARD_DURATION  0.4


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


    bool got_path;
    bool pos_received ;

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
        got_path=false;
        path_node_index =0;
        //The rows and cols needs to be one bigger than the length of the outer walls
        //grid_cost_map_pub = n.advertise<std::vector< std::vector<struct position_node> >("test",1);
        path_sub = n.subscribe<nav_msgs::GridCells>( "/nodes_generator/path", 1,&BrainNode::path_vector_function,this);
        object_pos_sub_ = n.subscribe<ras_msgs::Object_id>("/objects/object",1,&BrainNode::object_detected_function,this);
        robot_pos_sub_ = n.subscribe<localization::Position>("/position",1,&BrainNode::current_robot_position_function,this);

        
        requst_pub_ = n.advertise<std_msgs::Int32>("/grid_generator/update_query", 100);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",100);
        marker_object_pub = n.advertise<visualization_msgs::MarkerArray>( "/object_to_rvis_grid_map", 1);
        object_pos_pub= n.advertise<geometry_msgs::PointStamped>( "/object_pos", 1);
        pos_received = false;
    }
    ~BrainNode()
    {
        //delete motor_controller_;
    }



void path_vector_function(nav_msgs::GridCells msg){
        paths = msg.cells;
        //path_node_index=0;
        got_path=true;
   }

void get_next_path(){
    std::cout << "-----------here 3 -----------"<< std::endl;
    geometry_msgs::Point pt = paths[path_node_index];
    newx = pt.x;
    newy = pt.y;
    path_node_index++;
    //paths.erase(paths.begin());
    got_path=false;
    std::cout << "-----------here 4 -----------"<< std::endl;
}

void check_at_correct_place(){
    //std::cout << "-----------correct 1 -----------"<< std::endl;
    for(int i = -5; i <= 5;i++){
        for(int j = -10; j <= 10;j++){
        //std::cout << "-----------correct k:i -----------"<< std::endl;
            if((floor(robot_x)==floor(newx+i)) && (floor(robot_y)==floor(newy+j))){
                std::cout << "-----------check ************************** -----------"<< std::endl;
                get_next_path();

            }
        }
    }

}




void move_function(){
    //go_forward(1.0);
    //while not close
    std::cout << "inside"<< std::endl;
    double alpha = atan2(newy-robot_y,newx-robot_x);
    double dist_to_goal = std::sqrt((newx-robot_x)*(newx-robot_x) + (newy-robot_y)*(newy-robot_y));
    double beta = (robot_theta-alpha);
    double beta_2 = (alpha-robot_theta);
    // if(robot_theta >=0){
        if(beta>M_PI){        
            //Turn left
            std::cout << "---Turn left > PI----   Beta:"<< beta<< std::endl;
            rotate_2(1);
        }else if(beta>0.18){
            //Turn right
            std::cout << "---Turn right > 0.18----   Beta:"<< beta <<  std::endl;
            rotate_2(2);
        }else if(beta <(-M_PI)){
        // turn right
            std::cout << "---Turn right < (-PI)----   Beta:"<< beta <<  std::endl;
        rotate_2(2);
        }else if(beta < (-0.18)){
            //turn left
            std::cout << "---Turn left > 0.18----   Beta:"<< beta <<  std::endl;
            rotate_2(1);
        }else if(dist_to_goal>5){
            go_forward(std::max(20.0,dist_to_goal));
        }
//     }
//     else if(robot_theta<0){
   
// //if((std::abs(alpha-robot_theta) > 0.26) || (std::abs(2*M_PI-(alpha-robot_theta)) > 0.26)){
//         if(beta>0.18){
//             //Turn left
//             rotate_2(1);
//         }else if(beta < (-0.18)){
//             //turn left
//             rotate_2(2);
//         }else if(dist_to_goal>5){
//             go_forward(std::max(20.0,dist_to_goal));
//         }
//      }
    //if(((alpha-robot_theta) > 0.26) || ((alpha-robot_theta) < -0.26)){
    //    std::cout << "alpha:"<<alpha << ", theta:"<<robot_theta <<", ANGLE left: "<<(alpha-robot_theta)<< ", right: "<<  2*M_PI-(alpha-robot_theta) << std::endl;
    //    if(std::abs(alpha-robot_theta) > std::abs(2*M_PI-robot_theta+alpha)){
    //        //rotate(alpha-robot_theta);
    //    }
    //    else{
    //        //rotate(2*M_PI-robot_theta+alpha);
    //    }
    //}
    //else if (dist_to_goal>5){
        //go_forward(std::max(50.0,dist_to_goal));
    //}
    //std::cout << "-----------middle -----------"<< std::endl;
    //ros::spinOnce();
    //std::cout << "-----------middle 2-----------"<< std::endl;
    /*
    if(object_seen){
        move_to_object();
    }*/
    check_at_correct_place();
    //std::cout << "-----------end -----------"<< std::endl;
}

void move_to_object(){

    //rotate(angle_to_object);

    //perhaps move a little close

    //ask for determination - 
}

void rotate_2(int turn_direction){
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    if (turn_direction==2){
    twist_msg.angular.z = -3.0;
    //negativt är höger
    }
    else if (turn_direction==1){
        twist_msg.angular.z = 3.0;
        //vänster
    }
    else{
        twist_msg.angular.z = 0.0;
    }
    std::cout << "Turns: linear x: "<< twist_msg.linear.x << ", angular z: "<<twist_msg.angular.z<< std::endl;
    double start_time =ros::Time::now().toSec();
    while (ros::Time::now().toSec()-start_time<ROT_DURATION){
        twist_pub.publish(twist_msg);
    }

    twist_msg.angular.z = 0.0;
    twist_pub.publish(twist_msg);
}


// void rotate(double angle){
//     geometry_msgs::Twist twist_msg;
//     twist_msg.linear.x = 0.0;
//     if (angle<0.1){
//     twist_msg.angular.z = -3.0;}
//     else if (angle>0.1){
//         twist_msg.angular.z = 3.0;
//     }
//     else{
//         twist_msg.angular.z = 0.0;
//     }
//     std::cout << "Turns: linear x: "<< twist_msg.linear.x << ", angular z: "<<twist_msg.angular.z<< std::endl;
//     // double start_time =ros::Time::now().toSec();
//     // while (ros::Time::now().toSec()-start_time<ROT_DURATION){
//     twist_pub.publish(twist_msg);
// // }

//     // twist_msg.angular.z = 0.0;
//     //twist_pub.publish(twist_msg);
// }

void go_forward(double distance){
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.4;
    twist_msg.angular.z = 0.0;
    std::cout << "Forward: linear x: "<< twist_msg.linear.x << ", angular z: "<<twist_msg.angular.z<< std::endl;
    
    double start_time =ros::Time::now().toSec();
    while (ros::Time::now().toSec()-start_time<FORWARD_DURATION){
    twist_pub.publish(twist_msg);
     }

     twist_msg.linear.x = 0.0;
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
    //std::cout << "pos recived"<< std::endl;
    robot_x=msg.x*100.0;
    robot_y=msg.y*100.0;
    robot_theta=msg.theta;

    pos_received = true;
    }

int pos_received_func(){
    if (pos_received==true){
        return 1;
    }
    else {
        return 0;
    }
}
private:

    bool object_seen;

 int rows;
 int col; 
 int object_counter;
 int path_node_index;
 
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
    int got_path_game;
    counter=0;
    got_path_game=0;
    ros::init(argc, argv, "brain_node");
    BrainNode brain_node;
   

    // Control @ 10 Hz
    double control_frequency = 10.0;
  
    ros::Rate loop_rate(control_frequency);
    
    while(brain_node.n.ok())
    {   
        if(brain_node.got_path==true){
           //brain_node.go_forward(10);
           // std::cout << "-----------here 1 -----------"<< std::endl;
           brain_node.get_next_path();
           //std::cout << "------------Here 2-------------"<< std::endl;
           got_path_game=1;
        }


  

        if((brain_node.pos_received_func() == 1)&&(got_path_game==1)){
            //std::cout << "-----------here 5 -----------"<< std::endl;
            brain_node.move_function();
            //std::cout << "-----------here 6 -----------"<< std::endl;
        }


        counter++;
        ros::spinOnce();
        loop_rate.sleep();
     }

    return 0;
}
