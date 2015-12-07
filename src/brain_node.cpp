#include <sstream>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <cmath> 
#include "ros/ros.h"
#include "ras_msgs/Object_id.h"
#include "ras_msgs/Shape.h"
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
const double object_vicinity_threshold = 0.1;



//Memory for PID controller
double errorOld_rot=0.0;
double errorOld_lin=0.0;
double integral_rot=0.0;
double integral_lin=0.0;

//PID controller values
double Kp_rot_rot = 4.0;
double Ki_rot_rot = 0.0; 
double Kd_rot_rot = 0.0;
double Kp_l = 1.0;
double Kd_l = 0.0;
double Ki_l = 0.0;
double Kp_rot = 3.7;
double Kd_rot = 0.0;
double Ki_rot = 0.0;

double node_vicinity_threshold = 0.045;
double rotation_threshold = 0.3;

double control_frequency = 30.0; 

double alpha;
double lastTime;


bool flag_has_detected = false;

bool final_node = false;

bool flag_moving_forward = false;
bool flag_rotation = false;

struct timestamp{
    double time;
    double delta_trans;
    double delta_rot;
};


struct vector2{
    double x;
    double y;
};

struct vector2 closest_object;


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

    ras_msgs::Object_id object_to_classify;

    nav_msgs::OccupancyGrid grid_cost;
    nav_msgs::OccupancyGrid grid_obs;

   // ras_msgs::Object_id object_to_classify;

    geometry_msgs::Point Path[];
    geometry_msgs::PointStamped object_pos_to_cost_map;

    visualization_msgs::Marker marker_object;
    visualization_msgs::MarkerArray marker_object_vec;


    bool flag_object_detected;

    bool got_path;
    bool pos_received ;

    ros::NodeHandle n;

    ros::Subscriber object_classified_sub_;
    ros::Subscriber object_detected_sub;
    ros::Subscriber robot_pos_sub_;
    ros::Subscriber path_sub;
    ros::Subscriber odom_sub;

    ros::Publisher marker_object_pub;
    ros::Publisher object_pos_pub;
    ros::Publisher request_pub_;
    ros::Publisher request_back_path_pub;
    ros::Publisher twist_pub;
    ros::Publisher robot_pos_pub;

    BrainNode()
    {
        n = ros::NodeHandle("~");
        got_path=false;
        pos_received = false;
        flag_object_detected = false;
        path_node_index = 0;

        fill_timestamps();

        //The rows and cols needs to be one bigger than the length of the outer walls
        //grid_cost_map_pub = n.advertise<std::vector< std::vector<struct position_node> >("test",1);
        path_sub = n.subscribe<nav_msgs::GridCells>( "/nodes_generator/path", 10,&BrainNode::path_vector_function,this);
        object_classified_sub_ = n.subscribe<ras_msgs::Object_id>("/object/object",10,&BrainNode::object_classified_callback,this);
        object_detected_sub = n.subscribe<ras_msgs::Shape>("/object/shape",10,&BrainNode::object_detected_callback,this);
        robot_pos_sub_ = n.subscribe<localization::Position>("/position",10,&BrainNode::localization_callback,this);
        odom_sub = n.subscribe<motors::odometry>("/odometry",10,&BrainNode::odom_callback,this);
        
        request_pub_ = n.advertise<std_msgs::Int32>("/grid_generator/update_query", 10);
        request_back_path_pub = n.advertise<std_msgs::Int32>("/go_home", 10);
        twist_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",10);
        marker_object_pub = n.advertise<visualization_msgs::MarkerArray>( "/object_marker_to_rvis", 10);
        object_pos_pub= n.advertise<geometry_msgs::PointStamped>( "/object_pos", 10);
        robot_pos_pub = n.advertise<localization::Position>("/brain_position",10);
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
        
        //for (int i = 0; i< paths.size(); i++) std::cout << "node(" << i << "): x =" << paths[i].x << "  y =" << paths[i].y << std::endl;
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

    localization::Position pos_msg;
    pos_msg.x = robot_x;
    pos_msg.y = robot_y;
    pos_msg.theta = robot_theta;

    robot_pos_pub.publish(pos_msg);

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
    
    
    if((fabs(robot_x - newx) < node_vicinity_threshold) && (fabs(robot_y - newy) < node_vicinity_threshold))  return true;
    else return false;


}

void move_function()
{
    alpha = atan2(newy-robot_y,newx-robot_x);
    double dist_to_goal = std::sqrt((newx-robot_x)*(newx-robot_x) + (newy-robot_y)*(newy-robot_y));


    if(std::fabs(alpha - robot_theta) < std::fabs(-2.0 * M_PI + alpha - robot_theta)){
        if (std::fabs(alpha - robot_theta) < std::fabs(2.0 * M_PI + alpha - robot_theta)){
        beta = alpha - robot_theta;
        }
        else{
            beta = 2.0 * M_PI + alpha - robot_theta;
        }
    }
    else{
        beta = -2.0 * M_PI + alpha - robot_theta;
    }

    
    // std::cout << "Beta:"<< beta <<  std::endl;
    // std::cout << "ABS(Beta):"<< fabs(beta) <<  std::endl;

    if (fabs(beta)>rotation_threshold)
    {
        rotate(); // Stops and aligns 
    } 
    else
    {
        if(flag_rotation){

            stop_robot_and_wait(3);
            flag_rotation = false; //TODO: this is correct but can be confusing. If you want to remove this, think carefully.
        }
        go_forward(dist_to_goal);  
    }    
}

void rotate(){

    //Set some stuff true or false
    flag_moving_forward = false;
    if(!flag_rotation){ //This is set to true below
        integral_rot = 0.0;
        integral_lin = 0.0;
        errorOld_rot = beta;
    }

    geometry_msgs::Twist twist_msg;

    double error_rot;

    double proportional_rot = 0.0;
    double derivative_rot = 0.0;

    error_rot = beta;

    double now = ros::Time::now().toSec();
    if (flag_rotation){

        double dt = now - lastTime;
        integral_rot += error_rot*dt;
        derivative_rot = (error_rot - errorOld_rot)/dt;
        //Anti windup:
        if(std::fabs(integral_rot)>4.0){
            integral_rot = sign(integral_rot)*4.0;
        }

    }
    flag_rotation = true;  

    //PID controller to turn
    proportional_rot = error_rot;

    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = Kp_rot*proportional_rot + Ki_rot*integral_rot + Kd_rot*derivative_rot;
    if(twist_msg.angular.z > 5.0) twist_msg.angular.z = 5.0;
    if(twist_msg.angular.z < -5.0) twist_msg.angular.z = -5.0;

  // if(twist_msg.angular.z > 0.0) std::cout << "TURNING LEFT....  beta: "<< beta << ", alpha: "<< alpha << ", theta: " << robot_theta << std::endl;
  // if(twist_msg.angular.z < 0.0) std::cout << "TURNING RIGHT...  beta: "<< beta << ", alpha: "<< alpha << ", theta: " << robot_theta << std::endl;
    // std::cout << "integral " << integral_rot  << std::endl;

    errorOld_rot = error_rot;

    twist_pub.publish(twist_msg);
}




void go_forward(double distance)
{   
    //Set some stuff true or false
    flag_rotation = false;
    if(!flag_moving_forward){ //This is set to true below
        integral_rot = 0.0;
        integral_lin = 0.0;
        errorOld_lin = distance;
        errorOld_rot = beta;
    }

    geometry_msgs::Twist twist_msg;

    double error_lin;
    double error_rot;



    double proportional_lin = 0.0;
    double derivative_lin=0.0;
    double proportional_rot=0.0;
    double derivative_rot=0.0;

    error_lin = distance;
    error_rot = beta;

    //To be able to control with pid
    double now = ros::Time::now().toSec();
    if (flag_moving_forward){
        
        double dt = now - lastTime;

        integral_rot = integral_rot + error_rot*dt;
        derivative_rot = (error_rot - errorOld_rot)/dt;

        integral_lin = integral_lin + error_lin*dt;
        derivative_lin = (error_lin - errorOld_lin)/dt;
            //Anti windup:
        if(std::fabs(integral_rot)>4.0){
            integral_rot = sign(integral_rot)*4.0;
        }
        if(std::fabs(integral_lin)>4.0){
            integral_lin = sign(integral_lin)*4.0;
        }


    }
    lastTime = now;
    flag_moving_forward = true;  

    //PID controller to go forward
    proportional_lin = error_lin;
    
    //PID controller to align
    proportional_rot = error_rot;
    
    twist_msg.linear.x = Kp_l*proportional_lin + Ki_l*integral_lin + Kd_l*derivative_lin;
    if (twist_msg.linear.x>0.4) twist_msg.linear.x = 0.3;//0.4 was good
    if (twist_msg.linear.x<0.1) twist_msg.linear.x = 0.1;

    twist_msg.angular.z = Kp_rot*proportional_rot + Ki_rot*integral_rot + Kd_rot*derivative_rot;
    if(twist_msg.angular.z > 3.0) twist_msg.angular.z = 3.0;
    if(twist_msg.angular.z < -3.0) twist_msg.angular.z = -3.0;

    errorOld_lin = error_lin;
    errorOld_rot = error_rot;

   // std::cout << "GOING FORWARD....  beta: "<< beta << ", alpha: "<< alpha << ", theta: " << robot_theta << std::endl;

    twist_pub.publish(twist_msg);

}

void object_classified_callback(ras_msgs::Object_id msg){
     object_to_classify = msg;
     flag_object_detected = true;
}





void classify_object()
{
    //if (!flag_object_detected) return; //TODO: maybe do this.
    //if(!new_object_detected(object_to_classify.x,object_to_classify.y)) return;
    
    std::cout << "got message from akash"<< std::endl;
    //double x = (robot_x + object_to_classify.x*cos(robot_theta) - object_to_classify.y*sin(robot_theta));
    //double y = (robot_y + object_to_classify.x*sin(robot_theta) + object_to_classify.y*cos(robot_theta));

    double x = (robot_x + closest_object.x*cos(robot_theta) - closest_object.y*sin(robot_theta));
    double y = (robot_y + closest_object.x*sin(robot_theta) + closest_object.y*cos(robot_theta));

    //TODO: if old thing at this position, don't do anthing.


    //else if(object_to_classify.object==100){ //unknown}


        marker_object.color.a = 1.0; // Don't forget to set the alpha!
        marker_object.header.frame_id = "/map";
        marker_object.ns = "object";
        marker_object.scale.x = 0.05;
        marker_object.scale.y = 0.05;
        marker_object.scale.z = 0.05;
        marker_object.pose.orientation.x = 0;
        marker_object.pose.orientation.y = 0;
        marker_object.pose.orientation.z = 0;
        marker_object.pose.orientation.w = 1.0;
        if(object_to_classify.object==1){// Red Cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 0.0;
            std::cout<<" Red Cube "<<std::endl;
        }
        if(object_to_classify.object==2){// Blue Cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 1.0;
            std::cout<<" Blue Cube "<<std::endl;
        }
        if(object_to_classify.object ==3){// Green Cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
            std::cout<<" Green Cube "<<std::endl;
        }
        if(object_to_classify.object ==4){// Yellow cube
            marker_object.type = visualization_msgs::Marker::CUBE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
            std::cout<<" Yellow cube "<<std::endl;
        }
        if(object_to_classify.object ==5){// yellow Ball
            marker_object.type = visualization_msgs::Marker::SPHERE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
            std::cout<<" yellow Ball "<<std::endl;
        }
        if(object_to_classify.object==6){// red ball
            marker_object.type = visualization_msgs::Marker::SPHERE;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 0.0;
            std::cout<<" Red ball "<<std::endl;
        }
        if(object_to_classify.object ==7){// green cylinder
            marker_object.type = visualization_msgs::Marker::CYLINDER;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.0;
            marker_object.color.g = 1.0;
            marker_object.color.b = 0.0;
            std::cout<<" Green cylinder "<<std::endl;
        }
        if(object_to_classify.object ==8){// blue triangle
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 1.0;
            std::cout<<" Blue triangle "<<std::endl;
        }
        if(object_to_classify.object ==9){//purple cross
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.5;
            marker_object.color.g = 0.2;
            marker_object.color.b = 1.0;
            std::cout<<" Purple cross "<<std::endl;
        }
        if(object_to_classify.object ==10){// orange star aka patric
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.4;
            marker_object.color.b = 0.0;
            std::cout<<" Orange star "<<std::endl;
        }
         if(object_to_classify.object ==11){//purple star
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 0.5;
            marker_object.color.g = 0.2;
            marker_object.color.b = 1.0;
            std::cout<<" Purple star "<<std::endl;
        }
        if(object_to_classify.object ==12){//red hollow cube
            marker_object.type = visualization_msgs::Marker::LINE_LIST;
            marker_object.action = visualization_msgs::Marker::ADD;
            marker_object.color.r = 1.0;
            marker_object.color.g = 0.0;
            marker_object.color.b = 0.0;
            std::cout<<" Red hollow cube "<<std::endl;
        }



    //marker_object.header.stamp = object_to_classify.time;
    marker_object.header.stamp = ros::Time::now();
    marker_object.id = object_counter;
    marker_object.pose.position.x = x;
    marker_object.pose.position.y = y;
    

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


void object_detected_callback(ras_msgs::Shape msg){
    closest_object.x = msg.x;
    closest_object.y = msg.y;
    flag_has_detected = true;
}




void stop_robot_and_wait(int time_wait){
    std::cout << "waiting for " <<  time_wait << std::endl;
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    twist_pub.publish(twist_msg);
    //wait a little bit (1 second right now)

    ros::Rate loop_rate(control_frequency);
    int counter = 0;
    while(counter<time_wait)
    {
        loop_rate.sleep();
        counter++;
        ros::spinOnce();
    }
    

}

bool check_if_object_is_close(double x,double y){
   double angle_derp = atan2(y,x);
    std::cout << sqrt(x*x + y*y) << std::endl;
    if(sqrt(x*x + y*y)<=0.4 && std::fabs(angle_derp)<0.6 ){
        return true;
    }else{
        return false;
    }
}


//Returns true if we believe that this is a new object
bool new_object_detected(double x,double y){
    std::cout << "new object detected?" << std::endl;

    //If the object is close enought and within some angle
    //x, y local coordinates

    double x_global = robot_x + x*cos(robot_theta) - y*sin(robot_theta);
    double y_global = robot_y + x*sin(robot_theta) + y*cos(robot_theta);
    for(int i=0; i<marker_object_vec.markers.size(); i++)
    {
        double marker_x = marker_object_vec.markers[i].pose.position.x;
        double marker_y = marker_object_vec.markers[i].pose.position.y;

        //std::cout << "here " << i << std::endl;

        double dist = sqrt((marker_x - x_global)*(marker_x - x_global) + (marker_y - y_global)*(marker_y - y_global));
        std::cout << "object is on distance: "<< dist << std::endl;
        if (dist<=object_vicinity_threshold){
            return false;

        }

    }
    return true;
    
}


void stop_and_classify_object(){
    std::cout << "classifying" << std::endl;
    flag_object_detected = false;
    stop_robot_and_wait(100);
    classify_object();



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


 int rows;
 int col; 
 int object_counter;
 int path_node_index;
 
 double beta;
 double xEnd;
 double xStep;
 double angle_to_object;
 double newx, newy;
 double robot_x;
 double robot_y;
 double robot_theta;
};




bool setup(ros::NodeHandle nh){
    if (!nh.hasParam("/has_parameters")){
        return false;
    }

    nh.getParam("/control_Kp_rot_rot", Kp_rot_rot);
    nh.getParam("/control_Ki_rot_rot", Ki_rot_rot);
    nh.getParam("/control_Kd_rot_rot", Kd_rot_rot);

    nh.getParam("/control_Kp_l", Kp_l);
    nh.getParam("/control_Ki_l", Ki_l);
    nh.getParam("/control_Kd_l", Kd_l);

    nh.getParam("/control_Kp_rot", Kp_rot);
    nh.getParam("/control_Ki_rot", Ki_rot);
    nh.getParam("/control_Kd_rot", Kd_rot);

    nh.getParam("/node_vicinity_threshold", node_vicinity_threshold);
    nh.getParam("/rotation_threshold", rotation_threshold);
    


    return true;
}


int main(int argc, char **argv)
{

    std::cout << "starting burainuuuu" << std::endl;
    int timer = 0;
    bool time_ran_out = false;

    
    ros::init(argc, argv, "brain_node");
    BrainNode brain_node;

    if(!setup(brain_node.n)){
        std::cout << "phukk yooooo" << std::endl;
        return 1; //Error, no parameters
    }
        
    ros::Rate loop_rate(control_frequency);

    
    while(brain_node.n.ok())
    {
        if(!time_ran_out)
        {
            if (flag_has_detected  && brain_node.check_if_object_is_close(closest_object.x,closest_object.y) && brain_node.new_object_detected(closest_object.x,closest_object.y)){
                brain_node.stop_and_classify_object(); //This will do ros spin withing a loop so it doesn't return immediately
                flag_has_detected = false;
                //continue;
            }else{
                closest_object.x = -500.0;
                closest_object.y = -500.0;
            }

            if(brain_node.got_path==true && brain_node.pos_received)
            {
                if (brain_node.check_at_correct_place())  
                {
                    std::cout << "----------GO TO NEXT NODE-----------"<< std::endl;
                    brain_node.stop_robot_and_wait(5); //This will do ros spin withing a loop so it doesn't return immediately
                    brain_node.get_next_path();

                }

                if(!final_node)
                {
                    brain_node.move_function();

                }else
                {
                    brain_node.got_path = false;
                    final_node=false;
                    // Ask for new path
                    std_msgs::Int32 request_path_message_herp_derp;
                    request_path_message_herp_derp.data = 1;
                    brain_node.request_pub_.publish(request_path_message_herp_derp);

                }  
            }            
        }else
        {  
             if(brain_node.got_path==true && brain_node.pos_received)
            {
                if (brain_node.check_at_correct_place())  
                {
                    std::cout << "----------GO TO NEXT NODE-----------"<< std::endl;
                    brain_node.stop_robot_and_wait(5); //This will do ros spin withing a loop so it doesn't return immediately
                    brain_node.get_next_path();
                }

                brain_node.move_function(); 
            }            

        }




        timer++;
        if(timer == 6300) //3min30
        {
            time_ran_out = true;
            brain_node.stop_robot_and_wait(5);
            // Time to go home
            std_msgs::Int32 request_back_path;
            request_back_path.data = 1;
            brain_node.request_back_path_pub.publish(request_back_path);
            // Ask for new path
            std_msgs::Int32 request_path_message_herp_derp;
            request_path_message_herp_derp.data = 1;
            brain_node.request_pub_.publish(request_path_message_herp_derp);

        }
            



        ros::spinOnce();
        loop_rate.sleep();
     }

     std::cout << "WATTAFAKK MAAAEEEEEEN" << std::endl;
    return 0;
}
