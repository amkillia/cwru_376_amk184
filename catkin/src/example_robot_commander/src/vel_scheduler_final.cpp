
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <ros/ros.h>
//#include <cwru_base/Pose.h>
#include <iostream>
#include <fstream>
//#include <cwru_base/cRIOSensors.h>
#include <std_msgs/Bool.h>
using namespace std;

bool estop;
string check;


// set some dynamic limits...
const double v_max = 5.0; //1m/sec is a slow walk
const double v_min = 0.1; // if command velocity too low, robot won't move
const double a_max = 0.1; //1m/sec^2 is 0.1 g's
//const double a_max_decel = 0.1; // TEST
const double omega_max = 1.0; //1 rad/sec-> about 6 seconds to rotate 1 full rev
const double alpha_max = 0.25; //0.5 rad/sec^2-> takes 2 sec to get from rest to full omega
const double DT = 0.05; // choose an update rate of 20Hz; go faster with actual hardware

// globals for communication w/ callbacks:
double odom_vel_ = 0.0; // measured/published system speed
double odom_omega_ = 0.0; // measured/published system yaw rate (spin)
double odom_x_ = 0.0;
double odom_y_ = 0.0;
double odom_phi_ = 0.0;
double dt_odom_ = 0.0;
ros::Time t_last_callback_;
double dt_callback_=0.0;



// receive the pose and velocity estimates from the simulator (or the physical robot)
// copy the relevant values to global variables, for use by "main"
// Note: stdr updates odom only at 10Hz; Jinx is 50Hz (?)
void odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    //here's a trick to compute the delta-time between successive callbacks:
    dt_callback_ = (ros::Time::now() - t_last_callback_).toSec();
    t_last_callback_ = ros::Time::now(); // let's remember the current time, and use it next iteration

    if (dt_callback_ > 0.15) { // on start-up, and with occasional hiccups, this delta-time can be unexpectedly large
        dt_callback_ = 0.1; // can choose to clamp a max value on this, if dt_callback is used for computations elsewhere
        ROS_WARN("large dt; dt = %lf", dt_callback_); // let's complain whenever this happens
    }
    
    // copy some of the components of the received message into global vars, for use by "main()"
    // we care about speed and spin, as well as position estimates x,y and heading
    odom_vel_ = odom_rcvd.twist.twist.linear.x;
    odom_omega_ = odom_rcvd.twist.twist.angular.z;
    odom_x_ = odom_rcvd.pose.pose.position.x;
    odom_y_ = odom_rcvd.pose.pose.position.y;
    //odom publishes orientation as a quaternion.  Convert this to a simple heading
    // see notes above for conversion for simple planar motion
    double quat_z = odom_rcvd.pose.pose.orientation.z;
    double quat_w = odom_rcvd.pose.pose.orientation.w;
    odom_phi_ = 2.0*atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion

    // the output below could get annoying; may comment this out, but useful initially for debugging
    ROS_INFO("odom CB: x = %f, y= %f, phi = %f, v = %f, omega = %f", odom_x_, odom_y_, odom_phi_, odom_vel_, odom_omega_);
}

    void linear_motion (double length, ros::Publisher vel_cmd_publisher, ros::Rate rtimer)
    {
    
        double segment_length_done = 0.0; // need to compute actual distance travelled within the current segment
        double start_x = 0.0; // fill these in with actual values once odom message is received
        double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
        
        double start_phi = 0.0;

        double scheduled_vel = 0.0; //desired vel, assuming all is per plan
        double scheduled_omega = 0.0; //ashley added this, desired yaw rate
        double new_cmd_vel = 0.1; // value of speed to be commanded; update each iteration
        double new_cmd_omega = 0.0; // update spin rate command as well

        geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands

        cmd_vel.linear.x = 0.0; // initialize these values to zero
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        // let's wait for odom callback to start getting good values...
        odom_omega_ = 1000000; // absurdly high
        ROS_INFO("waiting for valid odom callback...");
        t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
        while (odom_omega_ > 1000) {
            rtimer.sleep();
            ros::spinOnce();
        }
        ROS_INFO("received odom message; proceeding");
        start_x = odom_x_;
        start_y = odom_y_;
        start_phi = odom_phi_;
        ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);

        // compute some properties of trapezoidal velocity profile plan velocity:
        double T_accel = v_max / a_max; //...assumes start from rest
        double T_decel = v_max / a_max; //(for same decel as accel); assumes brake to full halt
        double dist_accel = 0.5 * a_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
        double dist_decel = 0.5 * a_max * (T_decel * T_decel);; //same as ramp-up distance
        double dist_const_v = length - dist_accel - dist_decel; //if this is <0, never get to full spd
        double T_const_v = dist_const_v / v_max; //will be <0 if don't get to full speed
        double T_segment_tot = T_accel + T_decel + T_const_v; // expected duration of this move

        while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
        {
            ros::spinOnce(); // allow callbacks to populate fresh data
            // compute distance travelled so far:
            double delta_x = odom_x_ - start_x;
            double delta_y = odom_y_ - start_y;
            segment_length_done = sqrt(delta_x * delta_x + delta_y * delta_y);
             ROS_INFO("dist travelled: %f", segment_length_done);
            double dist_to_go = length - segment_length_done;

            //use segment_length_done to decide what vel should be, as per plan
            if (dist_to_go<= 0.0) { // at goal, or overshot; stop!
                scheduled_vel=0.0;
            }
            else if (dist_to_go <= dist_decel) { //possibly should be braking to a halt
                // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
                // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
                scheduled_vel = sqrt(2 * dist_to_go * a_max);
                ROS_INFO("braking zone: v_sched = %f",scheduled_vel);
            }
            else { // not ready to decel, so target vel is v_max, either accel to it or hold it
                scheduled_vel = v_max;
            }
        
            //how does the current velocity compare to the scheduled vel?
            if (odom_vel_ < scheduled_vel) {  // maybe we halted, e.g. due to estop or obstacle;
                // may need to ramp up to v_max; do so within accel limits
                double v_test = odom_vel_ + a_max*dt_callback_; // if callbacks are slow, this could be abrupt
                // operator:  c = (a>b) ? a : b;
                new_cmd_vel = (v_test < scheduled_vel) ? v_test : scheduled_vel; //choose lesser of two options
                // this prevents overshooting scheduled_vel
            } else if (odom_vel_ > scheduled_vel) { //travelling too fast--this could be trouble
                // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
                // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
                ROS_INFO("odom vel: %f; sched vel: %f",odom_vel_,scheduled_vel); //debug/analysis output; can comment this out
            
                double v_test = odom_vel_ - 1.2 * a_max*dt_callback_; //moving too fast--try decelerating faster than nominal a_max

                new_cmd_vel = (v_test > scheduled_vel) ? v_test : scheduled_vel; // choose larger of two options...don't overshoot scheduled_vel
            } else {
                new_cmd_vel = scheduled_vel; //silly third case: this is already true, if here.  Issue the scheduled velocity
            }
            ROS_INFO("cmd vel: %f",new_cmd_vel); // debug output

            cmd_vel.linear.x = new_cmd_vel;
            cmd_vel.angular.z = new_cmd_omega; // spin command; always zero, in this example
            if (dist_to_go <= 0.0) { //uh-oh...went too far already!
                cmd_vel.linear.x = 0.0;  //command vel=0
            }
            vel_cmd_publisher.publish(cmd_vel); // publish the command to robot0/cmd_vel
            rtimer.sleep(); // sleep for remainder of timed iteration
            if (dist_to_go <= 0.0) break; // halt this node when this segment is complete.
        }
    }

    void angular_motion (double turn, ros::Publisher vel_cmd_publisher, ros::Rate rtimer)
    {

        double segment_length_done = 0.0; // need to compute actual distance travelled within the current segment
        double start_x = 0.0; // fill these in with actual values once odom message is received
        double start_y = 0.0; // subsequent segment start coordinates should be specified relative to end of previous segment
        
        double start_phi = 0.0;

        double scheduled_omega = 0.0; //ashley added this, desired yaw rate
        double new_cmd_omega = 0.05; // update spin rate command as well

        geometry_msgs::Twist cmd_vel; //create a variable of type "Twist" to publish speed/spin commands

        cmd_vel.linear.x = 0.0; // initialize these values to zero
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        // let's wait for odom callback to start getting good values...
        odom_omega_ = 1000000; // absurdly high
        ROS_INFO("waiting for valid odom callback...");
        t_last_callback_ = ros::Time::now(); // initialize reference for computed update rate of callback
        while (odom_omega_ > 1000) {
            rtimer.sleep();
            ros::spinOnce();
        }
        ROS_INFO("received odom message; proceeding");
        start_x = odom_x_;
        start_y = odom_y_;
        start_phi = odom_phi_;
        ROS_INFO("start pose: x %f, y= %f, phi = %f", start_x, start_y, start_phi);

        // compute some properties of trapezoidal velocity profile plan:
        double T_accel = omega_max / alpha_max; //...assumes start from rest
        double T_decel = omega_max / alpha_max; //(for same decel as accel); assumes brake to full halt
        double dist_accel = 0.5 * alpha_max * (T_accel * T_accel); //distance rqd to ramp up to full speed
        double dist_decel = 0.5 * alpha_max * (T_decel * T_decel);; //same as ramp-up distance
        double dist_const_omega = turn - dist_accel - dist_decel; //if this is <0, never get to full spd
        double T_const_omega = dist_const_omega / omega_max; //will be <0 if don't get to full speed
        double T_segment_tot = T_accel + T_decel + T_const_omega; // expected duration of this move

        while (ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted (or ctl-C)
        {
            ros::spinOnce(); // allow callbacks to populate fresh data
            // compute angle travelled so far:

            ROS_INFO("start_phi: %f", start_phi);
            ROS_INFO("odom_phi: %f", odom_phi_);
            double delta_phi = odom_phi_ - start_phi;
            segment_length_done = delta_phi;

            ROS_INFO("dist travelled: %f", segment_length_done);
            double dist_to_go = turn  - segment_length_done;
            ROS_INFO("dist_to_go: %f", dist_to_go);

            //use segment_length_done to decide what vel should be, as per plan
            if (dist_to_go >= 0.0) { // at goal, or overshot; stop!
                scheduled_omega= 0.0;
            }
            else if (dist_to_go <= dist_decel) { //possibly should be braking to a halt
                // dist = 0.5*a*t_halt^2; so t_halt = sqrt(2*dist/a);   v = a*t_halt
                // so v = a*sqrt(2*dist/a) = sqrt(2*dist*a)
                ROS_INFO("dist_decel %f", dist_decel);
                ROS_INFO("alpha_max %f", alpha_max);
                scheduled_omega = -1*sqrt(-2 * (dist_to_go) * alpha_max);
                ROS_INFO("braking zone: v_sched = %f",scheduled_omega);
            }
            else { // not ready to decel, so target vel is v_max, either accel to it or hold it
                scheduled_omega = -1* omega_max;
            }
      

            //how does the current velocity compare to the scheduled vel?
            if (odom_omega_ > scheduled_omega) {  // maybe we halted, e.g. due to estop or obstacle;
                // may need to ramp up to v_max; do so within accel limits
                ROS_INFO("dt_callback_ %f", dt_callback_);
                double v_test = odom_omega_ +alpha_max*dt_callback_; // if callbacks are slow, this could be abrupt
                // operator:  c = (a>b) ? a : b;
                new_cmd_omega = (v_test < scheduled_omega) ? v_test : scheduled_omega; //choose lesser of two options
                // this prevents overshooting scheduled_vel
            } else if (odom_omega_ < scheduled_omega) { //travelling too fast--this could be trouble
                // ramp down to the scheduled velocity.  However, scheduled velocity might already be ramping down at a_max.
                // need to catch up, so ramp down even faster than a_max.  Try 1.2*a_max.
                ROS_INFO("odom omega: %f; sched omega: %f",odom_omega_,scheduled_omega); //debug/analysis output; can comment this out
                
                double omega_test = odom_omega_ - 1.2 * alpha_max*dt_callback_; //moving too fast--try decelerating faster than nominal a_max

                new_cmd_omega = (omega_test > scheduled_omega) ? omega_test : scheduled_omega; // choose larger of two options...don't overshoot scheduled_vel
            } else {
                new_cmd_omega = scheduled_omega; //silly third case: this is already true, if here.  Issue the scheduled velocity
            }
            ROS_INFO("cmd omega: %f",new_cmd_omega); // debug output

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = new_cmd_omega ; // spin command; 

            if (dist_to_go >= 0.0) { //uh-oh...went too far already!
                cmd_vel.angular.z = 0.0;  //command vel=0
            }

            vel_cmd_publisher.publish(cmd_vel); // publish the command to robot0/cmd_omega
            rtimer.sleep(); // sleep for remainder of timed iteration
            if (dist_to_go >= 0.0) break; // halt this node when this segment is complete.
        }
    }

    void estopCallback(const std_msgs::Bool::ConstPtr& estop) 
    {
        if (estop->data == true)
            check = "estop_off";  // means motors are ENABLED
        else if (estop->data == false)
            check = "estop_on";  // means motors are DISABLED

        cout<<check<<endl;
    }

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_scheduler"); // name of this node will be "minimal_publisher1"
    ros::NodeHandle nh; // get a ros nodehandle; standard yadda-yadda
    //create a publisher object that can talk to ROS and issue twist messages on named topic;
    // note: this is customized for stdr robot; would need to change the topic to talk to jinx, etc.
    ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/robot0/odom", 1, odomCallback);
    ros::Rate rtimer(1 / DT); // frequency corresponding to chosen sample period DT; the main loop will run this fast
    // receive odom messages and strip off the components we want to use
    // tested this OK w/ stdr
    
    // define the desired path length of this segment

    double vv[5] = { 4.75,-1.56179633,12.5,-1.56179633,9.0 };
    vector<double> pathVector (vv, vv + sizeof(vv) / sizeof(vv[0]) );


    for(long index=0; index < (long)pathVector.size(); ++index)
    {
        if(index % 2 == 0)
        {
           linear_motion (pathVector.at(index), vel_cmd_publisher, rtimer); 
        }
        else
        {
           angular_motion (pathVector.at(index), vel_cmd_publisher, rtimer); 
        }
    }
    
    ROS_INFO("completed move distance");

}

