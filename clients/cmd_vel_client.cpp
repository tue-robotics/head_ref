#include "actionlib/client/action_client.h"
#include "head_ref/HeadReferenceAction.h"

#include <geometry_msgs/Twist.h>

typedef actionlib::ActionClient<head_ref::HeadReferenceAction> HeadReferenceActionClient;

#define PI 3.14159265

HeadReferenceActionClient* ac;

int priority;
double pan_vel, tilt_vel;
std::string frame;

void cmdVelCallback(const geometry_msgs::TwistConstPtr& tw)
{
    if (tw->linear.x == 0.0 && tw->linear.y == 0 && tw->angular.z == 0)
        return;

    // send a goal to the action
    head_ref::HeadReferenceGoal goal;
    goal.target_point.header.frame_id = frame;
    goal.target_point.header.stamp = ros::Time::now();

    double dt = 3; //! TODO: Hardcoded values in this function
    double x = dt * (tw->linear.x * cos(tw->angular.z) + tw->linear.y * cos(PI/2 + tw->angular.z));
    double y = dt * (tw->linear.x * sin(tw->angular.z) + tw->linear.y * sin(PI/2 + tw->angular.z));

    double th = atan2(y, x);

    //! When only turning
    if (tw->linear.y*tw->linear.y+tw->linear.x*tw->linear.x < 0.1*0.1)
        if (tw->angular.z > 0)
            th = .2*PI;
        else if (tw->angular.z < 0)
            th = -.2*PI;

    //! Limit -.5*PI till .5*PI
    th = std::max( -.5*PI , std::min (th, .5*PI) );

    //! At least 1 m
    double r = std::max(1.0 , y*y+x*x );

    goal.target_point.point.x = cos(th) * r;
    goal.target_point.point.y = sin(th) * r;
    goal.target_point.point.z = 0;

    goal.goal_type = head_ref::HeadReferenceGoal::LOOKAT;

    goal.priority = 5;

    goal.pan_vel = .2;
    goal.tilt_vel = .2;

    goal.end_time = ros::Time::now().toSec() + 0.2;

    // Send goal
    ac->sendGoal(goal);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "look_around");

    ros::NodeHandle n("~");

    n.param("priority", priority, 10);
    n.param("pan_vel", pan_vel, .5);
    n.param("tilt_vel", tilt_vel, .5);
    n.param("frame", frame, std::string("/base_link"));

    ROS_INFO("Cmd vel client on frame '%s' initialized on priority %d with velocity (%3f,%3f)", frame.c_str(), priority, pan_vel, tilt_vel);

    ros::Subscriber s = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cmdVelCallback);

    ac = new HeadReferenceActionClient("head_ref/action_server");

    ros::spin();

    return 0;
}
