#include "actionlib/client/action_client.h"
#include "head_ref/HeadReferenceAction.h"

typedef actionlib::ActionClient<head_ref::HeadReferenceAction> HeadReferenceActionClient;

#define PI 3.14159265

HeadReferenceActionClient* ac;

int priority;
double duration, pan_vel, tilt_vel;
std::vector<float> pans, tilts;
std::vector<float> xs, ys, zs;
std::string frame_id;
unsigned int state = 0;

void panTiltCallback(const ros::TimerEvent& e)
{
    // send a goal to the action
    head_ref::HeadReferenceGoal goal;

    goal.goal_type = head_ref::HeadReferenceGoal::PAN_TILT;
    goal.priority = priority;
    goal.pan_vel = pan_vel;
    goal.tilt_vel = tilt_vel;
    goal.pan = pans[state];
    goal.tilt = tilts[state];

    HeadReferenceActionClient::GoalHandle gh = ac->sendGoal(goal);

    if (state + 1 == pans.size())
        state = 0;
    else
        state+=1;
}

void lookAtCallback(const ros::TimerEvent& e)
{
    // send a goal to the action
    head_ref::HeadReferenceGoal goal;
    geometry_msgs::PointStamped goal_pt;

    goal_pt.point.x = xs[state];
    goal_pt.point.y = ys[state];
    goal_pt.point.z = zs[state];
    goal_pt.header.frame_id = frame_id;

    goal.goal_type = head_ref::HeadReferenceGoal::LOOKAT;
    goal.priority = priority;
    goal.pan_vel = pan_vel;
    goal.tilt_vel = tilt_vel;
    goal.target_point = goal_pt;

    HeadReferenceActionClient::GoalHandle gh = ac->sendGoal(goal);
    std::cout << "Sent goal to (" << goal_pt.point.x << "," << goal_pt.point.y << "," << goal_pt.point.z << ")" << std::endl;

    if (state + 1 == xs.size())
        state = 0;
    else
        state+=1;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "look_around");

    ros::NodeHandle n("~");

    n.param("duration", duration, 3.0);
    n.param("priority", priority, 10);
    n.param("pan_vel", pan_vel, .5);
    n.param("tilt_vel", tilt_vel, .5);

    ROS_INFO("Look around client initialized on priority 10 with velocity (%3f,%3f)", pan_vel, tilt_vel);

    if (n.getParam("states/x", xs) && n.getParam("states/y", ys) && n.getParam("states/z", zs) && xs.size() == ys.size() && xs.size() == zs.size() && xs.size() > 0)
    {
        if ( n.getParam("frame_id", frame_id) )
        {
            ros::Timer timer = n.createTimer(ros::Duration(duration), lookAtCallback);

            ac = new HeadReferenceActionClient("head_ref/action_server");

            ros::spin();
        }
        else
        {
            ROS_ERROR("Please specify the frame_id for the head_look_around_client");
        }
    }
    else if (n.getParam("states/pan", pans) && n.getParam("states/tilt", tilts) && pans.size() == tilts.size() && pans.size() > 0)
    {
        ros::Timer timer = n.createTimer(ros::Duration(duration), panTiltCallback);

        ac = new HeadReferenceActionClient("head_ref/action_server");

        ros::spin();
    }
    else
    {
        ROS_ERROR("Please specify states/pan and states/tilt of the same length.");
    }
    return 0;
}
