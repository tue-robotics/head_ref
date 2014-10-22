#include "actionlib/client/action_client.h"
#include "head_ref/HeadReferenceAction.h"

typedef actionlib::ActionClient<head_ref::HeadReferenceAction> HeadReferenceActionClient;

#define PI 3.14159265

HeadReferenceActionClient* ac;

int priority;
double duration, pan_vel, tilt_vel;
std::vector<float> pans, tilts;
unsigned int state = 0;

void timerCallback(const ros::TimerEvent& e)
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

int main(int argc, char** argv){

    ros::init(argc, argv, "look_around");

    ros::NodeHandle n("~");

    n.param("duration", duration, 3.0);
    n.param("priority", priority, 10);
    n.param("pan_vel", pan_vel, .5);
    n.param("tilt_vel", tilt_vel, .5);

    ROS_INFO("Look around client initialized on priority 10 with velocity (%3f,%3f)", pan_vel, tilt_vel);

    if (n.getParam("states/pan", pans) && n.getParam("states/tilt", tilts) && pans.size() == tilts.size() && pans.size() > 0)
    {
        ros::Timer timer = n.createTimer(ros::Duration(duration), timerCallback);

        ac = new HeadReferenceActionClient("head_reference");

        ros::spin();
    }
    else
    {
        ROS_ERROR("Please specify states/pan and states/tilt of the same length.");
    }
    return 0;
}
