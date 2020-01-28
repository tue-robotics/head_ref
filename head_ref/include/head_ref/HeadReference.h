#ifndef HEAD_REF_HEAD_REFERENCES_H_
#define HEAD_REF_HEAD_REFERENCES_H_

#define PI 3.14159265

// ros
#include <ros/ros.h>

// Messages
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

// Actionlib
#include <actionlib/server/action_server.h>
#include <head_ref_msgs/HeadReferenceAction.h>

// tf
#include <tf/transform_listener.h>

typedef actionlib::ActionServer<head_ref_msgs::HeadReferenceAction> HeadReferenceActionServer;

/**
 * @brief The GoalInfo struct contains all info concerning a specific goal
 */
struct GoalInfo
{
  HeadReferenceActionServer::GoalHandle goal_handle;
//  ros::Time::stamp at_setpoint_stamp;
};


class HeadReference
{

    public:
        HeadReference();
        ~HeadReference();
        void generateReferences();

    private:
        void goalCallback(HeadReferenceActionServer::GoalHandle gh);
        void cancelCallback(HeadReferenceActionServer::GoalHandle gh);

        void measurementCallBack(const sensor_msgs::JointState& msg);

        void abortGoalWithSamePriority(unsigned int priority);

        bool targetToPanTilt(const tf::Stamped<tf::Point>& target, double& pan, double& tilt);
        void publishMarker(const tf::Stamped<tf::Point>& target);

        void checkTimeOuts();

        HeadReferenceActionServer* as_;
        std::vector<GoalInfo> goal_info_;

        bool float_topics_;
        ros::Publisher head_pub_, pan_pub_, tilt_pub_, marker_pub_;
        ros::Subscriber measurement_sub_;

        tf::TransformListener* tf_listener_;

        double current_pan_, current_tilt_, goal_error_tolerance_;
        
        std::string tf_prefix_;
        double default_pan_, default_tilt_;

        head_ref_msgs::HeadReferenceGoal lookat_and_freeze_goal_;
};

#endif /* HEAD_REF_HEAD_REFERENCES_H_ */
