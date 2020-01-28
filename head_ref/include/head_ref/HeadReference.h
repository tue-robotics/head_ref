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
  ros::Time at_setpoint_stamp;
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

        /**
         * @brief atSetpoint Checks if this goal has been at its setpoint sufficiently long
         * @param goal_info contains goal handle and stamp at which the goal was last at its setpoint
         * @param currently_at_setpoint whether it currently is at setpoint
         * @return at setpoing long enough
         */
        bool atSetpoint(GoalInfo &goal_info, bool currently_at_setpoint);

        HeadReferenceActionServer* as_;
        std::vector<GoalInfo> goal_info_;

        bool float_topics_;
        ros::Publisher head_pub_, pan_pub_, tilt_pub_, marker_pub_;
        ros::Subscriber measurement_sub_;

        tf::TransformListener* tf_listener_;

        double current_pan_, current_tilt_, goal_error_tolerance_;
        
        std::string tf_prefix_;
        double default_pan_, default_tilt_;

        /**
         * @brief at_setpoint_delay_ The neck should be at least this amount of seconds at setpoint before
         * the 'at_setpoint' field of the feedback message is set to True.
         *
         * This is useful, e.g., if there are vibrations between the actuator and the head: this might
         * cause the actuators to report that they're at setpoint while the head (=what matters) is still
         * shaking, indirectly resulting in blurry camera images.
         */
        double at_setpoint_delay_;

        head_ref_msgs::HeadReferenceGoal lookat_and_freeze_goal_;
};

#endif /* HEAD_REF_HEAD_REFERENCES_H_ */
