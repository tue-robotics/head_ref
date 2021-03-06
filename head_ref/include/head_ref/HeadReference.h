#ifndef HEAD_REF_HEAD_REFERENCES_H_
#define HEAD_REF_HEAD_REFERENCES_H_

// ros
#include <ros/ros.h>

// Messages
#include <sensor_msgs/JointState.h>

// Actionlib
#include <actionlib/server/action_server.h>
#include <head_ref_msgs/HeadReferenceAction.h>

// tf2
#include <tf2_ros/buffer.h>

#include <memory>

typedef actionlib::ActionServer<head_ref_msgs::HeadReferenceAction> HeadReferenceActionServer;


namespace urdf
{
  class Model;  // Forward declare urdf model class
}

namespace tf2_ros {
    class TransformListener; // Forward declare tf2_ros::TransformListener class
}

struct JointProps
{
  std::string name;
  double lower;
  double upper;
  double direction = 1.0;
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

        bool targetToPanTilt(const geometry_msgs::PointStamped& target, double& pan, double& tilt);
        void publishMarker(const geometry_msgs::PointStamped& target);

        void checkTimeOuts();

        /**
         * @brief getJointsInfo gets joint limits etc. using the urdf model on the param server for all joints
         * @return success or failure
         */
        bool getJointsInfo();

        /**
         * @brief getJointInfo updates the props of the joint
         * @param model urdf model to get the data from
         * @param props Joint info to be updated
         * @return success or failure
         */
        bool getJointInfo(const urdf::Model& model, JointProps& props);

        /**
         * @brief limitReferences Saturates a reference value
         * @param props joint properties of the relevant joint
         * @param reference value to be limited
         * @return limited value
         */
        double limitReferences(const JointProps &props, double reference);

        /**
         * @brief publishReferences saturates the references values (if applicable) and publishes these
         * @param goal goal to publish
         */
        void publishReferences(head_ref_msgs::HeadReferenceGoal& goal);

        std::unique_ptr<HeadReferenceActionServer> as_;
        std::vector<HeadReferenceActionServer::GoalHandle> goal_handles_;

        bool float_topics_;
        ros::Publisher head_pub_, pan_pub_, tilt_pub_, marker_pub_;
        ros::Subscriber measurement_sub_;

        tf2_ros::Buffer tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        double current_pan_, current_tilt_, goal_error_tolerance_;
        
        std::string tf_prefix_;
        std::string frame_mount_ = "head_mount";
        std::string frame_neck_ = "neck_tilt";
        std::string frame_head_ = "top_kinect/openni_camera";

        /// Joint properties
        JointProps pan_joint_props_, tilt_joint_props_;

        double default_pan_, default_tilt_;

        head_ref_msgs::HeadReferenceGoal lookat_and_freeze_goal_;
};

#endif /* HEAD_REF_HEAD_REFERENCES_H_ */
