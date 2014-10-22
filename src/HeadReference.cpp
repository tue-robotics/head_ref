#include "head_ref/HeadReference.h"

bool compByPriority(HeadReferenceActionServer::GoalHandle a, HeadReferenceActionServer::GoalHandle b) {
    return a.getGoal()->priority < b.getGoal()->priority;
}

void getResetPositionGoal(head_ref::HeadReferenceGoal& goal)
{
    goal.goal_type = head_ref::HeadReferenceGoal::PAN_TILT;
    goal.pan = 0;
    goal.tilt = 0.1;
}

HeadReference::HeadReference() :
    current_pan_(0),
    current_tilt_(0),
    goal_error_tolerance_(0.05)
{
    ros::NodeHandle nh;

    tf_listener_ = new tf::TransformListener(ros::Duration(10.0));

    // ROS publishers
    head_pub_ = nh.advertise<sensor_msgs::JointState>("/amigo/neck/references", 50);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("head_target_marker", 1);

    // ROS subscribers
    measurement_sub_ = nh.subscribe("/amigo/neck/measurements", 1, &HeadReference::measurementCallBack, this);

    // Setup action server
    as_ = new HeadReferenceActionServer(nh,"head_reference",false);
    as_->registerGoalCallback(boost::bind(&HeadReference::goalCallback, this, _1));
    as_->registerCancelCallback(boost::bind(&HeadReference::cancelCallback, this, _1));

    // Start action server
    as_->start();
}

HeadReference::~HeadReference()
{

}

void HeadReference::measurementCallBack(const sensor_msgs::JointState& msg) {
    for(unsigned int i = 0; i < msg.name.size(); ++i) {
        if (msg.name[i] == "neck_pan_joint") {
            current_pan_ = msg.position[i];
        } else if  (msg.name[i] == "neck_tilt_joint") {
            current_tilt_ = msg.position[i];
        }
    }
}

void HeadReference::goalCallback(HeadReferenceActionServer::GoalHandle gh)
{
    ROS_DEBUG_STREAM("HR: Goal Callback of priority " << (int) gh.getGoal()->priority);

    // abort goal with same priority
    abortGoalWithSamePriority(gh.getGoal()->priority);

    // Accept the goal
    gh.setAccepted();

    // Push back goal handle
    goal_handles_.push_back(gh);

    // Sort goal handles on priority
    std::sort(goal_handles_.begin(),goal_handles_.end(),compByPriority);
}

void HeadReference::abortGoalWithSamePriority(unsigned int priority)
{
    std::vector<HeadReferenceActionServer::GoalHandle>::iterator it = goal_handles_.begin();
    for(; it != goal_handles_.end(); ++it) {
        if (it->getGoal()->priority == priority) {
            head_ref::HeadReferenceResult result;
            result.error = "Client with same priority registered.";
            ROS_DEBUG_STREAM("HR: Client with same priority " << priority << " registered. Aborting old client.");
            it->setAborted(result);
            goal_handles_.erase(it);
            return;
        }
    }
}

void HeadReference::checkTimeOuts()
{
    for(std::vector<HeadReferenceActionServer::GoalHandle>::iterator it = goal_handles_.begin(); it != goal_handles_.end();)
    {
        double end_time = it->getGoal()->end_time;

        if (end_time > 0 && ros::Time::now().toSec() > end_time)
        {
            head_ref::HeadReferenceResult result;
            result.error = "TimeOut exceeded!";
            it->setAborted(result);
            it = goal_handles_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void HeadReference::cancelCallback(HeadReferenceActionServer::GoalHandle gh)
{
    ROS_DEBUG_STREAM("HR: Cancel callback with priority " << (int) gh.getGoal()->priority);

    // Find the goalhandle in the goal_handles_ vector
    std::vector<HeadReferenceActionServer::GoalHandle>::iterator it = std::find(goal_handles_.begin(), goal_handles_.end(), gh);

    // Check if element exist (just for safety) and erase the element
    if (it != goal_handles_.end()) { goal_handles_.erase(it); }
}

void HeadReference::generateReferences()
{
    checkTimeOuts();

    head_ref::HeadReferenceGoal goal;

    if (goal_handles_.size() > 0)
    {
        // Take the highest priority goal
        HeadReferenceActionServer::GoalHandle gh = goal_handles_.front();
        goal = *gh.getGoal();

        if (goal.goal_type == head_ref::HeadReferenceGoal::LOOKAT) {
            // Update pan and tilt
            tf::Stamped<tf::Point> tp;
            tf::pointStampedMsgToTF(goal.target_point,tp);
            tp.stamp_ = ros::Time();
            targetToPanTilt(tp, goal.pan, goal.tilt);
            publishMarker(tp);
        }

        // Check whether we are there
        head_ref::HeadReferenceFeedback fb;
        if (fabs(goal.pan - current_pan_) < goal_error_tolerance_ && fabs(goal.tilt - current_tilt_) < goal_error_tolerance_) {

            fb.at_setpoint = true;
            gh.publishFeedback(fb);

            return;
        }
    }
    else
    {
        getResetPositionGoal(goal);
    }

    ROS_DEBUG("Current head goal (pan/tilt): %.3f,%.3f",goal.pan,goal.tilt);

    // populate msg
    sensor_msgs::JointState head_ref;
    head_ref.name.push_back("neck_pan_joint");
    head_ref.name.push_back("neck_tilt_joint");

    head_ref.position.push_back(goal.pan);
    head_ref.position.push_back(goal.tilt);
    head_ref.velocity.push_back(goal.pan_vel);
    head_ref.velocity.push_back(goal.tilt_vel);

    //publish angles over ROS
    head_pub_.publish(head_ref);
}

bool HeadReference::targetToPanTilt(const tf::Stamped<tf::Point>& target, double& pan, double& tilt)
{
    tf::Stamped<tf::Point> target_HEAD_MOUNT;
    try {
        tf_listener_->transformPoint("/amigo/head_mount", target, target_HEAD_MOUNT);
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    tf::Stamped<tf::Point> target_NECK_TILT;
    try {
        tf_listener_->transformPoint("/amigo/neck_tilt", target, target_NECK_TILT);
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    double head_mount_to_neck;
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform("/amigo/head_mount", "/amigo/neck_tilt", ros::Time(), transform);
        head_mount_to_neck = transform.getOrigin().getX();
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    double neck_to_cam_vert;
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform("/amigo/neck_tilt", "/amigo/top_kinect/openni_camera", ros::Time(), transform);
        neck_to_cam_vert = transform.getOrigin().getZ();
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    pan = -atan2(target_HEAD_MOUNT.getY(), target_HEAD_MOUNT.getZ());

    double neck_to_target = target_NECK_TILT.length();

    double target_to_neck_vert = target_HEAD_MOUNT.getX() - head_mount_to_neck;

    double head_mount_to_target_flat = sqrt(target_HEAD_MOUNT.getY() * target_HEAD_MOUNT.getY() + target_HEAD_MOUNT.getZ() * target_HEAD_MOUNT.getZ());

    double tilt_basic = -atan(target_to_neck_vert / head_mount_to_target_flat);
    double tilt_camera_offset_correction =  asin(neck_to_cam_vert / neck_to_target);

    tilt = tilt_basic + tilt_camera_offset_correction;

    return true;
}

void HeadReference::publishMarker(const tf::Stamped<tf::Point>& target) {

    //create marker object
    visualization_msgs::Marker marker;

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    // Set the frame ID and timestamp.
    marker.header.frame_id = target.frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.ns = "head_target";
    marker.id = 0;

    // Set the marker type.
    marker.type = shape;

    // Set the marker action.
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.
    marker.pose.position.x = target.getX();
    marker.pose.position.y = target.getY();
    marker.pose.position.z = target.getZ();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.6;

    // Publish the marker
    marker_pub_.publish(marker);

}





