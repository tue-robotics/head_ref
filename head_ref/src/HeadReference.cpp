#include "head_ref/HeadReference.h"


bool compByPriority(GoalInfo a, GoalInfo b)
{
  return a.goal_handle.getGoal()->priority < b.goal_handle.getGoal()->priority;
}


HeadReference::HeadReference() :
    current_pan_(0),
    current_tilt_(0),
    goal_error_tolerance_(0.05)
{
    ros::NodeHandle nh("~");
    ros::NodeHandle gh;

    tf_listener_ = new tf::TransformListener(ros::Duration(10.0));

    // Setup action server
    as_ = new HeadReferenceActionServer(nh,"action_server",false);
    as_->registerGoalCallback(boost::bind(&HeadReference::goalCallback, this, _1));
    as_->registerCancelCallback(boost::bind(&HeadReference::cancelCallback, this, _1));

    // Start action server
    as_->start();
    
    // Get tf prefix
    ros::NodeHandle n("~");
    n.param<std::string>("tf_prefix", tf_prefix_, "");
    n.param<double>("default_pan", default_pan_, 0);
    n.param<double>("default_tilt", default_tilt_, 0);
    n.param<double>("delay", at_setpoint_delay_, 0.0);
    n.param<bool>("float_topics", float_topics_, false);
    tf_prefix_ = "/" + tf_prefix_;

    // ROS publishers
    if ( float_topics_ )
    {
        pan_pub_ = gh.advertise<std_msgs::Float64>("pan_controller/command", 1);
        tilt_pub_ = gh.advertise<std_msgs::Float64>("tilt_controller/command", 1);
    }
    else
    {
        head_pub_ = gh.advertise<sensor_msgs::JointState>("neck/references", 50);
    }
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("head_target_marker", 1);

    // ROS subscribers
    measurement_sub_ = gh.subscribe("neck/measurements", 5, &HeadReference::measurementCallBack, this);
    // measurements @ 100 Hz will need a queue of 100/25=4

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

    // ROBOCUP HACK
    if (gh.getGoal()->goal_type == head_ref_msgs::HeadReferenceGoal::LOOKAT_AND_FREEZE)
    {
        // Update pan and tilt
        tf::Stamped<tf::Point> tp;
        tf::pointStampedMsgToTF(gh.getGoal()->target_point,tp);
        tp.stamp_ = ros::Time();
        targetToPanTilt(tp, lookat_and_freeze_goal_.pan, lookat_and_freeze_goal_.tilt);
    }

    // abort goal with same priority
    abortGoalWithSamePriority(gh.getGoal()->priority);

    // Accept the goal
    gh.setAccepted();

    // Push back goal handle
    GoalInfo goal_info{gh, ros::Time(0)};
    goal_info_.push_back(goal_info);

    // Sort goal handles on priority
    std::sort(goal_info_.begin(), goal_info_.end(), compByPriority);
}


void HeadReference::abortGoalWithSamePriority(unsigned int priority)
{
    auto it = goal_info_.begin();
    for(; it != goal_info_.end(); ++it) {
      HeadReferenceActionServer::GoalHandle gh = it->goal_handle;
      if (gh.getGoal()->priority == priority) {
            head_ref_msgs::HeadReferenceResult result;
            result.error = "Client with same priority registered.";
            ROS_DEBUG_STREAM("HR: Client with same priority " << priority << " registered. Aborting old client.");
            gh.setAborted(result);
            goal_info_.erase(it);
            return;
        }
    }
}


void HeadReference::checkTimeOuts()
{
    for(auto it = goal_info_.begin(); it != goal_info_.end();)
    {
        HeadReferenceActionServer::GoalHandle gh = it->goal_handle;
        double end_time = gh.getGoal()->end_time;

        if (end_time > 0 && ros::Time::now().toSec() > end_time)
        {
            head_ref_msgs::HeadReferenceResult result;
            result.error = "TimeOut exceeded!";
            gh.setAborted(result);
            it = goal_info_.erase(it);
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

    for (auto it = goal_info_.begin(); it != goal_info_.end(); ++it)
    {
      if (it->goal_handle == gh)
      {
        goal_info_.erase(it);
        return;
      }
    }
    // If we end up here, something was wrong
    ROS_WARN_STREAM("Goal with ID " << gh.getGoalID() << " was not in the list, could not cancel");
}


void HeadReference::generateReferences()
{
    checkTimeOuts();

    head_ref_msgs::HeadReferenceGoal goal;

    if (goal_info_.size() > 0)
    {
        // Take the highest priority goal
        GoalInfo goal_info = goal_info_.front();  // This returns a *reference* to a goal info struct
        HeadReferenceActionServer::GoalHandle gh = goal_info.goal_handle;
        goal = *gh.getGoal();

        if (goal.goal_type == head_ref_msgs::HeadReferenceGoal::LOOKAT || goal.goal_type == head_ref_msgs::HeadReferenceGoal::LOOKAT_AND_FREEZE) {
            // Update pan and tilt
            tf::Stamped<tf::Point> tp;
            tf::pointStampedMsgToTF(goal.target_point,tp);
            tp.stamp_ = ros::Time();
            if (!targetToPanTilt(tp, goal.pan, goal.tilt))
            {
              gh.setAborted();
              goal_info_.erase (goal_info_.begin(), goal_info_.begin()+1);
              return;
            }
            publishMarker(tp);
        }

        // Check whether we are there
        head_ref_msgs::HeadReferenceFeedback fb;
        if (fabs(goal.pan - current_pan_) < goal_error_tolerance_ && fabs(goal.tilt - current_tilt_) < goal_error_tolerance_)
        {

            fb.at_setpoint = atSetpoint(goal_info, true);
            gh.publishFeedback(fb);
            return;
        }
        else
        {
            fb.at_setpoint = atSetpoint(goal_info, false);
            gh.publishFeedback(fb);
        }
    }
    else
    {
        goal.goal_type = head_ref_msgs::HeadReferenceGoal::PAN_TILT;
        goal.pan = default_pan_;
        goal.tilt = default_tilt_;
    }

    ROS_DEBUG("Current head goal (pan/tilt): %.3f,%.3f",goal.pan,goal.tilt);

    // ROBOCUP HACK
    if (goal.goal_type == head_ref_msgs::HeadReferenceGoal::LOOKAT_AND_FREEZE)
    {
        goal = lookat_and_freeze_goal_;
        goal.goal_type = head_ref_msgs::HeadReferenceGoal::PAN_TILT;
        goal.tilt_vel = 1.0;
        goal.pan_vel = 1.0;
    }

    //publish angles over ROS
    if ( float_topics_ )
    {
        // populate msgs
        std_msgs::Float64 pan_ref, tilt_ref;
        pan_ref.data = goal.pan;
        tilt_ref.data = goal.tilt;

        pan_pub_.publish(pan_ref);
        tilt_pub_.publish(tilt_ref);
    }
    else
    {
        // populate msg
        sensor_msgs::JointState head_ref;
        head_ref.name.push_back("neck_pan_joint");
        head_ref.name.push_back("neck_tilt_joint");

        head_ref.position.push_back(goal.pan);
        head_ref.position.push_back(goal.tilt);
        head_ref.velocity.push_back(goal.pan_vel);
        head_ref.velocity.push_back(goal.tilt_vel);

        head_pub_.publish(head_ref);
    }
}


bool HeadReference::targetToPanTilt(const tf::Stamped<tf::Point>& target, double& pan, double& tilt)
{
    tf::Stamped<tf::Point> target_HEAD_MOUNT;
    try {
        tf_listener_->transformPoint(tf_prefix_+"/head_mount", target, target_HEAD_MOUNT);
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    tf::Stamped<tf::Point> target_NECK_TILT;
    try {
        tf_listener_->transformPoint(tf_prefix_+"/neck_tilt", target, target_NECK_TILT);
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    double head_mount_to_neck;
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform(tf_prefix_+"/head_mount", tf_prefix_+"/neck_tilt", ros::Time(), transform);
        head_mount_to_neck = transform.getOrigin().getX();
    } catch(tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    double neck_to_cam_vert;
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform(tf_prefix_+"/neck_tilt", tf_prefix_+"/top_kinect/openni_camera", ros::Time(), transform);
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


bool HeadReference::atSetpoint(GoalInfo &goal_info, bool currently_at_setpoint)
{
  // If currently not at the setpoint, reset the stamp and return false
  if (!currently_at_setpoint)
  {
    goal_info.at_setpoint_stamp = ros::Time(0);
    return false;
  }

  // If at setpoint, check if it is already set
  ros::Time current_stamp = ros::Time::now();
  if (goal_info.at_setpoint_stamp == ros::Time(0))
  {
    goal_info.at_setpoint_stamp = current_stamp;
    return false;
  }

  // If it was already set, check if this was long enough
  // ToDo: does this result in jittery behavior?
  if ((current_stamp - goal_info.at_setpoint_stamp).toSec() > at_setpoint_delay_)
  {
    return true;
  }
  return false;

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
