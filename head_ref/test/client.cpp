#include "head_ref/HeadReference.h"
#include "actionlib/client/action_client.h"

typedef actionlib::ActionClient<head_ref_msgs::HeadReferenceAction> HeadReferenceActionClient;

int main(int argc, char** argv){

    if (argc < 2) {
        std::cerr << "Usage: rosrun head_ref test_client DURATION NODE_NAME FRAME X Y Z PAN TILT TYPE [0=pan/tilt,1=lookat] PRIORITY" << std::endl;
        return 1;
    }

    int TIME = 5;
    if (argc > 1) {
        TIME = atof(argv[1]);
    }

    std::string NAME = "test_client";
    if (argc > 2) {
        NAME = argv[2];
    }

    std::string FRAME = "map";
    if (argc > 3) {
        FRAME = argv[3];
    }

    double X = 0;
    if (argc > 4) {
        X = atof(argv[4]);
    }

    double Y = 0;
    if (argc > 5) {
        Y = atof(argv[5]);
    }

    double Z = 0;
    if (argc > 6) {
        Z = atof(argv[6]);
    }

    double PAN = 0;
    if (argc > 7) {
        PAN = atof(argv[7]);
    }

    double TILT = 0;
    if (argc > 8) {
        TILT = atof(argv[8]);
    }

    bool TYPE = true;
    if (argc > 9) {
        TYPE = atof(argv[9]);
    }

    int PRIORITY = 0;
    if (argc > 10) {
        PRIORITY = atof(argv[10]);
    }

    std::cout << "NAME: " << NAME << std::endl;
    std::cout << "FRAME: " << FRAME << std::endl;
    std::cout << "(X,Y,Z): " << "(" << X << "," << Y << "," << Z << ")" << std::endl;
    std::cout << "PAN/TILT: " << PAN << ";" << TILT << std::endl;
    std::cout << "TYPE: " << TYPE << std::endl;
    std::cout << "PRIORITY: " << PRIORITY << std::endl;
    std::cout << "DURATION: " << TIME << std::endl;

    ros::init(argc, argv, NAME.c_str());

    ros::NodeHandle n;

    actionlib::ActionClient<head_ref_msgs::HeadReferenceAction> ac("HeadReference");

    // send a goal to the action
    head_ref_msgs::HeadReferenceGoal goal;
    goal.target_point.header.frame_id = FRAME;
    goal.target_point.header.stamp = ros::Time::now();
    goal.target_point.point.x = X;
    goal.target_point.point.y = Y;
    goal.target_point.point.z = Z;
    goal.pan = PAN;
    goal.tilt = TILT;
    if (TYPE) {
        goal.goal_type = head_ref_msgs::HeadReferenceGoal::LOOKAT;
    } else {
        goal.goal_type = head_ref_msgs::HeadReferenceGoal::PAN_TILT;
    }
    goal.priority = PRIORITY;

    head_ref_msgs::HeadReferenceActionGoal g;
    ros::Duration(2).sleep(); // Let the actionclient initialize

    HeadReferenceActionClient::GoalHandle gh = ac.sendGoal(goal);

    ros::Rate r(10);
    ros::Time t_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t_start).toSec() < TIME)
    {
        r.sleep();
    }

    gh.cancel();

    return 0;
}
