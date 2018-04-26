#include "head_ref/HeadReference.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "head_ref_action_server");
    ros::NodeHandle nh;

    ros::Rate r(25);

    HeadReference hr;

	while (nh.ok()) {
		ros::spinOnce();
//        hr.generateReferences();
		r.sleep();
	}

    return 0;
}

