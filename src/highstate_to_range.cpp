#include "../include/highstate_to_range.h"

void state_callback(const unitree_legged_msgs::HighState& state){
    for (int i=0; i<3; ++i){
        msg.header.frame_id = "go1/" + (std::string) sensors[i];
        msg.range = state.rangeObstacle[i];
        pubs[i].publish(msg);
    }
}

int main(int argc, char** argv){
    printf("\n\n\n[i] started highstate_to_range\n");
    ros::init(argc, argv, "high2range");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "go1";
    transformStamped.child_frame_id = "go1_front";
    transformStamped.transform.translation.x = 0.30;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);

    transformStamped.header.frame_id = "go1";
    transformStamped.child_frame_id = "go1_left";
    transformStamped.transform.translation.x = -0.06;
    transformStamped.transform.translation.y = 0.10;
    transformStamped.transform.translation.z = 0.0;
    q.setRPY(0, 0, 1.5708f);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);

    transformStamped.header.frame_id = "go1";
    transformStamped.child_frame_id = "go1_right";
    transformStamped.transform.translation.x = -0.06;
    transformStamped.transform.translation.y = -0.10;
    transformStamped.transform.translation.z = 0.0;
    q.setRPY(0, 0, -1.5708f);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);


    msg.radiation_type = 0;
    if (argc==4) {
        msg.field_of_view = atof(argv[0]);
        msg.min_range = atof(argv[1]);
        msg.max_range = atof(argv[2]);
    }else{
        msg.field_of_view = 1.2f;
        msg.min_range = 0.1f;
        msg.max_range = 1.5f;
    }

    for (int i = 0; i < 3; ++i) {
        pubs[i] = nh.advertise<sensor_msgs::Range>("/range/" + (std::string) sensors[i], 1000);
    }

    sub = nh.subscribe("/high_state", 1, state_callback);
    ros::spin();

    printf("[i] ended highstate_to_range\n");
}