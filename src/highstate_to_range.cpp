#include "../include/highstate_to_range.h"

void state_callback(const unitree_legged_msgs::HighState& state){
    for (int i=0; i<3; ++i){
        msg.header.frame_id = "go1";
        msg.range = state.rangeObstacle[i];
        pubs[i].publish(msg);
    }
}

int main(int argc, char** argv){
    printf("\n\n\n[i] started highstate_to_range\n");
    ros::init(argc, argv, "high2range");
    ros::NodeHandle nh;

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