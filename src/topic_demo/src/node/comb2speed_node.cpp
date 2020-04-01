#include <comb2speed.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "comb2speed_node");
    zev::Comb2Speed mNode_class;
    ros::spin();
    return 0;
}