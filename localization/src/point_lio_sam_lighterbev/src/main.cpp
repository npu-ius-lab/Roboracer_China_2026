#include "point_lio_sam_lighterbev.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_lio_sam_lighterbev_node");
    ros::NodeHandle nh_private("~");

    PointLioSamLighterBev point_lio_slam(nh_private);

    ros::AsyncSpinner spinner(8); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
