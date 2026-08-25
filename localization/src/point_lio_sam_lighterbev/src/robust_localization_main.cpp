#include "robust_localizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointlio_robust_localizer");
    ros::NodeHandle private_nh("~");
    try
    {
        RobustMapLocalizer localizer(private_nh);
        ros::AsyncSpinner spinner(4);
        spinner.start();
        ros::waitForShutdown();
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("Robust localization startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
