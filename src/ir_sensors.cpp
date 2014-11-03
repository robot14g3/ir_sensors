#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "ir_sensors");
    ros::NodeHandle nh;

    ros::Publisher point_pub = nh.advertise<geometry_msgs::Pose>("measured_point", 10);
    ros::Subscriber pose_sub = nh.subscribe();

    tf::TransformListener listener;
    ros::Rate loop_rate(10.0);

    while (nh.ok()) {

        tf::StampedTransform transform;
        try {
            listener.lookupTransform("ir_sensors", "base_link", ros::Time(0), transform);

        }

        loop_rate.sleep();
    }

    return 0;
}
