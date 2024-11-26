#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_tf_broadcaster");
    ros::NodeHandle nh;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    ros::Rate rate(10);  // 10 Hz

    while (ros::ok()) {
        // Parent frame: map or world
        transformStamped.header.frame_id = "map";
        // Child frame: camera_link
        transformStamped.child_frame_id = "camera_link";
        transformStamped.header.stamp = ros::Time::now();

        // Set position (x, y, z)
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        // Set rotation (pitch = -90 degrees)
        tf2::Quaternion q;
        q.setRPY(0, M_PI_2, 0);  // Roll, Pitch, Yaw in radians
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // Publish the transform
        br.sendTransform(transformStamped);

        rate.sleep();
    }

    return 0;
}