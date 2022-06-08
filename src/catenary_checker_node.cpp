#include <vector>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <catenary_checker/catenary_checker.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


//! Global data
sensor_msgs::PointCloud2 pc;
tf2_ros::Buffer tf_buffer;
std::string base_frame, global_frame;
float plane_dist = 0.5;

ros::Publisher pc_publisher;
bool publish_pc = true;

void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr &pc_msg) {
    pc = *pc_msg;
}

//! Gets a point and checks if there exists
void checkCatenary(const geometry_msgs::PoseStamped &target_pose) {
    // Get the pose of the robot or die
    geometry_msgs::TransformStamped transformStamped;
    try{
         // get the location of the robot
         transformStamped = tf_buffer.lookupTransform(base_frame, global_frame,
                                  ros::Time(0));
         pcl::PointXYZ robot(static_cast<float>(transformStamped.transform.translation.x), 
                             static_cast<float>(transformStamped.transform.translation.y),
                             static_cast<float>(transformStamped.transform.translation.z));
         
         pcl::PointXYZ target(static_cast<float>(target_pose.pose.position.x),
                              static_cast<float>(target_pose.pose.position.y),
                              static_cast<float>(target_pose.pose.position.z));
         pcl::PointCloud<pcl::PointXYZ> pcl_pc;
         pcl::PCLPointCloud2 pcl_pc2;
         pcl_conversions::moveToPCL(pc, pcl_pc2);
         pcl::fromPCLPointCloud2(pcl_pc2, pcl_pc); // TODO: Avoid conversions!!

         auto points_2d = project2D(pcl_pc, robot, target, plane_dist);

         if (publish_pc) {

            auto points_3d = reproject_3D(points_2d, robot, target);

            pcl::toPCLPointCloud2(pcl_pc, pcl_pc2);
         
            sensor_msgs::PointCloud2 out_pc2;
            pcl_conversions::moveFromPCL(pcl_pc2, out_pc2);

            pc_publisher.publish(pcl_pc2);
         }
    }
    catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
    }


}

int main (int argc, char **argv) {
    // --- Inicializacion de ROS. No hace falta tocar
    std::string node_name = "catenary_checker";
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh, lnh("~");

    tf2_ros::TransformListener tfl(tf_buffer);

    //Subscribe the node to the point cloud from the ROS bag file. The topic has to be remapped to points2
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("points2", 1, pointCloudCb);

    base_frame = lnh.param<std::string>("base_frame_id", std::string("base_link"));
    global_frame = lnh.param<std::string>("global_frame_id", std::string("map"));
    plane_dist = lnh.param<float>("plane_dist", 0.5f);
    publish_pc = lnh.param<bool>("publish_pc", true);

    if (publish_pc) {
        pc_publisher = nh.advertise<sensor_msgs::PointCloud2>("plane_pc", 2);
    }

    ros::spin();

    return 0;
}