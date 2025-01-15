// src/example.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>       // std::cout
#include <typeinfo>       // operator typeid

#include <subpub/obsts.h>
//#include <pubsub/obst.h>

#include <vector>

// #include "pubsub.hpp"

ros::Publisher pub;
ros::Publisher pub2;

typedef pcl::PointXYZ              PointXYZ;

float   pointDist(pcl::PointXYZ &point){
    return (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)));
}

pcl::PointCloud<pcl::PointXYZ> ROI(pcl::PointCloud<pcl::PointXYZ> &pc){
    pcl::PointCloud<pcl::PointXYZ> roi;

    for (auto& point : pc.points){
        if (point.x> 0 && point.x < 10 && point.y > -10
            && point.y < 10 && pointDist(point) > 2
            && point.z < 2)
            roi.points.push_back(point);
    }
    return roi;
}

void    sort_PointCloud(pcl::PointCloud<pcl::PointXYZ> &PC){
    if (PC.points.size() <= 1)
        return ;
    for (int i = 0; i < PC.points.size() - 1; ++i){
        for (int j = 0; j < PC.points.size() - i - 1; ++j){
            if (pointDist(PC.points[j]) > pointDist(PC.points[j + 1]))
            {
                pcl::PointXYZ tmp = PC.points[j];
                PC.points[j] = PC.points[j + 1];
                PC.points[j + 1] = tmp;
            }
        }
    }
}

<<<<<<< HEAD:src/subpub/src/subpub.cpp
void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "No inliers found!" << std::endl;
        return;
    }

    // Now copy the inliers from input_cloud directly
    pcl::copyPointCloud<pcl::PointXYZ>(*input_cloud, *inliers, *inlierPoints);

    // Extract the inliers from the input cloud and store them in a new cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Remove the inliers from the input cloud
    extract.filter(*input_cloud);  // Update the input_cloud with the remaining points
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DownSampling(pcl::PointCloud<pcl::PointXYZ> &pc){

    pcl::PointCloud<pcl::PointXYZ>::Ptr outPC(new pcl::PointCloud<pcl::PointXYZ>());
=======
pcl::PointCloud<pcl::PointXYZ> DownSampling(pcl::PointCloud<pcl::PointXYZ> &pc){

    pcl::PointCloud<pcl::PointXYZ> outPC;
>>>>>>> parent of ffbcf80... Update pubsub.cpp:pubsub/src/pubsub.cpp

    subpub::obsts msg;
    std::vector<subpub::obst> point_elem;
    int count = 0;

    for (int i = 0; i < pc.points.size(); i ++){
        pc.points[i].x = static_cast<int>(round(pc.points[i].x));
        pc.points[i].y = static_cast<int>(round(pc.points[i].y));
        pc.points[i].z = static_cast<int>(round(pc.points[i].z));

        
        // if (pc.points[i].x < 5 && pc.points[i].x > -5
        //     && pc.points[i].y < 5 && pc.points[i].y > -5
        //     && pc.points[i].z > -5 && pc.points[i].z < 5)
        if(true)
        {
            bool point_exists = false;
            for (auto& point : outPC.points) {
                if (point.x == pc.points[i].x && point.y == pc.points[i].y && point.z == pc.points[i].z) {
                    point_exists = true;
                    break;
                }
            }
            if (!point_exists) {
                subpub::obst pt;
                pt.x = pc.points[i].x;
                pt.y = pc.points[i].y;
                pt.z = pc.points[i].z;
                
                outPC.points.push_back(pc.points[i]);
                point_elem.push_back(pt);
                std::cout<< "x: "<< pc.points[i].x << " y: " << pc.points[i].y << " z: " << pc.points[i].z << std::endl;
                count ++;
            }
        }
    }
    msg.obst_vector = point_elem;
    msg.id = count;
    pub2.publish(msg);
        std::cout << "==============================" << std::endl;

    return outPC;
}

void    cloud_cb (const sensor_msgs::PointCloud2 msg){

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(msg, pcl_pc);
    pcl::PointCloud<PointXYZ> input_cloud;
    pcl::fromPCLPointCloud2(pcl_pc, input_cloud);

    pcl::PointCloud<PointXYZ> roi_cloud;
    roi_cloud = ROI(input_cloud);

    sort_PointCloud(roi_cloud);
<<<<<<< HEAD:src/subpub/src/subpub.cpp
    pcl::PointCloud<PointXYZ>::Ptr tmp = DownSampling(roi_cloud);
    ransac(tmp);

=======
    pcl::PointCloud<PointXYZ> tmp = DownSampling(roi_cloud);
>>>>>>> parent of ffbcf80... Update pubsub.cpp:pubsub/src/pubsub.cpp
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg((*tmp), output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

int main(int argc, char **argv){
    ros::init (argc, argv, "pubsub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/lidar3D", 1, cloud_cb);
    
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    pub2 = nh.advertise<subpub::obsts> ("custom_msg", 1);
    ros::spin();
<<<<<<< HEAD:src/subpub/src/subpub.cpp
    return 0;
}
=======
}
>>>>>>> parent of ffbcf80... Update pubsub.cpp:pubsub/src/pubsub.cpp
