#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
    
int 
main (int argc, char **argv)
{
    ros::init(argc, argv, "test_playground_node");
    ROS_INFO("Program starts\n");
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/wu/Documents/biorobotics/recording/pcds/table_scene_mug_stereo_textured.pcd", *cloud);
    
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);


    // // Create the normal estimation class, and pass the input dataset to it
    // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    // ne.setInputCloud (cloud);

    // // Create an empty kdtree representation, and pass it to the normal estimation object.
    // // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    // ne.setSearchMethod (tree);

    // // Output datasets
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // // Use all neighbors in a sphere of radius 3cm
    // ne.setRadiusSearch (0.03);

    // // Compute the features
    // ne.compute (*cloud_normals);
    // std::cout<<cloud->size()<<std::endl;
    // std::cout<<cloud_normals->size()<<std::endl;
    
    ROS_INFO("Computation finished!");
    pcl::io::savePCDFileASCII("/home/wu/Documents/biorobotics/recording/pcds/binaryIntegral.pcd",*cloud);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);


   // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloud(cloud,"cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals,100,0.02,"cloudNormal",0);
    
    // pcl::visualization::PCLVisualizer viewer2("PCL Viewer2");
    // viewer.setBackgroundColor (0.0, 0.0, 0.5);
    // // viewer.addPointCloud(cloud);
    // viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud2, cloud_normals);

    while (!viewer.wasStopped ())
    {      
        viewer.spinOnce ();
        // viewer2.spinOnce();
    }
    return 0;
}