/* Adapted from Haowen Shi's voxel_grid_filter.cpp */

#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <boost/program_options.hpp>

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
#include "gripper_blaser_ros/PointCloudStitcher.h"

static std::mutex gCollectingMtx;
static std::condition_variable gCollectingCV;

static std::atomic<bool> gPublish(false);
static std::atomic<bool> gSave(false);
static std::atomic<bool> gClearAccum(false);


bool srv_callback(gripper_blaser_ros::PointCloudStitcher::Request &req,
                  gripper_blaser_ros::PointCloudStitcher::Response &res)
{
    if (req.publish)
    {
        ROS_INFO("Publishing stitched point cloud");
        gPublish = true;
    }
    if (req.save)
    {
        ROS_INFO("Saving stitched point cloud");
        gSave = true;
    }
    if (req.clear)
    {
        ROS_INFO("Clearing stitched point cloud");
        gClearAccum = true;
    }

    return true;
}

class PointCloudStitcher
{
    // ROS related
    ros::NodeHandle nh;
    ros::Subscriber pc_sub;
    ros::Publisher pc_pub;
    ros::Publisher stitched_pub;
    ros::ServiceServer service;

    std::string in_topic;
    std::string out_topic1;
    std::string out_topic2;
    std::string srv_name;

    bool verbose;
    bool debug;
    std::string directory;

    bool cropFilterOn;
    bool SORon;
    bool voxelFilterOn;

    std::vector<double> XYZpassMin;
    std::vector<double> XYZpassMax;

    float meanK;
    float stdThresh;

    float leafSize;
    

public:
    //pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
    pcl::CropBox<pcl::PointXYZ> crop_box;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> SOR;
    pcl::VoxelGrid<pcl::PointXYZ> vg_filter;

    std::mutex cloud_accum_mtx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_accum;
    tf::TransformListener listener;
    bool has_data = false;

    PointCloudStitcher() //: cloud_accum(new PointCloud())
    {
        try
        {
            nh.getParam("input_topic",in_topic);
            out_topic1 = in_topic + "/filtered";
            out_topic2 = in_topic + "/stitched";
            srv_name = in_topic + "/stitcher_srv";

            nh.getParam("verbose",verbose);
            nh.getParam("debug",debug);
            nh.getParam("directory",directory);

            nh.getParam("cropFilterOn",cropFilterOn);
            nh.getParam("SORon",SORon);
            nh.getParam("voxelFilterOn",voxelFilterOn);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
            return;
        }
        
        pc_sub = nh.subscribe(in_topic.c_str(), 1, &PointCloudStitcher::pcl_callback, this);
        pc_pub = nh.advertise<pcl::PCLPointCloud2>(out_topic1.c_str(), 1);
        stitched_pub = nh.advertise<pcl::PCLPointCloud2>(out_topic2.c_str(), 1);
        service = nh.advertiseService(srv_name.c_str(), srv_callback);

        cloud_accum = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        try
        {
            if(cropFilterOn){
                nh.getParam("XYZpassMin",XYZpassMin);
                nh.getParam("XYZpassMax",XYZpassMax);

                Eigen::Vector4f min_pt (XYZpassMin[0], XYZpassMin[1], XYZpassMin[2], 1.0f);
                Eigen::Vector4f max_pt (XYZpassMax[0], XYZpassMax[1], XYZpassMax[2], 1.0f);

                crop_box.setMin(min_pt);
                crop_box.setMax(max_pt);
            }

            if(SORon){
                nh.getParam("meanK",meanK);
                nh.getParam("stdThresh",stdThresh);

                SOR.setMeanK (meanK);
                SOR.setStddevMulThresh (stdThresh);
            }

            if(voxelFilterOn){
                nh.getParam("leafSize",leafSize);

                vg_filter.setLeafSize(leafSize, leafSize, leafSize);
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return;
        }
        
    }

    void pcl_callback(const sensor_msgs::PointCloud2 &input_pc)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr to_filter(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(input_pc, *to_filter);
        
        cloud_accum_mtx.lock();
        to_filter = filter_pc(to_filter);
        if(verbose) ROS_INFO("After return: %dx%d points!", to_filter->width, to_filter->height);
        *cloud_accum += *to_filter;
        cloud_accum = filter_pc(cloud_accum);
        cloud_accum_mtx.unlock();

        sensor_msgs::PointCloud2 filtered_pc;
        pcl::toROSMsg(*to_filter,filtered_pc);
        
        pc_pub.publish(filtered_pc);
        has_data = true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr to_filter){
        if(verbose) ROS_INFO("Before filter: %dx%d points!", to_filter->width, to_filter->height);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        ros::Time stamp = ros::Time::now();

        if(debug){
                std::string fileName = directory + "raw_" + std::to_string(stamp.sec) + "_" + std::to_string(stamp.nsec) + ".pcd";
                pcl::io::savePCDFileASCII (fileName, *to_filter);    
        }

        if(voxelFilterOn){
            vg_filter.setInputCloud(to_filter);
            vg_filter.filter(*filtered);
            to_filter = filtered;
            if(verbose) ROS_INFO("Voxel grid filter has %dx%d points!", filtered->width, filtered->height);
            if(debug){
                std::string fileName = directory + "vox_" + std::to_string(stamp.sec) + "_" + std::to_string(stamp.nsec) + ".pcd";
                pcl::io::savePCDFileASCII (fileName, *filtered);
            }
        }
        
        if(cropFilterOn){
            crop_box.setInputCloud(to_filter);
            crop_box.filter(*filtered);
            to_filter = filtered;
            if(verbose) ROS_INFO("Crop box filter has %dx%d points!", filtered->width, filtered->height);
            if(debug){
                std::string fileName = directory + "crop_" + std::to_string(stamp.sec) + "_" + std::to_string(stamp.nsec) + ".pcd";
                pcl::io::savePCDFileASCII (fileName, *filtered);
            }
        }

        if(SORon){
            SOR.setInputCloud(to_filter);
            SOR.filter(*filtered);
            if(verbose) ROS_INFO("SOR box filter has %dx%d points!", filtered->width, filtered->height);
            if(debug){
                std::string fileName = directory + "SOR_" + std::to_string(stamp.sec) + "_" + std::to_string(stamp.nsec) + ".pcd";
                pcl::io::savePCDFileASCII (fileName, *filtered);
            }
        }

        if(voxelFilterOn || cropFilterOn || SORon) return filtered;
        else return to_filter;

        if(verbose) ROS_INFO("After filter: %dx%d points!", to_filter->width, to_filter->height);

    }

    void publish_pc()
    {
        cloud_accum->header.frame_id = "base";
        cloud_accum->header.stamp = ros::Time::now().toNSec() / 1e3;
        stitched_pub.publish(cloud_accum);
    }

    void save_pc()
    {
        std::string fileName = directory + "Accum_" + std::to_string(ros::Time::now().sec) + "_" + std::to_string(ros::Time::now().nsec) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::savePCDFileASCII (fileName, *cloud_accum);
    }

    void clear_accum()
    {
        cloud_accum_mtx.lock();
        cloud_accum->points.clear();
        // Update height/width
        if (cloud_accum->height == 1)
        {
            cloud_accum->width = 0;
        }
        else
        {
            if (cloud_accum->width == 1)
            {
                cloud_accum->height = 0;
            }
            else
            {
                cloud_accum->width = cloud_accum->height = 0;
            }
        }
        cloud_accum_mtx.unlock();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_stitcher_node");
    PointCloudStitcher vfn;

    double rate = 10; //Hz
    ros::Time last_pub_t = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();

        if (gPublish)
        {
            vfn.publish_pc();
            gPublish = false;
            ROS_INFO("Published stitching");
        }

        if (gSave)
        {
            vfn.save_pc();
            gSave = false;
            ROS_INFO("Saved stitching");
        }

        if (gClearAccum)
        {
            vfn.clear_accum();
            gClearAccum = false;
            ROS_INFO("Cleared stitching");
        }

        // TODO: use conditional variable instead
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
