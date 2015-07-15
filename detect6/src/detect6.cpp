#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sstream>
#include <iostream>
#include <string>

#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include "geometry_msgs/Point.h"

#include <sstream>

#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>


unsigned int id = 1;
class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("data", 1000);

// %Tag(LOOP_RATE)%
    bool match(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud )
    {
        pcl::ModelCoefficients::Ptr coefficients_Sphere (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_Sphere (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg_Sphere;
        // Optional
        seg_Sphere.setOptimizeCoefficients (true);
        // Mandatory
        seg_Sphere.setModelType (pcl::SACMODEL_SPHERE);
        seg_Sphere.setMethodType (pcl::SAC_RANSAC);
        seg_Sphere.setDistanceThreshold (0.001);

        seg_Sphere.setInputCloud (cloud);
        seg_Sphere.segment (*inliers_Sphere, *coefficients_Sphere);

        if (inliers_Sphere->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return false;
        }
        //对球的半径有选择的过滤
        if (coefficients_Sphere->values[3] < 0.08 || coefficients_Sphere->values[3] > 0.2)
            return false;
        //由于暂时检测不到3米外的球，故对找到的3米外的球过滤
        // if (coefficients_Sphere->values[2] > 3)
        //return false;

        std::cerr << "Model coefficients: " << coefficients_Sphere->values[0] << " "
                  << coefficients_Sphere->values[1] << " "
                  << coefficients_Sphere->values[2] << " "
                  << coefficients_Sphere->values[3] << std::endl;
        //准备数据报文
        geometry_msgs::Point position;

        position.x = coefficients_Sphere->values[0];
        position.y = coefficients_Sphere->values[1];
        position.z = coefficients_Sphere->values[2];
        ROS_INFO("x:%lf y:%lf z:%lf", position.x, position.y, position.z);

        chatter_pub.publish(position);

        ros::spinOnce();
        //std::cerr << "Model inliers_Sphere: " << inliers_Sphere->indices.size () << std::endl;
        // loop_rate.sleep();
        //viewer.showCloud(cloud);
        //viewer.runOnVisualizationThreadOnce (viewerOneOff);

        return true;
        /*
        for (size_t i = 0; i < inliers->indices.size (); ++i)
            std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                       << cloud->points[inliers->indices[i]].y << " "
                                                       << cloud->points[inliers->indices[i]].z << std::endl;
        */
    }
    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
    {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        cout << "id:" << id++ <<endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new 	pcl::PointCloud<pcl::PointXYZ>);

        //voxelgrid并不是产生球面空洞的原因
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud_filtered->points.size ();
        while (cloud_filtered->points.size () > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;
        }
        // viewer.showCloud(cloud_filtered);
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        if ( !cloud_filtered->empty())
            tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.04); // 4cm
        ec.setMinClusterSize (200);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        viewer.showCloud(cloud_filtered);
        int j = 0;
        int cnt = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            //std::stringstream ss;
            //ss << "cloud_cluster_" << j << ".pcd";
            //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
            j++;
            cout << "cloud_cluster_" << j << endl;
            if ( match(cloud_cluster))
            {
                cout << ++cnt << " sphere have been detected " <<endl;
                //viewer.showCloud(cloud_cluster);


            }
            //viewer.showCloud(cloud_cluster);

        }

        //if (!viewer.wasStopped())
        //viewer.showCloud (cloud);
    }

    void run ()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "talker");




    SimpleOpenNIViewer v;
    v.run ();
    return 0;
}

