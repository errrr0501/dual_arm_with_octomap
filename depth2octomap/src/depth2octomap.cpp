#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
 
// PCL library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
 
// define pointcloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
 
using namespace std;
using namespace cv;

// Intrinsic Camera Parameters 
const double camera_factor = 1000;
const double camera_cx = 324.4005126953125;
const double camera_cy = 240.46707153320312;
const double camera_fx = 387.39300537109375;
const double camera_fy = 387.39300537109375;

// 387.39300537109375, 0.0, 324.4005126953125, 0.0, 387.39300537109375, 240.46707153320312, 0.0, 0.0, 1.0

// global variable：image martix & point cloud
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;
 
/***  RGB process  ***/
void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
    try
    {
        color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        cv::waitKey(1050); // contiune refresh image，frequency time is = int delay，ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    color_pic = color_ptr->image;
}
 
/***  Depth process  ***/
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::waitKey(1050);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
 
    depth_pic = depth_ptr->image;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_octomap");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, color_Callback);
    image_transport::Subscriber sub1 = it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
    ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/output", 1);
    // pointcloud variable
    PointCloud::Ptr cloud ( new PointCloud );
    sensor_msgs::PointCloud2 pub_pointcloud;
 
    double sample_rate = 1.0; // 1HZ
    ros::Rate naptime(sample_rate); // use to regulate loop rate
 
    while (ros::ok()) {
        // check depth image
        for (int m = 0; m < depth_pic.rows; m++){
            for (int n = 0; n < depth_pic.cols; n++){
                // get depth image value at (m,n)
                float d = depth_pic.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
                // d might not have value，if so, skip it 
                if (d == 0)
                    continue;
                // d have value，add one point at pointcloud
                pcl::PointXYZRGB p;
 
                // calculate spatial coordinates of this point
//                p.z = double(d) / camera_factor;
//                p.x = (n - camera_cx) * p.z / camera_fx;
//                p.y = (m - camera_cy) * p.z / camera_fy;
 
                // camera model is vertical
                p.x = double(d) / camera_factor;
                p.y = -(n - camera_cx) * p.x / camera_fx;
                p.z = -(m - camera_cy) * p.x / camera_fy;
 
                // get color from rgb image
                // rgb image is 3 channel with B, G, R
                p.b = color_pic.ptr<uchar>(m)[n*3];
                p.g = color_pic.ptr<uchar>(m)[n*3+1];
                p.r = color_pic.ptr<uchar>(m)[n*3+2];
 
                // add p to pointcloud
                cloud->points.push_back( p );
            }
        }
 
        // setting and save pointcloud
        cloud->height = 1;
        cloud->width = cloud->points.size();
        ROS_INFO("point cloud size = %d",cloud->width);
        cloud->is_dense = false;// transform pointcloud type
        pcl::toROSMsg(*cloud,pub_pointcloud);
        pub_pointcloud.header.frame_id = "camera_link";
        pub_pointcloud.header.stamp = ros::Time::now();
        // publish synthesis pointcloud
        pointcloud_publisher.publish(pub_pointcloud);
        // clean data and exit
        cloud->points.clear();
 
        ros::spinOnce(); //allow data update from callback;
        naptime.sleep(); // wait for remainder of specified period;
    }
}