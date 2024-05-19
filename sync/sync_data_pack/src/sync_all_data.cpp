#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <fstream>

using namespace sensor_msgs;
using namespace message_filters;

rosbag::Bag output_bag;

std::ofstream timestampFile;
// Updated callback function to include additional message types
void callback(const ImageConstPtr& image_color, const ImageConstPtr& image_depth, const PointCloud2ConstPtr& cloud, const NavSatFixConstPtr& gnss, const ImuConstPtr& imu) {
    // Get timestamps in nanoseconds
    uint64_t ts_image_color = image_color->header.stamp.toNSec();
    uint64_t ts_image_depth = image_depth->header.stamp.toNSec();
    uint64_t ts_cloud = cloud->header.stamp.toNSec();
    uint64_t ts_gnss = gnss->header.stamp.toNSec();
    uint64_t ts_imu = imu->header.stamp.toNSec();

    // Calculate the maximum gap between timestamps
    uint64_t max_gap = std::max({
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_color - ts_image_depth))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_color - ts_cloud))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_color - ts_gnss))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_color - ts_imu))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_depth - ts_cloud))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_depth - ts_gnss))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_image_depth - ts_imu))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_cloud - ts_gnss))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_cloud - ts_imu))),
        static_cast<uint64_t>(std::abs(static_cast<int64_t>(ts_gnss - ts_imu)))
    });


    // Define the maximum acceptable gap (in nanoseconds)
    const uint64_t MAX_ACCEPTABLE_GAP = 100000000; // 100 milliseconds

    // Check if the maximum gap is within the acceptable limit
    if (max_gap <= MAX_ACCEPTABLE_GAP) {
        ROS_INFO("Callback triggered, writing to bag");
        
        const ros::Time common_time = ros::Time::now();
        
        // Make copies of the const messages to modify them
        Image img_color = *image_color;
        Image img_depth = *image_depth;
        PointCloud2 p_cloud = *cloud;
        NavSatFix gps = *gnss;
        Imu im = *imu;

        // Set all timestamps to the common_time
        img_color.header.stamp = common_time;
        img_depth.header.stamp = common_time;
        p_cloud.header.stamp = common_time;
        gps.header.stamp = common_time;
        im.header.stamp = common_time;
        
        // Write the copies with the updated timestamps to the bag
        output_bag.write("/sync/camera/color/image_raw", common_time, img_color);
        output_bag.write("/sync/camera/depth/image_rect_raw", common_time, img_depth);
        output_bag.write("/sync/velodyne_points", common_time, p_cloud);
        output_bag.write("/sync/gnss", common_time, gps);
        output_bag.write("/sync/imu", common_time, im);
        
        // Open the file in append mode if it's not already open
        if (!timestampFile.is_open()) {
            timestampFile.open("/output/timestamps_100ms.txt", std::ios::out | std::ios::app);
        }

        // Write the timestamps to the file
        timestampFile << ts_image_color << ", "
                      << ts_image_depth << ", "
                      << ts_cloud << ", "
                      << ts_gnss << ", "
                      << ts_imu << ", "
                      << common_time << std::endl;
    } else {
        ROS_INFO("Max gap exceeded, skipping write.");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;

    if (argc != 3) {
        ROS_ERROR("Usage: sync_node <input_bag_path> <output_bag_path>");
        return -1;
    }

    std::string input_bag_file = argv[1];
    std::string output_bag_file = argv[2];

    // Open the input and output bags
    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);
    output_bag.open(output_bag_file, rosbag::bagmode::Write);

    // Publishers for the bag messages
    ros::Publisher image_color_pub = nh.advertise<Image>("/temporary/image_raw", 200);//50);
    ros::Publisher image_depth_pub = nh.advertise<Image>("/temporary/image_depth", 190);//50);
    ros::Publisher cloud_pub = nh.advertise<PointCloud2>("/temporary/velodyne_points", 400);//100);
    ros::Publisher gnss_pub = nh.advertise<NavSatFix>("/temporary/gnss", 450); //50);
    ros::Publisher imu_pub = nh.advertise<Imu>("/temporary/imu", 50);//10);

    // Subscribers for the message_filters
    // Make sure the subscribers are listening to the same "/temporary" topics
    message_filters::Subscriber<Image> image_color_sub(nh, "/temporary/image_raw", 200);
    message_filters::Subscriber<Image> image_depth_sub(nh, "/temporary/image_depth", 190);
    message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/temporary/velodyne_points", 400);
    message_filters::Subscriber<NavSatFix> gnss_sub(nh, "/temporary/gnss", 450);
    message_filters::Subscriber<Imu> imu_sub(nh, "/temporary/imu", 50);

    // Define the synchronization policy
    typedef sync_policies::ApproximateTime<Image, Image, PointCloud2, NavSatFix, Imu> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(12), image_color_sub, image_depth_sub, cloud_sub, gnss_sub, imu_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));



    std::vector<std::string> topics;
    topics.push_back("/camera/color/image_raw");
    topics.push_back("/camera/depth/image_rect_raw");
    topics.push_back("/velodyne_points");
    topics.push_back("/gnss");
    topics.push_back("/imu/data");

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));

    int imu_downsample_rate = 10; // Publish every 10th IMU message
    int imu_counter = 0; // Counter for downsampling

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == "/camera/color/image_raw") {
            ImageConstPtr image_color = m.instantiate<Image>();
            if (image_color != NULL) {
                image_color_pub.publish(image_color);
            }
        } else if (m.getTopic() == "/camera/depth/image_rect_raw") {
            ImageConstPtr image_depth = m.instantiate<Image>();
            if (image_depth != NULL) {
                image_depth_pub.publish(image_depth);
            }
        } else if (m.getTopic() == "/velodyne_points") {
            PointCloud2ConstPtr cloud = m.instantiate<PointCloud2>();
            if (cloud != NULL) {
                cloud_pub.publish(cloud);
            }
        } else if (m.getTopic() == "/gnss") {
            NavSatFixConstPtr gnss = m.instantiate<NavSatFix>();
            if (gnss != NULL) {
                gnss_pub.publish(gnss);
            }
        } else if (m.getTopic() == "/imu/data") {
            ImuConstPtr imu = m.instantiate<Imu>();
            if (imu != NULL) {
                imu_counter++;
                if (imu_counter % imu_downsample_rate == 0) { // Check if the counter is a multiple of the downsampling rate
                    imu_pub.publish(imu);
                    imu_counter = 0; // Reset the counter after publishing
                }
            }
        }

        // Process callbacks
        ros::spinOnce();
    }

    // Close the bags
    input_bag.close();
    output_bag.close();

    // Close the file before exiting the program
    if (timestampFile.is_open()) {
        timestampFile.close();
    }
    
    std::cout << "Processing complete!" << std::flush;
    return 0;
}
