#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace sensor_msgs;
using namespace message_filters;

rosbag::Bag output_bag;

void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& cloud) {
    // Save synchronized messages to the new bag
    ROS_INFO("Callback triggered, writing to bag");
    output_bag.write("/sync/camera/color/image_raw", ros::Time::now(), image);
    output_bag.write("/sync/velodyne_points", ros::Time::now(), cloud);
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

    // Open the input bag
    rosbag::Bag input_bag;
    input_bag.open(input_bag_file, rosbag::bagmode::Read);

    // Open the output bag
    output_bag.open(output_bag_file, rosbag::bagmode::Write);

    // Publishers for the bag messages
    ros::Publisher image_pub = nh.advertise<Image>("/temporary/image_raw", 20);
    ros::Publisher cloud_pub = nh.advertise<PointCloud2>("/temporary/velodyne_points", 10);

    // Subscribers for the message_filters
    message_filters::Subscriber<Image> image_sub(nh, "/temporary/image_raw", 20);
    message_filters::Subscriber<PointCloud2> cloud_sub(nh, "/temporary/velodyne_points", 10);

    typedef sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/color/image_raw"));
    topics.push_back(std::string("/velodyne_points"));

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == "/camera/color/image_raw") {
            ImageConstPtr image = m.instantiate<Image>();
            ROS_INFO("image raw");
            if (image != NULL) {
            	 ROS_INFO("Publishing to bag");
                image_pub.publish(image);
            }
        }

        if (m.getTopic() == "/velodyne_points") {
            PointCloud2ConstPtr cloud = m.instantiate<PointCloud2>();
            if (cloud != NULL) {
                cloud_pub.publish(cloud);
            }
        }
        // Ensure all callbacks are processed
    	ros::spinOnce();
    }

 

    // Close the bags
    input_bag.close();
    output_bag.close();

    std::cout << "Processing complete!" << std::flush;
    return 0;
}
