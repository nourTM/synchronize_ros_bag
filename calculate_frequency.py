import rosbag
import rospy

def calculate_frequency(bag_path):
    bag = rosbag.Bag(bag_path)
    
    topic_frequencies = {}

    for topic, msg, t in bag.read_messages():
        if topic not in topic_frequencies:
            topic_frequencies[topic] = 1
        else:
            topic_frequencies[topic] += 1

    bag_duration = bag.get_end_time() - bag.get_start_time()

    print("Topic Frequencies:")
    for topic, frequency in topic_frequencies.items():
        topic_frequency = frequency / bag_duration
        print(f"{topic}: {topic_frequency:.2f} Hz")

    bag.close()

if __name__ == "__main__":
    bag_path = "/output/sensor_data_nour.bag"
    calculate_frequency(bag_path)
