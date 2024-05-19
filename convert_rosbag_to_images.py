import os
import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge

FILENAME = 'output'
ROOT_DIR = '/ros_ws/scripts'
OUTPUT_DIR = '/ros_ws/output'
BAGFILE = os.path.join(OUTPUT_DIR, FILENAME + '.bag')

if __name__ == '__main__':
    bag = rosbag.Bag(BAGFILE)
    bridge = CvBridge()  # Initialize outside the loop for efficiency
    for i in range(2):
        if i == 0:
            TOPIC = '/sync/camera/depth/image_rect_raw'
            FOLDER_NAME = 'depth'
        else:
            TOPIC = '/sync/camera/color/image_raw'
            FOLDER_NAME = 'color'
        
        folder_path = os.path.join(OUTPUT_DIR, FOLDER_NAME)
        os.makedirs(folder_path, exist_ok=True)

        for k, (topic, msg, t) in enumerate(bag.read_messages(TOPIC)):
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            if FOLDER_NAME == 'depth':
                # Apply colormap for visualization if it's depth image
                cv_image = cv2.convertScaleAbs(cv_image, alpha=0.03)  # Scale image to uint8
                cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)

            timestamp = msg.header.stamp.to_nsec()  # Get nanosecond timestamp
            image_filename = f"{FOLDER_NAME}_{timestamp}.png"
            image_path = os.path.join(folder_path, image_filename)
            
            cv2.imwrite(image_path, cv_image)
            print(f'Saved: {image_filename} in {image_path}')

    bag.close()
    print('PROCESS COMPLETE')

