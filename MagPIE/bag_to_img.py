#!/usr/bin/env python

import rospy
import csv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        self.image_topic = "/camera/color/image_raw"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.save_dir = "/home/su/MagPIE/Talbot_UGV/magpie2Dataset_8/mav0/cam0/data"
        self.csv_file_path = '/home/su/MagPIE/Talbot_UGV/magpie2Dataset_8/mav0/cam0/data.csv'
        self.txt_file_path = '/home/su/Dev/ORB_SLAM3/Examples/Monocular-inertial/magpie_TimeStamps/traj_08.txt'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        self.counter = 0
        
        file_exists = os.path.isfile(self.csv_file_path)
        self.csv_file = open(self.csv_file_path, mode='a')
        self.txt_file = open(self.txt_file_path, mode='a')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        self.txt_writer = csv.writer(self.txt_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        if not file_exists or os.stat(self.csv_file_path).st_size == 0:
                self.csv_writer.writerow(['#timestamp [ns]', 'filename'])


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")  # Use "rgb16" if it’s a color image
            file_name = f"{msg.header.stamp}.png"
            file_path = os.path.join(self.save_dir, file_name)
            cv2.imwrite(file_path, cv_image)
            self.counter += 1
            self.csv_writer.writerow([msg.header.stamp, file_name])
            self.txt_writer.writerow([msg.header.stamp])
        
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        image_saver.run()
    except rospy.ROSInterruptException:
        pass