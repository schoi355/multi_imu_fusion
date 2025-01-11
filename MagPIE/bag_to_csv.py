import rospy
import csv
import os
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

class ROSNode:
    def __init__(self):
        rospy.init_node('ros_to_csv_node', anonymous=True)
        
        self.csv_file_path = 'dataset/Talbot_UGV/magpie2Dataset_8_imu4.csv'
        file_exists = os.path.isfile(self.csv_file_path)
        self.csv_file = open(self.csv_file_path, mode='a')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        # rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        rospy.Subscriber('/imu_chatter_B4', Imu, self.imu_callback)
        # rospy.Subscriber('/tango/transform/area_description_T_start_of_service', TransformStamped, self.tango_callback)
        # rospy.Subscriber('/tango/transform/start_of_service_T_device', TransformStamped, self.tango_callback)
        
        if not file_exists or os.stat(self.csv_file_path).st_size == 0:
                # self.csv_writer.writerow(['t', 'px', 'py', 'pz', 'rx', 'ry', 'rz', 'rw'])
                self.csv_writer.writerow(['t', 'gx', 'gy', 'gz', 'ax', 'ay', 'az'])


    def tf_callback(self, data):
        self.csv_writer.writerow([data.transforms[0].header.stamp, data.transforms[0].transform.translation.x, 
                                  data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z,
                                  data.transforms[0].transform.rotation.x, data.transforms[0].transform.rotation.y, 
                                  data.transforms[0].transform.rotation.z, data.transforms[0].transform.rotation.w])
        
    def imu_callback(self, data):
        self.csv_writer.writerow([data.header.stamp, data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z,
                                  data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])

    def tango_callback(self, data):
        self.csv_writer.writerow([data.header.stamp, data.transform.translation.x, 
                                  data.transform.translation.y, data.transform.translation.z,
                                  data.transform.rotation.x, data.transform.rotation.y, 
                                  data.transform.rotation.z, data.transform.rotation.w])
        
    def spin(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        finally:
            self.csv_file.close()

if __name__ == '__main__':
    ros_node = ROSNode()
    ros_node.spin()