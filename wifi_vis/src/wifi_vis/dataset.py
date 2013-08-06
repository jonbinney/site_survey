import rosbag
import numpy as np
from tf import transformations

class Datapoint:
    def __init__(self, signal, noise, channel, ssid, bssid, pose):
        '''
        Args:
            signal (int or float) - Signal strength (db).
            noise (int or float) - Noise (db).
            channel (int) - Wireless channel.
            ssid (str) - SSID of access point.
            bssid (str) - BSSID of access point.
            pose (geometry_msgs.msg.Pose) - Pose of robot for this measurement.
        '''
        self.signal = signal
        self.noise = noise
        self.channel = channel
        self.ssid = ssid
        self.bssid = bssid
        self.pose = pose
    
    def get_xytheta(self):
        q = self.pose.orientation
        roll, pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return np.array((self.pose.position.x, self.pose.position.y, yaw))

class Dataset:
    def __init__(self):
        self.bssid_dict = {}

    def from_bag(self, bag_filename):
        '''
        Read in data from a bagfile
        '''
        bag = rosbag.Bag(bag_filename)
        pose = None
        for topic, msg, t in bag.read_messages():
            if topic == '/odom_combined':
                pose = msg.pose.pose
            elif topic == '/ap_detection':
                if pose is None:
                    continue
                if not msg.bssid in self.bssid_dict:
                    self.bssid_dict[msg.bssid] = []

                d = Datapoint(msg.signal, msg.noise, msg.channel, msg.ssid, msg.bssid, pose)
                self.bssid_dict[msg.bssid].append(d)
        bag.close()
