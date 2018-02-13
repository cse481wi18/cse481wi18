import rosbag
from sensor_msgs.msg import PointCloud2


def pc_filter(topic, datatype, md5sum, msg_def, header):
    if datatype == 'sensor_msgs/PointCloud2':
        return True
    return False


class MockCamera(object):
    """A MockCamera reads saved point clouds.
    """

    def __init__(self):
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.

        Args:
            path: string, the path to a bag file with a single
                sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """
        bag = rosbag.Bag(path)
        for topic, msg, time in bag.read_messages(connection_filter=pc_filter):
            return msg
        bag.close()
        return None
