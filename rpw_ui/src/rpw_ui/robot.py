import rospy
from geometry_msgs.msg import Twist

class Robot:

    def __init__(self, id):
        self.id = id
        self.topic = "/{}/odom".format(id)
        rospy.loginfo("Robot {} created and listens to topic: {}".format(self.id, self.topic))


    def unsubscribe(self):

        pass