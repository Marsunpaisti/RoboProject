import rospy
from geometry_msgs.msg import Twist

class Robot:

    def __init__(self, id):
        self.id = id
        self.topic = "/{}/odom".format(id)
        rospy.loginfo("Robot {} created and listens to topic: {}".format(self.id, self.topic))
        self.x_pos = 0.0
        self.y_pos = 0.0
        # Subscribe
        self.sub = rospy.Subscriber(self.topic, Twist, callback=self.callback)

    def callback(self, msg):
        self.x_pos = msg.linear.x
        self.y_pos = msg.linear.y



    def unsubscribe(self):
        self.sub.unregister()
        pass