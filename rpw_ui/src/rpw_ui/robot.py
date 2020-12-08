import rospy

class Robot:

    def __init__(self, name):
        self.name = name
        self.topic = "/{}/odom".format(name)
        rospy.loginfo("Robot {} created and listens topic: {}".format(self.name, self.topic))

        pass