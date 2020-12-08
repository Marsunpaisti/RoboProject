#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "pidcontroller.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vector"
#include <sstream>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_kdl.h>

class Controller {
private:
public:
    bool haveCurrentPose;
    bool haveTargetPose;
    ros::Subscriber inputTargetSubscriber;
    ros::Subscriber odometrySubscriber;
    ros::Publisher outputPublisher;
    std::string robotName;
    geometry_msgs::Pose2D targetPose;
    geometry_msgs::Pose2D currentPose;
    void inputTargetCb(const geometry_msgs::Pose2D::ConstPtr pTargetPose);
    void inputOdometryCb(const nav_msgs::Odometry::ConstPtr pOdometryMsg);
    Controller(std::string robotName);
    void loop();
    void stop();
    PIDController angleController = PIDController(1.0 / 50.0, -0.65, 0.65, 1, -0.1, 0, true);
    PIDController distanceController = PIDController(1.0 / 50.0, 0, 0.6, -1.3, 0.1, 0, false);
};

void Controller::stop()
{
    geometry_msgs::Twist outputTwist;
    outputPublisher.publish(outputTwist);
}

void Controller::loop()
{
    if (!haveCurrentPose) {
        ROS_INFO("%s %s", robotName.c_str(),
            ": canceling loop. No odometry received yet");
        return;
    };

    // TODO Remove hardcoded target
    haveTargetPose = true;
    targetPose.x = 4;
    targetPose.y = -1;
    if (!haveTargetPose) {
        ROS_INFO("%s %s", robotName.c_str(),
            ": canceling loop. No target pose received yet");
        return;
    };

    double dx
        = targetPose.x - currentPose.x;
    double dy = targetPose.y - currentPose.y;
    double angleToTarget = atan2(dy, dx); // Returns -pi < ang < pi
    double angleDifference = angleToTarget - currentPose.theta;
    double distanceToTarget = sqrt(dx * dx + dy * dy);
    if (angleDifference > 3.1415926535) {
        angleDifference -= 3.1415926535 * 2;
    } else if (angleDifference < -3.1415926535) {
        angleDifference += 3.1415926535 * 2;
    }

    double angleSteer = 0;
    double linearSteer = 0;
    if (distanceToTarget > 0.05) {
        angleSteer = angleController.calculate(angleToTarget, currentPose.theta);
    } else { // Align self to target theta once we are at target location
        angleSteer = angleController.calculate(targetPose.theta, currentPose.theta);
    }
    if (distanceToTarget > 0.05 && fabs(angleDifference) < (10 * (3.14159 / 180))) { // Only run angle controller if we are roughly facing target
        linearSteer = distanceController.calculate(0, distanceToTarget);
    }

    /*
    ROS_INFO("COORDS TARGET: X: %f Y: %f, OURS: X: %f Y: %f", targetPose.x, targetPose.y, currentPose.x, currentPose.y);
    ROS_INFO("ANGLES TARGET: %f, OUR: %f, DIFFERENCE: %f", angleToTarget, currentPose.theta, angleDifference);
    */
    geometry_msgs::Twist outputTwist;
    outputTwist.angular.z = angleSteer;
    outputTwist.linear.x = linearSteer;
    ROS_INFO("OUTPUT: LINEAR X: %f Y:%f Z: %f, ANGULAR: X: %f, Y: %f, Z: %f", outputTwist.linear.x, outputTwist.linear.y, outputTwist.linear.z, outputTwist.angular.x, outputTwist.angular.y, outputTwist.angular.z);
    outputPublisher.publish(outputTwist);
}

void Controller::inputTargetCb(const geometry_msgs::Pose2D::ConstPtr pTargetPose)
{
    targetPose.theta = pTargetPose->theta;
    targetPose.x = pTargetPose->x;
    targetPose.y = pTargetPose->y;
    haveTargetPose = true;
}

void Controller::inputOdometryCb(const nav_msgs::Odometry::ConstPtr pOdometryMsg)
{
    double roll;
    double pitch;
    double yaw;

    // Converting quaternion to roll/pitch/yaw
    KDL::Rotation rot;
    tf::quaternionMsgToKDL(pOdometryMsg->pose.pose.orientation, rot);
    rot.GetRPY(roll, pitch, yaw);

    // Saving position information to class variables
    currentPose.x = pOdometryMsg->pose.pose.position.x;
    currentPose.y = pOdometryMsg->pose.pose.position.y;
    currentPose.theta = yaw;
    haveCurrentPose = true;
}

Controller::Controller(std::string robotName)
    : robotName(robotName)
{
    ros::NodeHandle nh;
    currentPose.x = 1;
    currentPose.y = 2;
    inputTargetSubscriber = nh.subscribe(robotName + "/controller_target", 0,
        &Controller::inputTargetCb, this);
    odometrySubscriber = nh.subscribe(robotName + "/odom", 0, &Controller::inputOdometryCb, this);
    outputPublisher = nh.advertise<geometry_msgs::Twist>(robotName + "/cmd_vel", 0);
    ROS_INFO("%s %s", robotName.c_str(), ": controller initialized");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voronoi_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);
    ROS_INFO("%s", "Voronoi controller started");
    std::vector<boost::shared_ptr<Controller> > controllers;
    boost::shared_ptr<Controller> testController(new Controller(""));
    controllers.push_back(testController);
    while (ros::ok()) {
        for (auto& pController : controllers) {
            pController->loop();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Send stop command for all
    for (auto& pController : controllers) {
        pController->stop();
    }
    return 0;
}
