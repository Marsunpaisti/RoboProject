#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "obstacle.h"
#include "pidcontroller.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "vector"
#include <memory>
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
    Controller(std::string robotName, std::vector<std::shared_ptr<Obstacle> >* allObstacles);
    void loop();
    void stop();
    PIDController angleController = PIDController(1.0 / 50.0, -3, 3, 3, -0.1, 0, true);
    PIDController distanceController = PIDController(1.0 / 50.0, 0, 0.60, -1.5, 0.0, 0, false);
    bool enableDistControl = false;
    long test_timer = 0;
    int test_angle_num = 0;
    Obstacle robotObstacle;
    std::vector<std::shared_ptr<Obstacle> >* allObstacles;
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
    /*
    test_timer += 50;
    if (test_timer > 25000) {
        test_timer -= 25000;
        test_angle_num += 1;
        if (test_angle_num >= 3)
            test_angle_num = 0;
    }
    haveTargetPose = true;
    targetPose.x = 3;
    targetPose.y = -3 + 3 * test_angle_num;
    targetPose.theta = 0;
    */

    if (!haveTargetPose) {
        ROS_INFO("%s %s", robotName.c_str(),
            ": canceling loop. No target pose received yet");
        return;
    };

    double dx
        = targetPose.x - currentPose.x;
    double dy = targetPose.y - currentPose.y;
    double distanceToTarget = sqrt(dx * dx + dy * dy);

    // Calculate smaller vector towards target and apply repulsion forces to it
    // Then use that as the actual controller target
    double unitTargetX = dx / distanceToTarget;
    double unitTargetY = dy / distanceToTarget;
    double dirVecLen = std::min(distanceToTarget, 0.15);
    double dirX = unitTargetX * dirVecLen;
    double dirY = unitTargetY * dirVecLen;

    for (auto pObstacle : *allObstacles) {
        if (pObstacle.get() != &robotObstacle) { // Ignore repulsive force from self
            Vector2 repulsion = pObstacle->getRepulsionVectorAtPoint(currentPose.x, currentPose.y, 0.2);
            if (repulsion.x > 0 || repulsion.y > 0) {
                ROS_INFO("X: %f  Y: %f repulsion applied", repulsion.x, repulsion.y);
            }
            dirX += repulsion.x;
            dirY += repulsion.y;
        }
    }

    double angleToTarget = atan2(dirY, dirX); // Returns -pi < ang < pi
    double angleDifference = angleToTarget - currentPose.theta;
    if (angleDifference > 3.1415926535) {
        angleDifference -= 3.1415926535 * 2;
    } else if (angleDifference < -3.1415926535) {
        angleDifference += 3.1415926535 * 2;
    }

    double angleSteer = 0;
    double linearSteer = 0;
    if (dirVecLen > 0.035) {
        angleSteer = angleController.calculate(angleToTarget, currentPose.theta);
    } else { // Align self to target theta once we are at target location
        angleSteer = angleController.calculate(targetPose.theta, currentPose.theta);
    }

    if (fabs(angleDifference) < (1 * (3.14159 / 180.0))) {
        enableDistControl = true;
    }
    if (fabs(angleDifference) > (15 * (3.14159 / 180.0))) {
        enableDistControl = false;
    }

    if (dirVecLen > 0.035 && enableDistControl) { // Only run angle controller if we are roughly facing target
        linearSteer = distanceController.calculate(0, dirVecLen);
        // Scale up throttle linearly when angle difference towards target is reduced
        // When angle goes from 15deg -> 0deg throttle multiplier goes from 0 -> 100%
        double throttleCorrectionMultiplier = (0.26179916666 - fabs(angleDifference)) / 0.26179916666;
        linearSteer = linearSteer * throttleCorrectionMultiplier;
    }

    /*
    ROS_INFO("COORDS TARGET: X: %f Y: %f, OURS: X: %f Y: %f", targetPose.x, targetPose.y, currentPose.x, currentPose.y);
    ROS_INFO("ANGLES TARGET: %f, OUR: %f, DIFFERENCE: %f", angleToTarget, currentPose.theta, angleDifference);
    */
    geometry_msgs::Twist outputTwist;
    outputTwist.angular.z = angleSteer;
    outputTwist.linear.x = linearSteer;
    //ROS_INFO("OUTPUT: LINEAR X: %f Y:%f Z: %f, ANGULAR: X: %f, Y: %f, Z: %f", outputTwist.linear.x, outputTwist.linear.y, outputTwist.linear.z, outputTwist.angular.x, outputTwist.angular.y, outputTwist.angular.z);
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
    robotObstacle._x = currentPose.x;
    robotObstacle._y = currentPose.y;
    haveCurrentPose = true;
    loop();
}

Controller::Controller(std::string robotName, std::vector<std::shared_ptr<Obstacle> >* allObstacles)
    : robotName(robotName)
    , robotObstacle(0, 0, 0.2)
    , allObstacles(allObstacles)
{
    ros::NodeHandle nh;
    inputTargetSubscriber = nh.subscribe(robotName + "/controller_target", 1,
        &Controller::inputTargetCb, this);
    odometrySubscriber = nh.subscribe(robotName + "/odom", 0, &Controller::inputOdometryCb, this);
    outputPublisher = nh.advertise<geometry_msgs::Twist>(robotName + "/cmd_vel", 0);
    ROS_INFO("%s %s", robotName.c_str(), ": controller initialized");
}

std::vector<float> string_vec_to_float_vec(std::vector<std::string> stringvec)
{
  std::vector<float> numvec;
  for (size_t i = 0; i < stringvec.size(); i++) {
    numvec.push_back(std::stof(stringvec.at(i)));
  }
  return numvec;
}

std::vector<std::string> split_string_by_spaces(std::string inputstr)
{
    std::stringstream tmpss(inputstr);
    std::vector<std::string> out;
    std::string tmps;
    while (getline(tmpss, tmps, ' ')) {
        out.push_back(tmps);
    }
    return out;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voronoi_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ROS_INFO("%s", "Voronoi controller started");

    // Create controller object for all robots given in launchfile
    std::vector<std::shared_ptr<Obstacle> > allObstacles;
    std::vector<std::shared_ptr<Controller> > controllers;
    std::string robotNames;
    bool paramSucceeded = nh.getParam("robot_names_set", robotNames);
    std::string obstx, obsty;
    bool xFound = nh.getParam("obst_x_set", obstx);
    bool yFound = nh.getParam("obst_y_set", obsty);
    std::cout << "xFound: " << xFound << '\n';
    std::cout << "yFound: " << yFound << '\n';
    std::vector<Obstacle> tmpObstVec;
    if (xFound && yFound) {
      float currx, nextx, curry, nexty, pdist, xdist, ydist, midx, midy;
      int pcount;
      std::vector<float> xvals = string_vec_to_float_vec(split_string_by_spaces(obstx));
      std::vector<float> yvals = string_vec_to_float_vec(split_string_by_spaces(obsty));
      std::cout << "length of xvals: " << xvals.size() << '\n';
      for (size_t i = 0; i < xvals.size()-1; i++) {
        currx = xvals.at(i);nextx = xvals.at(i+1);
        curry = yvals.at(i);nexty = yvals.at(i+1);
        pdist = (currx-nextx)*(currx-nextx)+(curry-nexty)*(curry-nexty);
        xdist = (currx-nextx);
        ydist = (curry-nexty);
        pcount = std::ceil(pdist*10);
        for (size_t i = 0; i < pcount; i++) {
          Obstacle currObst((nextx+i*xdist/pcount),(nexty+i*ydist/pcount),0.02);
          //currObst._x =(nextx-i*xdist/pcount);
          //currObst._y = (nexty-i*ydist/pcount);
          tmpObstVec.push_back(currObst);
          //std::shared_ptr<Obstacle> pObst(new Obstacle);
          //pObst = &currObst;
        }
      }
      for (size_t i = 0; i < tmpObstVec.size(); i++) {
        std::shared_ptr<Obstacle> pObst(&tmpObstVec.at(i));
        allObstacles.push_back(pObst);
      }
    }
    std::cout << "size of allObstacles: " << allObstacles.size() << '\n';
    //return 0;
    std::vector<std::string> robotNamesSplit = split_string_by_spaces(robotNames);
    for (std::string& robotName : robotNamesSplit) {
        std::shared_ptr<Controller> pRobotController(new Controller(robotName, &allObstacles));
        std::shared_ptr<Obstacle> pRobotObstacle(&pRobotController->robotObstacle);
        controllers.push_back(pRobotController);
        allObstacles.push_back(pRobotObstacle);
    }

    // Main loop
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Send stop command for all
    for (auto& pController : controllers) {
        pController->stop();
    }
    return 0;
}
