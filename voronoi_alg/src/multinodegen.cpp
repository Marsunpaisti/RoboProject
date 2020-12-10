
#include <chrono>
#include <iostream>
#include <math.h>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "voronoi_alg/RobotPos.h"

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

/*class Voronoicbc
{
public:
    Voronoicbc(const ros::NodeHandle& nh){
		  std::string nodenames;
		  bool retsucceeded = nh.getParam("robot_names_set", nodenames);
			std::vector<string> nodenameVec = split_string_by_spaces(nodenames);

			botLX=-1;botLY=-1;topRX=1;topRY=1;
			target_region_x = {-1,-1,1,1};
			target_region_y = {-1,1,1,-1};
      mnh = nh;
      targloc_pub = mnh.advertise<voronoi_alg::RobotPos>("target_locs", 1000);
      targloc_sub = mnh.subscribe("robot_locs", 1000, &Voronoicbc::robPosVecCb, this);
			targreg_sub = mnh.subscribe("target_region", 1000, &Voronoicbc::robRegVecCb, this);
      ros::spin();
    };
private:
	void robRegVecCb(const voronoi_alg::RobotPos::ConstPtr& msg){
		//botLX=msg->x.at(0);botLY=msg->y.at(0);topRX=msg->x.at(1);topRY=msg->y.at(1);
		target_region_x = msg->x;
		target_region_y = msg->y;
		//eli tässä funktiossa callback, ja robojen paikat vaan luetaan jostain... kait?
		//jos nyt toistaseks muokkais vaan tota robPosVecCb:tä.
		//rowVecToColVec
	}
    void robPosVecCb(const voronoi_alg::RobotPos::ConstPtr& msg){
			std::vector<std::vector<float>> posIn, posOut,
				targetAsRowVec{target_region_x,target_region_y};
			posIn.push_back(msg->x);
			posIn.push_back(msg->y);
			std::vector<std::vector<float>> targetAsColVec = rowVecToColVec(targetAsRowVec);
			float costh, sinth;
			cornersToRotangles(targetAsColVec, &costh, &sinth);
			//std::vector<std::vector<float>>
			float xmin, xmax, ymin, ymax;
			std::vector<std::vector<float>> generatedPoints =
				transformRobLocsAndCorners(targetAsColVec, rowVecToColVec(posIn),
				&xmin, &xmax, &ymin, &ymax);

			//std::size_t nbPoints = 2;
	    std::vector<std::vector<float>> centroids;
	    // Generate points
	    //std::vector<Vector2> points = generatePoints(nbPoints);
			Vector2 bottomLeftCorner, topRightCorner;
			//bottomLeftCorner.x=botLX;bottomLeftCorner.y=botLY;
			//topRightCorner.x=topRX;topRightCorner.y=topRY;
			bottomLeftCorner.x=xmin;bottomLeftCorner.y=ymin;
			topRightCorner.x=xmax;topRightCorner.y=ymax;
	    // Write generated points to generatedPoints:
	    //std::vector<std::vector<float>> generatedPoints = colVecToRowVec(posIn);//vector2toFloats(points);

	    VoronoiDiagram diagram = generateRandomDiagram(&generatedPoints,
				bottomLeftCorner, topRightCorner);

	    std::vector<std::vector<std::vector<float>>> polygons;

	    //count the edges:
	    countEdges(diagram);
	    //detect the polygons:
	    detectPolygon(diagram, &polygons);
	    //detect the point centers:
	    findAllCentroids(polygons, &centroids);
			posOut = rowVecToColVec(centroids);
			posOut = transformTargetLocs(posOut, targetAsColVec);
      voronoi_alg::RobotPos reply;
      //reply.x = targ_x;reply.y = targ_y;
			reply.x = posOut.at(0);reply.y = posOut.at(1);
      targloc_pub.publish(reply);
			std::string xcoord_as_str = std::to_string(reply.x.at(0));
			ROS_INFO("First target x: [%s]", xcoord_as_str.c_str());
			std::string ycoord_as_str = std::to_string(reply.y.at(0));
			ROS_INFO("First target y: [%s]", ycoord_as_str.c_str());
			std::string botlx_as_str = std::to_string(botLX);
			ROS_INFO("Bottom left x: [%s]", botlx_as_str.c_str());
    };
    float recXcoord = 0, botLX, botLY, topRX, topRY;
		std::vector<float> target_region_x, target_region_y;
    ros::Publisher targloc_pub;
    ros::Subscriber targloc_sub, targreg_sub;
    ros::NodeHandle mnh;
};*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    //std::vector<std::string> nodenames;
    std::string nodenames;
    bool retsucceeded = n.getParam("robot_names_set", nodenames);
    std::vector<std::string> nodenameVec = split_string_by_spaces(nodenames);
    while (ros::ok()) {
    }
    //std::cout << nodenames.size() << '\n';
    //std::cout << nodenames << '\n';
    //std::cout << "retsucceeded: " << retsucceeded << '\n';
    //ROS_INFO(std::to_string(nodenames.size()).c_str());
    /*for (size_t i = 0; i < nodenames.size(); i++) {
    std::cout << nodenames.at(i) << '\n';
    //ROS_INFO(nodenames.at(i).c_str());
  }*/
    //Voronoicbc instance(n);

    return 0;
}
