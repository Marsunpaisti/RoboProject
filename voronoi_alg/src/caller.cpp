/* FortuneAlgorithm
 * Copyright (C) 2018 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <math.h>
// SFML
//#include <SFML/Graphics.hpp>
// My includes
#include "FortuneAlgorithm.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "voronoi_alg/RobotPos.h"

constexpr float WINDOW_WIDTH = 600.0f;
constexpr float WINDOW_HEIGHT = 600.0f;
constexpr float POINT_RADIUS = 0.005f;
constexpr float OFFSET = 1.0f;


void rotateWithMat(float ax, float ay, float costh, float sinth, float *outx, float *outy)
{
  float newx, newy;
  newx = ax*costh - ay*sinth;
  newy = ax*sinth + ay*costh;
  *outx = newx;
  *outy = newy;
}
void cornersToRotangles(std::vector<std::vector<float>> corners, float *costh, float *sinth)
{
  float ax, ay, bx, by, newcth, newsth;
  ax = corners.at(0).at(0);ay = corners.at(0).at(1);
  bx = corners.at(1).at(0);by = corners.at(1).at(1);
  *costh = (by-ay)/sqrt((by-ay)*(by-ay)+(bx-ax)*(bx-ax));
  *sinth = (bx-ax)/sqrt((by-ay)*(by-ay)+(bx-ax)*(bx-ax));
}
std::vector<std::vector<float>> transformRobLocsAndCorners(std::vector<std::vector<float>> corners,
  std::vector<std::vector<float>> robLocs, float *xmin, float *xmax, float *ymin, float *ymax)
{
  //find the transformed robot locations:
  float costh, sinth, tfx, tfy;
  std::vector<float> tfLoc{0,0};
  cornersToRotangles(corners, &costh, &sinth);
  std::vector<std::vector<float>> transformedRobLocs;
  for (size_t i = 0; i < robLocs.size(); i++) {
    rotateWithMat(robLocs.at(i).at(0), robLocs.at(i).at(1), costh, sinth, &tfx, &tfy);
    tfLoc.at(0)=tfx;tfLoc.at(1)=tfy;
    transformedRobLocs.push_back(tfLoc);
  }
  //find the smallest and largest x and y value from the corner vector:
  float tmpXmin, tmpXmax, tmpYmin, tmpYmax;
  for (size_t i = 0; i < corners.size(); i++) {
    rotateWithMat(corners.at(i).at(0), corners.at(i).at(1), costh, sinth, &tfx, &tfy);
    if (i==0) {
      tmpXmin = tfx;
      tmpXmax = tfx;
      tmpYmin = tfy;
      tmpYmax = tfy;
    }
    if (tfx < tmpXmin) {
      tmpXmin=tfx;
    }
    if (tfx > tmpXmax) {
      tmpXmax=tfx;
    }
    if (tfy < tmpYmin) {
      tmpYmin=tfy;
    }
    if (tfy > tmpYmax) {
      tmpYmax=tfy;
    }
  }
  *xmin = tmpXmin;*xmax=tmpXmax;*ymin=tmpYmin;*ymax=tmpYmax;
  return transformedRobLocs;
}
std::vector<std::vector<float>> transformTargetLocs(
  std::vector<std::vector<float>> targetLocs,
  std::vector<std::vector<float>> corners)
{
  float costh, sinth, tfx, tfy;
  std::vector<float> tfLoc{0,0};
  cornersToRotangles(corners, &costh, &sinth);
  std::vector<std::vector<float>> transformedTargetLocs;
  for (size_t i = 0; i < targetLocs.size(); i++) {
    rotateWithMat(targetLocs.at(i).at(0), targetLocs.at(i).at(1), costh, -sinth, &tfx, &tfy);
    tfLoc.at(0)=tfx;tfLoc.at(1)=tfy;
    transformedTargetLocs.push_back(tfLoc);
  }
  return transformedTargetLocs;
}
void computeCentroid(const std::vector<std::vector<float>> vertices, std::vector<float>* centroid)
{
	float centroidX = 0, centroidY = 0;
	float det = 0, tempDet = 0;
	unsigned int j = 0;
	unsigned int nVertices = (unsigned int)vertices.size();

	for (unsigned int i = 0; i < nVertices; i++)
	{
		// closed polygon
		if (i + 1 == nVertices)
			j = 0;
		else
			j = i + 1;

		// compute the determinant
		tempDet = vertices[i][0] * vertices[j][1] - vertices[j][0]*vertices[i][1];
		det += tempDet;

		centroidX += (vertices[i][0] + vertices[j][0])*tempDet;
		centroidY += (vertices[i][1] + vertices[j][1])*tempDet;

		//std::cout << "vertice x: "<< vertices[i][0] << ", vertice y: " << vertices[i][1] << '\n';
	}

	// divide by the total mass of the polygon
	centroidX /= 3*det;
	centroidY /= 3*det;

  std::vector<float> newCentroid {centroidX, centroidY};
	*centroid = newCentroid;
}
void findAllCentroids(std::vector<std::vector<std::vector<float>>> polygons, std::vector<std::vector<float>>* centroids)
{
  std::vector<std::vector<float>> newCentroids;
  std::vector<float> newCentroid;
  for (size_t i=0; i<polygons.size(); i++){
    computeCentroid(polygons.at(i), &newCentroid);
    newCentroids.push_back(newCentroid);
  }
  *centroids = newCentroids;
}
std::vector<Vector2> generatePoints(int nbPoints)
{
    uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::cout << "seed: " << seed << '\n';
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution (0.0, 1.0);

    std::vector<Vector2> points;
    for (int i = 0; i < nbPoints; ++i)
        points.push_back(Vector2{distribution(generator), distribution(generator)});

    return points;
}
float cross(std::vector<float> a, std::vector<float> b)
{
  //cross product: assumes that input vectors have z=0 and computes the z-value
  //of the cross product.
  return a.at(0)*b.at(1)-a.at(1)*b.at(0);
}
bool is_counterclockwise(std::vector<std::vector<float>> points)
{
  //gets a list of points along the edge, determines if the points are in
  //counterclockwise order or not.

  //vectors from first to second and first to third point
  std::vector<float> p12, p13;
  p12.push_back(points.at(1).at(0)-points.at(0).at(0));
  p12.push_back(points.at(1).at(1)-points.at(0).at(1));
  p13.push_back(points.at(2).at(0)-points.at(0).at(0));
  p13.push_back(points.at(2).at(1)-points.at(0).at(1));
  //if the cross product of these vectors is greater than zero, the point 3 must
  //be left of the line from point 1 to point 2.
  return cross(p12,p13)>0;
}
void detectPolygon(VoronoiDiagram& diagram, std::vector<std::vector<std::vector<float>>>* polys)
{
  bool polygonFound = false;
  int polysFound = 0;
  std::vector<float> origvec{0,0}, destvec{0,0};
  std::vector<std::vector<float>> cornerPoints;
  std::vector<std::vector<std::vector<float>>> allPolys;
  for (std::size_t i = 0; i < diagram.getNbSites(); ++i)
  {
    const VoronoiDiagram::Site* site = diagram.getSite(i);
    Vector2 center = site->point;
    VoronoiDiagram::Face* face = site->face;
    VoronoiDiagram::HalfEdge* halfEdge = face->outerComponent;
    if (halfEdge == nullptr)
      continue;
    while (halfEdge->prev != nullptr)
    {
      halfEdge = halfEdge->prev;
      if (halfEdge == face->outerComponent)
        break;
    }
    VoronoiDiagram::HalfEdge* start = halfEdge;
    if (start->origin != nullptr)
    {
      origvec.at(0) = halfEdge->origin->point.x;
      origvec.at(1) = halfEdge->origin->point.y;
      cornerPoints.push_back(origvec);
    }
    while (halfEdge != nullptr)
    {
      if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
      {
        //Vector2 origin = (halfEdge->origin->point - center) * OFFSET + center;
        //Vector2 destination = (halfEdge->destination->point - center) * OFFSET + center;
        origvec.at(0) = halfEdge->origin->point.x;
        origvec.at(1) = halfEdge->origin->point.y;
        destvec.at(0) = halfEdge->destination->point.x;
        destvec.at(1) = halfEdge->destination->point.y;
        float diff_from_prev = (origvec.at(0)-cornerPoints.back().at(0))*
          (origvec.at(0)-cornerPoints.back().at(0)) +
          (origvec.at(1)-cornerPoints.back().at(1))*
          (origvec.at(1)-cornerPoints.back().at(1));
        if (diff_from_prev>0.0001)
          std::cout << "Edges in wrong order!" << '\n';
        cornerPoints.push_back(destvec);
      }
      halfEdge = halfEdge->next;
      if (halfEdge == start){
        polygonFound = true;
        break;
      }
    }
    //std::cout << "distance from start to end: " << diff_from_start << '\n';
    //std::cout << "Number of edges in latest polygon: " << cornerPoints.size() << '\n';
    if (is_counterclockwise(cornerPoints))
    {
      std::cout << "current polygon is counterclockwise" << '\n';
    } else {
      std::cout << "current polygon is clockwise" << '\n';
    }
    allPolys.push_back(cornerPoints);
    cornerPoints.clear();
    if (polygonFound)
      polysFound++;
    polygonFound = false;
    //if (polygonFound)
    //  break;
  }
  std::cout << "number of polygons found: " << polysFound << '\n';
  std::cout << "latest origin: x: " << origvec.at(0) << ", y: " << origvec.at(1) << '\n';
  *polys = allPolys;
}
void countEdges(VoronoiDiagram& diagram)
{
  int edgenumber = 0;
    for (std::size_t i = 0; i < diagram.getNbSites(); ++i)
    {
        const VoronoiDiagram::Site* site = diagram.getSite(i);
        Vector2 center = site->point;
        VoronoiDiagram::Face* face = site->face;
        VoronoiDiagram::HalfEdge* halfEdge = face->outerComponent;
        if (halfEdge == nullptr)
            continue;
        while (halfEdge->prev != nullptr)
        {
            halfEdge = halfEdge->prev;
            if (halfEdge == face->outerComponent)
                break;
        }
        VoronoiDiagram::HalfEdge* start = halfEdge;
        while (halfEdge != nullptr)
        {
            if (halfEdge->origin != nullptr && halfEdge->destination != nullptr)
            {
                Vector2 origin = (halfEdge->origin->point - center) * OFFSET + center;
                Vector2 destination = (halfEdge->destination->point - center) * OFFSET + center;
                edgenumber++;
                //drawEdge(window, origin, destination, sf::Color::Red);
            }
            halfEdge = halfEdge->next;
            if (halfEdge == start)
                break;
        }
    }
    std::cout << "edgenumber: " << edgenumber << std::endl;
}
std::vector<std::vector<float>> vector2toFloats(std::vector<Vector2> points)
{
  std::vector<float> p{0,0};
  std::vector<std::vector<float>> res;
  for (size_t i=0;i<points.size();i++){
    p.at(0)=points.at(i).x;p.at(1)=points.at(i).y;
    res.push_back(p);
  }
  return res;
}
std::vector<Vector2> floatsToVector2(std::vector<std::vector<float>> points)
{
  Vector2 p;
  std::vector<Vector2> res;
  for (size_t i=0;i<points.size();i++){
    p.x=points.at(i).at(0);p.y=points.at(i).at(1);
    res.push_back(p);
  }
  return res;
}
VoronoiDiagram generateRandomDiagram(
	std::vector<std::vector<float>>* generatedPoints,
	 Vector2 bottomLeftCorner, Vector2 topRightCorner)
{
	/*
    // Generate points
    std::vector<Vector2> points = generatePoints(nbPoints);
    // Write generated points to generatedPoints:
    std::vector<std::vector<float>> tmp = vector2toFloats(points);
    *generatedPoints = tmp;*/
		std::vector<Vector2> points = floatsToVector2(*generatedPoints);

    // Construct diagram
    FortuneAlgorithm algorithm(points);
    auto start = std::chrono::steady_clock::now();
    algorithm.construct();
    auto duration = std::chrono::steady_clock::now() - start;
    std::cout << "construction: " <<
		std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() <<
		 "ms" << '\n';

    // Bound the diagram
    start = std::chrono::steady_clock::now();
    algorithm.bound(Box{bottomLeftCorner.x-0.05, bottomLeftCorner.y-0.05,
			topRightCorner.x+0.05, topRightCorner.y+0.05}); // Take the bounding box slightly bigger than the intersection box
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "bounding: " <<
		std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() <<
		 "ms" << '\n';
    VoronoiDiagram diagram = algorithm.getDiagram();

    // Intersect the diagram with a box
    start = std::chrono::steady_clock::now();
    bool valid = diagram.intersect(Box{bottomLeftCorner.x, bottomLeftCorner.y,
			 topRightCorner.x, topRightCorner.y});
    duration = std::chrono::steady_clock::now() - start;
    std::cout << "intersection: " <<
		 std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() <<
		  "ms" << '\n';
    if (!valid)
        throw std::runtime_error("An error occured in the box intersection algorithm");

    return diagram;
}
void printPointsWithCentroids(std::vector<std::vector<float>> points, std::vector<std::vector<float>> centroids)
{
  //orig. and new x and y:
  float ox, oy, nx, ny;
  for (size_t i = 0; i < points.size(); i++) {
    ox = points.at(i).at(0);oy = points.at(i).at(1);
    nx = centroids.at(i).at(0);ny = centroids.at(i).at(1);
    std::cout << "old: <" << ox << "," << oy << ">, new: <" << nx << "," << ny << ">" << '\n';
  }
}
std::vector<std::vector<float>> colVecToRowVec(std::vector<std::vector<float>> colVec)
{
	std::vector<std::vector<float>> rowVec;
	std::vector<float> xRow, yRow;
	for (size_t i = 0; i < colVec.size(); i++) {
		xRow.push_back(colVec.at(i).at(0));
		yRow.push_back(colVec.at(i).at(1));
	}
	rowVec.push_back(xRow);
	rowVec.push_back(yRow);
	return rowVec;
}
std::vector<std::vector<float>> rowVecToColVec(std::vector<std::vector<float>> rowVec)
{
	std::vector<std::vector<float>> colVec;
	std::vector<float> col{0,0};
	for (size_t i = 0; i < rowVec.at(0).size(); i++) {
		col.at(0)=rowVec.at(0).at(i);
		col.at(1)=rowVec.at(1).at(i);
		colVec.push_back(col);
	}
	return colVec;
}

class Voronoicbc
{
public:
    Voronoicbc(const ros::NodeHandle& nh){
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
	    //print result:
	    //printPointsWithCentroids(generatedPoints,centroids);

        /*std::string xcoord_as_str = std::to_string(msg->x.at(0));
        ROS_INFO("I heard: [%s]", xcoord_as_str.c_str());
        if (targ_x.size()==1){
            targ_x.at(0)+=msg->x.at(0);
            targ_y.at(0)+=msg->y.at(0);
        } else {
            targ_x.push_back(0);targ_y.push_back(0);
        }*/
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
    //std::vector<float> targ_x;
    //std::vector<float> targ_y;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;
	Voronoicbc instance(n);
    /*std::size_t nbPoints = 2;
    std::vector<std::vector<float>> centroids;
    // Generate points
    std::vector<Vector2> points = generatePoints(nbPoints);
		Vector2 bottomLeftCorner, topRightCorner;
		bottomLeftCorner.x=-1;bottomLeftCorner.y=-1;
		topRightCorner.x=1;topRightCorner.y=1;
    // Write generated points to generatedPoints:
    std::vector<std::vector<float>> generatedPoints = vector2toFloats(points);

    VoronoiDiagram diagram = generateRandomDiagram(&generatedPoints,
			bottomLeftCorner, topRightCorner);

    std::vector<std::vector<std::vector<float>>> polygons;

    //count the edges:
    countEdges(diagram);
    //detect the first polygon:
    detectPolygon(diagram, &polygons);
    //detect the point centers:
    findAllCentroids(polygons, &centroids);
    //print result:
    printPointsWithCentroids(generatedPoints,centroids);*/

    return 0;
}
