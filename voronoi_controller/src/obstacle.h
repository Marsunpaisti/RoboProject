#ifndef _Obstacle_
#define _Obstacle_

#include <boost/shared_ptr.hpp>
#include <vector>

struct Vector2 {
    double x;
    double y;
};

class Obstacle {

public:
    constexpr static double repulsionMultiplier = 0.01;
    constexpr static double distanceThreshold = 1.5;
    double _x;
    double _y;
    double _radius;
    Obstacle(double x, double y, double radius);
    Vector2 getRepulsionVectorAtPoint(double x, double y, double radius);

private:
};

#endif