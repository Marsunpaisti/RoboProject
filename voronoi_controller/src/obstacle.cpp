#include "obstacle.h"
#include <math.h>

Obstacle::Obstacle(double x, double y, double radius)
    : _x(x)
    , _y(y)
    , _radius(radius)
{
}

Vector2 Obstacle::getRepulsionVectorAtPoint(double x, double y, double radius)
{
    // Calculate repulsion vector based on distance from circle with radius at x,y and return it
    // Repulsion vector magnitude scales down proportional to distance squared
    // Behaving like inverse gravitational force

    double dx = x - _x;
    double dy = y - _y;
    double vecDistance = sqrt(dx * dx + dy * dy);
    double radiusDistance = vecDistance - _radius - radius;
    double force = repulsionMultiplier / (radiusDistance);

    if (radiusDistance > distanceThreshold) {
        Vector2 res;
        res.x = 0;
        res.y = 0;
        return res;
    }

    double unitVecX = dx / vecDistance;
    double unitVecY = dy / vecDistance;
    Vector2 res;
    res.x = unitVecX * force;
    res.y = unitVecY * force;
    return res;
}