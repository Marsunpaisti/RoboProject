#include "pidcontroller.h"
#include "ros/ros.h"

PIDController::PIDController(double sampleTime, double min, double max, double Kp, double Kd, double Ki, bool isAngleController)
    : _sampleTime(sampleTime)
    , _max(max)
    , _min(min)
    , _Kp(Kp)
    , _Kd(Kd)
    , _Ki(Ki)
    , _isAngleController(isAngleController)
    , _prevError(0)
    , _isFirstLoop(true)
{
}

PIDController::~PIDController()
{
}

double PIDController::calculate(double setpoint, double measurement)
{
    double error = setpoint - measurement;
    if (_isAngleController) { // Avoid gimbal lock
        if (error > 3.1415926535) {
            error -= 3.1415926535 * 2;
        } else if (error < -3.1415926535) {
            error += 3.1415926535 * 2;
        }
    }

    _integral += error * _sampleTime * _Ki;

    if (_isFirstLoop) { // _prevError is initialized to 0 so on first loop it might cause a huge spike in D term unless ignored
        _prevError = error;
    }
    double derivative = (_prevError - error) / _sampleTime;
    if (_isAngleController) { // Gimbal lock must be avoided here as well
        derivative = (_prevError - error);
        if (derivative > 3.1415926535) {
            derivative -= 3.1415926535 * 2;
        } else if (derivative < -3.1415926535) {
            derivative += 3.1415926535 * 2;
        }
        derivative = derivative / _sampleTime;
    }
    double dTerm = derivative * _Kd;
    double pTerm = error * _Kp;
    double output = pTerm + dTerm + _integral;

    //ROS_INFO("SET: %f MEAS: %f P: %f D: %f I: %f", setpoint, measurement, pTerm, dTerm, _integral);
    if (output > _max) { // Clamp output and conditionally integrate as anti windup
        output = _max;
        _integral -= error * _sampleTime * _Ki;
    } else if (output < _min) {
        output = _min;
        _integral += error * _sampleTime * _Ki;
    }

    _prevError = error;
    _isFirstLoop
        = false;
    return output;
}
