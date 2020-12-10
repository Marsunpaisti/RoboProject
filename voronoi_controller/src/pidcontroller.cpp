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
    , _prevMeasurement(0)
    , _isFirstLoop(true)
    , _integral(0)
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
        _prevMeasurement = measurement;
    }
    double derivative = (measurement - _prevMeasurement) / _sampleTime;
    if (_isAngleController) { // Gimbal lock must be avoided here as well
        derivative = (measurement - _prevMeasurement);
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

    if (_isAngleController) {
        ROS_INFO("SET: %f MEAS: %f P: %f D: %f I: %f", setpoint, measurement, pTerm, dTerm, _integral);
    }
    if (output > _max) { // Clamp output and conditionally integrate as anti windup
        output = _max;
        _integral -= error * _sampleTime * _Ki;
    } else if (output < _min) {
        output = _min;
        _integral += error * _sampleTime * _Ki;
    }

    _prevMeasurement = measurement;
    _isFirstLoop
        = false;
    return output;
}
