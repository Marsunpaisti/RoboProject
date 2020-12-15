#ifndef _PIDController_
#define _PIDController_
class PIDController {

public:
    PIDController(double sampleTime, double min, double max, double Kp, double Kd, double Ki, bool isAngleController);
    double calculate(double setpoint, double measurement);
    ~PIDController();

private:
    double _sampleTime;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _prevMeasurement;
    double _integral;
    bool _isAngleController;
    bool _isFirstLoop = true;
};

#endif