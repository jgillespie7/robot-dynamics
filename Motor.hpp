#ifndef MOTOR_HPP_INCLUDED
#define MOTOR_HPP_INCLUDED

#include <vector>
#include "Wheel.hpp"

class Motor{
public:
    Motor(double, double, double, double, double);
    void addWheel(Wheel);
    double torque(double, double, double, double, double, double, double);
    void force(double, double, double, double, double, double, double&, double&, double&);
    double findWheelSpeed(double, double, double, double, double, double);
private:
    double stallTorque;
    double freeSpeed;
    double strafeR, driveF, turnCCW;
    std::vector<Wheel> wheels;
};


#endif // MOTOR_HPP_INCLUDED
