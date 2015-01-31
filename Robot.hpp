#ifndef ROBOT_HPP_INCLUDED
#define ROBOT_HPP_INCLUDED

#include <vector>
#include "Motor.hpp"

class Robot{
public:
    Robot(double);
    void addMotor(Motor);
    void acceleration(double, double, double, double, double, double, double&, double&, double&);
private:
    double mass;
    std::vector<Motor> motors;
};

#endif // ROBOT_HPP_INCLUDED
