#ifndef WHEEL_HPP_INCLUDED
#define WHEEL_HPP_INCLUDED

class Wheel{
public:
    Wheel(double, double, double, double, double, double);
    double torque(double, double, double, double);
    void force(double, double, double, double, double&, double&, double&);
private:
    double xPos, yPos;
    double radius;
    double width;
    double mu;
    double normalForce;
};


#endif // WHEEL_HPP_INCLUDED
