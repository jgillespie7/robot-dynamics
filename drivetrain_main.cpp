#include <iostream>
#include <fstream>
#include <math.h>
#include "Robot.hpp"

int main(){
    // Create an example robot, 140 lb mass
    Robot robot(140);
    // Left side motors+gearbox
    Motor left(50, 46, 0, 1, -1);
    /* Left side wheels, located 1 foot to left of CG
    and 1 ft front and rear of CG, 6 inch diameter,
    2 inch width, 1.2 CoF, 35 lbf normal force on each
    */
    Wheel lf(-1, 1, .5, .166667, 1.2, 35);
    Wheel lr(-1, -1, .5, .166667, 1.2, 35);
    // Attach left side wheels to motor
    left.addWheel(lf);
    left.addWheel(lr);
    // Right side
    Motor right(50, 46, 0, 1, 1);
    Wheel rf(1, 1, .5, .166667, 1.2, 35);
    Wheel rr(1, -1, .5, .166667, 1.2, 35);
    right.addWheel(rf);
    right.addWheel(rr);
    // Attach left and right motors to robot
    robot.addMotor(left);
    robot.addMotor(right);
    double dt = .01;
    double t = 0;
    double ay, ax, alpha;
    double vy=.01, vx=0, w=0;
    std::ofstream output;
    output.open ("output.dat");
    do {
        robot.acceleration(vx, vy, w, 0, 1, 0, ax, ay, alpha);
        output << t << '\t';
        output << vx << '\t' << vy << '\t' << w << '\t';
        output << ax << '\t' << ay << '\t' << alpha << '\n';
        t += dt;
        vx += dt*ax;
        vy += dt*ay;
        w += dt*alpha;
    } while (fabs(ax)>1 || fabs(ay)>1 || fabs(alpha)>1);
    output.close();
    return 0;
}
