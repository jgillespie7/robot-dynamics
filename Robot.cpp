#include "Robot.hpp"

const double gc = 32.1740486;

Robot::Robot(double m){
    mass = m;
}

void Robot::addMotor(Motor newMotor){
    motors.push_back(newMotor);
}

void Robot::acceleration(double vX, double vY, double omega,
                  double strafeRight, double driveForward, double turnCCW,
                  double& aX, double& aY, double& alpha){
    double forceX = 0;
    double forceY = 0;
    double moment = 0;
    for(std::vector<Motor>::iterator it = motors.begin();
        it != motors.end(); ++it){
            (*it).force(vX, vY, omega, strafeRight, driveForward, turnCCW, forceX, forceY, moment);
    }
    aX = gc * forceX / mass;
    aY = gc * forceY / mass;
    alpha = gc * moment / mass;
}
