#include "Robot.hpp"

const double gc = 32.1740486; //gravitational constant

//initialize robot
Robot::Robot(double m){
    mass = m;
}

//add motor to robot
void Robot::addMotor(Motor newMotor){
    motors.push_back(newMotor);
}

//calculate acceleration for robot for given velocity, rotation rate, and drive/turn commands
void Robot::acceleration(double vX, double vY, double omega,
                  double strafeRight, double driveForward, double turnCCW,
                  double& aX, double& aY, double& alpha){
    double forceX = 0;
    double forceY = 0;
    double moment = 0;
    for(std::vector<Motor>::iterator it = motors.begin();
        it != motors.end(); ++it){
            //add the forces and moment from each motor (and its attached wheels)
            (*it).force(vX, vY, omega, strafeRight, driveForward, turnCCW, forceX, forceY, moment);
    }
    //calculate accelerations from forces and moment
    aX = gc * forceX / mass;
    aY = gc * forceY / mass;
    alpha = gc * moment / mass;
}
