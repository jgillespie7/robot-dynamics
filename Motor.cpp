#include "Motor.hpp"
#include <math.h>
#include <iostream>
#include <stdexcept>

Motor::Motor(double st, double fs, double strafe, double drive, double turn){
    strafeR = strafe;
    driveF = drive;
    turnCCW = turn;
    stallTorque = st;
    freeSpeed = fs;
}

void Motor::addWheel(Wheel newWheel){
    wheels.push_back(newWheel);
}

double Motor::torque(double wheelSpeed, double vX, double vY, double omega,
                     double strafeRight, double driveForward, double turnCounterClockwise){
    double throttle = strafeRight*strafeR +
                      driveForward*driveF +
                      turnCounterClockwise*turnCCW;
    if (throttle > 1){
        throttle = 1;
    }
    else if (throttle < -1){
        throttle = -1;
    }
    double torque = (throttle*stallTorque) * (1 - wheelSpeed/(freeSpeed*throttle));
    for(std::vector<Wheel>::iterator it = wheels.begin();
        it != wheels.end(); ++it){
            torque += (*it).torque(wheelSpeed, vX, vY, omega);
    }
    return torque;
}

void Motor::force(double vX, double vY, double omega,
                  double strafeRight, double driveForward, double turnCounterClockwise,
                  double& forceX, double& forceY, double& moment){
    try {
        double wheelSpeed = findWheelSpeed(vX, vY, omega,
                                           strafeRight, driveForward, turnCounterClockwise);
        for(std::vector<Wheel>::iterator it = wheels.begin();
        it != wheels.end(); ++it){
            (*it).force(wheelSpeed, vX, vY, omega, forceX, forceY, moment);
        }
    } catch ( const std::runtime_error& e ){
    std::cerr << "Encountered runtime error: " << e.what() << '\n';
    }
}

double Motor::findWheelSpeed(double vX, double vY, double omega,
                             double strafeRight, double driveForward, double turnRight){
    double lower = -freeSpeed*2, upper = freeSpeed*2;
    double guess = (lower+upper)/2;
    int i = 0;
    while (fabs(torque(guess, vX, vY, omega, strafeRight, driveForward, turnRight))>0.001){
        i++;
        if (i>100){
            throw std::runtime_error( "Exceeded maximum iterations");
        }
        if (torque(guess, vX, vY, omega, strafeRight, driveForward, turnRight)>0){
            lower = guess;
        }
        else{
            upper = guess;
        }
        guess = (lower+upper)/2;
/*        std::cout << lower << '\t' << upper
            << '\t' << torque(guess, vX, vY, omega) << '\n';*/
    }
    return guess;
}
