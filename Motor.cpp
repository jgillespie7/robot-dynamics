#include "Motor.hpp"
#include <math.h>
#include <iostream>
#include <stdexcept>

//initialize motor with stall torque, free speed, and reactions to driving commands
/*For example, if drive is 1 and turn is -0.2, a command to drive forward at 50% (0.5),
 and turn clockwise at 100% will result in a command of:
    throttle = (0.5)*(1) + (-1)*(-0.2) = 0.5 + 0.2 = 0.7 -> 70%
*/
Motor::Motor(double st, double fs, double strafe, double drive, double turn){
    stallTorque = st;
    freeSpeed = fs;
    strafeR = strafe; //Reaction to a strafe right command
    driveF = drive; //Reaction to a drive forward command
    turnCCW = turn; //Reaction to a turn counter-clockwise command
}

//add wheel to motor
void Motor::addWheel(Wheel newWheel){
    wheels.push_back(newWheel);
}

//calculate torque on a motor system for a given wheelspeed and velocity
//this torque must be zero when the wheel-motor system is at steady state
//this is used in Motor::findWheelSpeed to determine the wheelspeed
double Motor::torque(double wheelSpeed, double vX, double vY, double omega,
                     double strafeRight, double driveForward, double turnCounterClockwise){
    //calculate throttle based on driving commands and motor properties
    double throttle = strafeRight*strafeR +
                      driveForward*driveF +
                      turnCounterClockwise*turnCCW;
    //throttle must be between -1 and 1
    if (throttle > 1){
        throttle = 1;
    }
    else if (throttle < -1){
        throttle = -1;
    }
    //calculate torque from the motor
    double torque = (throttle*stallTorque) * (1 - wheelSpeed/(freeSpeed*throttle));
    for(std::vector<Wheel>::iterator it = wheels.begin();
        it != wheels.end(); ++it){
            //add torque from each wheel
            torque += (*it).torque(wheelSpeed, vX, vY, omega);
    }
    return torque;
}

//Solve Motor::torque for zero, using the bisection method
//This finds the wheelspeed at which the torque is zero (the steady-state wheelspeed)
/*Note: this is not a general-purpose bisection method root finder. It assumes that torque is
monotonically decreasing with wheelspeed. If you wish to adapt it to another problem,
you may need to add additional logic to account for this*/
double Motor::findWheelSpeed(double vX, double vY, double omega,
                             double strafeRight, double driveForward, double turnRight){
    //Assume motor is never driven at more than twice its free speed
    double lower = -freeSpeed*2, upper = freeSpeed*2;
    double guess = (lower+upper)/2;
    int i = 0;
    //Iterate until the torque is within 0.001 ft-lbf of zero
    while (fabs(torque(guess, vX, vY, omega, strafeRight, driveForward, turnRight))>0.001){
        i++;
        //Give up after 100 iterations
        if (i>100){
            throw std::runtime_error( "Exceeded maximum iterations");
        }
        //If torque is positive, wheel will speed up, so the guess is too low
        //Set the guess as the lower bound
        if (torque(guess, vX, vY, omega, strafeRight, driveForward, turnRight)>0){
            lower = guess;
        }
        //If torque is negative, wheel will slow down, so the guess is too high
        //Set the guess as the upper bound
        else{
            upper = guess;
        }
        //Pick a guess at the midpoint of the upper and lower bounds
        guess = (lower+upper)/2;
    }
    return guess;
}

//Calculate the forces from the motor
void Motor::force(double vX, double vY, double omega,
                  double strafeRight, double driveForward, double turnCounterClockwise,
                  double& forceX, double& forceY, double& moment){
    try {
        //Get the steady-state wheelspeed
        double wheelSpeed = findWheelSpeed(vX, vY, omega,
                                           strafeRight, driveForward, turnCounterClockwise);
        for(std::vector<Wheel>::iterator it = wheels.begin();
        it != wheels.end(); ++it){
            //add up the forces and moment from each wheel
            (*it).force(wheelSpeed, vX, vY, omega, forceX, forceY, moment);
        }
    } catch ( const std::runtime_error& e ){
    std::cerr << "Encountered runtime error: " << e.what() << '\n';
    }
}
