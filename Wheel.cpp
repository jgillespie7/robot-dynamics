#include "Wheel.hpp"
#include <math.h>

//Initialize wheel
Wheel::Wheel(double x, double y, double r, double w, double cof, double n){
    xPos = x; //Position of wheel relative to center of mass. Right is positive
    yPos = y; //Position of wheel relative to COM. Further forward is positive
    radius = r; //Radius of wheel
    width = w; //Wheel width
    mu = cof; //Wheel coefficient of friction
    normalForce = n; //Normal force on the wheel
}

//Calculate the torque exerted on the wheel (reaction from longitudinal force)
double Wheel::torque(double wheelSpeed, double vX, double vY, double omega){
    vY += omega*xPos;
    vX -= omega*yPos;
    //Force is initially proportional to slip
    double k = sqrt(pow(vX,2) + pow(vY-radius*wheelSpeed,2));
    /*Assume force reaches a maximum at a slip of 1 ft/s
    and is constant thereafter*/
    if (fabs(k) > 1){
        k = 1;
    }
    //If there is no slip, force and torque are zero
    //Avoid dividing by zero
    if (k == 0){
        return 0;
    }
    //Calculate the force in the longitudinal direction
    double fY = -k*normalForce*mu*(vY - radius*wheelSpeed)/
        sqrt( pow(vX,2) + pow(vY-radius*wheelSpeed,2) );
    //Calculate and return the reaction torque
    return -fY*radius;
}

/*Calculate forces in longitudinal and lateral directions and the moment
on the wheel*/
void Wheel::force(double wheelSpeed, double vX, double vY, double omega,
                  double& forceX, double& forceY, double& moment){
    vY += omega*xPos;
    vX -= omega*yPos;
    //Force is initially proportional to slip
    double k = sqrt(pow(vX,2) + pow(vY-radius*wheelSpeed,2));
    /*Assume force reaches a maximum at a slip of 1 ft/s
    and is constant thereafter*/
    if (fabs(k) > 1){
        k = 1;
    }
    //Only calculate forces if there is slip
    if (k != 0){
            double fX = -k*normalForce*mu*(vX)/
                        sqrt( pow(vX,2) + pow(vY-radius*wheelSpeed,2) );
            double fY = -k*normalForce*mu*(vY - radius*wheelSpeed)/
                        sqrt( pow(vX,2) + pow(vY-radius*wheelSpeed,2) );
            forceX += fX;
            forceY += fY;
            //moment is counterclockwise about origin
            moment += fY*xPos - fX*yPos;
    }
}
