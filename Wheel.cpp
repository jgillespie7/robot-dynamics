#include "Wheel.hpp"
#include <math.h>

Wheel::Wheel(double x, double y, double r, double w, double cof, double n){
    xPos = x;
    yPos = y;
    radius = r;
    width = w;
    mu = cof;
    normalForce = n;
}

double Wheel::torque(double wheelSpeed, double vX, double vY, double omega){
    vY += omega*xPos;
    vX -= omega*yPos;
    double k = sqrt(pow(vX,2) + pow(vY-radius*wheelSpeed,2));
    if (fabs(k) > 1){
        k = 1;
    }
    double fY = -k*normalForce*mu*(vY - radius*wheelSpeed)/
        sqrt( pow(vX,2) + pow(vY-radius*wheelSpeed,2) );
    return -fY*radius;
}

void Wheel::force(double wheelSpeed, double vX, double vY, double omega,
                  double& forceX, double& forceY, double& moment){
    vY += omega*xPos;
    vX -= omega*yPos;
    double k = sqrt(pow(vX,2) + pow(vY-radius*wheelSpeed,2));
    if (fabs(k) > 1){
        k = 1;
    }
    double fX = -k*normalForce*mu*(vX)/
              sqrt( pow(vX,2) + pow(vY-radius*wheelSpeed,2) );
    double fY = -k*normalForce*mu*(vY - radius*wheelSpeed)/
         sqrt( pow(vX,2) + pow(vY-radius*wheelSpeed,2) );
    forceX += fX;
    forceY += fY;
    moment += fY*xPos - fX*yPos;
}
