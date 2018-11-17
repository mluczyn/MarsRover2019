#include <iostream>
#include <cmath>
#include "../include/ik/Roboarm.h"

using namespace std;

Roboarm::Roboarm(float l1, float l2, float l3, float a1, float a2, float a3) :
  lengthLink1(l1), lengthLink2(l2), lengthLink3(l3),
  angleLink1(a1), angleLink2(a2), angleLink3(a3) {

  velocityLink1 = 0;
  velocityLink2 = 0;
  velocityLink3 = 0;
}

void Roboarm::calc_velocities(float endEffectorVX, float endEffectorVY, float endEffectorPhi) {
  float c1 = cos(angleLink1);
  float s1 = sin(angleLink1);

  float c12 = cos(angleLink1 + angleLink2);
  float s12 = sin(angleLink1 + angleLink2);

  float c123 = cos(angleLink1 + angleLink2 + angleLink3);
  float s123 = sin(angleLink1 + angleLink2 + angleLink3);
    
  float l1 = lengthLink1;
  float l2 = lengthLink2;
  float l3 = lengthLink3;

  float eX = endEffectorVX;
  float eY = endEffectorVY;
  float eP = endEffectorPhi;

  velocityLink1 =
      (l2 * c12 * eX)
    + (l2 * s12 * eY)
    + (l2 * l3 * (c12 * s123 - s12 * c123) * eP);
    
  velocityLink2 =
      (-l1 * c1 - l2 * c12) * eX
    - (l1 * s1 + l2 * s12) * eY
    - (l1 * l3 * (c1 * s123 - s1 * c123) + l2 * l3 * (c12 * s123 - s12 * c123)) * eP;

  velocityLink3 =
      (l1 * c1 * eX)
    + (l1 * s1 * eY)
    + (l1 * l2 * (c1 * s12 - s1 * c12) + l1 * l3 * (c1 * s123 - s1 * c123)) * eP;
}

Roboarm::~Roboarm() {}