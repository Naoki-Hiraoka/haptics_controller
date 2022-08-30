#ifndef GRAVITYCOMPENSATIONHANDLER_H
#define GRAVITYCOMPENSATIONHANDLER_H

#include "GaitParam.h"

class GravityCompensationHandler {
public:
  // GravityCompensationHandlerだけでつかうパラメータ
  double gravityCompensationRatio = 1.0;

public:
  bool calcTorque(const GaitParam& gaitParam, double dt,// input
                  cnoid::BodyPtr& actRobotGch, cnoid::VectorX& o_gchTorque) const; // output
};

#endif
