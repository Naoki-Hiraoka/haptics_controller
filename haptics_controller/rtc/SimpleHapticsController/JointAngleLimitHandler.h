#ifndef JOINTANGLELIMITHANDLER_H
#define JOINTANGLELIMITHANDLER_H

#include "GaitParam.h"

class JointAngleLimitHandler {
public:
  // GravityCompensationHandlerだけでつかうパラメータ
  double softJointLimitPGain = 100.0;
  double softJointLimitMargin = 15.0 / 180.0 * M_PI; // 0以上

public:
  bool calcTorque(const GaitParam& gaitParam, double dt,// input
                  cnoid::VectorX& o_jalhTorque) const; // output
};

#endif
