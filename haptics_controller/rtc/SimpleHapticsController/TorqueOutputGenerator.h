#ifndef TORQUEOUTPUTGENERATOR_H
#define TORQUEOUTPUTGENERATOR_H

#include "GaitParam.h"

class TorqueOutputGenerator {
public:
  // GravityCompensationHandlerだけでつかうパラメータ
  cnoid::VectorX maxTorque; // 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上

public:
  void init(const GaitParam& gaitParam){
    maxTorque = gaitParam.maxTorque;
  }
  bool calcTorque(const GaitParam& gaitParam,// input
                  cnoid::BodyPtr& actRobotTqc) const; // output
};

#endif
