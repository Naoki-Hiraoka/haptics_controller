#include "ActToGenFrameConverter.h"
#include <cnoid/EigenUtil>

bool ActToGenFrameConverter::convertFrame(const GaitParam& gaitParam, double dt,// input
                                          std::vector<cnoid::Position>& o_actEEPose) const {

  std::vector<cnoid::Position> actEEPose(gaitParam.eeName.size(), cnoid::Position::Identity());
  // 各エンドエフェクタのactualの位置・力を計算
  for(int i=0;i<gaitParam.eeName.size(); i++){
    actEEPose[i] = gaitParam.actRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i];
  }

  o_actEEPose = actEEPose;

  return true;
}
