#include "RefToGenFrameConverter.h"

bool RefToGenFrameConverter::convertFrame(const GaitParam& gaitParam, double dt,// input
                                          std::vector<cnoid::Vector6>& o_refEEWrench /*endeffector frame. endeffector origin*/) const{ // output

  std::vector<cnoid::Vector6> refEEWrench(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    cnoid::Position refEEPoseRaw = gaitParam.refEEPoseRaw[i]; // reference frame
    cnoid::Vector6 refEEWrenchRaw = gaitParam.refEEWrenchRaw[i]; // reference frame. endeffector origin
    refEEWrench[i].head<3>() = refEEPoseRaw.linear().transpose() * refEEWrenchRaw.head<3>();
    refEEWrench[i].tail<3>() = refEEPoseRaw.linear().transpose() * refEEWrenchRaw.tail<3>();
  }

  o_refEEWrench = refEEWrench;

  return true;
}

