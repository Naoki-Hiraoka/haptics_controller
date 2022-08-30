#ifndef WORKSPACEFORCEHANDLER_H
#define WORKSPACEFORCEHANDLER_H

#include "GaitParam.h"

class WorkSpaceForceHandler {
public:
  // WorkSpaceForceHandlerだけでつかうパラメータ
  double floorHeight = 0.7; // generate frame. [m]. isHcRunning()中は変更されない
  double floorPGain = 10000.0; // 0以上
  double floorDGain = 500.0; // 0以上

  std::vector<std::vector<cnoid::Vector3> > legShape; // 要素数と順序はeeNameと同じ. endeffector frame. 単位[m]. isHcRunning()中は変更されない
protected:
  mutable cpp_filters::TwoPointInterpolator<double> currentFloorHeight = cpp_filters::TwoPointInterpolator<double>(0.0,0.0,0.0, cpp_filters::HOFFARBIB)
public:
  void init(const GaitParam& gaitParam){
    for(int i=0;i<gaitParam.eeName.size();i++){
      legShape.push_back(std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.105,0.065,0.0),cnoid::Vector3(-0.105,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}):
    }
  }

  // startAutoBalancer時に一回呼ばれる
  double reset(const GaitParam& gaitParam){
    // currentFloorHeight.reset(0.0);
    // currentFloorHeight.setGoal(floorHeight, 5.0);
  }

  // refEEWrenchからrfhTgtEEWrenchを計算する
  bool calcWrench(const GaitParam& gaitParam, double dt,// input
                  std::vector<cnoid::Vector6>& o_wsfhTgtEEWrench /*endeffector frame. endeffector origin*/) const; // output
};

#endif
