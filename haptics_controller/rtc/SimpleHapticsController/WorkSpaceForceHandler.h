#ifndef WORKSPACEFORCEHANDLER_H
#define WORKSPACEFORCEHANDLER_H

#include "GaitParam.h"

class WorkSpaceForceHandler {
public:
  // WorkSpaceForceHandlerだけでつかうパラメータ
  double floor_height = 0.7; // generate frame. [m]
  double floorPGain = 10000.0; // 0以上
  double floorDGain = 500.0; // 0以上

  std::vector<std::vector<cnoid::Vector3> > legShape; // 要素数と順序はeeNameと同じ. endeffector frame. 単位[m]
protected:

public:
  void init(const GaitParam& gaitParam){
    for(int i=0;i<gaitParam.eeName.size();i++){
      legShape.push_back(std::vector<cnoid::Vector3>{cnoid::Vector3(0.115,0.065,0.0),cnoid::Vector3(-0.105,0.065,0.0),cnoid::Vector3(-0.105,-0.065,0.0),cnoid::Vector3(0.115,-0.065,0.0)}):
    }
  }


  // refEEWrenchからrfhTgtEEWrenchを計算する
  bool convertFrame(const GaitParam& gaitParam, double dt,// input
                    std::vector<cnoid::Vector6>& o_rfhTgtEEWrench /*endeffector frame. endeffector origin*/) const; // output
};

#endif
