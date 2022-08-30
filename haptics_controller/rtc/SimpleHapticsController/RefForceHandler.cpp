#include "RefForceHandler.h"

// refEEWrenchからrfhTgtEEWrenchを計算する
bool RefForceHandler::calcWrench(const GaitParam& gaitParam, double dt,// input
                                 std::vector<cnoid::Vector6>& o_rfhTgtEEWrench /*endeffector frame. endeffector origin*/) const{ // output

  return true;
}
