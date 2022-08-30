#ifndef REFFORCEHANDLER_H
#define REFFORCEHANDLER_H

#include "GaitParam.h"
#include <cpp_filters/IIRFilter.h>

class RefForceHandler {
public:
  // RefForceHandlerだけでつかうパラメータ
  double wrenchHpfCutoffHz = 20.0;
  double wrenchLpfCutoffHz = 0.3;
  double wrenchHpfGain = 0.1;
  double wrenchLpfGain = 0.0;

protected:
  const double Q_BUTTERWORTH = 1.0 / std::sqrt(2.0);

  mutable std::vector<cpp_filters::IIRFilter<cnoid::Vector6> > wrenchLpfForHpf;
  mutable std::vector<cpp_filters::IIRFilter<cnoid::Vector6> > wrenchLpf;
public:
  void init(const GaitParam& gaitParam, const double& dt){
    for(int i=0;i<gaitParam.eeName.size();i++){
      wrenchLpfForHpf.push_back(cpp_filters::IIRFilter<cnoid::Vector6>());
      wrenchLpfForHpf.back().setParameterAsBiquad(this->wrenchHpfCutoffHz, Q_BUTTERWORTH, 1.0 / dt, cnoid::Vector6::Zero());
      wrenchLpf.push_back(cpp_filters::IIRFilter<cnoid::Vector6>());
      wrenchLpf.back().setParameterAsBiquad(this->wrenchLpfCutoffHz, Q_BUTTERWORTH, 1.0 / dt, cnoid::Vector6::Zero());
    }
  }

  // startHapticsController時に1回呼ばれる
  void reset(){
    for(int i=0;i<gaitParam.eeName.size();i++){
      wrenchLpfForHpf[i].reset(cnoid::Vector6::Zero());
      wrenchLpf[i].reset(cnoid::Vector6::Zero());
    }
  }

  // refEEWrenchからrfhTgtEEWrenchを計算する
  bool calcWrench(const GaitParam& gaitParam, double dt,// input
                  std::vector<cnoid::Vector6>& o_rfhTgtEEWrench /*endeffector frame. endeffector origin*/) const; // output
};

#endif
