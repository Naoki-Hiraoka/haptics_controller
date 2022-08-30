#ifndef SimpleHapticsController_H
#define SimpleHapticsController_H

#include <memory>
#include <map>
#include <time.h>
#include <mutex>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>

#include <cnoid/Body>

#include <cpp_filters/TwoPointInterpolator.h>

#include "SimpleHapticsControllerService_impl.h"
#include "GaitParam.h"
#include "RefToGenFrameConverter.h"
#include "RefForceHandler.h"
#include "WorkSpaceForceHandler.h"
#include "GravityCompensationHandler.h"
#include "JointAngleLimitHandler.h"
#include "TorqueOutputGenerator.h"

class SimpleHapticsController : public RTC::DataFlowComponentBase{
public:
  SimpleHapticsController(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onFinalize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool startHapticsController();
  bool stopHapticsController();
  bool setHapticsControllerParam(const OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam& i_param);
  bool getHapticsControllerParam(OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam& i_param);

protected:
  std::mutex mutex_;

  unsigned int debugLevel_;
  unsigned long long loop_;
  double dt_;

  class Ports {
  public:
    Ports();

    RTC::TimedDoubleSeq m_refTau_;
    RTC::InPort<RTC::TimedDoubleSeq> m_refTauIn_;
    std::vector<RTC::TimedDoubleSeq> m_refEEWrench_; // Reference World frame. EndEffector origin. 要素数及び順番はgaitParam_.eeNameと同じ
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedDoubleSeq> > > m_refEEWrenchIn_;
    std::vector<RTC::TimedPose3D> m_refEEPose_; // Reference World frame. 要素数及び順番はgaitParam_.eeNameと同じ
    std::vector<std::unique_ptr<RTC::InPort<RTC::TimedPose3D> > > m_refEEPoseIn_;
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;

    RTC::TimedDoubleSeq m_genTau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_genTauOut_;
    RTC::TimedPose3D m_genBasePose_; // Generate World frame
    RTC::OutPort<RTC::TimedPose3D> m_genBasePoseOut_;
    RTC::TimedDoubleSeq m_genBaseTform_;  // Generate World frame
    RTC::OutPort<RTC::TimedDoubleSeq> m_genBaseTformOut_; // for HrpsysSeqStateROSBridge

    SimpleHapticsControllerService_impl m_service0_;
    RTC::CorbaPort m_SimpleHapticsControllerServicePort_;

    // only for log
    RTC::TimedPoint3D m_genBasePos_; // Generate World frame
    RTC::OutPort<RTC::TimedPoint3D> m_genBasePosOut_; // for log
    RTC::TimedOrientation3D m_genBaseRpy_; // Generate World frame
    RTC::OutPort<RTC::TimedOrientation3D> m_genBaseRpyOut_; // for log
    std::vector<RTC::TimedPose3D> m_actEEPose_; // Generate World frame. 要素数及び順番はgaitParam_.eeNameと同じ
    std::vector<std::unique_ptr<RTC::OutPort<RTC::TimedPose3D> > > m_actEEPoseOut_;
  };
  Ports ports_;

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startHapcicsController() -> MODE_SYNC_TO_HC -> MODE_HC -> stopHapctisController() -> MODE_SYNC_TO_IDLE -> MODE_IDLE
      MODE_SYNC_TO*の時間はtransition_timeの時間をかけて遷移するが、少なくとも1周期はMODE_SYNC_TO*を経由する.
      MODE_SYNC_TO*では、基本的に次のMODEと同じ処理が行われるが、出力時に前回のMODEの出力から補間するような軌道に加工されることで出力の連続性を確保する
      補間している途中で別のmodeに切り替わることは無いので、そこは安心してプログラムを書いてよい(例外はonActivated). 同様に、remainTimeが突然減ったり増えたりすることもない
     */
    enum Mode_enum{ MODE_IDLE, MODE_SYNC_TO_HC, MODE_HC, MODE_SYNC_TO_IDLE};
    enum Transition_enum{ START_HC, STOP_HC};
    double hc_start_transition_time, hc_stop_transition_time;
  private:
    Mode_enum current, previous, next;
    double remain_time;
  public:
    ControlMode(){ reset(); hc_start_transition_time = 2.0; hc_stop_transition_time = 2.0;}
    void reset(){ current = previous = next = MODE_IDLE; remain_time = 0;}
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_HC:
        if(current == MODE_IDLE){ next = MODE_SYNC_TO_HC; return true; }else{ return false; }
      case STOP_HC:
        if(current == MODE_HC){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(double dt){
      if(current != next) {
        previous = current; current = next;
        switch(current){
        case MODE_SYNC_TO_HC:
          remain_time = hc_start_transition_time; break;
        case MODE_SYNC_TO_IDLE:
          remain_time = hc_stop_transition_time; break;
        default:
          break;
        }
      }else{
        previous = current;
        remain_time -= dt;
        if(remain_time <= 0.0){
          remain_time = 0.0;
          switch(current){
          case MODE_SYNC_TO_HC:
            current = next = MODE_HC; break;
          case MODE_SYNC_TO_IDLE:
            current = next = MODE_IDLE; break;
          default:
            break;
          }
        }
      }
    }
    double remainTime() const{ return remain_time;}
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isHCRunning() const{ return (current==MODE_SYNC_TO_HC) || (current==MODE_HC) ;}
    bool isSyncToHC() const{ return current==MODE_SYNC_TO_HC;}
    bool isSyncToHCInit() const{ return (current != previous) && isSyncToHC();}
    bool isSyncToIdle() const{ return current==MODE_SYNC_TO_IDLE;}
    bool isSyncToIdleInit() const{ return (current != previous) && isSyncToIdle();}
  };
  ControlMode mode_;
  cpp_filters::TwoPointInterpolator<double> idleToHcTransitionInterpolator_ = cpp_filters::TwoPointInterpolator<double>(0.0, 0.0, 0.0, cpp_filters::LINEAR);

  GaitParam gaitParam_;

  RefToGenFrameConverter refToGenFrameConverter_;
  RefForceHandler refForceHandler_;
  WorkSpaceForceHandler workSpaceForceHandler_;
  GravityCompensationHandler gravityCompensationHandler_;
  JointAngleLimitHandler jointAngleLimitHandler_;
  TorqueOutputGenerator torqueOutputGenerator_;

protected:
  // utility functions
  bool getProperty(const std::string& key, std::string& ret);

  static bool readInPortData(const GaitParam& gaitParam, SimpleHapticsController::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobot, std::vector<cnoid::Vector6>& refEEWrenchRaw, std::vector<cnoid::Position>& refEEPoseRaw, std::vector<cnoid::Position>& actEEPose);
  static bool execSimpleHapticsController(const SimpleHapticsController::ControlMode& mode, GaitParam& gaitParam, double dt, const RefToGenFrameConverter& refToGenFrameConverter, const RefForceHandler& refForceHandler, const WorkSpaceForceHandler& workSpaceForceHandler, const GravityCompensationHandler& gravityCompensationHandler, const JointAngleLimitHandler& jointAngleLimitHandler, const TorqueOutputGenerator& torqueOutputGenerator);
  static bool writeOutPortData(SimpleHapticsController::Ports& ports, const SimpleHapticsController::ControlMode& mode, cpp_filters::TwoPointInterpolator<double>& idleToAbcTransitionInterpolator, double dt, const GaitParam& gaitParam);
};

extern "C"
{
  void SimpleHapticsControllerInit(RTC::Manager* manager);
};

#endif // SimpleHapticsController_H
