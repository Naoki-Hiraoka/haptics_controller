#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <limits>
#include <cpp_filters/TwoPointInterpolator.h>
#include <cpp_filters/FirstOrderLowPassFilter.h>
#include <joint_limit_table/JointLimitTable.h>
#include "FootGuidedController.h"

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
public:
  // constant parameter
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Position> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

  std::vector<double> maxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for actRobot.

  const double g = 9.80665; // 重力加速度

public:
  // parameter

  std::vector<bool> jointControllable; // 要素数と順序はnumJoints()と同じ. falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). IKでは動かさない(ref値をそのまま). トルク計算では目標トルクを通常通り計算する. このパラメータはMODE_IDLEのときにしか変更されない

public:
  // from reference port
  cnoid::BodyPtr refRobotRaw; // actual. Model File frame (= generate frame)
  std::vector<cnoid::Vector6> refEEPoseRaw; // 要素数と順序はeeNameと同じ.Reference frame.
  std::vector<cnoid::Vector6> refEEWrenchRaw; // 要素数と順序はeeNameと同じ.Reference frame. EndEffector origin. ロボットが受ける力
  cnoid::BodyPtr actRobot; // actual. Model File frame (= generate frame)
  std::vector<cnoid::Position> actEEPose; // 要素数と順序はeeNameと同じ.generate frame

public:
  // AutoStabilizerの中で計算更新される.

  // refToGenFrameConverter
  std::vector<cnoid::Vector6> refEEWrench; // 要素数と順序はeeNameと同じ. endeffector frame. EndEffector origin. ロボットが受ける力

  // refForceHandler
  std::vector<cnoid::Vector6> rfhTgtEEWrench; // 要素数と順序はeeNameと同じ. endeffector frame. EndEffector origin. ロボットが受ける力

  // workSpaceForceHandler
  std::vector<cnoid::Vector6> wsfhTgtEEWrench; // 要素数と順序はeeNameと同じ. endeffector frame. EndEffector origin. ロボットが受ける力

  // gravityCompensationHandler
  cnoid::VectorX gchTorque; // 要素数と順序はrobot->numJoints()と同じ. ロボットが発揮するトルク
  cnoid::BodyPtr actRobotGch; // (actRobotと同じだが、逆動力学計算に使う)

  // jointAngleLimitHandler
  cnoid::VectorX jalhTorque; // 要素数と順序はrobot->numJoints()と同じ. ロボットが発揮するトルク

  // torqueOutputGenerator
  cnoid::BodyPtr actRobotTqc; // (actRobotと同じだが、uに指令関節トルクが入っている)

public:
  void init(const cnoid::BodyPtr& robot){
    maxTorque.resize(robot->numJoints(), std::numeric_limits<double>::max());
    jointLimitTables.resize(robot->numJoints());
    jointControllable.resize(robot->numJoints(), true);
    refRobotRaw = robot->clone();
    refRobotRaw->calcForwardKinematics(); refRobotRaw->calcCenterOfMass();
    actRobot = robot->clone();
    actRobot->calcForwardKinematics(); actRobot->calcCenterOfMass();
    actRobotGch = robot->clone();
    actRobotGch->calcForwardKinematics(); actRobotGch->calcCenterOfMass();
    actRobotTqc = robot->clone();
    actRobotTqc->calcForwardKinematics(); actRobotTqc->calcCenterOfMass();
    gchTorque = cnoid::VectorX::Zero(robot->numJoints());
    jalhTorque = cnoid::VectorX::Zero(robot->numJoints());
  }

  void push_backEE(const std::string& name_, const std::string& parentLink_, const cnoid::Position& localT_){
    eeName.push_back(name_);
    eeParentLink.push_back(parentLink_);
    eeLocalT.push_back(localT_);
    refEEWrenchRaw.push_back(cnoid::Vector6::Zero());
    refEEPoseRaw.push_back(cnoid::Position::Identity());
    refEEWrench.push_back(cnoid::Vector6::Zero());
    actEEPose.push_back(cnoid::Position::Identity());
    rfhTgtEEWrench.push_back(cnoid::Vector6::Zero());
    wsfhTgtEEWrench.push_back(cnoid::Vector6::Zero());
  }

  // startAutoStabilizer時に呼ばれる
  void reset(){
  }

  // 毎周期呼ばれる. 内部の補間器をdtだけ進める
  void update(double dt){
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // for debug
  mutable struct timeval prevTime;
  void resetTime() const { gettimeofday(&prevTime, NULL);}
  void printTime(const std::string& message="") const {
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    std::cerr << message << (currentTime.tv_sec - prevTime.tv_sec) + (currentTime.tv_usec - prevTime.tv_usec) * 1e-6 << std::endl;
  }
};


#endif
