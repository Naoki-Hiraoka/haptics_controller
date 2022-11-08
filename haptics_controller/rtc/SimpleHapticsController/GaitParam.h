#ifndef GAITPARAM_H
#define GAITPARAM_H

#include <sys/time.h>
#include <cnoid/EigenTypes>
#include <vector>
#include <limits>
#include <cpp_filters/TwoPointInterpolator.h>
#include <joint_limit_table/JointLimitTable.h>
#include <cnoid/Jacobian>

enum leg_enum{RLEG=0, LLEG=1, NUM_LEGS=2};

class GaitParam {
public:
  // constant parameter
  std::vector<std::string> eeName; // constant. 要素数2以上. 0番目がrleg, 1番目がllegという名前である必要がある
  std::vector<std::string> eeParentLink; // constant. 要素数と順序はeeNameと同じ. 必ずrobot->link(parentLink)がnullptrではないことを約束する. そのため、毎回robot->link(parentLink)がnullptrかをチェックしなくても良い
  std::vector<cnoid::Position> eeLocalT; // constant. 要素数と順序はeeNameと同じ. Parent Link Frame

  cnoid::VectorX maxTorque; // constant. 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上
  std::vector<std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > > jointLimitTables; // constant. 要素数と順序はnumJoints()と同じ. for actRobot.

  const double g = 9.80665; // 重力加速度

public:
  // parameter
  std::vector<cpp_filters::TwoPointInterpolator<double> > softMaxTorque; // 要素数と順序はnumJoints()と同じ. 単位は[Nm]. 0以上. refWrench以外のtorqueの上限
  std::vector<bool> jointControllable; // 要素数と順序はnumJoints()と同じ. falseの場合、qやtauはrefの値をそのまま出力する(writeOutputPort時にref値で上書き). IKでは動かさない(ref値をそのまま). トルク計算では目標トルクを通常通り計算する. このパラメータはMODE_IDLEのときにしか変更されない

  std::vector<std::vector<cnoid::Vector3> > eeVertices; // 要素数と順序はeeNameと同じ. endeffector座標系. このverticesがVirtual Work Space内に留まるように力が発生する

  double softJointAngleLimitMargin = 15.0 / 180.0 * M_PI; // [rad]. 0以上. 上下限にこの値よりも近づくと、jointanglelimit力が発生
  double jointAngleLimitGain = 100.0; // [N/rad]. 0以上

  double initialFloorHeight = 0.0; // [m]. generate frame
  double floorHeight = 0.6; // [m]. generate frame.
  double floorPGain = 2500.0;
  double floorDGain = 125.0;

  double qRefPGain = 0.0;
  double qRefDGain = 0.0;

public:
  // from reference port
  cnoid::BodyPtr refRobot; // actual. Model File frame (= generate frame)
  std::vector<cnoid::Position> refEEPose; // 要素数と順序はeeNameと同じ.Reference frame.
  std::vector<cnoid::Vector6> refEEWrench; // 要素数と順序はeeNameと同じ.Reference frame. EndEffector origin. 搭乗者が受ける力(ロボットが発揮する力)
  cnoid::BodyPtr actRobot; // actual. Model File frame (= generate frame)
  std::vector<cnoid::Position> actEEPose; // 要素数と順序はeeNameと同じ.generate frame
  std::vector<cnoid::Vector6> actEEVel; // 要素数と順序はeeNameと同じ.generate frame. endeffector origin
  std::vector<cnoid::JointPathPtr> actJointPath; // 要素数と順序はeeNameと同じ. actRobotに対応

public:
  // AutoStabilizerの中で計算更新される.

  cnoid::BodyPtr actRobotGch; // actRobotと同じだが、逆動力学計算に使う
  cpp_filters::TwoPointInterpolator<double> currentFloorHeight = cpp_filters::TwoPointInterpolator<double>(0.0,0.0,0.0,cpp_filters::HOFFARBIB); // [m]. generate frame.  滑らかに遷移する

  cnoid::BodyPtr actRobotTqc; // actRobotと同じだが、uに指令関節トルクが入っている

public:
  void init(const cnoid::BodyPtr& robot){
    maxTorque = cnoid::VectorX::Ones(robot->numJoints()) * std::numeric_limits<double>::max();
    jointLimitTables.resize(robot->numJoints());
    softMaxTorque.resize(robot->numJoints(), cpp_filters::TwoPointInterpolator<double>(std::numeric_limits<double>::max(),0.0,0.0,cpp_filters::HOFFARBIB));
    jointControllable.resize(robot->numJoints(), true);
    refRobot = robot->clone();
    refRobot->calcForwardKinematics(); refRobot->calcCenterOfMass();
    actRobot = robot->clone();
    actRobot->calcForwardKinematics(); actRobot->calcCenterOfMass();
    actRobotGch = robot->clone();
    actRobotGch->calcForwardKinematics(); actRobotGch->calcCenterOfMass();
    actRobotTqc = robot->clone();
    actRobotTqc->calcForwardKinematics(); actRobotTqc->calcCenterOfMass();
  }

  void push_backEE(const std::string& name_, const std::string& parentLink_, const cnoid::Position& localT_){
    eeName.push_back(name_);
    eeParentLink.push_back(parentLink_);
    eeLocalT.push_back(localT_);
    refEEWrench.push_back(cnoid::Vector6::Zero());
    refEEPose.push_back(cnoid::Position::Identity());
    refEEWrench.push_back(cnoid::Vector6::Zero());
    actEEPose.push_back(cnoid::Position::Identity());
    actEEVel.push_back(cnoid::Vector6::Zero());
    actJointPath.push_back(cnoid::JointPathPtr(new cnoid::JointPath(actRobot->rootLink(), actRobot->link(parentLink_))));
    eeVertices.push_back(std::vector<cnoid::Vector3>{cnoid::Vector3(0.1,0.05,0.0),cnoid::Vector3(-0.1,0.05,0.0),cnoid::Vector3(-0.1,-0.05,0.0),cnoid::Vector3(0.1,-0.05,0.0)});
  }

  // startHapcictController時に呼ばれる
  void reset(){
    currentFloorHeight.reset(this->initialFloorHeight);
    currentFloorHeight.setGoal(this->floorHeight, 10.0);
    for(int i=0;i<softMaxTorque.size();i++) softMaxTorque[i].reset(softMaxTorque[i].getGoal());
  }
  // 毎周期呼ばれる. 内部の補間器をdtだけ進める
  void update(double dt){
    currentFloorHeight.interpolate(dt);
    for(int i=0;i<softMaxTorque.size();i++) softMaxTorque[i].interpolate(dt);
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
