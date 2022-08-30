#include "SimpleHapticsController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
#include <limits>

static const char* SimpleHapticsController_spec[] = {
  "implementation_id", "SimpleHapticsController",
  "type_name",         "SimpleHapticsController",
  "description",       "SimpleHapticsController component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

SimpleHapticsController::Ports::Ports() :
  m_refTauIn_("refTauIn", m_refTau_),
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),

  m_genTauOut_("genTauOut", m_genTau_),
  m_genBasePoseOut_("genBasePoseOut", m_genBasePose_),
  m_genBaseTformOut_("genBaseTformOut", m_genBaseTform_),

  m_genBasePosOut_("genBasePosOut", m_genBasePos_),
  m_genBaseRpyOut_("genBaseRpyOut", m_genBaseRpy_),

  m_SimpleHapticsControllerServicePort_("SimpleHapticsControllerService"){
}

SimpleHapticsController::SimpleHapticsController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t SimpleHapticsController::onInitialize(){

  // add ports
  this->addInPort("refTauIn", this->ports_.m_refTauIn_);
  this->addInPort("qAct", this->ports_.m_qActIn_);
  this->addInPort("dqAct", this->ports_.m_dqActIn_);
  this->addOutPort("genTauOut", this->ports_.m_genTauOut_);
  this->addOutPort("genBasePoseOut", this->ports_.m_genBasePoseOut_);
  this->addOutPort("genBaseTformOut", this->ports_.m_genBaseTformOut_);
  this->addOutPort("genBasePosOut", this->ports_.m_genBasePosOut_);
  this->addOutPort("genBaseRpyOut", this->ports_.m_genBaseRpyOut_);
  this->ports_.m_SimpleHapticsControllerServicePort_.registerProvider("service0", "SimpleHapticsControllerService", this->ports_.m_service0_);
  this->addPort(this->ports_.m_SimpleHapticsControllerServicePort_);
  {
    // load dt
    std::string buf; this->getProperty("dt", buf);
    this->dt_ = std::stod(buf);
    if(this->dt_ <= 0.0){
      this->getProperty("exec_cxt.periodic.rate", buf);
      double rate = std::stod(buf);
      if(rate > 0.0){
        this->dt_ = 1.0/rate;
      }else{
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "dt is invalid" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
  }

  {
    // load robot model
    cnoid::BodyLoader bodyLoader;
    std::string fileName; this->getProperty("model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    cnoid::BodyPtr robot = bodyLoader.load(fileName);
    if(!robot){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    this->gaitParam_.init(robot);

    // generate JointParams
    for(int i=0;i<this->gaitParam_.actRobotTqc->numJoints();i++){
      cnoid::LinkPtr joint = this->gaitParam_.actRobotTqc->joint(i);
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
      this->gaitParam_.maxTorque[i] = std::max(climit * gearRatio * torqueConst, 0.0);
    }
    std::string jointLimitTableStr; this->getProperty("joint_limit_table",jointLimitTableStr);
    std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTables = joint_limit_table::readJointLimitTablesFromProperty (this->gaitParam_.actRobotTqc, jointLimitTableStr);
    for(size_t i=0;i<jointLimitTables.size();i++){
      this->gaitParam_.jointLimitTables[jointLimitTables[i]->getSelfJoint()->jointId()].push_back(jointLimitTables[i]);
    }
  }


  {
    // load end_effector
    std::string endEffectors; this->getProperty("end_effectors", endEffectors);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    while(std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      //   name, parentLink, (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      if(!this->gaitParam_.refRobotRaw->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Position localT;
      localT.translation() = localp;
      localT.linear() = localR;

      this->gaitParam_.push_backEE(name, parentLink, localT);
    }
  }

  {
    // add more ports (ロボットモデルやEndEffectorの情報を使って)

    // 各EndEffectorにつき、ref<name>WrenchInというInPortをつくる
    this->ports_.m_refEEWrenchIn_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_refEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "ref"+this->gaitParam_.eeName[i]+"WrenchIn";
      this->ports_.m_refEEWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_refEEWrench_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_refEEWrenchIn_[i]));
    }

    // 各EndEffectorにつき、ref<name>PoseInというInPortをつくる
    this->ports_.m_refEEPoseIn_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_refEEPose_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "ref"+this->gaitParam_.eeName[i]+"PoseIn";
      this->ports_.m_refEEPoseIn_[i] = std::make_unique<RTC::InPort<RTC::TimedPose3D> >(name.c_str(), this->ports_.m_refEEPose_[i]);
      this->addInPort(name.c_str(), *(this->ports_.m_refEEPoseIn_[i]));
    }

    // 各EndEffectorにつき、act<name>PoseOutというOutPortをつくる
    this->ports_.m_actEEPoseOut_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_actEEPose_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "act"+this->gaitParam_.eeName[i]+"PoseOut";
      this->ports_.m_actEEPoseOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPose3D> >(name.c_str(), this->ports_.m_actEEPose_[i]);
      this->addOutPort(name.c_str(), *(this->ports_.m_actEEPoseOut_[i]));
    }

    // 各EndEffectorにつき、tgt<name>WrenchOutというOutPortをつくる
    this->ports_.m_tgtEEWrenchOut_.resize(this->gaitParam_.eeName.size());
    this->ports_.m_tgtEEWrench_.resize(this->gaitParam_.eeName.size());
    for(int i=0;i<this->gaitParam_.eeName.size();i++){
      std::string name = "tgt"+this->gaitParam_.eeName[i]+"WrenchOut";
      this->ports_.m_tgtEEWrenchOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_tgtEEWrench_[i]);
      this->addOutPort(name.c_str(), *(this->ports_.m_tgtEEWrenchOut_[i]));
    }
  }

  this->refForceHandler_.init(this->gaitParam_, this->dt_);
  this->workSpaceForceHandler_.init(this->gaitParam_);
  this->torqueOutputGenerator_.init(this->gaitParam_);

  // initialize parameters
  this->loop_ = 0;

  return RTC::RTC_OK;
}

// static function
bool SimpleHapticsController::readInPortData(SimpleHapticsController::Ports& ports, cnoid::BodyPtr refRobotRaw, cnoid::BodyPtr actRobot, std::vector<cnoid::Vector6>& refEEWrenchRaw, std::vector<cnoid::Position>& refEEPoseRaw, std::vector<cnoid::Position>& actEEPose){
  bool qAct_updated = false;

  if(ports.m_refTauIn_.isNew()){
    ports.m_refTauIn_.read();
    if(ports.m_refTau_.data.length() == refRobotRaw->numJoints()){
      for(int i=0;i<ports.m_refTau_.data.length();i++){
        if(std::isfinite(ports.m_refTau_.data[i])) refRobotRaw->joint(i)->u() = ports.m_refTau_.data[i];
      }
    }
  }

  for(int i=0;i<ports.m_refEEWrenchIn_.size();i++){
    if(ports.m_refEEWrenchIn_[i]->isNew()){
      ports.m_refEEWrenchIn_[i]->read();
      if(ports.m_refEEWrench_[i].data.length() == 6){
        for(int j=0;j<6;j++){
          if(std::isfinite(ports.m_refEEWrench_[i].data[j])) refEEWrenchOrigin[i][j] = ports.m_refEEWrench_[i].data[j];
        }
      }
    }
  }

  for(int i=0;i<ports.m_refEEPoseIn_.size();i++){
    if(ports.m_refEEPoseIn_[i]->isNew()){
      ports.m_refEEPoseIn_[i]->read();
      if(std::isfinite(ports.m_refEEPose_[i].data.position.x) && std::isfinite(ports.m_refEEPose_[i].data.position.y) && std::isfinite(ports.m_refEEPose_[i].data.position.z) &&
         std::isfinite(ports.m_refEEPose_[i].data.orientation.r) && std::isfinite(ports.m_refEEPose_[i].data.orientation.p) && std::isfinite(ports.m_refEEPose_[i].data.orientation.y)){
        cnoid::Position pose;
        refEEPoseRaw[i].translation()[0] = ports.m_refEEPose_[i].data.position.x;
        refEEPoseRaw[i].translation()[1] = ports.m_refEEPose_[i].data.position.y;
        refEEPoseRaw[i].translation()[2] = ports.m_refEEPose_[i].data.position.z;
        refEEPoseRaw[i].linear() = cnoid::rotFromRpy(ports.m_refEEPose_[i].data.orientation.r, ports.m_refEEPose_[i].data.orientation.p, ports.m_refEEPose_[i].data.orientation.y);
      }
    }
  }

  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    if(ports.m_qAct_.data.length() == actRobot->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) actRobot->joint(i)->q() = ports.m_qAct_.data[i];
      }
      qAct_updated = true;
    }
  }
  if(ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();
    if(ports.m_dqAct_.data.length() == actRobot->numJoints()){
      for(int i=0;i<ports.m_dqAct_.data.length();i++){
        if(std::isfinite(ports.m_dqAct_.data[i])) actRobot->joint(i)->dq() = ports.m_dqAct_.data[i];
      }
    }
  }
  actRobot->calcForwardKinematics();

  for(int i=0;i<gaitParam.eeName.size(); i++){
    actEEPose[i] = actRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i];
  }

  return qAct_updated;
}

// static function
bool SimpleHapticsController::execSimpleHapticsController(const SimpleHapticsController::ControlMode& mode, GaitParam& gaitParam, double dt, const RefToGenFrameConverter& refToGenFrameConverter, const RefForceHandler& refForceHandler, const WorkSpaceForceHandler& workSpaceForceHandler, const GravityCompensationHandler& gravityCompensationHandler, const JointAngleLimitHandler& jointAngleLimitHandler, const TorqueOutputGenerator& torqueOutputGenerator) {
  if(mode.isSyncToHCInit()){ // startAutoBalancer直後の初回. gaitParamのリセット
  }

  // FootOrigin座標系を用いてrefRobotRawをgenerate frameに投影しrefRobotとする
  refToGenFrameConverter.convertFrame(gaitParam, dt,
                                      gaitParam.refEEWrench);

  refForceHandler.calcWrench(gaitParam, dt,
                             gaitParam.rfhTgtEEWrench);

  workSpaceForceHandler.calcWrench(gaitParam, dt,
                                   gaitParam.wsfhTgtWrench);

  gravityCompensationHandler.calcTorque(gaitParam, dt,
                                        gaitParam.actRobotGch, gaitParam.gchTorque);

  jointAngleLimitHandler.calcTorque(gaitParam, dt,
                                    gaitParam.japhTorque);

  torqueOutputGenerator.calcTorque(gaitParam,
                                   gaitParam.actRobotTqc);

  return true;
}

// static function
bool SimpleHapticsController::writeOutPortData(SimpleHapticsController::Ports& ports, const SimpleHapticsController::ControlMode& mode, cpp_filters::TwoPointInterpolator<double>& idleToAbcTransitionInterpolator, double dt, const GaitParam& gaitParam){
  if(mode.isSyncToABC()){
    if(mode.isSyncToABCInit()){
      idleToAbcTransitionInterpolator.reset(0.0);
    }
    idleToAbcTransitionInterpolator.setGoal(1.0,mode.remainTime());
    idleToAbcTransitionInterpolator.interpolate(dt);
  }else if(mode.isSyncToIdle()){
    if(mode.isSyncToIdleInit()){
      idleToAbcTransitionInterpolator.reset(1.0);
    }
    idleToAbcTransitionInterpolator.setGoal(0.0,mode.remainTime());
    idleToAbcTransitionInterpolator.interpolate(dt);
  }

  {
    // tau
    ports.m_genTau_.tm = ports.m_qAct_.tm;
    ports.m_genTau_.data.length(gaitParam.actRobotTqc->numJoints());
    for(int i=0;i<gaitParam.actRobotTqc->numJoints();i++){
      if(mode.now() == SimpleHapticsController::ControlMode::MODE_IDLE || !gaitParam.jointControllable[i]){
        ports.m_genTau_.data[i] = gaitParam.refRobotRaw->joint(i)->u();
      }else if(mode.isSyncToABC() || mode.isSyncToIdle()){
        double ratio = idleToAbcTransitionInterpolator.value();
        ports.m_genTau_.data[i] = gaitParam.refRobotRaw->joint(i)->u() * (1.0 - ratio) + gaitParam.actRobotTqc->joint(i)->u() * ratio;
      }else{
        ports.m_genTau_.data[i] = gaitParam.actRobotTqc->joint(i)->u();
      }
    }
    ports.m_genTauOut_.write();
  }

  {
    // basePose
    cnoid::Position basePose = gaitParam.actRobot->rootLink()->T();
    cnoid::Vector3 basePos = basePose.translation();
    cnoid::Matrix3 baseR = basePose.linear();
    cnoid::Vector3 baseRpy = cnoid::rpyFromRot(basePose.linear());

    ports.m_genBasePose_.tm = ports.m_qAct_.tm;
    ports.m_genBasePose_.data.position.x = basePos[0];
    ports.m_genBasePose_.data.position.y = basePos[1];
    ports.m_genBasePose_.data.position.z = basePos[2];
    ports.m_genBasePose_.data.orientation.r = baseRpy[0];
    ports.m_genBasePose_.data.orientation.p = baseRpy[1];
    ports.m_genBasePose_.data.orientation.y = baseRpy[2];
    ports.m_genBasePoseOut_.write();

    ports.m_genBaseTform_.tm = ports.m_qRef_.tm;
    ports.m_genBaseTform_.data.length(12);
    for(int i=0;i<3;i++){
      ports.m_genBaseTform_.data[i] = basePos[i];
    }
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        ports.m_genBaseTform_.data[3+i*3+j] = baseR(i,j);// row major
      }
    }
    ports.m_genBaseTformOut_.write();

    ports.m_genBasePos_.tm = ports.m_qAct_.tm;
    ports.m_genBasePos_.data.x = basePos[0];
    ports.m_genBasePos_.data.y = basePos[1];
    ports.m_genBasePos_.data.z = basePos[2];
    ports.m_genBasePosOut_.write();
    ports.m_genBaseRpy_.tm = ports.m_qAct_.tm;
    ports.m_genBaseRpy_.data.r = baseRpy[0];
    ports.m_genBaseRpy_.data.p = baseRpy[1];
    ports.m_genBaseRpy_.data.y = baseRpy[2];
    ports.m_genBaseRpyOut_.write();
  }

  for(int i=0;i<gaitParam.eeName.size();i++){
    ports.m_actEEPose_[i].tm = ports.m_qAct_.tm;
    ports.m_actEEPose_[i].data.position.x = gaitParam.actEEPose[i].translation()[0];
    ports.m_actEEPose_[i].data.position.y = gaitParam.actEEPose[i].translation()[1];
    ports.m_actEEPose_[i].data.position.z = gaitParam.actEEPose[i].translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(gaitParam.actEEPose[i].linear());
    ports.m_actEEPose_[i].data.orientation.r = rpy[0];
    ports.m_actEEPose_[i].data.orientation.p = rpy[1];
    ports.m_actEEPose_[i].data.orientation.y = rpy[2];
    ports.m_actEEPoseOut_[i]->write();
  }

  return true;
}

RTC::ReturnCode_t SimpleHapticsController::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  this->loop_++;

  if(!SimpleHapticsController::readInPortData(this->dt_, this->ports_, this->gaitParam_.refRobotRaw, this->gaitParam_.actRobotRaw, this->gaitParam_.refEEWrenchOrigin, this->gaitParam_.refEEPoseRaw, this->gaitParam_.actEEPose)) return RTC::RTC_OK;  // qAct が届かなければ何もしない

  this->mode_.update(this->dt_);
  this->gaitParam_.update(this->dt_);

  if(this->mode_.isABCRunning()) {
    if(this->mode_.isSyncToABCInit()){ // startAutoBalancer直後の初回. 内部パラメータのリセット
      this->gaitParam_.reset();
      this->refForceHandler_.reset();
      this->workSpaceForceHandler_.reset();
    }
    SimpleHapticsController::execSimpleHapticsController(this->mode_, this->gaitParam_, this->dt_, this->refToGenFrameConverter_, this->refForceHandler_, this->workSpaceForceHandler_, this->gravityCompensationHandler_, this->jointAngleLimitHandler_, this->torqueOutputGenerator_);
  }

  SimpleHapticsController::writeOutPortData(this->ports_, this->mode_, this->idleToAbcTransitionInterpolator_, this->dt_, this->gaitParam_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t SimpleHapticsController::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  this->mode_.reset();
  this->idleToAbcTransitionInterpolator_.reset(0.0);
  return RTC::RTC_OK;
}
RTC::ReturnCode_t SimpleHapticsController::onDeactivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t SimpleHapticsController::onFinalize(){ return RTC::RTC_OK; }

bool SimpleHapticsController::startHapticsController(){
  if(this->mode_.setNextTransition(ControlMode::START_HC)){
    std::cerr << "[" << m_profile.instance_name << "] start HapticsController mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_HC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] HapticsController is already started" << std::endl;
    return false;
  }
}
bool SimpleHapticsController::stopAutoBalancer(){
  if(this->mode_.setNextTransition(ControlMode::STOP_HC)){
    std::cerr << "[" << m_profile.instance_name << "] stop HapticsController mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] HapticsController is already stopped" << std::endl;
    return false;
  }
}
bool SimpleHapticsController::setSimpleHapticsControllerParam(const OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);

  // ignore i_param.ee_name
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    for(int i=0;i<this->gaitParam_.jointControllable.size();i++) this->gaitParam_.jointControllable[i] = false;
    for(int i=0;i<i_param.controllable_joints.length();i++){
      cnoid::LinkPtr joint = this->gaitParam_.genRobot->link(std::string(i_param.controllable_joints[i]));
      if(joint) this->gaitParam_.jointControllable[joint->jointId()] = true;
    }
  }
  this->mode_.hc_start_transition_time = std::max(i_param.hc_start_transition_time, 0.01);
  this->mode_.hc_stop_transition_time = std::max(i_param.hc_stop_transition_time, 0.01);

  return true;
}
bool SimpleHapticsController::getSimpleHapticsControllerParam(OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  i_param.ee_name.length(this->gaitParam_.eeName.size());
  for(int i=0;i<this->gaitParam_.eeName.size();i++) i_param.ee_name[i] = this->gaitParam_.eeName[i].c_str();
  std::vector<std::string> controllable_joints;
  for(int i=0;i<this->gaitParam_.jointControllable.size();i++) if(this->gaitParam_.jointControllable[i]) controllable_joints.push_back(this->gaitParam_.genRobot->joint(i)->name());
  i_param.controllable_joints.length(controllable_joints.size());
  for(int i=0;i<controllable_joints.size();i++) i_param.controllable_joints[i] = controllable_joints[i].c_str();
  i_param.hc_start_transition_time = this->mode_.hc_start_transition_time;
  i_param.hc_stop_transition_time = this->mode_.hc_stop_transition_time;

  return true;
}

bool SimpleHapticsController::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
}

extern "C"{
    void SimpleHapticsControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(SimpleHapticsController_spec);
        manager->registerFactory(profile, RTC::Create<SimpleHapticsController>, RTC::Delete<SimpleHapticsController>);
    }
};
