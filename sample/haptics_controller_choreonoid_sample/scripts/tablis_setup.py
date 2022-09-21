#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

import OpenRTM_aist
import OpenHRP
from hrpsys.RobotHardwareService_idl import *
from haptics_controller.SimpleHapticsControllerService_idl import *

from haptics_controller import *

class TABLIS_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
    hc = None
    hc_svc = None
    hc_version = None

    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['hc', "SimpleHapticsController"],
            ['log', "DataLogger"]
            ]

    def connectComps(self):
        super(TABLIS_HrpsysConfigurator, self).connectComps()
        if self.hc:
            connectPorts(self.rh.port("q"), self.hc.port("qAct"))
            connectPorts(self.rh.port("dq"), self.hc.port("dqAct"))
            if self.st:
                disconnectPorts(self.st.port("tau"), self.rh.port("tauRef"))
                connectPorts(self.st.port("tau"), self.hc.port("refTauIn"))
                connectPorts(self.hc.port("genTauOut"), self.rh.port("tauRef"))
            else:
                disconnectPorts(self.sh.port("tqOut"), self.rh.port("tauRef"))
                connectPorts(self.sh.port("tqOut"), self.hc.port("refTauIn"))
                connectPorts(self.hc.port("genTauOut"), self.rh.port("tauRef"))

    def setupLogger(self):
        if self.hc:
            self.log_svc.add("TimedDoubleSeq","hc_genTauOut")
            rtm.connectPorts(rtm.findRTC("hc").port("genTauOut"),rtm.findRTC("log").port("hc_genTauOut"))
            for ee in ["rleg", "lleg", "rarm", "larm"]:
                self.log_svc.add("TimedPose3D","hc_act" + ee + "PoseOut")
                rtm.connectPorts(rtm.findRTC("hc").port("act" + ee + "PoseOut"),rtm.findRTC("log").port("hc_act" + ee + "PoseOut"))
        super(TABLIS_HrpsysConfigurator, self).setupLogger()

    def startABSTIMP (self):
        self.rh_svc.setJointControlMode("all",OpenHRP.RobotHardwareService.TORQUE)
        self.rh_svc.setServoTorqueGainPercentage("all",100)
        self.rh_svc.setServoGainPercentage("all",0)
        self.hc_svc.startHapticsController()

if __name__ == '__main__':
    hcf = TABLIS_HrpsysConfigurator("TABLIS")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
