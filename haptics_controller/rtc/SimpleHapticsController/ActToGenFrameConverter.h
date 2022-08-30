#ifndef ACTTOGENFRAMECONVERTER_H
#define ACTTOGENFRAMECONVERTER_H

#include "GaitParam.h"

class ActToGenFrameConverter {
public:
  // ActToGenFrameConverterだけでつかうパラメータ

public:
  // actual frameで表現されたactRobotRawをgenerate frameに投影しactRobotとし、各種actual値をgenerate frameに変換する
  bool convertFrame(const GaitParam& gaitParam, double dt, // input
                    std::vector<cnoid::Position>& o_actEEPose) const; // output
};

#endif
