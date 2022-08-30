#ifndef REFTOGENFRAMECONVERTER_H
#define REFTOGENFRAMECONVERTER_H

#include "GaitParam.h"
#include <cnoid/Body>

class RefToGenFrameConverter {
public:
  // RefToGenFrameConverterだけでつかうパラメータ

public:
  // reference frameで表現されたrefRobotRawをgenerate frameに投影しrefRobotとし、各種referencec値をgenerate frameに変換する
  bool convertFrame(const GaitParam& gaitParam, double dt,// input
                    std::vector<cnoid::Vector6>& o_refEEWrench /*endeffector frame. endeffector origin*/) const; // output
};

#endif
