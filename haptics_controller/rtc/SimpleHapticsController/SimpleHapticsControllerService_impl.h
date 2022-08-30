// -*-C++-*-
#ifndef SimpleHapticsControllerSERVICESVC_IMPL_H
#define SimpleHapticsControllerSERVICESVC_IMPL_H

#include "auto_stabilizer/idl/SimpleHapticsControllerService.hh"

class SimpleHapticsController;

class SimpleHapticsControllerService_impl
  : public virtual POA_OpenHRP::SimpleHapticsControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  SimpleHapticsControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~SimpleHapticsControllerService_impl();
  CORBA::Boolean startHapticsController();
  CORBA::Boolean stopHapticsController();
  CORBA::Boolean setSimpleHapticsControllerParam(const OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam& i_param);
  CORBA::Boolean getSimpleHapticsControllerParam(OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam_out i_param);
  //
  //
  void setComp(SimpleHapticsController *i_comp);
private:
  SimpleHapticsController *comp_;
};

#endif
