#include "SimpleHapticsControllerService_impl.h"
#include "SimpleHapticsController.h"

SimpleHapticsControllerService_impl::SimpleHapticsControllerService_impl()
{
}

SimpleHapticsControllerService_impl::~SimpleHapticsControllerService_impl()
{
}

CORBA::Boolean SimpleHapticsControllerService_impl::startHapticsController()
{
  return this->comp_->startHapticsController();
};

CORBA::Boolean SimpleHapticsControllerService_impl::stopHapticsController()
{
  return this->comp_->stopHapticsController();
};

CORBA::Boolean SimpleHapticsControllerService_impl::setHapticsControllerParam(const OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam& i_param)
{
  return this->comp_->setHapticsControllerParam(i_param);
};

CORBA::Boolean SimpleHapticsControllerService_impl::getHapticsControllerParam(OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam_out i_param)
{
  i_param = new OpenHRP::SimpleHapticsControllerService::SimpleHapticsControllerParam();
  return this->comp_->getHapticsControllerParam(*i_param);
};

void SimpleHapticsControllerService_impl::setComp(SimpleHapticsController *i_comp)
{
  this->comp_ = i_comp;
}
