#ifndef VIRTUAL_MECHANISM_FACTORY_H
#define VIRTUAL_MECHANISM_FACTORY_H

#include "virtual_mechanism/virtual_mechanism_interface.h"

namespace virtual_mechanism
{

enum order_t {FIRST,SECOND};
enum model_type_t {GMR,GMR_NORMALIZED,SPLINE};

class VirtualMechanismAbstractFactory
{
public:
    virtual VirtualMechanismInterface* Build(const Eigen::MatrixXd& data, const order_t order, const model_type_t model_type) = 0;
    virtual VirtualMechanismInterface* Build(const std::string model_name, const order_t order, const model_type_t model_type) = 0;
};

class VirtualMechanismFactory : public VirtualMechanismAbstractFactory
{
public:
    virtual VirtualMechanismInterface* Build(const Eigen::MatrixXd& data, const order_t order, const model_type_t model_type);
    virtual VirtualMechanismInterface* Build(const std::string model_name, const order_t order, const model_type_t model_type);
protected:
    VirtualMechanismInterface* CreateEmptyMechanism(const order_t order, const model_type_t model_type);
    template<typename ORDER>  VirtualMechanismInterface* SelectModel(const model_type_t model_type);
};

} // namespace

#endif
