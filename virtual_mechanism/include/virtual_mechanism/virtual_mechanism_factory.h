#ifndef VIRTUAL_MECHANISM_FACTORY_H
#define VIRTUAL_MECHANISM_FACTORY_H

#include "virtual_mechanism/virtual_mechanism_interface.h"

namespace virtual_mechanism_interface
{

enum order_t {FIRST,SECOND};
enum model_type_t {GMR,GMR_NORMALIZED,SPLINE};

class VirtualMechanismAbstractFactory
{

public:
    virtual virtual_mechanism_interface::VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const Eigen::MatrixXd& data) = 0;
    virtual virtual_mechanism_interface::VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const std::string& model_name) = 0;
};

class VirtualMechanismFactory : public VirtualMechanismAbstractFactory
{

public:
    virtual virtual_mechanism_interface::VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const Eigen::MatrixXd& data);
    virtual virtual_mechanism_interface::VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const std::string& model_name);

protected:
    virtual_mechanism_interface::VirtualMechanismInterface* CreateEmptyMechanism(const order_t order, const model_type_t model_type);
    template<typename ORDER>  virtual_mechanism_interface::VirtualMechanismInterface* SelectModel(const model_type_t model_type);
};

} // namespace

#endif
