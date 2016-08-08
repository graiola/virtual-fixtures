#ifndef VIRTUAL_MECHANISM_FACTORY_H
#define VIRTUAL_MECHANISM_FACTORY_H

#include "virtual_mechanism/virtual_mechanism_interface.h"

namespace virtual_mechanism
{

enum order_t {FIRST,SECOND};
enum model_type_t {GMR,GMR_NORMALIZED}; // SPLINE

/*class VirtualMechanismAbstractFactory
{
public:
    virtual VirtualMechanismInterface* Build(const Eigen::MatrixXd& data, const order_t order, const model_type_t model_type) = 0;
    virtual VirtualMechanismInterface* Build(const std::string model_name, const order_t order, const model_type_t model_type) = 0;
};*/

class VirtualMechanismFactory// : public VirtualMechanismAbstractFactory
{
public:
    VirtualMechanismFactory();
    VirtualMechanismInterface* Build(const Eigen::MatrixXd& data, const order_t order, const model_type_t model_type);
    VirtualMechanismInterface* Build(const std::string model_name, const order_t order, const model_type_t model_type);
    VirtualMechanismInterface* Build(const Eigen::MatrixXd& data); // With default order and model_type
    VirtualMechanismInterface* Build(const std::string model_name); // With default order and model_type
    void SetDefaultPreferences(const order_t order, const model_type_t model_type);
    void SetDefaultPreferences(const std::string order, const std::string model_type);
protected:
    VirtualMechanismInterface* CreateEmptyMechanism(const order_t order, const model_type_t model_type);
    template<typename ORDER>  VirtualMechanismInterface* SelectModel(const model_type_t model_type);
    order_t default_order_;
    model_type_t default_model_type_;
};

} // namespace

#endif
