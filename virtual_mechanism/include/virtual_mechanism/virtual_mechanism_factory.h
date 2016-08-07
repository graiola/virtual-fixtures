#ifndef VIRTUAL_MECHANISM_FACTORY_H
#define VIRTUAL_MECHANISM_FACTORY_H

#include "virtual_mechanism/virtual_mechanism_interface.h"
#include "virtual_mechanism/virtual_mechanism_gmr.h"
#include "virtual_mechanism/virtual_mechanism_spline.h"

namespace virtual_mechanism_factory
{

using namespace virtual_mechanism_interface;
using namespace virtual_mechanism_gmr;
using namespace virtual_mechanism_spline;

enum order_t {FIRST,SECOND};
enum model_type_t {GMR,GMR_NORMALIZED,SPLINE};

class VirtualMechanismAbstractFactory
{
public:
    virtual VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const Eigen::MatrixXd& data) = 0;
    virtual VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const std::string& model_name) = 0;
};

class VirtualMechanismFactory : public VirtualMechanismAbstractFactory
{
    typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
    typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

public:

    virtual VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const Eigen::MatrixXd& data)
    {
        VirtualMechanismInterface* vm_ptr = NULL;
        try
        {
            vm_ptr = CreateEmptyMechanism(order,model_type);
            vm_ptr->CreateModelFromData(data);
            vm_ptr->Init();
        }
        catch(const std::runtime_error& e)
        {
           ROS_ERROR("Error in the factory: %s",e.what());
        }
        return vm_ptr;
    }

    VirtualMechanismInterface* Build(const order_t order, const model_type_t model_type, const std::string& model_name)
    {
        VirtualMechanismInterface* vm_ptr = NULL;
        try
        {
            vm_ptr = CreateEmptyMechanism(order,model_type);
            vm_ptr->CreateModelFromFile(model_name);
            vm_ptr->Init();
        }
        catch(const std::runtime_error& e)
        {
           ROS_ERROR("Error in the factory: %s",e.what());
        }
        return vm_ptr;
    }

protected:

    VirtualMechanismInterface* CreateEmptyMechanism(const order_t order, const model_type_t model_type)
    {
         VirtualMechanismInterface* vm_ptr = NULL;

         switch(order)
         {
           case FIRST:
             vm_ptr = SelectModel<VMP_1ord_t>(model_type);
             break;
           case SECOND:
             vm_ptr = SelectModel<VMP_2ord_t>(model_type);
             break;
           default:
             vm_ptr = SelectModel<VMP_1ord_t>(model_type);
             break;
         }
         return vm_ptr;
    }

    template<typename ORDER>  VirtualMechanismInterface* SelectModel(const model_type_t model_type)
    {
        VirtualMechanismInterface* vm_ptr = NULL;
        switch(model_type)
        {
           case GMR:
            vm_ptr = new VirtualMechanismGmr<ORDER>();
            break;
           case GMR_NORMALIZED:
            vm_ptr = new VirtualMechanismGmrNormalized<ORDER>();
            break;
           default:
            vm_ptr = new VirtualMechanismGmr<ORDER>();
            break;
        }
        return vm_ptr;
    }
};

} // namespace

#endif
