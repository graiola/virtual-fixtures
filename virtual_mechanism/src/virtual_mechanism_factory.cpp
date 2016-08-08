#include "virtual_mechanism/virtual_mechanism_factory.h"
#include "virtual_mechanism/virtual_mechanism_gmr.h"
#include "virtual_mechanism/virtual_mechanism_spline.h"

//using namespace virtual_mechanism_interface;
//using namespace virtual_mechanism_gmr;
//using namespace virtual_mechanism_spline;
using namespace Eigen;
using namespace std;

namespace virtual_mechanism
{

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

VirtualMechanismFactory::VirtualMechanismFactory()
{
    default_order_ = FIRST;
    default_model_type_ = GMR;
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const MatrixXd& data, const order_t order, const model_type_t model_type)
{
    VirtualMechanismInterface* vm_ptr = NULL;
    try
    {
        vm_ptr = CreateEmptyMechanism(order,model_type);
        vm_ptr->CreateModelFromData(data);
        vm_ptr->Init();
    }
    catch(const runtime_error& e)
    {
       ROS_ERROR("Error in the factory: %s",e.what());
    }
    return vm_ptr;
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const string model_name, const order_t order, const model_type_t model_type)
{
    VirtualMechanismInterface* vm_ptr = NULL;
    try
    {
        vm_ptr = CreateEmptyMechanism(order,model_type);
        vm_ptr->CreateModelFromFile(model_name);
        vm_ptr->Init();
    }
    catch(const runtime_error& e)
    {
       ROS_ERROR("Error in the factory: %s",e.what());
    }
    return vm_ptr;
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const MatrixXd& data)
{
    return Build(data,default_order_,default_model_type_);
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const string model_name)
{
    return Build(model_name,default_order_,default_model_type_);
}

void VirtualMechanismFactory::SetDefaultPreferences(const order_t order, const model_type_t model_type)
{
    default_order_ = order;
    default_model_type_ = model_type;
}

void VirtualMechanismFactory::SetDefaultPreferences(const string order, const string model_type)
{
    if (order == "first")
        default_order_ = FIRST;
    else if (order == "second")
        default_order_ = SECOND;
    else
        throw new runtime_error("VirtualMechanismFactory: Wrong order.");

    if (model_type == "gmr")
        default_model_type_ = GMR;
    else if (model_type == "gmr_normalized")
        default_model_type_ = GMR_NORMALIZED;
    else
        throw new runtime_error("VirtualMechanismFactory: Wrong model_type.");
}

VirtualMechanismInterface* VirtualMechanismFactory::CreateEmptyMechanism(const order_t order, const model_type_t model_type)
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
     }
     return vm_ptr;
}

template<typename ORDER>  VirtualMechanismInterface* VirtualMechanismFactory::SelectModel(const model_type_t model_type)
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
    }
    return vm_ptr;
}

} // namespace
