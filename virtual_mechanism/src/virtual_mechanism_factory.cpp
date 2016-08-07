#include "virtual_mechanism/virtual_mechanism_factory.h"

using namespace virtual_mechanism_interface;
using namespace virtual_mechanism_gmr;
using namespace virtual_mechanism_spline;
using namespace Eigen;
using namespace std;

typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

namespace virtual_mechanism_factory
{

VirtualMechanismInterface* VirtualMechanismFactory::Build(const order_t order, const model_type_t model_type, const MatrixXd& data)
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

VirtualMechanismInterface* VirtualMechanismFactory::Build(const order_t order, const model_type_t model_type, const string& model_name)
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
       default:
         vm_ptr = SelectModel<VMP_1ord_t>(model_type);
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
       default:
        vm_ptr = new VirtualMechanismGmr<ORDER>();
        break;
    }
    return vm_ptr;
}

} // namespace
