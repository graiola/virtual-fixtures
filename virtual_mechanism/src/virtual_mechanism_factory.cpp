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

/*virtual void VirtualMechanismFactory::FromStringToEnum(const std::string order, const std::string model_type,
                                                       order_t& order_out, model_type_t& model_type_out)
{
    if (order == "first")
        order_out = FIRST;
    else if (order == "second")
        order_out = SECOND;
    else
        order_out = FIRST; // Default

    if (model_type == "gmr")
        model_type_out = GMR;
    else if (model_type == "gmr_normalized")
        model_type_out = GMR_NORMALIZED;
    else
        model_type_out = GMR; // Default
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const std::string order, const std::string model_type, const MatrixXd& data)
{
    order_t order_enum;
    model_type_t model_type_enum;
    FromStringToEnum(order,model_type,order_enum,model_type_enum);
    return Build(order_enum,model_type_enum,data);
}

VirtualMechanismInterface* VirtualMechanismFactory::Build(const std::string order, const std::string model_type, const string model_name)
{
    order_t order_enum;
    model_type_t model_type_enum;
    FromStringToEnum(order,model_type,order_enum,model_type_enum);
    return Build(order_enum,model_type_enum,model_name);
}
*/
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
