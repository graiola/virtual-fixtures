#ifndef VIRTUAL_MECHANISM_FACTORY_H
#define VIRTUAL_MECHANISM_FACTORY_H

#include "virtual_mechanism/virtual_mechanism_interface.h"
#include "virtual_mechanism/virtual_mechanism_gmr.h"
#include "virtual_mechanism/virtual_mechanism_spline.h"

namespace
{

using namespace virtual_mechanism_interface;
using namespace virtual_mechanism_gmr;
using namespace virtual_mechanism_spline;

namespace virtual_mechanism_factory
{

class VirtualMechanismAbstractFactory
{
public:

    virtual VirtualMechanismInterface* Build(const int order, const std::string& model_type, const Eigen::MatrixXd& data) = 0;
    virtual VirtualMechanismInterface* Build(const int order, const std::string& model_type, const std::string& model_name) = 0;
};

class VirtualMechanismFactory : public VirtualMechanismAbstractFactory
{
    typedef VirtualMechanismInterfaceFirstOrder VMP_1ord_t;
    typedef VirtualMechanismInterfaceSecondOrder VMP_2ord_t;

public:

    virtual VirtualMechanismInterface* Build(const int order, const std::string& model_type, const Eigen::MatrixXd& data)
    {
        VirtualMechanismInterface* vm_ptr = NULL;

        if(order == 1)
            if(std::strcmp(model_type.c_str(),"gmr_normalized"))
                vm_ptr = new VirtualMechanismGmrNormalized<VMP_1ord_t>(data);
            else if(std::strcmp(model_type.c_str(),"gmr"))
                vm_ptr = new VirtualMechanismGmr<VMP_1ord_t>(data);
        else if(order == 2)
            if(std::strcmp(model_type.c_str(),"gmr_normalized"))
                vm_ptr = new VirtualMechanismGmrNormalized<VMP_2ord_t>(data);
            else if(std::strcmp(model_type.c_str(),"gmr"))
                vm_ptr = new VirtualMechanismGmr<VMP_2ord_t>(data);

        return vm_ptr;
    }

    VirtualMechanismInterface* Build(const int order, const std::string& model_type, const std::string& model_name)
    {
        VirtualMechanismInterface* vm_ptr = NULL;

        if(order == 1)
            if(std::strcmp(model_type.c_str(),"gmr_normalized"))
                vm_ptr = new VirtualMechanismGmrNormalized<VMP_1ord_t>(model_name);
            else if(std::strcmp(model_type.c_str(),"gmr"))
                vm_ptr = new VirtualMechanismGmr<VMP_1ord_t>(model_name);
        else if(order == 2)
            if(std::strcmp(model_type.c_str(),"gmr_normalized"))
                vm_ptr = new VirtualMechanismGmrNormalized<VMP_2ord_t>(model_name);
            else if(std::strcmp(model_type.c_str(),"gmr"))
                vm_ptr = new VirtualMechanismGmr<VMP_2ord_t>(model_name);

        return vm_ptr;
    }
};

} // namespace

} // namespace

#endif
