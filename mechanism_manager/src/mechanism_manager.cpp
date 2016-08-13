#include "mechanism_manager/mechanism_manager.h"

namespace mechanism_manager
{

  using namespace virtual_mechanism;
  using namespace tool_box;
  using namespace Eigen;

VirtualMechanismAutom::VirtualMechanismAutom(const double phase_dot_preauto_th, const double phase_dot_th)
{
    assert(phase_dot_th > 0.0);
    assert(phase_dot_preauto_th > phase_dot_th);
    phase_dot_preauto_th_ = phase_dot_preauto_th;
    phase_dot_th_ = phase_dot_th;
    state_ = MANUAL;
    loopCnt = 0;
}

void VirtualMechanismAutom::Step(const double phase_dot,const double phase_dot_ref, bool collision_detected)
{
    if(true)
    {
        switch(state_)
        {
            case MANUAL:
                if(phase_dot >= phase_dot_preauto_th_)
                    state_ = PREAUTO;
                break;
            case PREAUTO:
                if(phase_dot <= (phase_dot_ref + phase_dot_th_))
                    state_ = AUTO;
                break;
            case AUTO:
                if(collision_detected)
                    state_ = MANUAL;
                break;
        }
    }
    else // Two states version
    {
            if((phase_dot <= (phase_dot_ref + phase_dot_th_)) && (phase_dot >= (phase_dot_ref - phase_dot_th_)))
                state_ = AUTO;
            else
                state_ = MANUAL;
    }
}

bool VirtualMechanismAutom::GetState()
{
    bool activate_vm;
    switch(state_)
    {
        case MANUAL:
            activate_vm = false;
            break;
        case PREAUTO:
            activate_vm = false;
            break;
        case AUTO:
            activate_vm = true;
            break;
    }

    if(loopCnt%1000==0)
    {
        switch(state_)
        {
            case MANUAL:
                PRINT_INFO("VirtualMechanismAutom: MANUAL");
                break;
            case PREAUTO:
                PRINT_INFO("VirtualMechanismAutom: PREAUTO");
                break;
            case AUTO:
                PRINT_INFO("VirtualMechanismAutom: AUTO");
                break;
        }
    }
    loopCnt++;

    return activate_vm;
}

MechanismManager::MechanismManager(int position_dim)
{
      if(!ReadConfig())
      {
        throw new std::runtime_error("MechanismManager: Can not read config file");
      }

      assert(position_dim == 1 || position_dim == 2);
      position_dim_ = position_dim;

      // Resize
      f_K_.resize(position_dim_);
      f_B_.resize(position_dim_);
      f_vm_.resize(position_dim_);
      err_pos_.resize(position_dim_);
      err_vel_.resize(position_dim_);
      f_prev_.resize(position_dim_);

      // Clear
      f_K_.fill(0.0);
      f_B_.fill(0.0);
      f_vm_.fill(0.0);
      err_pos_.fill(0.0);
      err_vel_.fill(0.0);
      f_prev_.fill(0.0);

      // Chached
      on_guide_prev_ = false;
      nb_vm_prev_ = 0;

      loopCnt = 0;

      pkg_path_ = ros::package::getPath(ROS_PKG_NAME);
/*
#ifdef USE_ROS_RT_PUBLISHER
      try
      {
          rt_publishers_vector_.AddPublisher(ros_node,"phase",&phase_);
          rt_publishers_vector_.AddPublisher(ros_node,"phase_dot",&phase_dot_);
          rt_publishers_vector_.AddPublisher(ros_node,"scale",&scales_);
          rt_publishers_vector_.AddPublisher(ros_node,"scale_hard",&scales_hard_);
          rt_publishers_vector_.AddPublisher(ros_node,"scale_soft",&scales_soft_);
          rt_publishers_vector_.AddPublisher(ros_node,"scale_t",&scales_t_);
          rt_publishers_vector_.AddPublisher(ros_node,"phase_dot_ref",&phase_dot_ref_);
      }
      catch(const std::runtime_error& e)
      {
        PRINT_ERROR("Failed to create the real time publishers: %s",e.what());
      }
#endif
*/
}

MechanismManager::~MechanismManager()
{
      for(int i=0;i<vm_vector_.size();i++)
        delete vm_vector_[i];
      for(int i=0;i<vm_autom_.size();i++)
        delete vm_autom_[i];
}

void MechanismManager::ExpandVectors(vm_t* const vm_tmp_ptr)
{
    //VectorXd empty_vect_N(position_dim_);
    //MatrixXd empty_mat_NxN(position_dim_,position_dim_);
    //MatrixXd empty_mat_Nx1(position_dim_,1);

    vm_vector_.push_back(vm_tmp_ptr);
    vm_fades_.push_back(DynSystemFirstOrder(10.0));
    vm_autom_.push_back(new VirtualMechanismAutom(phase_dot_preauto_th_,phase_dot_th_));

    PushBack(0.0,scales_);
    PushBack(0.0,scales_t_);
    PushBack(0.0,scales_hard_);
    PushBack(0.0,scales_soft_);
    PushBack(0.0,phase_);
    PushBack(0.0,phase_dot_);
    PushBack(0.0,phase_ddot_);
    PushBack(0.0,phase_ref_);
    PushBack(0.0,phase_dot_ref_);
    PushBack(0.0,phase_ddot_ref_);
    PushBack(0.0,fade_);
/*
#ifdef USE_ROS_RT_PUBLISHER
    rt_publishers_vector_.PushBackEmptyAll();
#endif
*/
}

bool MechanismManager::ReadConfig()
{
    YAML::Node main_node = CreateYamlNodeFromPkgName(ROS_PKG_NAME);
    if (const YAML::Node& curr_node = main_node["mechanism_manager"])
    {
        std::string vm_order, vm_model_type;
        curr_node["vm_order"] >> vm_order;
        curr_node["vm_model_type"] >> vm_model_type;
        curr_node["escape_factor"] >> escape_factor_;
        curr_node["phase_dot_th"] >> phase_dot_th_;
        curr_node["phase_dot_preauto_th"] >> phase_dot_preauto_th_;
        assert(phase_dot_th_ > 0.0);
        assert(phase_dot_preauto_th_ > phase_dot_th_);
        assert(escape_factor_ > 0.0);

        vm_factory_.SetDefaultPreferences(vm_order,vm_model_type);

        return true;
    }
    else
        return false;
}

void MechanismManager::InsertVM(std::string& model_name)
{
    std::string model_complete_path(pkg_path_+"/models/gmm/"+model_name); // FIXME change the folder for splines
    //std::cout << "Creating the guide from file... "<< model_complete_path << std::endl;
    PRINT_INFO("Creating the guide from file... " << model_complete_path);
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock
    vm_t* vm_tmp_ptr = NULL;
    try
    {
        vm_tmp_ptr = vm_factory_.Build(model_complete_path);
        vm_tmp_ptr->setName(model_name);
        ExpandVectors(vm_tmp_ptr);
        PRINT_INFO("Guide number#" << vm_vector_.size()-1 <<" created");
    }
    catch(...)
    {
        PRINT_ERROR("Impossible to create the guide... "<<model_complete_path);
    }
    guard.unlock(); // Unlock
}

void MechanismManager::InsertVM(const MatrixXd& data)
{
    PRINT_INFO("Creating the guide from data...");
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock
    vm_t* vm_tmp_ptr = NULL;
    try
    {
        vm_tmp_ptr = vm_factory_.Build(data);
        vm_tmp_ptr->setName("guide_"+std::to_string(vm_vector_.size()-1));
        ExpandVectors(vm_tmp_ptr);
        PRINT_INFO("Guide number#"<< vm_vector_.size()-1 <<" created");
    }
    catch(...)
    {
        PRINT_ERROR("Impossible to create the guide from data...");
    }
    guard.unlock(); // Unlock
}

void MechanismManager::InsertVM()
{
    std::string model_name;
    std::cout << "Insert model name: " << std::endl;
    std::cin >> model_name;
    InsertVM(model_name);
}

void MechanismManager::SaveVM(const int idx, std::string& model_name)
{
    std::string model_complete_path(pkg_path_+"/models/gmm/"+model_name); // FIXME change the folder for splines
    PRINT_INFO("Saving guide number#"<<idx<<" to " << model_complete_path);
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    if(idx < vm_vector_.size())
    {
        if(!vm_vector_[idx]->SaveModelToFile(model_complete_path))
            PRINT_ERROR("Impossible to save the file " << model_complete_path);
    }
    else
    {
        PRINT_INFO("Guide number#"<<idx<<" not available");
    }
    guard.unlock();
}

void MechanismManager::SaveVM(const int idx)
{
    std::string model_name;
    std::cout << "Insert model name: " << std::endl;
    std::cin >> model_name;
    SaveVM(idx,model_name);
}

void MechanismManager::DeleteVM(const int idx)
{
   boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
   guard.lock();
   if(idx < vm_autom_.size())
   {
       delete vm_autom_[idx];
       vm_autom_.erase(vm_autom_.begin()+idx);
   }
   if(idx < vm_vector_.size())
   {
       PRINT_INFO("Deleting guide number#"<<idx);

       delete vm_vector_[idx];
       vm_vector_.erase(vm_vector_.begin()+idx);
       vm_fades_.erase(vm_fades_.begin()+idx);

       Delete(idx,scales_);
       Delete(idx,scales_t_);
       Delete(idx,scales_hard_);
       Delete(idx,scales_soft_);
       Delete(idx,phase_);
       Delete(idx,phase_dot_);
       Delete(idx,phase_ddot_);
       Delete(idx,phase_ref_);
       Delete(idx,phase_dot_ref_);
       Delete(idx,phase_ddot_ref_);
       Delete(idx,fade_);

       PRINT_INFO("Delete of guide number#"<<idx<<" complete");
/*
#ifdef USE_ROS_RT_PUBLISHER
       rt_publishers_vector_.RemoveAll(idx);
#endif
*/
   }
   else
       PRINT_WARNING("Impossible to remove guide number#"<<idx);

   guard.unlock();
}

void MechanismManager::Update(const VectorXd& robot_position, const VectorXd& robot_velocity, double dt, VectorXd& f_out, const scale_mode_t scale_mode)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {

        for(int i=0; i<vm_vector_.size();i++)
        {
            // Update the virtual mechanisms states
            vm_vector_[i]->Update(robot_position,robot_velocity,dt);

            // Compute the scale for each mechanism
            scales_(i) = vm_vector_[i]->getScale(robot_position,escape_factor_);

            // Retrain variables for plots
            phase_(i) = vm_vector_[i]->getPhase();
            phase_dot_(i) = vm_vector_[i]->getPhaseDot();
            phase_ddot_(i) = vm_vector_[i]->getPhaseDotDot();
            phase_ref_(i) = vm_vector_[i]->getPhaseRef();
            phase_dot_ref_(i) = vm_vector_[i]->getPhaseDotRef();
            phase_ddot_ref_(i) = vm_vector_[i]->getPhaseDotDotRef();
            fade_(i) = vm_vector_[i]->getFade();
        }

        f_out.fill(0.0); // Reset the force
        double sum = scales_.sum();

        // Compute the global scales
        for(int i=0; i<vm_vector_.size();i++)
        {
          switch(scale_mode)
          {
            case HARD:
                scales_soft_(i) = vm_vector_[i]->getScale(robot_position,escape_factor_);
                scales_hard_(i) = scales_(i)/sum;
                scales_(i) =  scales_hard_(i);
                break;
            case POTENTIAL:
                scales_(i) = vm_vector_[i]->getScale(robot_position,escape_factor_);
                break;
            case SOFT:
                scales_soft_(i) = vm_vector_[i]->getScale(robot_position,escape_factor_);
                //scales_hard_(i) = scales_(i)/(sum + std::numeric_limits<double>::epsilon()); // To avoid numerical issues
                scales_hard_(i) = scales_(i)/sum;
                scales_(i) =  scales_soft_(i) * scales_hard_(i);
                break;
            default:
              break;
          }
        }

        // For each mechanism that is not active (low scale value), remove the force component tangent to
        // the active mechanism jacobian. In this way we avoid to be locked if one or more guide overlap in a certain area.
        // Use a first order filter to gently remove these components.
        for(int i=0; i<vm_vector_.size();i++)
            if(scales_hard_(i) > 1.0/static_cast<double>(vm_vector_.size()))
                for(int j=0; j<vm_vector_.size();j++)
                    if(j!=i)
                        scales_t_(j) = vm_fades_[j].IntegrateForward();
                    else
                        scales_t_(j) = vm_fades_[j].IntegrateBackward();

        // Compute the force for each mechanism, remove the antagonist force components
        for(int i=0; i<vm_vector_.size();i++)
        {
            err_pos_ = vm_vector_[i]->getState() - robot_position;
            f_K_ .noalias() = vm_vector_[i]->getK() * err_pos_;
            err_vel_ = vm_vector_[i]->getStateDot() - robot_velocity;
            f_B_.noalias() = vm_vector_[i]->getB() * err_vel_;

            // Sum spring force + damping force for the current mechanism
            f_vm_ = f_K_ + f_B_;

            f_out += scales_(i) * f_vm_;
            for(int j=0; j<vm_vector_.size();j++)
            {
                if(j!=i)
                    f_out -= scales_(i) * scales_t_(i) * vm_vector_[j]->getJacobianVersor() * f_vm_.dot(vm_vector_[j]->getJacobianVersor());
            }
        }
        f_prev_ = f_out;
        nb_vm_prev_ = vm_vector_.size();
    }
    else
        f_out = f_prev_; // Keep the previous force while the vectors are updating
/*
#ifdef USE_ROS_RT_PUBLISHER
   rt_publishers_vector_.PublishAll();
#endif
*/
}

void MechanismManager::Stop()
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        for(int i=0;i<vm_vector_.size();i++)
          vm_vector_[i]->Stop();
    }
}

void MechanismManager::GetVmPosition(const int idx, Eigen::VectorXd& position)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        if(idx < vm_vector_.size())
            vm_vector_[idx]->getState(position);
    }
}

void MechanismManager::GetVmVelocity(const int idx, Eigen::VectorXd& velocity)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        if(idx < vm_vector_.size())
            vm_vector_[idx]->getStateDot(velocity);
    }
}

double MechanismManager::GetPhase(const int idx)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        if(idx < vm_vector_.size())
            return vm_vector_[idx]->getPhase();
        else
            return 0.0;
    }
}

double MechanismManager::GetScale(const int idx)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        if(idx < vm_vector_.size())
            return scales_(idx);
        else
            return 0.0;
    }
}

int MechanismManager::GetNbVms()
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
        return vm_vector_.size();
    else
        return nb_vm_prev_;
}

bool MechanismManager::OnVm()
{
    bool on_guide = false;
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        for(int i=0;i<scales_.size();i++)
        {
            if(scales_(i) > 0.9) // We are on a guide if it's scale is ... (so that we are on it)
                on_guide = true;
        }
    }
    else
        on_guide = on_guide_prev_;

    on_guide_prev_ = on_guide;
    return on_guide;
}

void MechanismManager::UpdateVM(MatrixXd& data, const int idx)
{
        std::cout << "Updating guide number#"<< idx << std::endl;
        boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
        guard.lock();
        if(idx < vm_vector_.size())
        {
            std::cout << "Updating..." << std::endl;
            vm_vector_[idx]->CreateModelFromData(data);
            //vm_vector_[idx]->AlignAndUpateGuide(data);
            std::cout << "...DONE!" << std::endl;
        }
        else
        {
            std::cout << "Guide not available, creating a new guide..." << std::endl;
            InsertVM(data);
        }

        guard.unlock();
        std::cout << "Updating of guide number#"<< idx << " complete." << std::endl;
}
/*
void MechanismManager::UpdateVM_no_rt(double* const data, const int n_rows, const int idx)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    UpdateVM_no_rt(mat,idx);
}
*/

void MechanismManager::ClusterVM(MatrixXd& data)
{
    //std::string file_path = "/home/sybot/gennaro_output/cropped_data_" + std::to_string(loopCnt++); zzz
    std::cout << "Crop incoming data" << std::endl;
    if(CropData(data))
    {
        //tool_box::WriteTxtFile(file_path.c_str(),data);

        std::cout << "Clustering the incoming data" << std::endl;
        boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
        guard.lock();
        if(vm_vector_.size()>0)
        {
            ArrayXd resps(vm_vector_.size());
            ArrayXi h(vm_vector_.size());
            ArrayXd::Index max_resp_idx;
            int dofs = 10; // WTF
            double old_resp, new_resp;
            for(int i=0;i<vm_vector_.size();i++)
            {
                old_resp = vm_vector_[i]->GetResponsability();
                new_resp = vm_vector_[i]->ComputeResponsability(data);

                try
                {
                 h(i) = lratiotest(old_resp,new_resp, dofs);
                }

                catch(...)
                {
                    std::cerr << "Something is wrong with lratiotest, skipping the clustering..." << std::endl;
                    break;
                }

                if(h(i) == 1)
                    resps(i) = -std::numeric_limits<double>::infinity();
                else
                    resps(i) = new_resp;
            }

            if((h == 1).all())
            {
                std::cout << "Creating a new guide..." << std::endl;
                InsertVM(data);
                std::cout << "...DONE!" << std::endl;
            }
            else
            {
                 resps.maxCoeff(&max_resp_idx); // Break the tie
                 UpdateVM(data,max_resp_idx);
            }
        }
        else
        {
            std::cout << "No guides available, creating a new guide..." << std::endl;
            InsertVM(data);
        }
        guard.unlock();
        std::cout << "Clustering complete" << std::endl;
    }
    else
        std::cerr << "Impossible to update guide, data is empty" << std::endl;
}

/*void MechanismManager::ClusterVM_no_rt(double* const data, const int n_rows)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    ClusterVM_no_rt(mat);
}*/

} // namespace
