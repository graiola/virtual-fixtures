#include "mechanism_manager/mechanism_manager.h"

////////// Function Approximator
#include <functionapproximators/FunctionApproximatorGMR.hpp>
#include <functionapproximators/MetaParametersGMR.hpp>
#include <functionapproximators/ModelParametersGMR.hpp>

namespace mechanism_manager
{

  using namespace virtual_mechanism_gmr;
  using namespace virtual_mechanism_spline;
  using namespace virtual_mechanism_interface;
  using namespace DmpBbo;
  using namespace tool_box;
  using namespace Eigen;

  typedef FunctionApproximatorGMR fa_t;
  
VirtualMechanismAutom::VirtualMechanismAutom(const double phase_dot_preauto_th, const double phase_dot_th, const double r_th)
{
    assert(phase_dot_th > 0.0);
    assert(phase_dot_preauto_th > phase_dot_th);
    assert(r_th > 0.0);
    phase_dot_preauto_th_ = phase_dot_preauto_th;
    phase_dot_th_ = phase_dot_th;
    r_th_ = r_th;
    state_ = MANUAL;
}

void VirtualMechanismAutom::Step(const double phase_dot,const double phase_dot_ref, const double r)
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
            //if((phase_dot < (phase_dot_ref - phase_dot_th_))) // + NOT INERTIA CONDITION? MAYBE ACCELERATION CONDITION OR FORCE CONDITION
            if((std::abs(r) > (r_th_)))
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
    return activate_vm;
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

void MechanismManager::InsertVM_no_rt(std::string& model_name)
{

    std::string model_complete_path(pkg_path_+"/models/gmm/"+model_name); // FIXME change the folder for splines

    std::cout << "LOADING..."<< model_complete_path << std::endl;

    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock

    std::cout << "CREATING..."<< model_complete_path << std::endl;

    vm_t* vm_tmp_ptr = NULL;
    //vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path);

    //bool on_guide = false;
    /*for(int i=0;i<scales_.size();i++)
        if(scales_(i) > 0.9)
            on_guide = true;*/

    //if(vm_vector_.size() == 0 || on_guide) // NOTE: We should be in free mode if vm_vector_ is empty otherwise we have jumps on the force.
    //{
    try
    {
        if(second_order_)
            vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path);
            //vm_vector_.push_back(vm_tmp_ptr); // NOTE the vm always works in xyz so we use position_dim_
        else
            vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path);
            //vm_vector_.push_back(new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path));

        VectorXd empty_vect(position_dim_);

        vm_state_.push_back(empty_vect);
        vm_state_dot_.push_back(empty_vect);
        vm_jacobian_.push_back(empty_vect);

        vm_vector_.push_back(vm_tmp_ptr);
        vm_vector_.back()->setWeightedDist(use_weighted_dist_);
        vm_vector_.back()->setExecutionTime(execution_time_);

        //filter_alpha_.push_back(new filters::M3DFilter(3)); // 3 = Average filter
        //filter_alpha_.back()->SetN(n_samples_filter_);

        if(second_order_)
        {
            dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setInertia(inertia_);
            dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKr(Kr_);
        }

        //delete vm_tmp_ptr;

        //getchar();
        //std::cout << "OK" << std::endl;

        PushBack(0.0,scales_);
        PushBack(0.0,phase_);
        PushBack(0.0,phase_dot_);
        PushBack(0.0,phase_ddot_);
        PushBack(0.0,phase_ref_);
        PushBack(0.0,phase_dot_ref_);
        PushBack(0.0,phase_ddot_ref_);
        PushBack(0.0,fade_);
        PushBack(0.0,alpha_);

        std::cout << "DONE..."<< model_complete_path << std::endl;

    }
    catch(...)
    {
        std::cout << "ERROR, IMPOSSIBLE TO CREATE THE GUIDE..."<< model_complete_path << std::endl;
    }


    /*std::cout << "Size of vm_vector_: " << vm_vector_.size() << std::endl;
        for(int i=0;i<vm_vector_.size();i++)
            std::cout << "Pointer: " << i+1 << vm_vector_[i] << std::endl;*/

    guard.unlock(); // Unlock

#ifdef USE_ROS_RT_PUBLISHER
    rt_publishers_vector_.PushBackEmptyAll();
#endif


}

void MechanismManager::InsertVM_no_rt()
{

    std::string model_name;
    std::cout << "Insert model name: " << std::endl;
    std::cin >> model_name;

    InsertVM_no_rt(model_name);

}

void MechanismManager::InsertVM()
{
    // Insert only if there are no guides or I am currently on a guide
    thread_insert_ = boost::thread(boost::bind(&MechanismManager::InsertVM_no_rt, this));
}

void MechanismManager::InsertVM(std::string& model_name)
{
    // Insert only if there are no guides or I am currently on a guide
    thread_insert_ = boost::thread(boost::bind(&MechanismManager::InsertVM_no_rt, this, model_name));
}

void MechanismManager::Delete(const int idx, VectorXd& vect)
{
    int n = vect.size()-idx-1;
    vect.segment(idx,n) = vect.tail(n);
    vect.conservativeResize(vect.size()-1);
}

void MechanismManager::PushBack(const double value, VectorXd& vect)
{
    int n = vect.size();
    vect.conservativeResize(n+1,NoChange);
    vect(n) = value;
}

void MechanismManager::DeleteVM(const int idx)
{
    // Delete only if I am not on the guide to erase

    //MechanismManager::DeleteVM_no_rt(idx);
    thread_delete_ = boost::thread(&MechanismManager::DeleteVM_no_rt, this, idx);
    //thread_delete_.join();
}

void MechanismManager::DeleteVM_no_rt(const int& idx)
{

   std::cout << "DELETE GUIDE N#"<< idx << std::endl;

   boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
   guard.lock();
   if(idx < vm_vector_.size())
   {
       delete vm_vector_[idx];
       vm_vector_.erase(vm_vector_.begin()+idx);
       vm_state_.erase(vm_state_.begin()+idx);
       vm_state_dot_.erase(vm_state_dot_.begin()+idx);
       vm_jacobian_.erase(vm_jacobian_.begin()+idx);
       //filter_alpha_.erase(filter_alpha_.begin()+idx);

       Delete(idx,scales_);
       Delete(idx,phase_);
       Delete(idx,phase_dot_);
       Delete(idx,phase_ddot_);
       Delete(idx,phase_ref_);
       Delete(idx,phase_dot_ref_);
       Delete(idx,phase_ddot_ref_);
       Delete(idx,fade_);
       Delete(idx,alpha_);

#ifdef USE_ROS_RT_PUBLISHER
       rt_publishers_vector_.RemoveAll(idx);
#endif
   }
   guard.unlock();

   std::cout << "DELETE COMPLETE"<< idx << std::endl;

}

bool MechanismManager::ReadConfig(std::string file_path)
{
    YAML::Node main_node;

    try
    {
        main_node = YAML::LoadFile(file_path);
    }
    catch(...)
    {
        return false;
    }

    //std::string prob_mode_tmp;

    // Retrain basic parameters for all the virtual mechanisms
    //main_node["prob_mode"] >> prob_mode_tmp;
    main_node["use_weighted_dist"] >> use_weighted_dist_;
    main_node["position_dim"] >> position_dim_;
    main_node["K"] >> K_;
    main_node["B"] >> B_;
    main_node["escape_factor"] >> escape_factor_;

    if (const YAML::Node& active_guide_node = main_node["active_guide"])
    {
        active_guide_node["execution_time"] >> execution_time_;
        active_guide_node["Kf"] >> Kf_;
        active_guide_node["Bf"] >> Bf_;
        active_guide_node["n_samples_filter"] >> n_samples_filter_;
        active_guide_node["phase_dot_th"] >> phase_dot_th_;
        active_guide_node["r_th"] >> r_th_;
        active_guide_node["pre_auto_th"] >> pre_auto_th_;
        active_guide_node["fade_gain"] >> fade_gain_;
    }

    if(const YAML::Node& second_order_node = main_node["second_order"])
    {
        second_order_node["inertia"] >> inertia_;
        second_order_node["Kr"] >> Kr_;
        second_order_ = true;
    }
    else
    {
        inertia_ = 0.1;
        Kr_ = 0.1;
        second_order_ = false;
    }

    assert(K_ >= 0.0);
    assert(B_ >= 0.0);
    assert(inertia_ > 0.0);
    assert(Kr_ > 0.0);
    assert(n_samples_filter_ > 0);
    assert(phase_dot_th_ > 0.0);
    assert(pre_auto_th_ > phase_dot_th_);
    assert(r_th_ > 0.0);

    /*
    if (prob_mode_tmp == "hard")
        prob_mode_ = HARD;
    else if (prob_mode_tmp == "potential")
        prob_mode_ = POTENTIAL;
    else if (prob_mode_tmp == "soft")
        prob_mode_ = SOFT;
    else
        prob_mode_ = POTENTIAL; // Default
    */
    return true;
}


MechanismManager::MechanismManager()
{

#ifdef INCLUDE_ROS_CODE
      pkg_path_ = ros::package::getPath("mechanism_manager");
      std::string config_file_path(pkg_path_+"/config/cfg.yml");
      if(ReadConfig(config_file_path))
      {
        ROS_INFO("Loaded config file: %s",config_file_path.c_str());
      }
      else
      {
        ROS_ERROR("Can not load config file: %s",config_file_path.c_str());
      }
#else
      pkg_path_ = "/home/sybot/workspace/virtual-fixtures/mechanism_manager"; // FIXME
      //pkg_path_ = PATH_TO_PKG;
      std::string config_file_path(pkg_path_+"/config/cfg.yml");
      bool file_read = ReadConfig(config_file_path);
      assert(file_read);
#endif

      //vm_vector_.size() = vm_vector_.size();
      //assert(vm_vector_.size() >= 1);

      // Resize
      orientation_dim_ = 4; // NOTE orientation dimension is fixed, quaternion (w q1 q2 q3)
      tmp_eigen_vector_.resize(position_dim_);
      robot_position_.resize(position_dim_);
      robot_velocity_.resize(position_dim_);
      robot_orientation_.resize(orientation_dim_);
      f_pos_.resize(position_dim_);
      f_pos_prev_.resize(position_dim_);
      f_ori_.resize(3); // NOTE The dimension is always 3 for rpy


      f_rob_.resize(position_dim_);
      t_versor_.resize(position_dim_);
      f_rob_t_ = 0.0;
      f_rob_n_ = 0.0;
      f_rob_norm_ = 1.0;


      // Clear
      tmp_eigen_vector_.fill(0.0);
      robot_position_.fill(0.0);
      robot_velocity_.fill(0.0);
      robot_orientation_ << 1.0, 0.0, 0.0, 0.0;
      f_pos_.fill(0.0);
      f_pos_prev_.fill(0.0);
      f_ori_.fill(0.0);

      //vm_vector_.reserve(6);
      //vm_state_.reserve(6);
      //vm_state_dot_.reserve(6);

      // Chached
      on_guide_prev_ = false;
      nb_vm_prev_ = 0;

#ifdef USE_ROS_RT_PUBLISHER
    try
    {
        ros_node_.Init("mechanism_manager");
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"phase",&phase_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"scale",&scales_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"alpha",&alpha_);
    }
    catch(const std::runtime_error& e)
    {
      ROS_ERROR("Failed to create the real time publishers: %s",e.what());
    }
#endif
}

MechanismManager::~MechanismManager()
{
      for(int i=0;i<vm_vector_.size();i++)
      {
        delete vm_vector_[i];
        //delete filter_phase_dot_[i];
        //delete filter_phase_ddot_[i];
        //delete filter_alpha_[i];
        //delete vm_autom_[i];
      }

      thread_insert_.join();
      thread_delete_.join();
}

void MechanismManager::Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr, const prob_mode_t prob_mode)
{
    assert(dt > 0.0);
    dt_ = dt;

    // FIXME We assume no orientation
    use_orientation_ = false;

    robot_position_ = VectorXd::Map(robot_position_ptr, position_dim_);
    robot_velocity_ = VectorXd::Map(robot_velocity_ptr, position_dim_);

    Update(prob_mode);

    VectorXd::Map(f_out_ptr, position_dim_) = f_pos_;
}

void MechanismManager::Update(const VectorXd& robot_pose, const VectorXd& robot_velocity, double dt, VectorXd& f_out, const prob_mode_t prob_mode)
{
    assert(dt > 0.0);
    dt_ = dt;

    assert(robot_velocity.size() == position_dim_);
    robot_velocity_ = robot_velocity;

    if(robot_pose.size() == position_dim_)
    {
        assert(f_out.size() == position_dim_);
        use_orientation_ = false;
        robot_position_ = robot_pose;
    }
    else if(robot_pose.size() == position_dim_ + orientation_dim_)
    {
        assert(f_out.size() == position_dim_ + 3);
        use_orientation_ = true;
        robot_position_ = robot_pose.segment(0,position_dim_);
        robot_orientation_ = robot_pose.segment(position_dim_,4);
    }

    Update(prob_mode);

    if(use_orientation_)
        f_out << f_pos_, f_ori_;
    else
        f_out = f_pos_;
}
void MechanismManager::Update(const prob_mode_t prob_mode)
{ 
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        // Update the virtual mechanisms states, compute single probabilities
        for(int i=0; i<vm_vector_.size();i++)
        {
            //vm_vector_[i]->Update(robot_position_,robot_velocity_,dt_,scales_(i)); // Add scales here to scale also on the vm
            vm_vector_[i]->Update(robot_position_,robot_velocity_,dt_);

            // Retrain variables for plots
            phase_(i) = vm_vector_[i]->getPhase();
            phase_dot_(i) = vm_vector_[i]->getPhaseDot();
            phase_ddot_(i) = vm_vector_[i]->getPhaseDotDot();
            phase_ref_(i) = vm_vector_[i]->getPhaseRef();
            phase_dot_ref_(i) = vm_vector_[i]->getPhaseDotRef();
            phase_ddot_ref_(i) = vm_vector_[i]->getPhaseDotDotRef();
            fade_(i) = vm_vector_[i]->getFade();

            // Retrain position/velocity and jacobian from the virtual mechanisms
            vm_vector_[i]->getState(vm_state_[i]);
            vm_vector_[i]->getStateDot(vm_state_dot_[i]);
            vm_vector_[i]->getJacobian(vm_jacobian_[i]);

            /////// HACKKKK
            // Robot force components
            /*f_rob_ = (robot_position_-vm_state_[i]);
            f_rob_norm_ = f_rob_.norm();
            t_versor_ = vm_jacobian_[i]/vm_jacobian_[i].norm();
            f_rob_t_ = f_rob_.dot(t_versor_); // tangent
            f_rob_n_ = (f_rob_ - f_rob_t_ * t_versor_).norm(); // normal
            // compute escape_factor based on the tangentforce applied by the user/robot
            //alpha_(i) = std::abs(f_rob_t_/f_rob_norm_)*(1.0-escape_factor_)+escape_factor_;
            alpha_(i) = std::abs(f_rob_n_/f_rob_norm_)*(escape_factor_-1.0)+1.0; // opposte
            //alpha_(i) = escape_factor_;
            alpha_(i) = filter_alpha_[i]->Step(alpha_(i));
            scales_(i) = vm_vector_[i]->getGaussian(robot_position_,alpha_(i));*/
            /////// END HACKKKK

            // Compute the gaussian activations
            //scales_(i) = vm_vector_[i]->getGaussian(robot_position_);
            scales_(i) = vm_vector_[i]->getGaussian(robot_position_,escape_factor_);
        }

        f_pos_.fill(0.0); // Reset the force
        double sum = scales_.sum();

        for(int i=0; i<vm_vector_.size();i++)
        {
          // Compute the probabilities
          switch(prob_mode)
          {
            case HARD:
                scales_(i) =  scales_(i)/sum;
                break;
            case POTENTIAL:
                scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
                break;
            case SOFT:
                //scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_)) * scales_(i)/(sum + std::numeric_limits<double>::epsilon()); // To avoid numerical issues
                scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_)) * scales_(i)/sum; // To avoid numerical issues
                break;
            default:
              break;
          }
            f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_state_[i] - robot_position_) + vm_vector_[i]->getB() * (vm_state_dot_[i] - robot_velocity_)); // Sum over all the vms
          //f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_vector_[i]->getState() - robot_position_) + vm_vector_[i]->getB() * (vm_vector_[i]->getStateDot() - robot_velocity_)); // Sum over all the vms           
        }
        f_pos_prev_ = f_pos_;
        nb_vm_prev_ = vm_vector_.size();
        //guard_.unlock();
    }
    else
        f_pos_ = f_pos_prev_; // Keep the previous force while the vectors are updating


#ifdef USE_ROS_RT_PUBLISHER
    rt_publishers_vector_.PublishAll();
#endif

}

void MechanismManager::GetVmPosition(const int idx, double* const position_ptr)
{
    tmp_eigen_vector_ = VectorXd::Map(position_ptr, position_dim_);
    GetVmPosition(idx,tmp_eigen_vector_);
    VectorXd::Map(position_ptr, position_dim_) = tmp_eigen_vector_;
}

void MechanismManager::GetVmVelocity(const int idx, double* const velocity_ptr)
{
    tmp_eigen_vector_ = VectorXd::Map(velocity_ptr, position_dim_);
    GetVmVelocity(idx,tmp_eigen_vector_);
    VectorXd::Map(velocity_ptr, position_dim_) = tmp_eigen_vector_;
}

void MechanismManager::GetVmPosition(const int idx, Eigen::VectorXd& position)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        if(idx < vm_vector_.size())
            vm_vector_[idx]->getState(position);
        //guard_.unlock();
    }
}

void MechanismManager::GetVmVelocity(const int idx, Eigen::VectorXd& velocity)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        if(idx < vm_vector_.size())
            vm_vector_[idx]->getStateDot(velocity);
        //guard_.unlock();
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
        //guard_.unlock();
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
        //guard_.unlock();
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
            if(scales_(i) > 0.9)
                on_guide = true;
    }
    else
        on_guide = on_guide_prev_;

    on_guide_prev_ = on_guide;
    return on_guide;
}

} // namespace
