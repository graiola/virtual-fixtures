#include "mechanism_manager/mechanism_manager.h"
#include "mechanism_manager/mechanism_manager_server.h"

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
  
VirtualMechanismAutom::VirtualMechanismAutom(const double phase_dot_preauto_th, const double phase_dot_th, const double r_th)
{
    assert(phase_dot_th > 0.0);
    assert(phase_dot_preauto_th > phase_dot_th);
    assert(r_th > 0.0);
    phase_dot_preauto_th_ = phase_dot_preauto_th;
    phase_dot_th_ = phase_dot_th;
    r_th_ = r_th;
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
                //if((phase_dot < (phase_dot_ref - phase_dot_th_))) // + NOT INERTIA CONDITION? MAYBE ACCELERATION CONDITION OR FORCE CONDITION
                //if((std::abs(r) > (r_th_)))
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
                std::cout << "****" <<std::endl;
                std::cout << "MANUAL" <<std::endl;
                break;
            case PREAUTO:
                std::cout << "****" <<std::endl;
                std::cout << "PREAUTO" <<std::endl;
                break;
            case AUTO:
                std::cout << "****" <<std::endl;
                std::cout << "AUTO" <<std::endl;
                break;
        }
    }
    loopCnt++;

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

void MechanismManager::InitGuide(vm_t* const vm_tmp_ptr)
{
    VectorXd empty_vect_N(position_dim_);
    MatrixXd empty_mat_NxN(position_dim_,position_dim_);
    MatrixXd empty_mat_Nx1(position_dim_,1);

    vm_state_.push_back(empty_vect_N);
    vm_state_dot_.push_back(empty_vect_N);
    vm_K_.push_back(empty_mat_NxN);
    vm_B_.push_back(empty_mat_NxN);
    vm_vector_.push_back(vm_tmp_ptr);


    vm_vector_.back()->setWeightedDist(use_weighted_dist_);
    vm_jacobian_.push_back(empty_mat_Nx1);
    f_vm_.push_back(empty_vect_N);
    t_versor_.push_back(empty_vect_N);

    vm_fades_.push_back(DynSystemFirstOrder(0.001,10.0));

    if(use_active_guide_)
    {
        vm_vector_.back()->setExecutionTime(execution_time_);
        // Autom
        vm_autom_.push_back(new VirtualMechanismAutom(phase_dot_preauto_th_,phase_dot_th_,r_th_));
    }

    if(second_order_)
    {
        dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setInertia(inertia_);
        dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKr(Kr_);
    }

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
    PushBack(0.0,r_);

#ifdef USE_ROS_RT_PUBLISHER
    rt_publishers_vector_.PushBackEmptyAll();
#endif
}

void MechanismManager::InsertVM_no_rt(std::string& model_name)
{
    std::string model_complete_path(pkg_path_+"/models/gmm/"+model_name); // FIXME change the folder for splines
    std::cout << "Creating the guide... "<< model_complete_path << std::endl;
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock
    vm_t* vm_tmp_ptr = NULL;
    try
    {
        if(second_order_)
            vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path);
        else
            vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path);

        InitGuide(vm_tmp_ptr);

        std::cout << "Guide number#" << vm_vector_.size()-1 << " created." << std::endl;
    }
    catch(...)
    {
        std::cerr << "Impossible to create the guide... "<< model_complete_path << std::endl;
    }

    guard.unlock(); // Unlock
}

void MechanismManager::InsertVM_no_rt(const MatrixXd& data)
{
    std::cout << "Creating the guide from data... "<< std::endl;
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock
    vm_t* vm_tmp_ptr = NULL;
    try
    {
        if(second_order_)
            vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,data);
        else
            vm_tmp_ptr = new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,data);

        InitGuide(vm_tmp_ptr);

        std::cout << "Guide number#" << vm_vector_.size()-1 << " created." << std::endl;
    }
    catch(...)
    {
        std::cerr << "Impossible to create the guide from data... " << std::endl;
    }

    guard.unlock(); // Unlock
}

void MechanismManager::InsertVM_no_rt()
{
    std::string model_name;
    std::cout << "Insert model name: " << std::endl;
    std::cin >> model_name;
    InsertVM_no_rt(model_name);
}

void MechanismManager::SaveVM_no_rt(const int idx)
{
    std::string model_name;
    std::cout << "Insert model name: " << std::endl;
    std::cin >> model_name;
    std::string model_complete_path(pkg_path_+"/models/gmm/"+model_name); // FIXME change the folder for splines

    std::cout << "Saving guide number#"<< idx << " to " << model_complete_path << std::endl;
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    if(idx < vm_vector_.size())
    {
        std::cout << "Saving..." << std::endl;
        if(vm_vector_[idx]->SaveGMMToTxt(model_complete_path))
            std::cout << "...DONE!" << std::endl;
        else
            std::cerr << "Impossible to save the file " << model_complete_path << std::endl;
    }
    else
    {
        std::cout << "Guide number#" << idx << "not available..." << std::endl;
    }
    guard.unlock();
    std::cout << "Saving of guide number#"<< idx << " complete." << std::endl;
}

void MechanismManager::InsertVM()
{
    async_thread_insert_->AddHandler(boost::bind(&MechanismManager::InsertVM_no_rt, this));
    async_thread_insert_->Trigger();
}

void MechanismManager::InsertVM(const MatrixXd& data)
{
    async_thread_insert_->AddHandler(boost::bind(static_cast<void (MechanismManager::*)(const MatrixXd&)>(&MechanismManager::InsertVM_no_rt), this, data));
    async_thread_insert_->Trigger();
}

void MechanismManager::InsertVM(std::string& model_name)
{
    async_thread_insert_->AddHandler(boost::bind(static_cast<void (MechanismManager::*)(std::string&)>(&MechanismManager::InsertVM_no_rt), this, model_name));
    async_thread_insert_->Trigger();
}

void MechanismManager::SaveVM(const int idx)
{
    async_thread_save_->AddHandler(boost::bind(&MechanismManager::SaveVM_no_rt, this, idx));
    async_thread_save_->Trigger();
}

void MechanismManager::DeleteVM(const int idx)
{
    async_thread_delete_->AddHandler(boost::bind(&MechanismManager::DeleteVM_no_rt, this, idx));
    async_thread_delete_->Trigger();
}

void MechanismManager::DeleteVM_no_rt(const int& idx)
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
       std::cout << "Deleting guide number#"<< idx << std::endl;

       delete vm_vector_[idx];
       vm_vector_.erase(vm_vector_.begin()+idx);
       vm_state_.erase(vm_state_.begin()+idx);
       vm_state_dot_.erase(vm_state_dot_.begin()+idx);
       vm_K_.erase(vm_K_.begin()+idx);
       vm_B_.erase(vm_B_.begin()+idx);
       vm_jacobian_.erase(vm_jacobian_.begin()+idx);
       f_vm_.erase(f_vm_.begin()+idx);
       t_versor_.erase(t_versor_.begin()+idx);
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
       Delete(idx,r_);

       std::cout << "Delete of guide  number#"<< idx << " complete." << std::endl;

#ifdef USE_ROS_RT_PUBLISHER
       rt_publishers_vector_.RemoveAll(idx);
#endif
   }
   else
       std::cerr << "Impossible to remove guide number#"<< idx << std::endl;

   guard.unlock();
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

    // Retrain basic parameters for all the virtual mechanisms
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
        //active_guide_node["n_samples_filter"] >> n_samples_filter_;
        active_guide_node["phase_dot_th"] >> phase_dot_th_;
        active_guide_node["phase_dot_preauto_th"] >> phase_dot_preauto_th_;
        active_guide_node["r_th"] >> r_th_;
        active_guide_node["fade_gain"] >> fade_gain_;
        use_active_guide_ = true;

        assert(Kf_ > 0.0);
        assert(Bf_ > 0.0);
        assert(fade_gain_ > 0.0);
        assert(phase_dot_th_ > 0.0);
        assert(phase_dot_preauto_th_ > phase_dot_th_);
        assert(r_th_ > 0.0);
    }
    else
    {
        Kf_ = 0.0;
        Bf_ = 0.0;
        fade_gain_ = 1.0;
        use_active_guide_ = false;
    }

    if(const YAML::Node& second_order_node = main_node["second_order"])
    {
        second_order_node["inertia"] >> inertia_;
        second_order_node["Kr"] >> Kr_;
        second_order_ = true;

        assert(inertia_ > 0.0);
        assert(Kr_ > 0.0);
    }
    else
    {
        inertia_ = 0.1;
        Kr_ = 0.1;
        second_order_ = false;
    }

    assert(K_.size() == B_.size());
    for(unsigned int i=0; i<K_.size(); i++)
    {
        assert(K_[i] >= 0.0);
        assert(B_[i] >= 0.0);
    }

    return true;
}

MechanismManager::MechanismManager()
{
      //Eigen::initParallel();

      async_thread_insert_ = new AsyncThread();
      async_thread_delete_ = new AsyncThread();
      async_thread_update_ = new AsyncThread();
      async_thread_save_   = new AsyncThread();

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

      // Resize
      orientation_dim_ = 4; // NOTE orientation dimension is fixed, quaternion (w q1 q2 q3)
      tmp_eigen_vector_.resize(position_dim_);
      robot_position_.resize(position_dim_);
      robot_velocity_.resize(position_dim_);
      robot_orientation_.resize(orientation_dim_);
      f_pos_.resize(position_dim_);
      f_K_.resize(position_dim_);
      f_B_.resize(position_dim_);
      err_pos_.resize(position_dim_);
      err_vel_.resize(position_dim_);
      f_pos_prev_.resize(position_dim_);
      f_ori_.resize(3); // NOTE The dimension is always 3 for rpy

      // Clear
      tmp_eigen_vector_.fill(0.0);
      robot_position_.fill(0.0);
      robot_velocity_.fill(0.0);
      robot_orientation_ << 1.0, 0.0, 0.0, 0.0;
      f_pos_.fill(0.0);
      f_K_.fill(0.0);
      f_B_.fill(0.0);
      err_pos_.fill(0.0);
      err_vel_.fill(0.0);
      f_pos_prev_.fill(0.0);
      f_ori_.fill(0.0);

      // Chached
      on_guide_prev_ = false;
      nb_vm_prev_ = 0;

      loopCnt = 0;

      collision_detected_ = true; // Let's start not active

#ifdef USE_ROS_RT_PUBLISHER
    try
    {
        ros_node_.Init("mechanism_manager");
        //rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"robot_position",&robot_position_);
        //rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"robot_velocity",&robot_velocity_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"phase",&phase_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"phase_dot",&phase_dot_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"scale",&scales_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"scale_hard",&scales_hard_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"scale_soft",&scales_soft_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"scale_t",&scales_t_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"phase_dot_ref",&phase_dot_ref_);
    }
    catch(const std::runtime_error& e)
    {
      ROS_ERROR("Failed to create the real time publishers: %s",e.what());
    }
#endif


    mm_server_ = new MechanismManagerServer(this,ros_node_.GetNode());
}


MechanismManager::~MechanismManager()
{
      for(int i=0;i<vm_vector_.size();i++)
        delete vm_vector_[i];

      for(int i=0;i<vm_autom_.size();i++)
        delete vm_autom_[i];

      delete async_thread_insert_;
      delete async_thread_delete_;
      delete async_thread_update_;
      delete async_thread_save_;

      delete mm_server_;
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

void MechanismManager::CheckForGuideActivation(const int idx)
{
    //const double r = vm_vector_[idx]->getR();
    const double phase_dot = vm_vector_[idx]->getPhaseDot();
    const double phase_dot_ref = vm_vector_[idx]->getPhaseDotRef();
    vm_autom_[idx]->Step(phase_dot,phase_dot_ref,collision_detected_);
    if(vm_autom_[idx]->GetState())
        vm_vector_[idx]->setActive(true);
    else
        vm_vector_[idx]->setActive(false);

    //r_(idx) = r;
}

void MechanismManager::Update(const prob_mode_t prob_mode)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    if(guard.try_lock())
    {
        // Update the virtual mechanisms states, compute single probabilities
        for(int i=0; i<vm_vector_.size();i++)
        {
            if(use_active_guide_)
                CheckForGuideActivation(i);

            if(second_order_)
            {
                //vm_vector_[i]->Update(robot_position_,robot_velocity_,dt_,scales_(i)); // Add scales here to scale also on the vm
                vm_vector_[i]->Update(robot_position_,robot_velocity_,dt_);
            }
            else
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
            vm_vector_[i]->getK(vm_K_[i]);
            vm_vector_[i]->getB(vm_B_[i]);
            vm_vector_[i]->getJacobian(vm_jacobian_[i]);

            // Compute the gaussian activations
            //scales_(i) = vm_vector_[i]->getGaussian(robot_position_);
            //scales_(i) = vm_vector_[i]->getGaussian(robot_position_,escape_factor_);
            //scales_(i) = vm_vector_[i]->getDistance(robot_position_);
            scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
            //scales_(i) = vm_vector_[i]->PolynomScale(robot_position_);
        }

        f_pos_.fill(0.0); // Reset the force
        double sum = scales_.sum();

        for(int i=0; i<vm_vector_.size();i++)
        {
          // Compute the probabilities
          switch(prob_mode)
          {
            case HARD:
                scales_soft_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
                scales_hard_(i) = scales_(i)/sum;
                scales_(i) =  scales_hard_(i);
                break;
            case POTENTIAL:
                scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
                break;
            case SOFT:
                scales_soft_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
                //scales_soft_(i) = vm_vector_[i]->PolynomScale(robot_position_);
                //scales_hard_(i) = scales_(i)/(sum + std::numeric_limits<double>::epsilon()); // To avoid numerical issues
                scales_hard_(i) = scales_(i)/sum;
                scales_(i) =  scales_soft_(i) * scales_hard_(i);
                //scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_)) * scales_(i)/sum; // To avoid numerical issues
                break;
            default:
              break;
          }
          err_pos_ = vm_state_[i] - robot_position_;
          f_K_ .noalias() = vm_K_[i] * err_pos_;
          err_vel_ = vm_state_dot_[i] - robot_velocity_;
          f_B_.noalias() = vm_B_[i] * err_vel_;

          // Jacobian versor
          t_versor_[i] = vm_jacobian_[i]/vm_jacobian_[i].norm();

          f_vm_[i] = f_K_ + f_B_;

          //f_pos_ += scales_(i) * (f_K_ + f_B_ ); // Sum over all the vms
          //f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_vector_[i]->getState() - robot_position_) + vm_vector_[i]->getB() * (vm_vector_[i]->getStateDot() - robot_velocity_)); // Sum over all the vms
        }

        for(int i=0; i<vm_vector_.size();i++)
            if(scales_hard_(i) > 1.0/static_cast<double>(vm_vector_.size()))
                for(int j=0; j<vm_vector_.size();j++)
                    if(j!=i)
                        scales_t_(j) = vm_fades_[j].IntegrateForward();
                    else
                        scales_t_(j) = vm_fades_[j].IntegrateBackward();

        for(int i=0; i<vm_vector_.size();i++)
        {
            f_pos_ += scales_(i) * f_vm_[i];
            for(int j=0; j<vm_vector_.size();j++)
            {
                if(j!=i)
                    f_pos_ -= scales_(i) * scales_t_(i) * t_versor_[j] * f_vm_[i].dot(t_versor_[j]);
            }
        }

        f_pos_prev_ = f_pos_;
        nb_vm_prev_ = vm_vector_.size();
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

void MechanismManager::UpdateVM(double* data, const int n_rows, const int idx)
{
    async_thread_update_->AddHandler(boost::bind(&MechanismManager::UpdateVM_no_rt, this, data, n_rows, idx));
    async_thread_update_->Trigger();
}

void MechanismManager::UpdateVM(MatrixXd& data, const int idx)
{
    async_thread_update_->AddHandler(boost::bind(&MechanismManager::UpdateVM_no_rt, this, data, idx));
    async_thread_update_->Trigger();
}

void MechanismManager::ClusterVM(double* data, const int n_rows)
{
    async_thread_update_->AddHandler(boost::bind(&MechanismManager::ClusterVM_no_rt, this, data, n_rows));
    async_thread_update_->Trigger();
}

void MechanismManager::ClusterVM(MatrixXd& data)
{
    async_thread_update_->AddHandler(boost::bind(&MechanismManager::ClusterVM_no_rt, this, data));
    async_thread_update_->Trigger();
}

void MechanismManager::UpdateVM_no_rt(MatrixXd& data, const int idx)
{
        std::cout << "Updating guide number#"<< idx << std::endl;
        boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
        guard.lock();
        if(idx < vm_vector_.size())
        {
            std::cout << "Updating..." << std::endl;
            vm_vector_[idx]->UpdateGuide(data);
            //vm_vector_[idx]->AlignAndUpateGuide(data);
            std::cout << "...DONE!" << std::endl;
        }
        else
        {
            std::cout << "Guide not available, creating a new guide..." << std::endl;
            InsertVM_no_rt(data);
        }

        guard.unlock();
        std::cout << "Updating of guide number#"<< idx << " complete." << std::endl;
}

void MechanismManager::UpdateVM_no_rt(double* const data, const int n_rows, const int idx)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    UpdateVM_no_rt(mat,idx);
}
/*
void MechanismManager::ClusterVM_no_rt(MatrixXd& data)
{
    std::cout << "Crop incoming data" << std::endl;
    if(CropData(data))
    {
        std::cout << "Clustering the incoming data" << std::endl;
        boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
        guard.lock();
        if(vm_vector_.size()>0)
        {
            //VectorXd new_resp(vm_vector_.size());
            //VectorXd old_resp(vm_vector_.size());
            double new_resp, old_resp;
            ArrayXd resp_change_in_percentage(vm_vector_.size());
            ArrayXd::Index max_resp_idx;
            for(int i=0;i<vm_vector_.size();i++)
            {

                old_resp = vm_vector_[i]->GetResponsability();
                new_resp = vm_vector_[i]->ComputeResponsability(data);

                resp_change_in_percentage(i) = ((new_resp - old_resp)/old_resp) * 100;

                std::cout << "Responsability of guide number# " << i << " : " << resp_change_in_percentage(i) << std::endl;

            }

            if((resp_change_in_percentage < -30.0).all())
            {

                std::cout << "Creating a new guide..." << std::endl;
                InsertVM_no_rt(data);
                std::cout << "...DONE!" << std::endl;
            }
            else
            {
                 resp_change_in_percentage.maxCoeff(&max_resp_idx);
                 std::cout << "Updating guide number# "<<max_resp_idx<< std::endl;
                 UpdateVM_no_rt(data,max_resp_idx);
                 std::cout << "...DONE!" << std::endl;
            }

        }
        else
        {
            std::cout << "No guides available, creating a new guide..." << std::endl;
            InsertVM_no_rt(data);
        }

        guard.unlock();
        std::cout << "Clustering complete" << std::endl;
    }
    else
        std::cerr << "Impossible to update guide, data is empty" << std::endl;
}
*/

void MechanismManager::ClusterVM_no_rt(MatrixXd& data)
{
    //std::string file_path = "/home/sybot/gennaro_output/cropped_data_" + std::to_string(loopCnt++);
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
                InsertVM_no_rt(data);
                std::cout << "...DONE!" << std::endl;
            }
            else
            {
                 resps.maxCoeff(&max_resp_idx); // Break the tie
                 UpdateVM_no_rt(data,max_resp_idx);
            }
        }
        else
        {
            std::cout << "No guides available, creating a new guide..." << std::endl;
            InsertVM_no_rt(data);
        }

        guard.unlock();
        std::cout << "Clustering complete" << std::endl;
    }
    else
        std::cerr << "Impossible to update guide, data is empty" << std::endl;
}

void MechanismManager::ClusterVM_no_rt(double* const data, const int n_rows)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    ClusterVM_no_rt(mat);
}

} // namespace
