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
    bool on_guide = false;

    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock

    for(int i=0;i<scales_.size();i++)
        if(scales_(i) > 0.9)
            on_guide = true;

    if(vm_vector_.size() == 0 || on_guide) // NOTE: We should be in free mode if vm_vector_ is empty otherwise we have jumps on the force.
    {
        if(second_order_)
        {
            vm_vector_.push_back(new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path)); // NOTE the vm always works in xyz so we use position_dim_
            dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setInertia(inertia_);
            dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKr(Kr_);
        }
        else
            vm_vector_.push_back(new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>(position_dim_,K_,B_,Kf_,Bf_,fade_gain_,model_complete_path));

        vm_vector_.back()->setWeightedDist(use_weighted_dist_);
        vm_vector_.back()->setExecutionTime(execution_time_);

        vm_state_.push_back(VectorXd(position_dim_));
        vm_state_dot_.push_back(VectorXd(position_dim_));
    }

    PushBack(0.0,scales_);
    PushBack(0.0,phase_);
    PushBack(0.0,phase_dot_);
    PushBack(0.0,phase_ddot_);
    PushBack(0.0,phase_ref_);
    PushBack(0.0,phase_dot_ref_);
    PushBack(0.0,phase_ddot_ref_);
    PushBack(0.0,fade_);

    guard.unlock(); // Unlock

#ifdef USE_ROS_RT_PUBLISHER
    rt_publishers_vector_.PushBackEmptyAll();
#endif
}

void MechanismManager::InsertVM(std::string& model_name)
{
    // Insert only if there are no guides or I am currently on a guide

    //MechanismManager::InsertVM_no_rt(model_name);
    thread_insert_ = boost::thread(&MechanismManager::InsertVM_no_rt, this, model_name);
    //thread_insert_.join();
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
   boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
   guard.lock();
   if(idx < vm_vector_.size())
   {
       vm_vector_.erase(vm_vector_.begin()+idx);
       vm_state_.erase(vm_state_.begin()+idx);
       vm_state_dot_.erase(vm_state_dot_.begin()+idx);

       Delete(idx,scales_);
       Delete(idx,phase_);
       Delete(idx,phase_dot_);
       Delete(idx,phase_ddot_);
       Delete(idx,phase_ref_);
       Delete(idx,phase_dot_ref_);
       Delete(idx,phase_ddot_ref_);
       Delete(idx,fade_);

#ifdef USE_ROS_RT_PUBLISHER
       rt_publishers_vector_.RemoveAll(idx);
#endif
   }
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

    std::string prob_mode_tmp;

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

      // Clear
      tmp_eigen_vector_.fill(0.0);
      robot_position_.fill(0.0);
      robot_velocity_.fill(0.0);
      robot_orientation_ << 1.0, 0.0, 0.0, 0.0;
      f_pos_.fill(0.0);
      f_pos_prev_.fill(0.0);
      f_ori_.fill(0.0);

#ifdef USE_ROS_RT_PUBLISHER
    try
    {
        ros_node_.Init("mechanism_manager");
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"phase",&phase_);
        rt_publishers_vector_.AddPublisher(ros_node_.GetNode(),"scale",&scales_);
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
    //if(guard_.try_lock())
    //{
        //std::cout << "Inside" << std::endl;

        // Update the virtual mechanisms states, compute single probabilities
        for(int i=0; i<vm_vector_.size();i++)
        {
            vm_vector_[i]->Update(robot_position_,robot_velocity_,dt_,scales_(i)); // Add scales here to scale also on the vm

            scales_(i) = vm_vector_[i]->getGaussian(robot_position_);

            // Retrain variables for plots
            phase_(i) = vm_vector_[i]->getPhase();
            phase_dot_(i) = vm_vector_[i]->getPhaseDot();
            phase_ddot_(i) = vm_vector_[i]->getPhaseDotDot();
            phase_ref_(i) = vm_vector_[i]->getPhaseRef();
            phase_dot_ref_(i) = vm_vector_[i]->getPhaseDotRef();
            phase_ddot_ref_(i) = vm_vector_[i]->getPhaseDotDotRef();
            fade_(i) = vm_vector_[i]->getFade();

            vm_vector_[i]->getState(vm_state_[i]);
            vm_vector_[i]->getStateDot(vm_state_dot_[i]);
        }

        f_pos_.fill(0.0); // Reset the force

        for(int i=0; i<vm_vector_.size();i++)
        {
          // Compute the conditional probabilities
          switch(prob_mode)
          {
            case HARD:
                scales_(i) =  scales_(i)/scales_.sum();
                break;
            case POTENTIAL:
                scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
                break;
            case SOFT:
                scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_)) * scales_(i)/scales_.sum();
                break;
            default:
              break;
          }

            f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_state_[i] - robot_position_) + vm_vector_[i]->getB() * (vm_state_dot_[i] - robot_velocity_)); // Sum over all the vms
          //f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_vector_[i]->getState() - robot_position_) + vm_vector_[i]->getB() * (vm_vector_[i]->getStateDot() - robot_velocity_)); // Sum over all the vms           
        }
        f_pos_prev_ = f_pos_;
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
        /*if(idx < vm_vector_.size())
        {
            tmp_eigen_vector_ = VectorXd::Map(position_ptr, position_dim_);
            vm_vector_[idx]->getState(tmp_eigen_vector_);
            VectorXd::Map(position_ptr, position_dim_) = tmp_eigen_vector_;
        }*/

    tmp_eigen_vector_ = VectorXd::Map(position_ptr, position_dim_);
    GetVmPosition(idx,tmp_eigen_vector_);
    VectorXd::Map(position_ptr, position_dim_) = tmp_eigen_vector_;
}

void MechanismManager::GetVmVelocity(const int idx, double* const velocity_ptr)
{
    /*if(idx < vm_vector_.size())
    {
        tmp_eigen_vector_ = VectorXd::Map(velocity_ptr, position_dim_);
        vm_vector_[idx]->getStateDot(tmp_eigen_vector_);
        VectorXd::Map(velocity_ptr, position_dim_) = tmp_eigen_vector_;
    }*/

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
    {
        return vm_vector_.size();
    }
}

} // namespace




/*
bool MechanismManager::ReadConfig(std::string file_path) // FIXME Switch to ros param server
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
	
	// Retrain the parameters from yaml file
	std::vector<std::string> model_names;
    //std::vector<std::vector<double> > quat_start;
    //std::vector<std::vector<double> > quat_end;
	std::string models_path(pkg_path_+"/models/");
    std::string prob_mode_string, mechanism_type;
    int position_dim;
    double K, B, Kf, Bf, inertia, fade_gain, Kr, Kfi;
    bool normalize, second_order;
	
	main_node["models"] >> model_names;
    main_node["quat_start"] >> quat_start_;
    main_node["quat_end"] >> quat_end_;   
	main_node["prob_mode"] >> prob_mode_string;
	main_node["use_weighted_dist"] >> use_weighted_dist_;
    main_node["execution_time"] >> execution_time_;
	main_node["use_active_guide"] >> use_active_guide_;
    main_node["position_dim"] >> position_dim;
    main_node["K"] >> K;
    main_node["B"] >> B;
    main_node["Kf"] >> Kf;
    main_node["Bf"] >> Bf;
    main_node["Kfi"] >> Kfi;
    main_node["n_samples_filter"] >> n_samples_filter_;
    main_node["phase_dot_th"] >> phase_dot_th_;
    main_node["r_th"] >> r_th_;
    main_node["pre_auto_th"] >> pre_auto_th_;
    main_node["normalize"] >> normalize;
    main_node["mechanism_type"] >> mechanism_type;
    main_node["escape_factor"] >> escape_factor_;
    main_node["f_norm"] >> f_norm_;
    main_node["second_order"] >> second_order;
    main_node["inertia"] >> inertia;
    main_node["Kr"] >> Kr;
    main_node["fade_gain"] >> fade_gain;

    assert(position_dim == 2 || position_dim == 3);
    position_dim_ = position_dim;

    assert(K >= 0.0);
    assert(B >= 0.0);
    assert(inertia > 0.0);
    assert(Kr > 0.0);
    assert(n_samples_filter_ > 0);
    assert(phase_dot_th_ > 0.0);
    assert(pre_auto_th_ > phase_dot_th_);
    assert(r_th_ > 0.0);


	if (prob_mode_string == "hard")
	    prob_mode_ = HARD;
	else if (prob_mode_string == "potential")
	    prob_mode_ = POTENTIAL;
	else if (prob_mode_string == "soft")
	    prob_mode_ = SOFT;
    else if (prob_mode_string == "escape")
        prob_mode_ = ESCAPE;
	else
	    prob_mode_ = POTENTIAL; // Default


    if (mechanism_type == "gmm")
        mechanism_type_ = GMM;
    else if (mechanism_type == "spline")
        mechanism_type_ = SPLINE;
    else
        mechanism_type_ = GMM; // Default


    // Create the virtual mechanisms starting from the models
 	for(int i=0;i<model_names.size();i++)
	{

        std::vector<std::vector<double> > data;


        switch(mechanism_type_)
        {
          case GMM:
           {
            ReadTxtFile((models_path+"gmm/"+model_names[i]).c_str(),data);
            ModelParametersGMR* model_parameters_gmr = ModelParametersGMR::loadGMMFromMatrix(models_path+"gmm/"+model_names[i]);
            boost::shared_ptr<fa_t> fa_tmp_shr_ptr(new FunctionApproximatorGMR(model_parameters_gmr)); // Convert to shared pointer
            if(normalize)
            {
                if(second_order)
                {
                    vm_vector_.push_back(new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceSecondOrder>(position_dim_,K,B,Kf,Bf,fade_gain,fa_tmp_shr_ptr)); // NOTE the vm always works in xyz so we use position_dim_
                    dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setInertia(inertia);
                    dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKr(Kr);
                    dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKfi(Kfi);
                }
                else
                    vm_vector_.push_back(new VirtualMechanismGmrNormalized<VirtualMechanismInterfaceFirstOrder>(position_dim_,K,B,Kf,Bf,fade_gain,fa_tmp_shr_ptr)); // NOTE the vm always works in xyz so we use position_dim_
            }
            else
            {
                if(second_order)
                {
                    vm_vector_.push_back(new VirtualMechanismGmr<VirtualMechanismInterfaceSecondOrder>(position_dim_,K,B,Kf,Bf,fade_gain,fa_tmp_shr_ptr));
                    dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setInertia(inertia);
                    dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKr(Kr);
                    dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKfi(Kfi);
                }
                else
                    vm_vector_.push_back(new VirtualMechanismGmr<VirtualMechanismInterfaceFirstOrder>(position_dim_,K,B,Kf,Bf,fade_gain,fa_tmp_shr_ptr));
            }
            }
            break;
          case SPLINE:
            if(second_order)
            {
                vm_vector_.push_back(new VirtualMechanismSpline<VirtualMechanismInterfaceSecondOrder>(position_dim_,K,B,Kf,Bf,fade_gain,models_path+"spline/"+model_names[i]));
                dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setInertia(inertia);
                dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKr(Kr);
                dynamic_cast<VirtualMechanismInterfaceSecondOrder*>(vm_vector_.back())->setKfi(Kfi);
            }
            else
                vm_vector_.push_back(new VirtualMechanismSpline<VirtualMechanismInterfaceFirstOrder>(position_dim_,K,B,Kf,Bf,fade_gain,models_path+"spline/"+model_names[i]));
            break;
          default:
            break;
        }
    }
	return true;
}
 */
/*
MechanismManager::MechanismManager()
{
      //position_dim_ = 2; // NOTE position dimension is fixed, xyz
      orientation_dim_ = 4; // NOTE orientation dimension is fixed, quaternion (w q1 q2 q3)

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

      // Number of virtual mechanisms
      vm_vector_.size() = vm_vector_.size();
      assert(vm_vector_.size() >= 1);

      // Initialize the virtual mechanisms and support vectors
      for(int i=0; i<vm_vector_.size();i++)
      {
         //vm_vector_[i]->Init();
         vm_vector_[i]->Init(quat_start_[i],quat_end_[i]);
         vm_vector_[i]->setWeightedDist(use_weighted_dist_[i]);
         vm_vector_[i]->setExecutionTime(execution_time_[i]);
         vm_state_.push_back(VectorXd(position_dim_));
         vm_state_dot_.push_back(VectorXd(position_dim_));
         vm_jacobian_.push_back(VectorXd(position_dim_));
         vm_quat_.push_back(VectorXd(orientation_dim_));
         // HACK
         filter_phase_dot_.push_back(new filters::M3DFilter(3)); // 3 = Average filter
         filter_phase_ddot_.push_back(new filters::M3DFilter(3));
         vm_autom_.push_back(new VirtualMechanismAutom(pre_auto_th_,phase_dot_th_,r_th_)); // phase_dot_preauto_th, phase_dot_th
         activated_.push_back(false); // NOTE we assume the guide not active at the beginning
      }
      for(int i=0; i<vm_vector_.size();i++)
      {
        filter_phase_dot_[i]->SetN(n_samples_filter_);
        filter_phase_ddot_[i]->SetN(n_samples_filter_);
      }

      // Some Initializations
      tmp_eigen_vector_.resize(position_dim_);
      use_orientation_ = false;
      active_guide_.resize(vm_vector_.size(),false);
      scales_.resize(vm_vector_.size());
      phase_.resize(vm_vector_.size());
      escape_factors_.resize(vm_vector_.size());
      //Kf_.resize(vm_vector_.size());
      phase_dot_.resize(vm_vector_.size());
      phase_ddot_.resize(vm_vector_.size());
      f_rob_t_.resize(vm_vector_.size());
      f_rob_n_.resize(vm_vector_.size());
      robot_position_.resize(position_dim_);
      robot_velocity_.resize(position_dim_);
      robot_orientation_.resize(orientation_dim_);
      orientation_error_.resize(3); // rpy
      cross_prod_.resize(3);
      f_pos_.resize(position_dim_);
      f_ori_.resize(3); // NOTE The dimension is always 3 for rpy
      f_rob_.resize(position_dim_);
      t_versor_.resize(position_dim_);
      escape_field_.resize(vm_vector_.size());
      phase_dot_filt_.resize(vm_vector_.size());
      phase_ddot_filt_.resize(vm_vector_.size());
      phase_dot_ref_.resize(vm_vector_.size());
      phase_ddot_ref_.resize(vm_vector_.size());
      phase_ref_.resize(vm_vector_.size());
      phase_dot_ref_upper_.resize(vm_vector_.size());
      phase_dot_ref_lower_.resize(vm_vector_.size());
      fade_.resize(vm_vector_.size());
      r_.resize(vm_vector_.size());
      torque_.resize(vm_vector_.size());

      // Clear
      tmp_eigen_vector_.fill(0.0);
      scales_.fill(0.0);
      phase_.fill(0.0);
      escape_factors_.fill(escape_factor_);
      f_rob_t_.fill(0.0);
      f_rob_n_.fill(0.0);
      phase_dot_.fill(0.0);
      phase_ddot_.fill(0.0);
      robot_position_.fill(0.0);
      robot_velocity_.fill(0.0);
      orientation_error_.fill(0.0);
      cross_prod_.fill(0.0);
      robot_orientation_ << 1.0, 0.0, 0.0, 0.0;
      f_pos_.fill(0.0);
      f_ori_.fill(0.0);
      f_rob_.fill(0.0);
      t_versor_.fill(0.0);
      escape_field_.fill(0.0);
      phase_dot_filt_.fill(0.0);
      phase_ddot_filt_.fill(0.0);
      phase_dot_ref_.fill(0.0);
      phase_ddot_ref_.fill(0.0);
      phase_ref_.fill(0.0);
      phase_dot_ref_upper_.fill(0.0);
      phase_dot_ref_lower_.fill(0.0);
      fade_.fill(0.0);
      r_.fill(0.0);
      torque_.fill(0.0);

      #ifdef USE_ROS_RT_PUBLISHER
      try
      {
          ros_node_.Init("mechanism_manager");
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase",phase_.size(),&phase_);
          //rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"Kf",Kf_.size(),&Kf_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_dot",phase_dot_.size(),&phase_dot_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_ddot",phase_ddot_.size(),&phase_ddot_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"scales",scales_.size(),&scales_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"robot_velocity_vect",robot_velocity_.size(),&robot_velocity_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"robot_position_vect",robot_position_.size(),&robot_position_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"f_pos",f_pos_.size(),&f_pos_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"escape_factors",escape_factors_.size(),&escape_factors_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"f_rob_n",f_rob_n_.size(),&f_rob_n_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"f_rob_t",f_rob_t_.size(),&f_rob_t_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"escape_field",escape_field_.size(),&escape_field_);
          //rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"escape_field_compare",escape_field_compare_.size(),&escape_field_compare_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_dot_filt",phase_dot_filt_.size(),&phase_dot_filt_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_ddot_filt",phase_ddot_filt_.size(),&phase_ddot_filt_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_ddot_ref",phase_ddot_ref_.size(),&phase_ddot_ref_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_ref",phase_ref_.size(),&phase_ref_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_dot_ref",phase_dot_ref_.size(),&phase_dot_ref_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_dot_ref_upper",phase_dot_ref_upper_.size(),&phase_dot_ref_upper_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"phase_dot_ref_lower",phase_dot_ref_lower_.size(),&phase_dot_ref_lower_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"fade",fade_.size(),&fade_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"r",r_.size(),&r_);
          rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"torque",torque_.size(),&torque_);
          //rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"p_dot_integrated",p_dot_integrated_.size(),&p_dot_integrated_);
          //rt_publishers_values_.AddPublisher(ros_node_.GetNode(),"prob",prob_.size(),&prob_);
          //rt_publishers_pose_.AddPublisher(ros_node_.GetNode(),"tracking_reference",tracking_reference_.size(),&tracking_reference_);
          rt_publishers_path_.AddPublisher(ros_node_.GetNode(),"robot_pos",robot_position_.size(),&robot_position_);
          //rt_publishers_wrench_.AddPublisher(ros_node_.GetNode(),"f_rob",f_pos_.size(),&f_pos_);

          //std::string topic_name = "f_rob";
          //std::string root_name = "map";
          //boost::shared_ptr<RealTimePublisherWrench> tmp_ptr = boost::make_shared<RealTimePublisherWrench>(ros_node_.GetNode(),topic_name,root_name);
          //rt_publishers_wrench_.AddPublisher(tmp_ptr,&f_pos_);
          for(int i=0; i<vm_vector_.size();i++)
          {
            std::string topic_name = "vm_pos_" + std::to_string(i+1);
            rt_publishers_path_.AddPublisher(ros_node_.GetNode(),topic_name,vm_state_[i].size(),&vm_state_[i]);
            //topic_name = "vm_ker_" + std::to_string(i+1);
            //boost::shared_ptr<RealTimePublisherMarkers> tmp_ptr = boost::make_shared<RealTimePublisherMarkers>(ros_node_.GetNode(),topic_name,root_name_);
            //rt_publishers_markers_.AddPublisher(tmp_ptr,&vm_kernel_[i]);
          }
      }
      catch(const std::runtime_error& e)
      {
        ROS_ERROR("Failed to create the real time publishers: %s",e.what());
      }
      #endif

      // Define the scale threshold to check when a guide is more "probable"
      scale_threshold_ = 1.0/static_cast<double>(vm_vector_.size());
      
      if(vm_vector_.size()>1)
          scale_threshold_ = scale_threshold_ + 0.2;

}
  */

