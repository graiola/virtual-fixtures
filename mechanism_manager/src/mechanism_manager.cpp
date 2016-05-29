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

    /*
    if(loopcnt_%100==0)
    {
        switch(state_)
        {
            case MANUAL:
                std::cout << "MANUAL" << std::endl;
                break;
            case PREAUTO:
                std::cout << "PREAUTO" << std::endl;
                break;
            case AUTO:
                std::cout << "AUTO" << std::endl;
                break;
        }
    }
    loopcnt_++;
    */

    return activate_vm;
}

void MechanismManager::Stop()
{
    for(int i=0;i<vm_nb_;i++)
    {
      vm_vector_[i]->Stop();
    }

}

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


        /*if(normalize)
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
        }*/
    }
	return true;
}
  
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
      vm_nb_ = vm_vector_.size();
      assert(vm_nb_ >= 1);

      // Initialize the virtual mechanisms and support vectors
      for(int i=0; i<vm_nb_;i++)
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
      for(int i=0; i<vm_nb_;i++)
      {
        filter_phase_dot_[i]->SetN(n_samples_filter_);
        filter_phase_ddot_[i]->SetN(n_samples_filter_);
      }

      // Some Initializations
      tmp_eigen_vector_.resize(position_dim_);
      use_orientation_ = false;
      active_guide_.resize(vm_nb_,false);
      scales_.resize(vm_nb_);
      phase_.resize(vm_nb_);
      escape_factors_.resize(vm_nb_);
      //Kf_.resize(vm_nb_);
      phase_dot_.resize(vm_nb_);
      phase_ddot_.resize(vm_nb_);
      f_rob_t_.resize(vm_nb_);
      f_rob_n_.resize(vm_nb_);
      robot_position_.resize(position_dim_);
      robot_velocity_.resize(position_dim_);
      robot_orientation_.resize(orientation_dim_);
      orientation_error_.resize(3); // rpy
      cross_prod_.resize(3);
      /*prev_orientation_error_.resize(3);
      orientation_integral_.resize(3);
      orientation_derivative_.resize(3);*/
      f_pos_.resize(position_dim_);
      f_ori_.resize(3); // NOTE The dimension is always 3 for rpy
      f_rob_.resize(position_dim_);
      t_versor_.resize(position_dim_);
      escape_field_.resize(vm_nb_);
      //escape_field_compare_.resize(vm_nb_);
      phase_dot_filt_.resize(vm_nb_);
      phase_ddot_filt_.resize(vm_nb_);
      phase_dot_ref_.resize(vm_nb_);
      phase_ddot_ref_.resize(vm_nb_);
      phase_ref_.resize(vm_nb_);
      phase_dot_ref_upper_.resize(vm_nb_);
      phase_dot_ref_lower_.resize(vm_nb_);
      fade_.resize(vm_nb_);
      r_.resize(vm_nb_);
      //p_dot_integrated_.resize(vm_nb_);
      torque_.resize(vm_nb_);

      // Clear
      tmp_eigen_vector_.fill(0.0);
      scales_.fill(0.0);
      phase_.fill(0.0);
      escape_factors_.fill(escape_factor_);
      f_rob_t_.fill(0.0);
      f_rob_n_.fill(0.0);
      //Kf_.fill(0.0);
      phase_dot_.fill(0.0);
      phase_ddot_.fill(0.0);
      robot_position_.fill(0.0);
      robot_velocity_.fill(0.0);
      orientation_error_.fill(0.0);
      cross_prod_.fill(0.0);
      /*prev_orientation_error_.fill(0.0);
      orientation_integral_.fill(0.0);
      orientation_derivative_.fill(0.0);*/
      robot_orientation_ << 1.0, 0.0, 0.0, 0.0;
      f_pos_.fill(0.0);
      f_ori_.fill(0.0);
      f_rob_.fill(0.0);
      t_versor_.fill(0.0);
      escape_field_.fill(0.0);
      //escape_field_compare_.fill(0.0);
      phase_dot_filt_.fill(0.0);
      phase_ddot_filt_.fill(0.0);
      phase_dot_ref_.fill(0.0);
      phase_ddot_ref_.fill(0.0);
      phase_ref_.fill(0.0);
      phase_dot_ref_upper_.fill(0.0);
      phase_dot_ref_lower_.fill(0.0);
      fade_.fill(0.0);
      r_.fill(0.0);
      //p_dot_integrated_.fill(0.0);
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
          for(int i=0; i<vm_nb_;i++)
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
      scale_threshold_ = 1.0/static_cast<double>(vm_nb_);
      
      if(vm_nb_>1)
          scale_threshold_ = scale_threshold_ + 0.2;

}
  
MechanismManager::~MechanismManager()
{
      //for(int i=0;i<vm_vector_.size();i++)


      for(int i=0;i<vm_nb_;i++)
      {
        delete vm_vector_[i];
        delete filter_phase_dot_[i];
        delete filter_phase_ddot_[i];
        delete vm_autom_[i];
      }
}

/*void MechanismManager::MoveForward()
{
    for(int i=0; i<vm_nb_;i++)
      vm_vector_[i].move_forward_ = true;
}

void MechanismManager::MoveBackward()
{
    for(int i=0; i<vm_nb_;i++)
      vm_vector_[i].move_forward_ = false;
}*/

/*void MechanismManager::Update(const VectorXd& robot_pose, const VectorXd& robot_velocity, double dt, VectorXd& f_out, bool force_applied, bool move_forward)
{
    for(int i=0; i<vm_nb_;i++)
    {
      if(move_forward)
        vm_vector_[i]->moveForward();
      else
        vm_vector_[i]->moveBackward();
    }
    Update(robot_pose,robot_velocity,dt,f_out,force_applied);
}*/

/*void MechanismManager::Update(const VectorXd& robot_pose, const VectorXd& robot_velocity, double dt, VectorXd& f_out, bool force_applied)
{   
    for(int i=0; i<vm_nb_;i++)
    {
      //if (force_applied == false && scales_(i) >= scale_threshold_ && use_active_guide_[i] == true)
      if (force_applied == false && use_active_guide_[i] == true)
      {
	      vm_vector_[i]->setActive(true);
	      active_guide_[i] = true;
	      //std::cout << "Active " <<i<< std::endl;
      }
      else
	  {
	      vm_vector_[i]->setActive(false);
	      active_guide_[i] = false;
	      //std::cout << "Deactive "<<i<< std::endl;
	  }
    }
    Update(robot_pose,robot_velocity,dt,f_out);
}*/

void MechanismManager::GetVmPosition(const int idx, double* const position_ptr)
{
    assert(idx <= vm_vector_.size());
    tmp_eigen_vector_ = VectorXd::Map(position_ptr, position_dim_);
    vm_vector_[idx]->getState(tmp_eigen_vector_);
    VectorXd::Map(position_ptr, position_dim_) = tmp_eigen_vector_;
}
void MechanismManager::GetVmVelocity(const int idx, double* const velocity_ptr)
{
    assert(idx <= vm_vector_.size());
    tmp_eigen_vector_ = VectorXd::Map(velocity_ptr, position_dim_);
    vm_vector_[idx]->getStateDot(tmp_eigen_vector_);
    VectorXd::Map(velocity_ptr, position_dim_) = tmp_eigen_vector_;
}

/*void MechanismManager::Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr, const bool user_force_applied)
{
    user_force_applied_ = user_force_applied;
    Update(robot_position_ptr,robot_velocity_ptr,dt,f_out_ptr);
}*/

/*void MechanismManager::Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr)
{


    //phase_dot_ref_upper_(0) = phase_dot_ref_(0) + range_;
    //phase_dot_ref_lower_(0) = phase_dot_ref_(0) - range_;


    //if((phase_dot_filt_(0) <= (phase_dot_ref_(0) + range_)) && (phase_dot_filt_(0) >= (phase_dot_ref_(0) - range_)))
    //{
    //    //std::cout << "GOOOOOOOOOOOOOOOOOOOOOOOOO" << std::endl;
//
     //   user_force_applied_ = false;
    //}
    //else
     //   user_force_applied_ = true;



    //if(std::abs(phase_ddot_filt_(0)) > 2.0)

    //user_force_applied_ = user_force_applied;
    Update(robot_position_ptr,robot_velocity_ptr,dt,f_out_ptr);
}*/

void MechanismManager::Update(const double* robot_position_ptr, const double* robot_velocity_ptr, double dt, double* f_out_ptr)
{
    assert(dt > 0.0);
    dt_ = dt;

    // FIXME We assume no orientation
    use_orientation_ = false;

    robot_position_ = VectorXd::Map(robot_position_ptr, position_dim_);
    robot_velocity_ = VectorXd::Map(robot_velocity_ptr, position_dim_);

    Update();

    VectorXd::Map(f_out_ptr, position_dim_) = f_pos_;
}

/*void MechanismManager::Update(const VectorXd& robot_pose, const VectorXd& robot_velocity, double dt, VectorXd& f_out, const bool user_force_applied)
{
    user_force_applied_ = user_force_applied;
    Update(robot_pose,robot_velocity,dt,f_out);
}*/

void MechanismManager::Update(const VectorXd& robot_pose, const VectorXd& robot_velocity, double dt, VectorXd& f_out)
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

    Update();

    if(use_orientation_)
        f_out << f_pos_, f_ori_;
    else
        f_out = f_pos_;
}
void MechanismManager::Update()
{

	// Update the virtual mechanisms states, compute single probabilities
	for(int i=0; i<vm_nb_;i++)
	{
        // Check for activation
        phase_dot_filt_(i) = filter_phase_dot_[i]->Step(phase_dot_(i)); // FIXME: change to multi virtual mechanisms
        phase_ddot_filt_(i) = filter_phase_ddot_[i]->Step(phase_ddot_(i)); // FIXME: change to multi virtual mechanisms

        r_(i) = vm_vector_[i]->getR();
        torque_(i) = vm_vector_[i]->getTorque();

        vm_autom_[i]->Step(phase_dot_filt_(i),phase_dot_ref_(i),r_(i));
        activated_[i] = vm_autom_[i]->GetState();


        phase_dot_ref_upper_(i) = phase_dot_ref_(i) + phase_dot_th_;
        phase_dot_ref_lower_(i) = phase_dot_ref_(i) - phase_dot_th_;

        //if (force_applied == false && scales_(i) >= scale_threshold_ && use_active_guide_[i] == true)
        if (activated_[i] == true && use_active_guide_[i] == true)
        {
            vm_vector_[i]->setActive(true);
            //active_guide_[i] = true;
            //std::cout << "Active" <<std::endl;
        }
        else
        {
            vm_vector_[i]->setActive(false);
            //active_guide_[i] = false;
            //std::cout << "Deactive" <<std::endl;
        }

      vm_vector_[i]->Update(robot_position_,robot_velocity_,dt_,scales_(i)); // Add scales here to scale also on the vm
	  switch(prob_mode_) 
	  {
	    case HARD:
          scales_(i) = vm_vector_[i]->getGaussian(robot_position_);
	      break;
	    case POTENTIAL:
          //scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
	      break;
	    case SOFT:
          scales_(i) = vm_vector_[i]->getGaussian(robot_position_);
	      break;
        case ESCAPE:
          scales_(i) = vm_vector_[i]->getGaussian(robot_position_);
          break;
	    default:
	      break;
	  }
	  
	  // Take the phase for each vm (For plots)
	  phase_(i) = vm_vector_[i]->getPhase();
      phase_dot_(i) = vm_vector_[i]->getPhaseDot();
      phase_ddot_(i) = vm_vector_[i]->getPhaseDotDot();
      phase_ref_(i) = vm_vector_[i]->getPhaseRef();
      phase_dot_ref_(i) = vm_vector_[i]->getPhaseDotRef();
      phase_ddot_ref_(i) = vm_vector_[i]->getPhaseDotDotRef();
      fade_(i) = vm_vector_[i]->getFade();
      //p_dot_integrated_(i) = vm_vector_[i]->getPDotIntegrated();

      // Compute the force from the vms
      vm_vector_[i]->getState(vm_state_[i]);
      vm_vector_[i]->getStateDot(vm_state_dot_[i]);
      vm_vector_[i]->getQuaternion(vm_quat_[i]);
      vm_vector_[i]->getJacobian(vm_jacobian_[i]);

      // Robot force components
      f_rob_ = vm_vector_[i]->getK() * (robot_position_-vm_state_[i]) + vm_vector_[i]->getB() * robot_velocity_;
      t_versor_ = vm_jacobian_[i]/vm_jacobian_[i].norm();
      f_rob_t_(i) = f_rob_.dot(t_versor_); // tangent
      f_rob_n_(i) = (f_rob_ - f_rob_t_(i) * t_versor_).norm(); // normal

      // compute escape_factor based on the normal force applied by the user/robot
      escape_factors_(i) = escape_factor_/f_norm_ * std::abs(f_rob_n_(i));
      if(escape_factors_(i) > escape_factor_)
          escape_factors_(i) = escape_factor_;
	}

    sum_ = scales_.sum();
    f_pos_.fill(0.0); // Reset the force
    f_ori_.fill(0.0); // Reset the force

	for(int i=0; i<vm_nb_;i++)
	{
	  // Compute the conditional probabilities
	  switch(prob_mode_) 
	  {
	    case HARD:
            scales_(i) =  scales_(i)/sum_;
            break;
	    case POTENTIAL:
            scales_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
            break;
	    case SOFT:
            escape_field_(i) = std::exp(-escape_factor_*vm_vector_[i]->getDistance(robot_position_));
            scales_(i) = escape_field_(i) * scales_(i)/sum_;
            break;
        case ESCAPE:
            //escape_field_compare_(i) = escape_factors_(i);
            escape_field_(i) = std::exp(-escape_factors_(i)*vm_vector_[i]->getDistance(robot_position_));
            scales_(i) = escape_field_(i) * scales_(i)/sum_;
            break;
	    default:
	      break;
	  }

	  //vm_vector_[i]->getLocalKernel(vm_kernel_[i]);
	  //K_ = vm_vector_[i]->getK();
      //B_ = vm_vector_[i]->getB();

      if(use_orientation_)
      {

        cross_prod_.noalias() = vm_quat_[i].segment<3>(1).cross(robot_orientation_.segment<3>(1));

        orientation_error_(0) = robot_orientation_(0) * vm_quat_[i](1)- vm_quat_[i](0) * robot_orientation_(1) - cross_prod_(0);
        orientation_error_(1) = robot_orientation_(0) * vm_quat_[i](2)- vm_quat_[i](0) * robot_orientation_(2) - cross_prod_(1);
        orientation_error_(2) = robot_orientation_(0) * vm_quat_[i](3)- vm_quat_[i](0) * robot_orientation_(3) - cross_prod_(2);

        //orientation_integral_ = orientation_integral_ + orientation_error_ * dt_;
        //orientation_derivative_ = (orientation_error_ - prev_orientation_error_)/dt_;

        //f_ori_ += scales_(i) * (5.0 * orientation_error_ + 0.0 * orientation_integral_ + 0.0 * orientation_derivative_);

        //prev_orientation_error_ = orientation_error_;

        //f_ori_(0)  += scales_(i) * 5.0 * (robot_orientation_(0) * vm_quat_[i](1)- vm_quat_[i](0) * robot_orientation_(1));
        //f_ori_(1)  += scales_(i) * 5.0 * (robot_orientation_(0) * vm_quat_[i](2)- vm_quat_[i](0) * robot_orientation_(2));
        //f_ori_(2)  += scales_(i) * 5.0 * (robot_orientation_(0) * vm_quat_[i](3)- vm_quat_[i](0) * robot_orientation_(3));
        f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_state_[i] - robot_position_) + vm_vector_[i]->getB() * (vm_state_dot_[i] - robot_velocity_)); // Sum over all the vms
      }
      else
      {
        f_pos_ += scales_(i) * (vm_vector_[i]->getK() * (vm_state_[i] - robot_position_) + vm_vector_[i]->getB() * (vm_state_dot_[i] - robot_velocity_)); // Sum over all the vms
      }
      
    }	   

    /*if(loopCnt%1000==0)
    {
        std::cout << "** ****" <<std::endl;
        std::cout << "** AFTR **" <<std::endl;
        std::cout << scales_ <<std::endl;
    }*/
    //loopCnt++;

    /*if(loopCnt%100==0)
    {
        std::cout << "** ****" <<std::endl;

    }*/


    //loopCnt++;

	//UpdateTrackingReference(robot_position);
	
	#ifdef USE_ROS_RT_PUBLISHER
        rt_publishers_values_.PublishAll();
        //rt_publishers_wrench_.PublishAll();
        rt_publishers_path_.PublishAll();
	#endif
}
  
}
