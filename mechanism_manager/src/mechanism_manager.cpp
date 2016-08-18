#include "mechanism_manager/mechanism_manager.h"

namespace mechanism_manager
{

  using namespace virtual_mechanism;
  using namespace tool_box;
  using namespace Eigen;

MechanismManager::MechanismManager(int position_dim)
{
      if(!ReadConfig())
      {
        PRINT_ERROR("MechanismManager: Can not read config file");
      }

      assert(position_dim == 1 || position_dim == 2);
      position_dim_ = position_dim;

      // Resize
      f_K_.resize(position_dim_);
      f_B_.resize(position_dim_);
      f_vm_.resize(position_dim_);
      err_pos_.resize(position_dim_);
      err_vel_.resize(position_dim_);

      // Clear
      f_K_.fill(0.0);
      f_B_.fill(0.0);
      f_vm_.fill(0.0);
      err_pos_.fill(0.0);
      err_vel_.fill(0.0);

      loopCnt = 0;

      pkg_path_ = ros::package::getPath(ROS_PKG_NAME);

      rt_idx_ = 0;
      no_rt_idx_ = 1;
}

MechanismManager::~MechanismManager()
{
    for(size_t i=0;i<2;i++)
        vm_buffers_[i].clear();
}

void MechanismManager::ExpandVectors(vm_t* const vm_tmp_ptr)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock(); // Lock

    std::vector<GuideStruct>& no_rt_buffer = vm_buffers_[no_rt_idx_];
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    no_rt_buffer.clear();

    // Copy
    for (size_t i = 0; i < rt_buffer.size(); i++)
      no_rt_buffer.push_back(rt_buffer[i]); // FIXME possible problems in the copy!!! (fade)

    GuideStruct new_guide;
    new_guide.name = vm_tmp_ptr->getName();
    new_guide.fade = DynSystemFirstOrder(10.0); // FIXME since it's a dynamic system, it should be a pointer or in the vm
    new_guide.scale = 0.0;
    new_guide.scale_t = 0.0;
    new_guide.guide = boost::shared_ptr<vm_t>(vm_tmp_ptr);

    no_rt_buffer.push_back(new_guide);

    // Circular swap
    rt_idx_ = (rt_idx_ + 1) % 2;
    no_rt_idx_ = (no_rt_idx_ + 1) % 2;

    guard.unlock(); // Unlock
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
    PRINT_INFO("Creating the guide from file... " << model_complete_path);
    vm_t* vm_tmp_ptr = NULL;
    try
    {
        vm_tmp_ptr = vm_factory_.Build(model_complete_path);
    }
    catch(...)
    {
        PRINT_WARNING("Impossible to create the guide... "<<model_complete_path);
        return;
    }

    vm_tmp_ptr->setName(model_name);
    ExpandVectors(vm_tmp_ptr);

    PRINT_INFO("... Done!");
}

void MechanismManager::InsertVM(const MatrixXd& data)
{
    PRINT_INFO("Creating the guide from data...");
    vm_t* vm_tmp_ptr = NULL;
    try
    {
        vm_tmp_ptr = vm_factory_.Build(data);
    }
    catch(...)
    {
        PRINT_WARNING("Impossible to create the guide from data...");
        return;
    }

    vm_tmp_ptr->setName("new_guide");
    ExpandVectors(vm_tmp_ptr);

    PRINT_INFO("... Done!");
}

void MechanismManager::SaveVM(const int idx)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx<rt_buffer.size())
    {
        std::string model_complete_path(pkg_path_+"/models/gmm/"+rt_buffer[idx].name);
        PRINT_INFO("Saving guide number#"<<idx<<" to " << model_complete_path);

        if(!rt_buffer[idx].guide->SaveModelToFile(model_complete_path))
            PRINT_ERROR("Impossible to save the file " << model_complete_path);
        else
             PRINT_INFO("Saving complete");
    }
    else
        PRINT_WARNING("Guide number#"<<idx<<" not available");
    guard.unlock();
}

void MechanismManager::DeleteVM(const int idx)
{
   PRINT_INFO("Deleting guide number#"<<idx);
   bool delete_complete = false;

   boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
   guard.lock(); // Lock

   std::vector<GuideStruct>& no_rt_buffer = vm_buffers_[no_rt_idx_];
   std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
   no_rt_buffer.clear();

   // Copy all the guides, except the one to delete
   // It will be deleted from the no_rt_buffer at the next Delete or Insert
   // thanks to the clear()
   for (size_t i = 0; i < rt_buffer.size(); i++)
   {
       if(i != idx)
            no_rt_buffer.push_back(rt_buffer[i]);
       else
           delete_complete = true;
   }

   // Circular swap
   rt_idx_ = (rt_idx_ + 1) % 2;
   no_rt_idx_ = (no_rt_idx_ + 1) % 2;

   guard.unlock();

   if(delete_complete)
        PRINT_INFO("Delete of guide number#"<<idx<<" complete");
   else
       PRINT_WARNING("Impossible to remove guide number#"<<idx);
}

void MechanismManager::GetVmName(const int idx, std::string& name)
{
    PRINT_INFO("Get name of guide number#"<<idx);
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx<rt_buffer.size())
    {
        name = rt_buffer[idx].name;
    }
    else
        PRINT_WARNING("Guide number#"<<idx<<" not available");
    guard.unlock();
}

void MechanismManager::GetVmNames(std::vector<std::string>& names)
{
    PRINT_INFO("Get the guides name");
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    names.resize(rt_buffer.size());
    for(size_t i=0;i<rt_buffer.size();i++)
    {
        names[i] = rt_buffer[i].name;
    }
    guard.unlock();
}

void MechanismManager::UpdateVM(MatrixXd& data, const int idx)
{
        /*std::cout << "Updating guide number#"<< idx << std::endl;
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
        std::cout << "Updating of guide number#"<< idx << " complete." << std::endl;*/
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
    /*std::cout << "Crop incoming data" << std::endl;
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
        std::cerr << "Impossible to update guide, data is empty" << std::endl;*/
}

/*void MechanismManager::ClusterVM_no_rt(double* const data, const int n_rows)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    ClusterVM_no_rt(mat);
}*/

///// RT METHODS

void MechanismManager::Update(const VectorXd& robot_position, const VectorXd& robot_velocity, double dt, VectorXd& f_out, const scale_mode_t scale_mode)
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];

    double sum = 0.0;
    for(int i=0; i<rt_buffer.size();i++)
    {
        // Update the virtual mechanisms states
        rt_buffer[i].guide->Update(robot_position,robot_velocity,dt);
        // Compute the scale for each mechanism
        rt_buffer[i].scale = rt_buffer[i].guide->getScale(robot_position,escape_factor_);
        sum += rt_buffer[i].scale;
    }

    f_out.fill(0.0); // Reset the force

    // Compute the global scales
    for(int i=0; i<rt_buffer.size();i++)
    {
      rt_buffer[i].scale_hard = rt_buffer[i].scale/sum;
      switch(scale_mode)
      {
        case HARD:
            rt_buffer[i].scale =  rt_buffer[i].scale_hard;
            break;
        case SOFT:
            rt_buffer[i].scale =  rt_buffer[i].scale * rt_buffer[i].scale_hard;
            break;
        default:
          PRINT_ERROR("Wrong scale_mode.");
          break;
      }
    }

    // For each mechanism that is not active (low scale value), remove the force component tangent to
    // the active mechanism jacobian. In this way we avoid to be locked if one or more guide overlap in a certain area.
    // Use a first order filter to gently remove these components.
    for(int i=0; i<rt_buffer.size();i++)
        if(rt_buffer[i].scale_hard > 1.0/static_cast<double>(rt_buffer.size()))
            for(int j=0; j<rt_buffer.size();j++)
                if(j!=i)
                    rt_buffer[j].scale_t = rt_buffer[j].fade.IntegrateForward();
                else
                    rt_buffer[j].scale_t = rt_buffer[j].fade.IntegrateBackward();

    // Compute the force for each mechanism, remove the antagonist force components
    for(int i=0; i<rt_buffer.size();i++)
    {
        err_pos_ = rt_buffer[i].guide->getState() - robot_position;
        f_K_ .noalias() = rt_buffer[i].guide->getK() * err_pos_;
        err_vel_ = rt_buffer[i].guide->getStateDot() - robot_velocity;
        f_B_.noalias() = rt_buffer[i].guide->getB() * err_vel_;

        // Sum spring force + damping force for the current mechanism
        f_vm_ = f_K_ + f_B_;

        f_out += rt_buffer[i].scale * f_vm_;
        for(int j=0; j<rt_buffer.size();j++)
        {
            if(j!=i)
                f_out -= rt_buffer[i].scale * rt_buffer[i].scale_t * rt_buffer[j].guide->getJacobianVersor() * f_vm_.dot(rt_buffer[j].guide->getJacobianVersor());
        }
    }
}

void MechanismManager::GetVmPosition(const int idx, Eigen::VectorXd& position)
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx < rt_buffer.size())
        rt_buffer[idx].guide->getState(position);
}

void MechanismManager::GetVmVelocity(const int idx, Eigen::VectorXd& velocity)
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx < rt_buffer.size())
        rt_buffer[idx].guide->getStateDot(velocity);
}

double MechanismManager::GetPhase(const int idx)
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx < rt_buffer.size())
        return rt_buffer[idx].guide->getPhase();
    else
        return 0.0;
}

double MechanismManager::GetScale(const int idx)
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx < rt_buffer.size())
        return rt_buffer[idx].scale;
    else
        return 0.0;
}

int MechanismManager::GetNbVms()
{
    // NO RT Version
    //PRINT_INFO("Get number of guides");
    //boost::recursive_mutex::scoped_lock guard(mtx_);
    //std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    //return rt_buffer.size();

     std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
     return rt_buffer.size();
}

bool MechanismManager::OnVm()
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];

    bool on_guide = false;

    for(int i=0;i<rt_buffer.size();i++)
    {
        if(rt_buffer[i].scale > 0.9) // We are on a guide if it's scale is ... (so that we are on it)
            on_guide = true;
    }

    return on_guide;
}

void MechanismManager::Stop()
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    for(int i=0;i<rt_buffer.size();i++)
        rt_buffer[i].guide->Stop();
}

} // namespace
