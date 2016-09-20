/**
 * @file   mechanism_manager.cpp
 * @brief  Manager of guides.
 * @author Gennaro Raiola
 *
 * This file is part of virtual-fixtures, a set of libraries and programs to create
 * and interact with a library of virtual guides.
 * Copyright (C) 2014-2016 Gennaro Raiola, ENSTA-ParisTech
 *
 * virtual-fixtures is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * virtual-fixtures is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with virtual-fixtures.  If not, see <http://www.gnu.org/licenses/>.
 */

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

      guide_unique_id_ = 0;

      rt_idx_ = 0;
      no_rt_idx_ = 1;

      scale_mode_ = SOFT; // By default use soft guides

      merge_th_ = 0;
}

MechanismManager::~MechanismManager()
{
    for(size_t i=0;i<2;i++)
        vm_buffers_[i].clear();
}

void MechanismManager::AddNewVm(vm_t* const vm_tmp_ptr, std::string& name)
{
    if(!CheckForNamesCollision(name))
    {
        boost::recursive_mutex::scoped_lock guard(mtx_);
        //guard.lock(); // Lock

        if(scale_mode_ == HARD)
        {
            PRINT_WARNING("Impossible to insert the guide while in HARD mode.");
            return;
        }

        std::vector<GuideStruct>& no_rt_buffer = vm_buffers_[no_rt_idx_];
        std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
        no_rt_buffer.clear();

        // Copy
        for (size_t i = 0; i < rt_buffer.size(); i++)
          no_rt_buffer.push_back(rt_buffer[i]); // FIXME possible problems in the copy!!! (fade)

        GuideStruct new_guide;
        new_guide.name = name;
        new_guide.scale = 0.0;
        new_guide.scale_t = 0.0;
        new_guide.guide = boost::shared_ptr<vm_t>(vm_tmp_ptr);
        new_guide.fade = boost::shared_ptr<DynSystemFirstOrder>(new DynSystemFirstOrder(10.0)); // FIXME since it's a dynamic system, it should be a pointer or in the vm

        // Define a new ros node with the same name as the guide
#ifdef USE_ROS_RT_PUBLISHER
        new_guide.guide->InitRtPublishers(name);
#endif
        // Add the new guide to the buffer
        no_rt_buffer.push_back(new_guide);

        // Circular swap
        rt_idx_ = (rt_idx_ + 1) % 2;
        no_rt_idx_ = (no_rt_idx_ + 1) % 2;

        //guard.unlock(); // Unlock

        PRINT_INFO("... Done!");
    }
    else
        PRINT_WARNING("Impossible to insert the guide, guide already existing.");
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

void MechanismManager::InsertVm(std::string& model_name)
{
    if(model_name.empty())
    {
        PRINT_WARNING("Impossible to create the guide, empty name.");
        return;
    }

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

    AddNewVm(vm_tmp_ptr,model_name);
}

void MechanismManager::InsertVm(const MatrixXd& data)
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

    std::string default_name = "guide_"+std::to_string(++guide_unique_id_);
    AddNewVm(vm_tmp_ptr,default_name);
}

void MechanismManager::InsertVm(double* data, const int n_rows)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    InsertVm(mat);
}

void MechanismManager::UpdateVm(MatrixXd& data, const int idx)
{
    PRINT_INFO("Update the guide.");

    boost::recursive_mutex::scoped_lock guard(mtx_);
    //guard.lock(); // Lock

    if(scale_mode_ == HARD)
    {
        PRINT_WARNING("Impossible to update the guide while in HARD mode.");
        return;
    }

    std::vector<GuideStruct>& no_rt_buffer = vm_buffers_[no_rt_idx_];
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    no_rt_buffer.clear();

    if(idx<rt_buffer.size())
    {
        // Clone the vm to update
        vm_t* vm_tmp_ptr = NULL;
        vm_tmp_ptr = rt_buffer[idx].guide->Clone();

        // Update
        // Behavior:
        //  - Spline: substitute the model
        //  - GMR: incremental training
        vm_tmp_ptr->CreateModelFromData(data);
        //vm_tmp_ptr->AlignAndUpateGuide(data);

        GuideStruct updated_guide;
        updated_guide.name = rt_buffer[idx].name;
        updated_guide.scale = rt_buffer[idx].scale;
        updated_guide.scale_t = rt_buffer[idx].scale_t;
        updated_guide.guide = boost::shared_ptr<vm_t>(vm_tmp_ptr);
        updated_guide.fade = rt_buffer[idx].fade; // Copy the shared pointer

        for (size_t i = 0; i < rt_buffer.size(); i++)
        {
            if(i != idx)
                 no_rt_buffer.push_back(rt_buffer[i]); // Copy all the vms except the one to update
            else
                 no_rt_buffer.push_back(updated_guide);
        }

        // Circular swap
        rt_idx_ = (rt_idx_ + 1) % 2;
        no_rt_idx_ = (no_rt_idx_ + 1) % 2;
    }
    else
        PRINT_WARNING("Impossible to update the guide.");

    //guard.unlock(); // Unlock
}

void MechanismManager::ClusterVm(MatrixXd& data)
{
    // TODO Check if the guide is a probabilistic one
    // otherwise skip

    if(CropData(data))
    {
        boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
        guard.lock(); // Lock

        std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
        if(rt_buffer.size()>0 && merge_th_ > 0)
        {
            ArrayXd resps(rt_buffer.size());
            ArrayXi h(rt_buffer.size());
            ArrayXd::Index max_resp_idx;
            //int dofs = 10; // WTF Export that
            double old_resp, new_resp;
            for(int i=0;i<rt_buffer.size();i++)
            {
                old_resp = rt_buffer[i].guide->GetResponsability();
                new_resp = rt_buffer[i].guide->ComputeResponsability(data);
                try
                {
                    h(i) = lratiotest(old_resp,new_resp, merge_th_);
                }
                catch(...)
                {
                    PRINT_WARNING("Something is wrong with lratiotest, skipping the clustering.");
                    break;
                }

                if(h(i) == 1)
                    resps(i) = -std::numeric_limits<double>::infinity();
                else
                    resps(i) = new_resp;
            }

            if((h == 1).all())
            {
                //PRINT_INFO("Creating a new guide.");
                InsertVm(data);
            }
            else
            {
                //PRINT_INFO("Update guide: " << max_resp_idx);
                resps.maxCoeff(&max_resp_idx); // Break the tie
                UpdateVm(data,max_resp_idx);
            }
        }
        else
        {
            //PRINT_INFO("No guide available, creating a new one");
            InsertVm(data);
        }
        guard.unlock();
    }
    else
        PRINT_WARNING("Impossible to update guide, data is empty.");
}

void MechanismManager::ClusterVm(double* const data, const int n_rows)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    ClusterVm(mat);
}

/*
void MechanismManager::UpdateVM_no_rt(double* const data, const int n_rows, const int idx)
{
    MatrixXd mat = MatrixXd::Map(data,n_rows,position_dim_);
    UpdateVM_no_rt(mat,idx);
}
*/

void MechanismManager::SaveVm(const int idx)
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

void MechanismManager::DeleteVm(const int idx)
{
   PRINT_INFO("Deleting guide number#"<<idx);
   bool delete_complete = false;

   boost::recursive_mutex::scoped_lock guard(mtx_);
   //guard.lock(); // Lock
   if(scale_mode_ == HARD)
   {
       PRINT_WARNING("Impossible to delete the guide while in HARD mode.");
       return;
   }
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
    //PRINT_INFO("Get the guides name");
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

void MechanismManager::SetVmName(const int idx, std::string& name)
{
    PRINT_INFO("Set name of guide number#"<<idx);
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    if(idx<rt_buffer.size())
    {
        if(!CheckForNamesCollision(name))
        {
            rt_buffer[idx].name = name;
#ifdef USE_ROS_RT_PUBLISHER
            rt_buffer[idx].guide->InitRtPublishers(name);
#endif
        }
        else
            PRINT_WARNING("Name already used, please change it");
    }
    else
        PRINT_WARNING("Guide number#"<<idx<<" not available");
    guard.unlock();
}

void MechanismManager::SetVmMode(const scale_mode_t mode)
{
    PRINT_INFO("Set mode for the virtual mechanisms");
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();

    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];

    if(rt_buffer.size()>0)
    {
        switch(mode)
        {
          case HARD:
            while(!OnVm()) // Pass to Hard when on guide
              boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            scale_mode_ = HARD;
            PRINT_INFO("Set mode to HARD");
            break;
          case SOFT:
            scale_mode_ = SOFT;
            PRINT_INFO("Set mode to SOFT");
            break;
          default:
            scale_mode_ = SOFT;
            PRINT_INFO("Set mode to SOFT");
            break;
        }
    }
    else
        PRINT_WARNING("Can not change guide mode, no guide available.");

    guard.unlock();
}
scale_mode_t& MechanismManager::GetVmMode()
{
    return scale_mode_;
}


void MechanismManager::SetMergeThreshold(int merge_th)
{
    assert(merge_th >= 0 && merge_th <= 100); //0: Don't merge, 100: Merge all
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    merge_th_ = merge_th;
    guard.unlock();
    PRINT_INFO("Set Merge threshold: "<< merge_th);
}

void MechanismManager::GetMergeThreshold(int& merge_th)
{
    boost::unique_lock<mutex_t> guard(mtx_, boost::defer_lock);
    guard.lock();
    merge_th = merge_th_;
    guard.unlock();
    PRINT_INFO("Get Merge threshold: "<< merge_th);
}

bool MechanismManager::CheckForNamesCollision(const std::string& name)
{
    bool collision = false;
    boost::recursive_mutex::scoped_lock guard(mtx_);
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];

    for(size_t i = 0; i<rt_buffer.size(); i++)
    {
        if(std::strcmp(name.c_str(),rt_buffer[i].name.c_str()) == 0)
            collision = true;
    }

    return collision;
}

///// RT METHODS

void MechanismManager::Update(const VectorXd& robot_position, const VectorXd& robot_velocity, double dt, VectorXd& f_out)
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];

    double sum = 0.0;
    for(int i=0; i<rt_buffer.size();i++)
    {
        // Compute the scale for each mechanism
        rt_buffer[i].scale = rt_buffer[i].guide->getScale(robot_position,escape_factor_);
        // Update the virtual mechanisms states
        rt_buffer[i].guide->Update(robot_position,robot_velocity,dt,rt_buffer[i].scale);
        sum += rt_buffer[i].scale;
    }

    f_out.fill(0.0); // Reset the force

    // Compute the global scales
    for(int i=0; i<rt_buffer.size();i++)
    {
      rt_buffer[i].scale_hard = rt_buffer[i].scale/sum;
      switch(scale_mode_)
      {
        case HARD:
            rt_buffer[i].scale =  rt_buffer[i].scale_hard;
            break;
        case SOFT:
            rt_buffer[i].scale =  rt_buffer[i].scale * rt_buffer[i].scale_hard;
            break;
        default:
          rt_buffer[i].scale =  rt_buffer[i].scale * rt_buffer[i].scale_hard; // Soft
          break;
      }
    }

    // For each mechanism that is not active (low scale value), remove the force component tangent to
    // the active mechanism jacobian. In this way we avoid to be locked if one or more guide overlap in a certain area.
    // Use a first order filter to gently remove these components.

    //1) Find the active mechanism (i.e. the one with the max scale)
    double max_scale = 0.0;
    int i_active = 0;
    for(int i=0; i<rt_buffer.size();i++)
    {
        if(rt_buffer[i].scale_hard > max_scale)
        {
            i_active = i;
            max_scale = rt_buffer[i].scale_hard;
        }
    }
    //2) Activate the filters
    for(int j=0; j<rt_buffer.size();j++)
        if(j==i_active) // active
            rt_buffer[j].scale_t = rt_buffer[j].fade->IntegrateForward(); // -> 1
        else // not active
            rt_buffer[j].scale_t = rt_buffer[j].fade->IntegrateBackward(); // -> 0

    //3) Compute the force for each mechanism, remove the antagonist force components
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
                f_out -= rt_buffer[i].scale * rt_buffer[j].scale_t * rt_buffer[j].guide->getJacobianVersor() * f_vm_.dot(rt_buffer[j].guide->getJacobianVersor());
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

void MechanismManager::SetMode(const scale_mode_t mode)
{
    scale_mode_ = mode;
}

void MechanismManager::Stop()
{
    std::vector<GuideStruct>& rt_buffer = vm_buffers_[rt_idx_];
    for(int i=0;i<rt_buffer.size();i++)
        rt_buffer[i].guide->Stop();
}

} // namespace
