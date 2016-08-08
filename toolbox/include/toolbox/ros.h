#ifndef ROS_H
#define ROS_H

////////// STD
#include <iterator>

////////// Eigen
#include <eigen3/Eigen/Core>

////////// ROS
#include <ros/ros.h>
#include <ros/package.h>
#ifdef USE_ROS_RT_PUBLISHER
  #include <std_msgs/Float64MultiArray.h>
  #include <geometry_msgs/PoseStamped.h>
  #include <geometry_msgs/WrenchStamped.h>
  #include <visualization_msgs/Marker.h>
  #include <nav_msgs/Path.h>
  #include <realtime_tools/realtime_publisher.h>
#endif

namespace tool_box
{

inline std::string GetYamlFilePath(std::string pkg_name)
{
    return ros::package::getPath(pkg_name) + "/config/cfg.yml";
}

class RosNode
{
    public:
        RosNode(std::string ros_node_name)
        {
          Init(ros_node_name);
        }

        RosNode()
        {
          init_ = false;
        }

        void Init(std::string ros_node_name)
        {
          int argc = 1;
          char* arg0 = strdup(ros_node_name.c_str());
          char* argv[] = {arg0, 0};
          ros::init(argc, argv, ros_node_name,ros::init_options::NoSigintHandler);
          free (arg0);
          if(ros::master::check()){
              ros_nh_ptr_ = new ros::NodeHandle(ros_node_name);
          }
          else
          {
              std::string err("roscore not found... Did you start the server?");
              throw std::runtime_error(err);
          }
          init_ = true;
        }

        ~RosNode(){if(ros_nh_ptr_!=NULL && init_ == true){ros_nh_ptr_->shutdown(); delete ros_nh_ptr_;}}

        const ros::NodeHandle& GetNode()
        {
          if(init_ == true)
            return *ros_nh_ptr_;
          else
          {
            std::string err("RosNode not initialized");
            throw std::runtime_error(err);
          }
        }

    protected:
        ros::NodeHandle* ros_nh_ptr_;
        bool init_;
};


#ifdef USE_ROS_RT_PUBLISHER

class RealTimePublisherVector
{
    public:

        /** Initialize the real time publisher. */
        RealTimePublisherVector(const ros::NodeHandle& ros_nh, const std::string topic_name)
        {
            // Checks
            assert(topic_name.size() > 0);
            topic_name_ = topic_name;
            pub_ptr_.reset(new rt_publisher_t(ros_nh,topic_name,10));

        }
        /** Publish the topic. */
        inline void Publish(const Eigen::Ref<const Eigen::VectorXd>& in)
        {
            if(pub_ptr_ && pub_ptr_->trylock())
            {
                //pub_ptr_->msg_.header.stamp = ros::Time::now();
                int data_size = pub_ptr_->msg_.data.size();
                //assert(data_size >= in.size());
                for(int i = 0; i < data_size; i++)
                {
                    pub_ptr_->msg_.data[i] = in(i);
                }
                pub_ptr_->unlockAndPublish();
            }
        }

        /** Remove an element in the vector. */
        inline void Remove(const int idx)
        {
            if(pub_ptr_)
            {
                pub_ptr_->lock();
                pub_ptr_->msg_.data.erase(pub_ptr_->msg_.data.begin()+idx);
                pub_ptr_->unlock();
            }
        }

        /** Resize the vector. */
        inline void Resize(const int dim)
        {
            if(pub_ptr_)
            {
                pub_ptr_->lock();
                pub_ptr_->msg_.data.resize(dim);
                pub_ptr_->unlock();
            }
        }

        /** Add a new element at the back. */
        inline void PushBackEmpty()
        {
            if(pub_ptr_)
            {
                pub_ptr_->lock();
                pub_ptr_->msg_.data.push_back(0.0);
                pub_ptr_->unlock();
            }
        }

        inline std::string getTopic(){return topic_name_;}

    private:
        typedef realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> rt_publisher_t;
        std::string topic_name_;
        int msg_size_;
        boost::shared_ptr<rt_publisher_t > pub_ptr_;
};

template <class RealTimePublisher_t>
class RealTimePublishers
{
    public:

        RealTimePublishers(){};

        // Add a RealTimePublisher already created
        void AddPublisher(boost::shared_ptr<RealTimePublisher_t> pub_ptr, Eigen::VectorXd* vector_ptr)
        {
            assert(pub_ptr!=false);
            // Put it into the map with his friend
            map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
        }

        // Add a new fresh RealTimePublisher
        void AddPublisher(const ros::NodeHandle& ros_nh, const std::string topic_name, Eigen::VectorXd* vector_ptr)
        {
            boost::shared_ptr<RealTimePublisher_t> pub_ptr = boost::make_shared<RealTimePublisher_t>(ros_nh,topic_name);
            assert(pub_ptr!=false);
            // Put it into the map with his friend
            map_[pub_ptr->getTopic()] = std::make_pair(vector_ptr,pub_ptr);
        }

        // Remove an element from all the publishers
        void RemoveAll(const int idx)
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->Remove(idx);
        }

        // Resize all the publishers
        void ResizeAll(const int dim)
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->Resize(dim);
        }

        // Push back an empty value in all the publishers
        void PushBackEmptyAll()
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->PushBackEmpty();
        }

        // Publish!
        void PublishAll()
        {
            for(pubs_map_it_t iterator = map_.begin(); iterator != map_.end(); iterator++)
                iterator->second.second->Publish(*iterator->second.first);
        }
    private:

        typedef std::map<std::string,std::pair<Eigen::VectorXd*,boost::shared_ptr<RealTimePublisher_t> > > pubs_map_t;
        typedef typename pubs_map_t::iterator pubs_map_it_t;

        pubs_map_t map_;
};

#endif // RT PUBLISHERS

} // namespace

#endif
