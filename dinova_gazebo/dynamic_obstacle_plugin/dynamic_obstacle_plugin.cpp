#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <thread>
// #include <ignition/math4/ignition/math/Vector3.hh>
// #include <ignition/math4/ignition/math/Pose3.hh>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
  class ObstaclePlugin : public ModelPlugin {
    public:
        ObstaclePlugin() {
         
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
            nh = new ros::NodeHandle;
            object_pose_sub = nh->subscribe("/dynamic_object/desired_pose", 1, &ObstaclePlugin::ObjectCallback, this);
            // tf_broadcaster.reset(new tf::TransformBroadcaster());


            this->model = _parent;
            world = this->model->GetWorld();
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ObstaclePlugin::OnUpdate, this));

                // Print a message indicating that the plugin is loaded
            std::cout << "DynamicObstaclePLugin is loaded!" << std::endl;


        }

        void OnUpdate()
        {
            object_pose = this->model->WorldPose();

            count++;
            if (count >= 10)
            {
                // // publish tf coordinates for RViz
                static tf2_ros::TransformBroadcaster br;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.frame_id = "world";
                transformStamped.child_frame_id = "object";

                transformStamped.header.stamp = ros::Time::now();
                transformStamped.transform.translation.x = object_pose.Pos().X();
                transformStamped.transform.translation.y = object_pose.Pos().Y();
                transformStamped.transform.translation.z = object_pose.Pos().Z();

                transformStamped.transform.rotation.x = object_pose.Rot().X();
                transformStamped.transform.rotation.y = object_pose.Rot().Y();
                transformStamped.transform.rotation.z = object_pose.Rot().Z();
                transformStamped.transform.rotation.w = object_pose.Rot().W();

                br.sendTransform(transformStamped);

                count = 0;
            }
            
        }

        void ObjectCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
        {
            new_pose.Pos().X() = poseMsg->position.x;
            new_pose.Pos().Y() = poseMsg->position.y;
            new_pose.Pos().Z() = poseMsg->position.z;
            // new_pose.Rot().X() = poseMsg->orientation.x;
            // new_pose.Rot().Y() = poseMsg->orientation.y;
            // new_pose.Rot().Z() = poseMsg->orientation.z;
            // new_pose.Rot().W() = poseMsg->orientation.w;

            this->model->SetWorldPose(new_pose);
        }     

    private:
        //Gazebo objects
        physics::ModelPtr model;
        gazebo::physics::WorldPtr world;
        event::ConnectionPtr updateConnection;

        //Ros objects
        ros::NodeHandle *nh;
        ros::Subscriber object_pose_sub;
        
        ignition::math::Pose3d new_pose;

        ignition::math::Pose3d object_pose;

        uint count{0};
  
  };
  GZ_REGISTER_MODEL_PLUGIN(ObstaclePlugin)
}
