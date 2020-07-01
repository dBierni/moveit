//
// Created by db on 23.06.2020.
//

#include <moveit/ompl_interface/bolt_multiple_graphs_context.h>
#include <moveit_ompl/ompl_rosparam.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>

ompl_interface::BoltMultipleGraphsContext::BoltMultipleGraphsContext(ModelBasedStateSpacePtr state_space)
: state_space_(state_space), BoltMultipleGraphsMonitor{state_space->getRobotModel()} //, ompl::tools::bolt::Bolt{state_space}
{
  bolt_.reset(new ompl::tools::bolt::Bolt(state_space_));
//  graphs_monitor_ = std::make_shared<ompl_interface::BoltMultipleGraphsMonitor>(state_space_->getRobotModel());

//  getMonitorThread()  = std::make_shared<std::thread>(&ompl_interface::BoltMultipleGraphsMonitor::monitor, this,
//          std::move(getExitSignalFuture()), bolt_);
//
//   monitor_th_.reset(new std::thread(&static_cast<ompl_interface::BoltMultipleGraphsMonitor &>(*this)));
  loadParameters();
  bolt_->load(1, true);
  monitor_th_ = std::unique_ptr<std::thread>(new std::thread(&ompl_interface::BoltMultipleGraphsMonitor::monitor, this,
          std::move(getExitSignalFuture()), bolt_));
}


bool ompl_interface::BoltMultipleGraphsContext::loadParameters()
{
  moveit_ompl::loadOMPLParameters(bolt_);
//  moveit_ompl::loadOMPLParameters(&static_cast<ompl::tools::bolt::Bolt &>(*this));
  if (bolt_->getSparseGraphsSize() == 0 )
    return false;

  return true;

}

void ompl_interface::BoltMultipleGraphsMonitor::monitor(std::future<void> future, ompl::tools::bolt::BoltPtr bolt)
{
  ros::Duration(5.0).sleep();
  std::vector<std::future<void>> futures;
  ompl::base::State *state = bolt->getSpaceInformation()->allocState();
  geometry_msgs::TransformStamped transformStamped;
//  transformStamped = tfBuffer.lookupTransform("base_link_arm", "world",ros::Time(0));
// const Eigen::Isometry3d &global_pose = tf2::transformToEigen(tf_buffer_->lookupTransform("world", "tool0",ros::Time(0)));
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup("arm_UR");
//  double *d = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;



  std::vector<double> joint_values;

  pose_ = Eigen::Isometry3d(Eigen::Translation3d(robot_state_->getGlobalLinkTransform("base_link_arm").translation())
                            * Eigen::Quaterniond(robot_state_->getGlobalLinkTransform("tool0").rotation()));
  Eigen::Quaterniond q(pose_.rotation());
  futures.push_back(std::async(*BoltTaskGraphGeneratorPtr(new BoltTaskGraphGenerator(*robot_state_)),bolt,state));

  bool ik = robot_state_->setFromIK(joint_model_group, pose_, 10, 0.1);
  ROS_WARN_STREAM("IK: "<< ik);


//  robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
  robot_state_->copyJointGroupPositions(joint_model_group, state->as<ModelBasedStateSpace::StateType>()->values);


  ROS_WARN_STREAM("Rozmiar: ??");
  ROS_INFO_STREAM("Pose:  x:" << pose_.translation().x() <<"| y: " <<pose_.translation().y() << "| z: "<<pose_.translation().z());

  std::vector<ompl::tools::bolt::SparseEdge> edg = bolt->getSparseGraph(0)->getNeighborsEdges(state, 8.2, 1);

  ROS_WARN_STREAM("Rozmiar: " << edg.size());

  while (future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
//    bool ik = robot_state_->setFromIK(joint_model_group, pose_, 10, 0.1);
//    ROS_WARN_STREAM("IK: "<< ik);
//    ROS_INFO_STREAM("Pose:  x:" << pose_.translation().x() <<"| y: " <<pose_.translation().y() << "| z: "<<pose_.translation().z());
//    ROS_INFO_STREAM("Pose:  qx:" <<q.x() <<"| qy: " <<q.y() << "| qz: "<<q.z()
//                            << "| qw: "<<q.w());
//    robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
//    ROS_ERROR("------------------------");
//    for (std::size_t i = 0; i < joint_values.size(); ++i)
//    {
//
//      ROS_INFO("Joint  %f ", joint_values[i]);
//    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

//ompl_interface::BoltTaskGraphGeneratorPtr ompl_interface::BoltMultipleGraphsMonitor::allocTaskGraphGenerator(ompl::tools::bolt::BoltPtr bolt)
//{
//  return  BoltTaskGraphGeneratorPtr(new BoltTaskGraphGenerator(bolt));
//}