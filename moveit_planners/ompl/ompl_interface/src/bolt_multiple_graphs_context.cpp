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
: state_space_(state_space), BoltMultipleGraphsMonitor{state_space->getRobotModel()}
{
  bolt_ = std::make_shared<ompl::tools::bolt::Bolt>(state_space_);//>reset(new ompl::tools::bolt::Bolt(state_space_));

//  graphs_monitor_ = std::make_shared<ompl_interface::BoltMultipleGraphsMonitor>(state_space_->getRobotModel());
//  monitor_th_.reset(new std::thread(&static_cast<ompl_interface::BoltMultipleGraphsMonitor &>(*this)));
  loadParameters();
  bolt_->load(1, true);
  setMonitorBolt(bolt_);
  monitor_th_ = std::unique_ptr<std::thread>(new std::thread(&ompl_interface::BoltMultipleGraphsMonitor::monitor, this,
          std::move(getExitSignalFuture())));
}


bool ompl_interface::BoltMultipleGraphsContext::loadParameters()
{
  moveit_ompl::loadOMPLParameters(bolt_);
//  moveit_ompl::loadOMPLParameters(&static_cast<ompl::tools::bolt::Bolt &>(*this));
  if (bolt_->getSparseGraphsSize() == 0 )
    return false;

  return true;

}
ompl_interface::BoltMultipleGraphsMonitor::BoltMultipleGraphsMonitor(moveit::core::RobotModelConstPtr  robot_model)
{
//  robot_state_ = std::make_shared<robot_state::RobotState>(robot_model);

  visuals_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_tool/graph/task"));
//  visuals_->loadPlanningSceneMonitor();
  visuals_->loadMarkerPub(true);
//      tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(5.0)));
//      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
//      ros::Duration(0.2).sleep();

  robot_state_.reset(new robot_state::RobotState(robot_model));
};
void ompl_interface::BoltMultipleGraphsMonitor::monitor(std::future<void> future)
{
  ros::Duration(5.0).sleep();
  //std::vector<std::future<void>> futures;
  ompl::base::State *state = bolt_->getSpaceInformation()->allocState();
  geometry_msgs::TransformStamped transformStamped;
//  transformStamped = tfBuffer.lookupTransform("base_link_arm", "world",ros::Time(0));
// const Eigen::Isometry3d &global_pose = tf2::transformToEigen(tf_buffer_->lookupTransform("world", "tool0",ros::Time(0)));
  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup("arm_UR");
//  double *d = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  std::vector<double> joint_values;

  pose_ = Eigen::Isometry3d(Eigen::Translation3d(robot_state_->getGlobalLinkTransform("base_link_arm").translation())
                            * Eigen::Quaterniond(robot_state_->getGlobalLinkTransform("tool0").rotation()));
  pose_.translation().x() +=pose_.translation().x();
  pose_.translation().y() +=pose_.translation().y();
  pose_.translation().z() +=pose_.translation().z();
  Eigen::Quaterniond q(pose_.rotation());

 // futures.push_back(std::async(*BoltTaskGraphGeneratorPtr(new BoltTaskGraphGenerator(*robot_state_)),bolt,state));
 ROS_WARN_STREAM("bolt size" << bolt_->getSparseGraphsSize());
  ompl::tools::bolt::SparseGraphPtr sp = bolt_->getSparseGraph(0);
  bool ik = robot_state_->setFromIK(joint_model_group, pose_, 10, 0.1);
  ROS_WARN_STREAM("IK: "<< ik);

//  robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
  robot_state_->copyJointGroupPositions(joint_model_group, state->as<ModelBasedStateSpace::StateType>()->values);
  visuals_->publishSphere(stateToPoint(state),rviz_visual_tools::RED,rviz_visual_tools::XXXLARGE);
  visuals_->trigger();
  visuals_->enableBatchPublishing(true);
  visualizeGraph(0);

  ROS_WARN_STREAM("Rozmiar: ??");
  ROS_INFO_STREAM("Pose:  x:" << pose_.translation().x() <<"| y: " <<pose_.translation().y() << "| z: "<<pose_.translation().z());
//  std::vector<ompl::tools::bolt::SparseEdge> edg = bolt->getSparseGraph(0)->getNeighborsEdges(state, 8.2, 1);
  //ompl::tools::bolt::SparseAdjList graph = bolt_->getSparseGraph(0)->getNeighborGraph(state, 8.2, 1);

  //bool vis = visualizeGraph(graph);

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
bool ompl_interface::BoltMultipleGraphsMonitor::visualizeGraph(size_t index)
{
  ros::Time start_time = ros::Time::now();
  ROS_WARN_STREAM("visualizeGraph");
  if(bolt_->getSparseGraphsSize() < index)
  {
    ROS_WARN_STREAM("Can not visualize graph. Graph with index"<< index <<" does not exist. Max graph index is "<<
                                                               bolt_->getSparseGraphsSize());
    return false;
  }

  ompl::tools::bolt::SparseGraphPtr sp = bolt_->getSparseGraph(index);
  BOOST_FOREACH(const otb::SparseEdge edge, boost::edges(sp->getGraph()))
        {
          otb::SparseVertex v1 = boost::source(edge, sp->getGraph());
          otb::SparseVertex v2 = boost::target(edge, sp->getGraph());
          std::cout <<"( " <<v1<< " , " << v2 <<" )"<<std::endl;

          visuals_->publishLine(stateToPoint(sp->getGraph()[v1].state_),
                  stateToPoint(sp->getGraph()[v2].state_), rviz_visual_tools::RED);

        }
  ros::Time end = ros::Time::now() ;
  double time = (end - start_time).toSec();

  OMPL_WARN("CZas print %f", time);
  visuals_->trigger();
  visuals_->enableBatchPublishing(true);
  return true;

}
bool ompl_interface::BoltMultipleGraphsMonitor::visualizeGraph(ompl::tools::bolt::SparseAdjList graph)
{
  ros::Time start_time = ros::Time::now();
  ROS_WARN_STREAM("visualizeGraph");

  BOOST_FOREACH(const otb::SparseEdge edge, boost::edges(graph))
        {
          otb::SparseVertex v1 = boost::source(edge, graph);
          otb::SparseVertex v2 = boost::target(edge, graph);
          std::cout <<"( " <<v1<< " , " << v2 <<" )"<<std::endl;

          visuals_->publishLine(stateToPoint(graph[v1].state_),
                                stateToPoint(graph[v2].state_));
        }
  ros::Time end = ros::Time::now() ;
  double time = (end - start_time).toSec();

  OMPL_WARN("CZas print %f", time);
  visuals_->trigger();
  visuals_->enableBatchPublishing(true);
  return true;

}
geometry_msgs::Point ompl_interface::BoltMultipleGraphsMonitor::stateToPoint(const ompl::base::State* state)
{
  robot_state_->setJointGroupPositions(robot_state_->getJointModelGroup("arm_UR"),
          state->as<ModelBasedStateSpace::StateType>()->values);
//  robot_state_->update();
  return visuals_->convertPoint(robot_state_->getGlobalLinkTransform("tool0").translation());;
}

//ompl_interface::BoltTaskGraphGeneratorPtr ompl_interface::BoltMultipleGraphsMonitor::allocTaskGraphGenerator(ompl::tools::bolt::BoltPtr bolt)
//{
//  return  BoltTaskGraphGeneratorPtr(new BoltTaskGraphGenerator(bolt));
//}