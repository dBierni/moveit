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
: state_space_(state_space), BoltMultipleGraphsMonitor{state_space->getRobotModel(),state_space}
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
ompl_interface::BoltMultipleGraphsMonitor::BoltMultipleGraphsMonitor(moveit::core::RobotModelConstPtr  robot_model,
                                                                     const ModelBasedStateSpacePtr state_space)
                                                                     : state_space_(state_space)
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
  std::vector<std::future<void>> futures;
  pose_ = Eigen::Isometry3d(Eigen::Translation3d(robot_state_->getGlobalLinkTransform("base_link").translation())
                            * Eigen::Quaterniond(robot_state_->getGlobalLinkTransform("H1_base_link").rotation()));
  ROS_INFO_STREAM("Pose robot:  x:" << pose_.translation().x() <<"| y: " <<pose_.translation().y() << "| z: "<<pose_.translation().z());

  ompl::base::State *state = bolt_->getSpaceInformation()->allocState();
  futures.push_back(std::async(BoltTaskGraphGenerator(*robot_state_, &bolt_->getSparseGraphFull(0),
          joint_model_group_name_, 8.2, 1),pose_, state_space_));

  while (future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
    // futures.push_back(std::async(*BoltTaskGraphGeneratorPtr(new BoltTaskGraphGenerator(*robot_state_)),bolt,state));

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


//bool ompl_interface::BoltMultipleGraphsMonitor::visualizeGraph(size_t index)
//{
//  ros::Time start_time = ros::Time::now();
//  ROS_WARN_STREAM("visualizeGraph");
//  if(bolt_->getSparseGraphsSize() < index)
//  {
//    ROS_WARN_STREAM("Can not visualize graph. Graph with index"<< index <<" does not exist. Max graph index is "<<
//                                                               bolt_->getSparseGraphsSize());
//    return false;
//  }
//
//  ompl::tools::bolt::SparseGraphPtr sp = bolt_->getSparseGraph(index);
//  BOOST_FOREACH(const otb::SparseEdge edge, boost::edges(sp->getGraph()))
//        {
//          otb::SparseVertex v1 = boost::source(edge, sp->getGraph());
//          otb::SparseVertex v2 = boost::target(edge, sp->getGraph());
//          std::cout <<"( " <<v1<< " , " << v2 <<" )"<<std::endl;
//
//          visuals_->publishLine(stateToPoint(sp->getGraph()[v1].state_),
//                  stateToPoint(sp->getGraph()[v2].state_), rviz_visual_tools::RED);
//
//        }
//  ros::Time end = ros::Time::now() ;
//  double time = (end - start_time).toSec();
//
//  OMPL_WARN("CZas print %f", time);
//  visuals_->trigger();
//  visuals_->enableBatchPublishing(true);
//  return true;
//
//}
//
//geometry_msgs::Point ompl_interface::BoltMultipleGraphsMonitor::stateToPoint(const ompl::base::State* state)
//{
//  robot_state_->setJointGroupPositions(robot_state_->getJointModelGroup("arm_UR"),
//          state->as<ModelBasedStateSpace::StateType>()->values);
////  robot_state_->update();
//  return visuals_->convertPoint(robot_state_->getGlobalLinkTransform("tool0").translation());;
//}

//ompl_interface::BoltTaskGraphGeneratorPtr ompl_interface::BoltMultipleGraphsMonitor::allocTaskGraphGenerator(ompl::tools::bolt::BoltPtr bolt)
//{
//  return  BoltTaskGraphGeneratorPtr(new BoltTaskGraphGenerator(bolt));
//}


ompl_interface::BoltTaskGraphGenerator::BoltTaskGraphGenerator(moveit::core::RobotState robot_state ,std::pair<Eigen::Isometry3d,
        ompl::tools::bolt::SparseGraphPtr> *graph_full, const std::string &group_name, const double nn_radius, size_t index)
{
  robot_state_.reset(new moveit::core::RobotState(robot_state));
  joint_model_group_name_ = group_name;
  nn_radius_ = nn_radius;
  graph_full_.reset(graph_full);
  visuals_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_tool/graph/"+ std::to_string(index)));
  visuals_->loadMarkerPub(true);
}

void ompl_interface::BoltTaskGraphGenerator::operator()(const Eigen::Isometry3d &current_pose, const ModelBasedStateSpacePtr state_space)
{
  ROS_WARN("Void operator() thread");
  ompl::base::State *state = state_space->allocState();
  std::vector<double> offset_v1, offset_v2, v1_vec, v2_vec;
  ompl::base::State *state_v1{nullptr}, *state_v2{nullptr};
  ompl::tools::bolt::SparseAdjList graph;
  graph_full_->first.translation().z() = 0.1; // TODO  ompl-rosparam not loaded ?
  ROS_INFO_STREAM("Pose graph:  x:" << graph_full_->first.translation().x() <<"| y: " <<graph_full_->first.translation().y() << "| z: "<<graph_full_->first.translation().z());

  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(joint_model_group_name_);
  const Eigen::Isometry3d  & state_pose = getSearchPose(current_pose);
  bool ik = poseToModelSpaceState(state_pose, joint_model_group, state);
  bool diff = false;
  if (ik)
  {
    robot_state_->copyJointGroupPositions(joint_model_group, state->as<ModelBasedStateSpace::StateType>()->values);
    bool nn = (graph_full_->second->getNeighborGraph(state,nn_radius_, graph));
int test = 0;
//    bool vis = visualizeGraph(0);
BOOST_FOREACH(const ompl::tools::bolt::SparseEdge & edge, boost::edges(graph))
      {
            otb::SparseVertex v1 = boost::source(edge, graph);
            otb::SparseVertex v2 = boost::target(edge,graph);


            //  ROS_WARN_STREAM("----------------------------------------");
//            if(state_v1 != graph[v1].state_)
//            {
//             // ROS_INFO_STREAM("Diff state v1" << graph[v1].state_ <<"  /" << state_v1 );
//
//              offset_v1 = getOffsetForState(graph[v1].state_, joint_model_group, state_space, current_pose);
//              state_v1 = graph[v1].state_;
//            }
//            ROS_ERROR_STREAM("------------" );
//            if(state_v2 != graph[v2].state_)
//            {
//         //     ROS_INFO_STREAM("Diff state v2" << graph[v2].state_ <<"  /" << state_v2 );
//              offset_v2 = getOffsetForState(graph[v2].state_, joint_model_group, state_space, current_pose);
//              state_v2 = graph[v2].state_;
//            }
//
//              if(offset_v1.size() > 0 && offset_v2.size() > 0)
//              {
//                v1_vec.resize(offset_v1.size());
//                v2_vec.resize(offset_v2.size());
//
//                state_space->copyToReals(v1_vec,graph[v1].state_);
//                state_space->copyToReals(v2_vec,graph[v2].state_);
////              visuals_->enableBatchPublishing(true);
////
////              visuals_->publishLine(stateToPoint(graph[v1].state_),stateToPoint(graph[v2].state_),
////                                    visuals_->getColorWithID(0), rviz_visual_tools::XXLARGE);
////              visuals_->enableBatchPublishing(true);
//
////              for(std::size_t i=0; i < v1_vec.size();i++)
////              {
////                ROS_INFO_STREAM("v1: " << v1_vec[i] );
////              }
//                std::transform (v1_vec.begin(), v1_vec.end(), offset_v1.begin(), v1_vec.begin(), std::minus<double>());
//                std::transform (v2_vec.begin(), v2_vec.end(), offset_v2.begin(), v2_vec.begin(), std::minus<double>());
//
////              for(std::size_t i=0; i < v1_vec.size();i++)
////              {
////                ROS_INFO_STREAM("new: " << v1_vec[i] );
////              }
//                state_space->copyFromReals(graph[v1].state_,v1_vec);
//                state_space->copyFromReals(graph[v2].state_,v2_vec);
//
//                Eigen::Isometry3d pose2 = stateToPoint(graph[v1].state_);
//                ROS_INFO_STREAM("new pose 1:  x:" << pose2.translation().x() <<"| y: " <<pose2.translation().y() << "| z: "<<pose2.translation().z());
//
//                Eigen::Isometry3d pose3 = stateToPoint(graph[v2].state_);
//                ROS_INFO_STREAM("new pose 2:  x:" << pose3.translation().x() <<"| y: " <<pose3.translation().y() << "| z: "<<pose3.translation().z());
////
////              visuals_->publishLine(stateToPoint(graph[v1].state_),stateToPoint(graph[v2].state_),
////                                    visuals_->getColorWithID(2), rviz_visual_tools::XXLARGE);
////              visuals_->enableBatchPublishing(true);
////
////              visuals_->trigger();
//                ROS_WARN_STREAM("----------------------------------------");


//              }

          }
  }
  ROS_WARN("Void operator() end");

}

Eigen::Isometry3d ompl_interface::BoltTaskGraphGenerator::getSearchPose(const Eigen::Isometry3d &pose)
{
  const Eigen::Isometry3d &graph_root = graph_full_->first;
  Eigen::Isometry3d result = Eigen::Isometry3d( Eigen::Translation3d(
          (pose.translation().x()-graph_root.translation().x())+pose.translation().x(),
          (pose.translation().y()-graph_root.translation().y())+pose.translation().y(),
          (pose.translation().z()-graph_root.translation().z())+pose.translation().z())
                  * Eigen::Quaterniond(pose.rotation()));
  //ROS_INFO_STREAM("result:  x:" << result.translation().x() <<"| y: " <<result.translation().y() << "| z: "<<result.translation().z());

  return result;
}

bool ompl_interface::BoltTaskGraphGenerator::poseToModelSpaceState(const Eigen::Isometry3d &pose,
                                                                                 const robot_state::JointModelGroup* joint_model_group,
                                                                                 ompl::base::State *state)
{
  bool ik = robot_state_->setFromIK(joint_model_group, pose, 10,0.1);
  if(ik)
    robot_state_->copyJointGroupPositions(joint_model_group, state->as<ModelBasedStateSpace::StateType>()->values);

 // ROS_INFO_STREAM("IK: " << ik);
  return ik;
}

bool ompl_interface::BoltTaskGraphGenerator::visualizeGraph(std::size_t color_id)
{
  ros::Time start_time = ros::Time::now();
  ompl::tools::bolt::SparseAdjList graph = graph_full_->second->getGraph();

  BOOST_FOREACH(const otb::SparseEdge edge, boost::edges(graph))
        {
          otb::SparseVertex v1 = boost::source(edge, graph);
          otb::SparseVertex v2 = boost::target(edge, graph);
          visuals_->publishLine(stateToPoint(graph[v1].state_),stateToPoint(graph[v2].state_),
                 visuals_->getColorWithID(color_id));
        }
  ros::Time end = ros::Time::now() ;
  double time = (end - start_time).toSec();

  OMPL_WARN("Visualization time %f", time);
  visuals_->trigger();
  visuals_->enableBatchPublishing(true);
  return true;
}

Eigen::Isometry3d ompl_interface::BoltTaskGraphGenerator::stateToPoint(const ompl::base::State* state)
{
  robot_state_->setJointGroupPositions(robot_state_->getJointModelGroup(joint_model_group_name_),
                                       state->as<ModelBasedStateSpace::StateType>()->values);
  robot_state_->update();

  return robot_state_->getGlobalLinkTransform("H1_base_link");
}
std::vector<double> ompl_interface::BoltTaskGraphGenerator::getOffsetForState(ompl::base::State *from,
                                                                      const robot_state::JointModelGroup* joint_model_group,
                                                                      const ModelBasedStateSpacePtr  state_space,
                                                                      const Eigen::Isometry3d & current_pose)
{
  std::vector<double> current,base, offset;
  Eigen::Isometry3d pose = stateToPoint(from);
  ROS_INFO_STREAM("Pose z grafu current:  x:" << pose.translation().x() <<"| y: " <<pose.translation().y() << "| z: "<<pose.translation().z());

  pose.translation().x() -=  (current_pose.translation().x()- graph_full_->first.translation().x()) ;
  pose.translation().y() -=  (current_pose.translation().y()- graph_full_->first.translation().y()) ;
  pose.translation().z() -=  (current_pose.translation().z()- graph_full_->first.translation().z());

  ROS_INFO_STREAM("Pose przesuniete base:  x:" << pose.translation().x() <<"| y: " <<pose.translation().y() << "| z: "<<pose.translation().z());
  ompl::base::State *to = state_space->allocState();
  bool ik = poseToModelSpaceState(pose, joint_model_group, to);
  if(ik)
  {
    state_space->copyToReals(current, from);

    state_space->copyToReals(base, to);
    state_space->freeState(to);

    assert(current.size() == base.size());

    offset.resize(current.size());
    for(std::size_t i; i < current.size();i++)
    {
      ROS_INFO_STREAM("current: " << current[i] );
      ROS_ERROR_STREAM("base: " << base[i] );
      offset[i] = (current[i] - base[i]);
      ROS_ERROR_STREAM("offset: " << offset[i] );
    }
  }
  return offset;
}