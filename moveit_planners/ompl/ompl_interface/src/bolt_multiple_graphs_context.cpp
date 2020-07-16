//
// Created by db on 23.06.2020.
//

#include <moveit/ompl_interface/bolt_multiple_graphs_context.h>
#include <moveit_ompl/ompl_rosparam.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include<math.h>

ompl_interface::BoltMultipleGraphsContext::BoltMultipleGraphsContext(ModelBasedStateSpacePtr state_space)
: state_space_(state_space), BoltMultipleGraphsMonitor{state_space->getRobotModel(),state_space}
{
  bolt_ = std::make_shared<ompl::tools::bolt::Bolt>(state_space_);//>reset(new ompl::tools::bolt::Bolt(state_space_));

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
  bool diff  = true;
  while (future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {

    pose_ = Eigen::Isometry3d(Eigen::Translation3d(robot_state_->getGlobalLinkTransform("base_link_arm").translation())
                              * Eigen::Quaterniond(robot_state_->getGlobalLinkTransform("H1_base_link").rotation()));
    ROS_INFO_STREAM("Pose robot:  x:" << pose_.translation().x() <<"| y: " <<pose_.translation().y() << "| z: "<<pose_.translation().z());
    if (diff)
    {
      const double &nn_radius = (bolt_->getSpaceInformation()->getMaximumExtent());
      for(size_t i =0; i < bolt_->getSparseGraphsSize(); i++)
      {
        ompl::base::State *state = state_space_->allocState();
        ROS_WARN_STREAM("THREAD: " << i << " SPARSE GRAPHS: " <<  bolt_->getSparseGraphsSize() << " addr: " << state);
        futures.push_back(std::async(BoltTaskGraphGenerator(*robot_state_, &bolt_->getSparseGraphFull(i),
                                                            joint_model_group_name_, pose_, i),nn_radius, state_space_,
                                                                    bolt_->getTaskGraph(), std::move(state)));
      }
      diff = false;
    }


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
        ompl::tools::bolt::SparseGraphPtr> *graph_full, const std::string &group_name, const Eigen::Isometry3d &current_pose, size_t index)
        : current_pose_(current_pose),
          joint_model_group_name_(group_name),
          index_(index)

{
  robot_state_.reset(new moveit::core::RobotState(robot_state));
  graph_full_.reset(graph_full);
  visuals_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/moveit_visual_tool/graph/"+ std::to_string(index)));
  visuals_->loadMarkerPub(true);

}

void ompl_interface::BoltTaskGraphGenerator::operator()(const double nn_radius, const ModelBasedStateSpacePtr state_space,
        const ompl::tools::bolt::TaskGraphPtr task_graph, ompl::base::State *state)
{
  ROS_WARN("Void operator() thread");

 // auto *diff_state = state_space->allocState();
  ompl::base::State *state_v1{nullptr}, *state_v2{nullptr};
  bool v1_new{false}, v2_new{false};
  ompl::tools::bolt::SparseAdjList graph;
  std::vector<ompl::tools::bolt::SparseVertex> vertex_to_delete;
  ROS_INFO_STREAM("Pose graph:  x:" << graph_full_->first.translation().x() <<"| y: " <<graph_full_->first.translation().y() << "| z: "<<graph_full_->first.translation().z());

  const robot_state::JointModelGroup* joint_model_group = robot_state_->getJointModelGroup(joint_model_group_name_);
  const Eigen::Isometry3d  & state_pose = getSearchPose(current_pose_);
  ROS_INFO_STREAM("Search pose:  x:" << state_pose.translation().x() <<"| y: " << state_pose.translation().y() << "| z: "<<state_pose.translation().z());

  bool diff_ik =true; // poseToModelSpaceState(current_pose_, joint_model_group, diff_state, 0.1);
  bool ik = poseToModelSpaceState(state_pose, joint_model_group, state, 0.1);

  if (!ik || !diff_ik)
    ROS_WARN_STREAM("Can not obtain pose. Inverse kinematic for nearest nighbors  failed");

  if (ik && diff_ik)
  {
    bool vis = visualizeGraph(0, graph_full_->second->getGraph());
    nn_radius_ = (nn_radius / 2.0) ;//- std::min(state_space->distance(state, diff_state), (nn_radius / 4));
  //  ROS_INFO_STREAM("Distance:" << state_space->distance(diff_state, state) << " Radius: " << nn_radius_ );

    robot_state_->copyJointGroupPositions(joint_model_group, state->as<ModelBasedStateSpace::StateType>()->values);
    bool nn = (graph_full_->second->getNeighborGraph(state,nn_radius_ , graph));
    ros::Time start_time = ros::Time::now();

    std::vector<double> test_v1= {0,0,0,0,0,0};
//    std::vector<double> test_v2= {0.743084,-1.00789,-1.76727,-0.393337,-2.33229,0.761168};
    std::vector<double> test_v2= {3.14,0,0,0,0,0};

    BOOST_FOREACH(const ompl::tools::bolt::SparseEdge & edge, boost::edges(graph))
      {
          otb::SparseVertex v1 = boost::source(edge, graph);
          otb::SparseVertex v2 = boost::target(edge,graph);

        if(state_v1 != graph[v1].state_)
          {
            v1_new = OffsetState(graph[v1].state_, joint_model_group);
            state_v1 = graph[v1].state_;}
          if(state_v2 != graph[v2].state_ && v1_new)
          {
            v2_new = OffsetState(graph[v2].state_, joint_model_group);
            state_v2 = graph[v2].state_;
          }

          if(!v2_new || !v1_new)
            continue;

        // If program reach here, it means, that robot can reach both v1 and v2.

            //  ROS_ERROR_STREAM("DIST: " << state_space->distance(state,state_v1));
//              ROS_ERROR_STREAM("DIST2: " << state_space->distance(state,state_v2));

              visuals_->enableBatchPublishing(true);

              visuals_->publishLine(stateToPoint(state_v1), stateToPoint(state_v2),
                                    visuals_->getColorWithID(2), rviz_visual_tools::LARGE);

              visuals_->enableBatchPublishing(true);
              visuals_->trigger();
              task_graph->addEdge(edge, graph, 1);


      }
    ros::Time end = ros::Time::now() ;
    double time = (end - start_time).toSec();
    ROS_WARN_STREAM("Graph " << index_ <<" time: " << time <<"s." <<" Could not obtain " << vertex_to_delete.size()
    <<" vertices " );
  //  state_space->freeState(state);

//    for(auto it = vertex_to_delete.begin(); it != vertex_to_delete.end(); it++)
//    {
//  //    state_space->freeState(graph[*it].state_);
//      boost::remove_vertex(*it, graph);
//    }
    //visualizeGraph(2, task_graph->getGraph());
  }
}

Eigen::Isometry3d ompl_interface::BoltTaskGraphGenerator::getSearchPose(const Eigen::Isometry3d &pose)
{
  const Eigen::Isometry3d &graph_root = graph_full_->first;
  Eigen::Isometry3d result;
  if(std::abs(pose.translation().x() - graph_root.translation().x()) < 1.2 )
  {
    result = Eigen::Isometry3d(Eigen::Translation3d(
            (pose.translation().x() - graph_root.translation().x()) + pose.translation().x(),
            (pose.translation().y() - graph_root.translation().y()) + pose.translation().y(),
            (pose.translation().z() - graph_root.translation().z()) + pose.translation().z())
                                                 * Eigen::Quaterniond(pose.rotation()));
  }
  else
  {
    result = Eigen::Isometry3d(Eigen::Translation3d((((pose.translation().x() + graph_root.translation().x()) / 2.0)
                                                    + pose.translation().x()),
                                                    (((pose.translation().y() + graph_root.translation().y()) / 2.0)
                                                    + pose.translation().y()),
                                                    (((pose.translation().z() + graph_root.translation().z()) / 2.0)
                                                    + pose.translation().z()))
                                                            * Eigen::Quaterniond(pose.rotation()));
  }

  return result;
}

bool ompl_interface::BoltTaskGraphGenerator::poseToModelSpaceState(const Eigen::Isometry3d &pose,
                                                                                 const robot_state::JointModelGroup* joint_model_group,
                                                                                 ompl::base::State *state , double timeout)
{
//  bool ik = robot_state_->setFromIK(joint_model_group, pose , 0.0001);
  bool ik = robot_state_->setFromIK(joint_model_group, pose , timeout);
  if(ik)
  {
    robot_state_->copyJointGroupPositions(joint_model_group, state->as<ModelBasedStateSpace::StateType>()->values);
    robot_state_->update();
  }

//  if(ik ==0)
//    ROS_WARN_STREAM("Can not obtain pose. Inverse kinematic failed");
  return ik;
}

bool ompl_interface::BoltTaskGraphGenerator::visualizeGraph(std::size_t color_id, const ompl::tools::bolt::SparseAdjList &graph,
        bool base)
{
  ros::Time start_time = ros::Time::now();

  BOOST_FOREACH(auto edge, boost::edges(graph))
        {
          auto v1 = boost::source(edge, graph);
          auto v2 = boost::target(edge, graph);
          if(base)
            visuals_->publishLine(pointWithDiff(stateToPoint(graph[v1].state_)),pointWithDiff(stateToPoint(graph[v2].state_)),
                   visuals_->getColorWithID(color_id));
          else
            visuals_->publishLine(stateToPoint(graph[v1].state_),stateToPoint(graph[v2].state_),
                                  visuals_->getColorWithID(color_id),rviz_visual_tools::MEDIUM);
        }
  ros::Time end = ros::Time::now() ;
  double time = (end - start_time).toSec();

  OMPL_WARN("Visualization time %f", time);
  visuals_->trigger();
  visuals_->enableBatchPublishing(true);
  return true;
}

bool ompl_interface::BoltTaskGraphGenerator::visualizeGraph(std::size_t color_id, const ompl::tools::bolt::TaskAdjList &graph)
{
  ros::Time start_time = ros::Time::now();

  BOOST_FOREACH(auto edge, boost::edges(graph))
        {
          auto v1 = boost::source(edge, graph);
          auto v2 = boost::target(edge, graph);
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
bool ompl_interface::BoltTaskGraphGenerator::OffsetState(ompl::base::State *from,
                                                                      const robot_state::JointModelGroup* joint_model_group)
{
  Eigen::Isometry3d pose = stateToPoint(from);
  return (poseToModelSpaceState(pointWithDiff(pose), joint_model_group, from));
}

Eigen::Isometry3d ompl_interface::BoltTaskGraphGenerator::pointWithDiff(Eigen::Isometry3d pose)
{
  pose.translation().x() -=  (current_pose_.translation().x() - graph_full_->first.translation().x()) ;
  pose.translation().y() -=  (current_pose_.translation().y() - graph_full_->first.translation().y()) ;
  pose.translation().z() -=  (current_pose_.translation().z() - graph_full_->first.translation().z());

  return pose;
}