//
// Created by db on 23.06.2020.
//

#ifndef SRC_BOLT_MULTIPLE_GRAPHS_CONTEXT_H
#define SRC_BOLT_MULTIPLE_GRAPHS_CONTEXT_H

// Bolt
#include <bolt_core/Bolt.h>
#include <bolt_core/SparseGenerator.h>
#include <bolt_core/SparseCriteria.h>
#include <bolt_core/SparseMirror.h>

//Moveit/ROS
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_ros/transform_listener.h>

// C++
#include <thread>
#include <future>
#include <tf2_ros/transform_listener.h>


namespace ompl_interface
{


OMPL_CLASS_FORWARD(BoltMultipleGraphsContext);
OMPL_CLASS_FORWARD(BoltMultipleGraphsMonitor);
OMPL_CLASS_FORWARD(BoltTaskGraphGenerator);

class  BoltTaskGraphGenerator //: public ompl::tools::bolt::Bolt
{
public:
//    BoltTaskGraphGenerator(const moveit::core::RobotModelConstPtr robot_model)
    BoltTaskGraphGenerator(moveit::core::RobotState robot_state)
//    : bolt_(bolt), robot_state_(robot_state)
    {
     robot_state_.reset(new moveit::core::RobotState(robot_state));
    }

    void operator()(ompl::tools::bolt::BoltPtr bolt, ompl::base::State *state)
    {
      ROS_WARN("Void operator() thread");
    }

protected:

private:
    moveit::core::RobotStatePtr robot_state_;

};

class BoltMultipleGraphsMonitor //: public ompl::tools::bolt::Bolt
{
public:
    BoltMultipleGraphsMonitor(moveit::core::RobotModelConstPtr  robot_model);

    bool visualizeGraph(size_t index);
    bool visualizeGraph(ompl::tools::bolt::SparseAdjList graph);
    geometry_msgs::Point stateToPoint(const ompl::base::State* state);


    void monitor(std::future<void> futureObj);
    BoltTaskGraphGeneratorPtr allocTaskGraphGenerator(ompl::tools::bolt::BoltPtr bolt);
    bool setMonitorBolt(ompl::tools::bolt::BoltPtr bolt)
    {
      bolt_ = bolt;
      return true;
    }

    std::future<void> getExitSignalFuture()
    {
      std::future<void> future_obj_;
      future_obj_ = exit_signal_.get_future();
      return future_obj_;
    }

    void stopThreadMonitor()
    {
      ROS_ERROR_STREAM("stopThreadMonitor");
      if(!monitor_th_->joinable())
      {
        exit_signal_.set_value();
        monitor_th_->join();
      }

    }

protected:
    std::promise<void> exit_signal_;
    std::unique_ptr<std::thread> monitor_th_;

private:

    robot_state::RobotStatePtr robot_state_;
    ompl::tools::bolt::BoltPtr bolt_;
//    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::Isometry3d pose_;
    bool parameters;
    moveit_visual_tools::MoveItVisualToolsPtr visuals_;


    };

class BoltMultipleGraphsContext: public BoltMultipleGraphsMonitor
{
public:
       BoltMultipleGraphsContext(const ModelBasedStateSpacePtr state_space);

      ~BoltMultipleGraphsContext()
      {
        stopThreadMonitor();
      }

//    virtual ~BoltMultipleGraphsContext() = default;

      bool loadParameters();

      bool initializeGraphInfo();

      bool startThreadMonitor();



        //    void monitor(std::future<void> futureObj);

      ompl::tools::bolt::BoltPtr getBolt()
      {
        return bolt_;
      }

protected:

private:
    ModelBasedStateSpacePtr state_space_;

    ompl::tools::bolt::BoltPtr bolt_;



};

}



#endif //SRC_BOLT_MULTIPLE_GRAPHS_CONTEXT_H
