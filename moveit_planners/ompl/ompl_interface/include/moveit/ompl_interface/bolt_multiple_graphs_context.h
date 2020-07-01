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

//Moveit
#include <moveit/ompl_interface/model_based_planning_context.h>

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
    BoltMultipleGraphsMonitor(moveit::core::RobotModelConstPtr  robot_model)
    {
      robot_state_ = std::make_shared<robot_state::RobotState>(robot_model);
      parameters = false;
//      tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(5.0)));
//      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
//      ros::Duration(0.2).sleep();

//      robot_state_.reset(new robot_state::RobotState(robot_model));
    };

//    ~BoltMultipleGraphsMonitor()
//    {
//      stopThreadMonitor();
//    };
//    void operator()()
//    {
//      ROS_WARN("Void operator() thread");
//    }
  //  virtual bool startThreadMonitor() = 0;

    void monitor(std::future<void> futureObj, ompl::tools::bolt::BoltPtr bolt);
    BoltTaskGraphGeneratorPtr allocTaskGraphGenerator(ompl::tools::bolt::BoltPtr bolt);


    std::future<void> getExitSignalFuture()
    {
      std::future<void> future_obj_;
      future_obj_ = exit_signal_.get_future();
      return future_obj_;
    }

    void stopThreadMonitor()
    {
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
//    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    Eigen::Isometry3d pose_;
    bool parameters;

};

class BoltMultipleGraphsContext: public BoltMultipleGraphsMonitor
{
public:
       BoltMultipleGraphsContext(const ModelBasedStateSpacePtr state_space);

//      ~BoltMultipleGraphsContext()
//      {
//        stopThreadMonitor();
//      }

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
