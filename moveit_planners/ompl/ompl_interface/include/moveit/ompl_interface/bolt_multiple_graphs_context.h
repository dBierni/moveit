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

namespace ompl_interface
{


OMPL_CLASS_FORWARD(BoltMultipleGraphsContext);
OMPL_CLASS_FORWARD(BoltMultipleGraphsMonitor);

class BoltMultipleGraphsMonitor //: public ompl::tools::bolt::Bolt
{
public:
    BoltMultipleGraphsMonitor(moveit::core::RobotModelConstPtr  robot_model)
    {
      robot_state_ = std::make_shared<robot_state::RobotState>(robot_model);
    };

//    ~BoltMultipleGraphsMonitor()
//    {
//      stopThreadMonitor();
//    };
    void operator()()
    {
      ROS_WARN("Void operator() thread");
    }
  //  virtual bool startThreadMonitor() = 0;

    void monitor(std::future<void> futureObj, ompl::tools::bolt::BoltPtr bolt);

    void allocBoltTaskGraphGenerator();

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

    Eigen::Isometry3d pose_;

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

      void allocBoltTaskGraphGenerator();

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
