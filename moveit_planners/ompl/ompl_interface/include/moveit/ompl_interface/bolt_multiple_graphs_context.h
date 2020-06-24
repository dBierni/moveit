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
    BoltMultipleGraphsMonitor()
    {

    };


    virtual bool startThreadMonitor() = 0;

    void monitor(std::future<void> futureObj, ompl::tools::bolt::BoltPtr bolt);

    void allocBoltTaskGraphGenerator();

    std::future<void> getExitSignalFuture()
    {
      return exit_signal_.get_future();
    }

    void stopThreadMonitor()
    {
      exit_signal_.set_value();
    }

protected:

private:

    std::promise<void> exit_signal_;

    Eigen::Isometry3d pose_;
};

class BoltMultipleGraphsContext: public BoltMultipleGraphsMonitor
{
public:
    BoltMultipleGraphsContext(ModelBasedStateSpacePtr state_space);

//    virtual ~BoltMultipleGraphsContext() = default;

    bool loadParameters();

    bool initializeGraphInfo();

   bool startThreadMonitor() override ;


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
