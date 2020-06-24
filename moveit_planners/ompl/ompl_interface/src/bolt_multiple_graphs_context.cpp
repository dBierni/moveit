//
// Created by db on 23.06.2020.
//

#include <moveit/ompl_interface/bolt_multiple_graphs_context.h>
#include <moveit_ompl/ompl_rosparam.h>


ompl_interface::BoltMultipleGraphsContext::BoltMultipleGraphsContext(ModelBasedStateSpacePtr state_space)
: state_space_(state_space)//, ompl::tools::bolt::Bolt{state_space}
{
  bolt_ = std::make_shared<ompl::tools::bolt::Bolt>(state_space_);
}


bool ompl_interface::BoltMultipleGraphsContext::loadParameters()
{
  moveit_ompl::loadOMPLParameters(bolt_);
//  moveit_ompl::loadOMPLParameters(&static_cast<ompl::tools::bolt::Bolt &>(*this));
  std::string file_path;
  std::string name_ = "planning_context_manager";

  for (auto it = bolt_->getGraphsInfo().begin(); it != bolt_->getGraphsInfo().end(); it++)
  {
    if (!moveit_ompl::getFilePath(file_path, it->name_,
                                  "ros/ompl_storage"))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to find file path for experience framework");
     bolt_->getGraphsInfo().erase(it--);
    }
    file_path.clear();
  }
  if (bolt_->getGraphsInfo().size() == 0 )
    return false;

 return true;

}

void ompl_interface::BoltMultipleGraphsContext::allocBoltTaskGraphGenerator()
{
  class BoltTaskGraphGenerator //: public ompl::tools::bolt::Bolt
  {
    public:
    BoltTaskGraphGenerator(ompl::tools::bolt::BoltPtr bolt)
    {

    }

    protected:

    private:
    ompl::tools::bolt::BoltPtr bolt_;

  };
}

bool ompl_interface::BoltMultipleGraphsContext::startThreadMonitor()
{
//  std::future<void> future_obj_;
  auto future_obj_ = getExitSignalFuture();
  std::thread th(&ompl_interface::BoltMultipleGraphsMonitor::monitor, this, std::move(future_obj_), bolt_);
}

void ompl_interface::BoltMultipleGraphsMonitor::monitor(std::future<void> future, ompl::tools::bolt::BoltPtr bolt)
{
  while (future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}