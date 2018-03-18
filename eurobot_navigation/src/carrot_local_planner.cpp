#include <carrot_local_planner/carrot_local_planner.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(carrot_local_planner::CarrotLocalPlanner, nav_core::BaseLocalPlanner)

namespace carrot_local_planner
{

    CarrotLocalPlanner::CarrotLocalPlanner() : tf_(NULL),costmap_ros_(NULL),initialized_(false)
    {
    }

    CarrotLocalPlanner::~CarrotLocalPlanner()
    {
    }

    void CarrotLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // check if the plugin is already initialized
        if(!initialized_)
        {
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            goal_reached_ = false;
  
            // set initialized flag
            initialized_ = true;

            ROS_DEBUG("CarrotLocalPlanner: plugin initialized.");
        } 
        else
        {
            ROS_WARN("CarrotLocalPlanner: plugin has already been initialized, doing nothing.");
        }
    }

    bool CarrotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        return true;
    }

    bool CarrotLocalPlanner::isGoalReached()
    {
        if (goal_reached_)
        {
            ROS_INFO("CarrotLocalPlanner: GOAL Reached!");
        }
        return goal_reached_;
    }

    bool CarrotLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        // store the global plan
        global_plan_.clear();
        global_plan_ = plan;
        return true;
    }

}
