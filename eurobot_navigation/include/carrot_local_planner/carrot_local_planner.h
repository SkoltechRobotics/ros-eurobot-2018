#ifndef CARROT_LOCAL_PLANNER_H_
#define CARROT_LOCAL_PLANNER_H_

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
//#include <base_local_planner/costmap_model.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// transforms
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>

using std::string;

namespace carrot_local_planner 
{

    class CarrotLocalPlanner : public nav_core::BaseLocalPlanner 
    {
        public:
            
            /**
             * @brief Default constructor of the teb plugin
             */
            CarrotLocalPlanner();
            
            /**
             * @brief  Destructor of the plugin
             */
            ~CarrotLocalPlanner();

            /**
             * @brief Constructs the local planner
             * @param name The name to give this instance of the local planner
             * @param tf A pointer to a transform listener
             * @param costmap_ros The cost map to use for assigning costs to local plans
             */
            void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

            /** overridden classes from interface nav_core::BaseLocalPlanner **/

            /**
             * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
             * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
             * @return True if a valid velocity command was found, false otherwise
             */
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            /**
             * @brief  Check if the goal pose has been achieved by the local planner
             * @return True if achieved, false otherwise
             */
            bool isGoalReached();

            /**
             * @brief  Set the plan that the local planner is following
             * @param plan The plan to pass to the local planner
             * @return True if the plan was updated successfully, false otherwise
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        private:
            // external objects (store weak pointers)
            costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
            costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
            tf::TransformListener* tf_; //!< pointer to Transform Listener

            // internal objects (memory management owned)
            std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
            bool goal_reached_; //!< store whether the goal is reached or not
            

            // flags
            bool initialized_; //!< Keeps track about the correct initialization of this class
    };
};
#endif
