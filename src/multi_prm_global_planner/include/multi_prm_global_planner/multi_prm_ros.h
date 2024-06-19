#ifndef MULTI_PRM_ROS_H
#define MULTI_PRM_ROS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <vector>
#include <visualization_msgs/Marker.h>

struct Node{
  double x;
  double y;
  int node_id;
  int parent_id;
  std::vector<Node*> neighbors; 
};

namespace MULTI_PRM_planner{

    class MultiPRMPlannerROS : public nav_core::BaseGlobalPlanner
    {
    public:
    
        MultiPRMPlannerROS();

        MultiPRMPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        ~MultiPRMPlannerROS();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */

        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

        bool collision(double x, double y);

        std::pair<double, double> sampleFree();

        double getEuDistance(double x1, double x2, double y1, double y2);

        void getNeighbors(std::vector<Node>& nodes);

        bool obstacleFree(Node node1, Node node2);

        std::vector<int> solveShortestPath();







    protected:

      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      std::string frame_id_;
      ros::Publisher plan_pub_;

    private:
      ros::Publisher marker_pub_;
      double plan_time_out;
      unsigned int max_nodes_num;
      unsigned int max_neighbors_num;

      bool initialized_;
      std::vector<Node> nodes;
      double resolution_;


    };
}
#endif