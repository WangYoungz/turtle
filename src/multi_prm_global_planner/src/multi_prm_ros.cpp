#include <multi_prm_global_planner/multi_prm_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(MULTI_PRM_planner::MultiPRMPlannerROS, nav_core::BaseGlobalPlanner)

namespace MULTI_PRM_planner{

    MultiPRMPlannerROS::MultiPRMPlannerROS() :
    costmap_(nullptr),
    initialized_(false)
    {}

    MultiPRMPlannerROS::MultiPRMPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(costmap_ros),
    initialized_(false)
    {
        initialize(name, costmap_ros);
    }

    MultiPRMPlannerROS::~MultiPRMPlannerROS()
    {}

  //变量初始化
    void MultiPRMPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap();
            frame_id_ = costmap_ros->getGlobalFrameID();
        
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1); //发布全局计算
            marker_pub_ = private_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 ); //发布可视化扩展过程
        
            resolution_ = costmap_->getResolution(); //地图分辨率

            double max_nodes_num_;
            double max_neighbors_num_;

            private_nh.param("plan_time_out",plan_time_out,10.0); //规划超时。默认10s
            private_nh.param("max_nodes_num",max_nodes_num_,2000.0); //最大采样点
            private_nh.param("max_neighbors_num",max_neighbors_num_, 10.0); //最大采样点

            max_nodes_num = static_cast<unsigned int>(max_nodes_num_);
            max_neighbors_num = static_cast<unsigned int>(max_neighbors_num_);

            ROS_INFO("Multi prm planner initialized successfully");
            initialized_ = true;
        }
        else
        {
        ROS_WARN("This planner has already been initialized... doing nothing");
        }        

    }

    bool MultiPRMPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan){
        plan.clear();

        if(this->collision(start.pose.position.x, start.pose.position.y))
        {
        ROS_WARN("failed to get a path.start point is obstacle.");
        return false;
        }

        if(this->collision(goal.pose.position.x, goal.pose.position.y))
        {
        ROS_WARN("failed to get a path.goal point is obstacle.");
        return false;
        }

        Node start_node;
        start_node.x = start.pose.position.x;
        start_node.y = start.pose.position.y;
        start_node.node_id = 0;
        start_node.parent_id = -1;
        nodes.push_back(start_node);

        Node goal_node;
        goal_node.x = goal.pose.position.x;
        goal_node.y = goal.pose.position.y;
        goal_node.node_id = 1;
        nodes.push_back(goal_node);

        unsigned int seed = 0;
        double start_time = ros::Time::now().toSec();
        while(ros::ok()){
            if( (ros::Time::now().toSec()-start_time) > plan_time_out)
            {
                ROS_WARN("failed to get a path.time out.");
                return false;
            }
            std::vector<std::pair<double, double>> samples;
            int num = 1;
            while (ros::ok() && nodes.size() < max_nodes_num-1)
            {
                std::pair<double, double> sample;
                sample = sampleFree();
                Node node;
                node.x = sample.first;
                node.y = sample.second;
                node.node_id = num++;
                nodes.push_back(node);
                ROS_INFO("(%f, %f)", sample.first, sample.second);
            }



            // getNeighbors(nodes);

            // std::vector<int> path1 = solveShortestPath();

            // ros::Time plan_time = ros::Time::now();
            // geometry_msgs::PoseStamped pose;
            // pose.pose.position.x = nodes[0].x;
            // pose.pose.position.y = nodes[0].y;
            // pose.pose.position.z = 0.0;
            // pose.pose.orientation.x = 0.0;
            // pose.pose.orientation.y = 0.0;
            // pose.pose.orientation.z = 0.0;
            // pose.pose.orientation.w = 1.0;

            // for(int i = 0; i < path1.size(); i++){
            //     pose.header.frame_id = this->frame_id_;
            //     pose.header.stamp = plan_time;
            //     pose.pose.position.x = nodes[path1[i]].x;
            //     pose.pose.position.y = nodes[path1[i]].y;
            //     plan.push_back(pose);
            // }
            // plan[0].pose.orientation = start.pose.orientation;
            // plan[plan.size()-1].pose.orientation = goal.pose.orientation;

            // nav_msgs::Path path_pose;
            // path_pose.header.frame_id = this->frame_id_;
            // path_pose.header.stamp = ros::Time::now();
            // path_pose.poses = plan;
            // plan_pub_.publish(path_pose);

            return true;
        }
        ROS_WARN("failed to get a path.");
        return false;
    }

    //检查点是否为障碍物
    bool MultiPRMPlannerROS::collision(double x, double y){
        unsigned int mx, my;
        if(!this->costmap_->worldToMap(x, y, mx, my)){
            return true;
        }
        if(mx >= this->costmap_->getSizeInCellsX()
                     || my >= this->costmap_->getSizeInCellsY()
                      || mx <= 0 || my <= 0){
                        return true;
                      } 
        if(costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            return true;
        }
        return false;
    }

    //生成一个随机点
    std::pair<double, double> MultiPRMPlannerROS::sampleFree(){
        unsigned int seed = 0;
        unsigned int x = 0.0, y = 0.0;
        double mx = 0, my = 0;
        unsigned int map_size_x = costmap_->getSizeInCellsX();
        unsigned int map_size_y = costmap_->getSizeInCellsY();
        std::pair<double, double> random_point;
        while(ros::ok()){
            srand(ros::Time::now().toSec() + seed++);
            x = rand() % map_size_x;
            srand(ros::Time::now().toSec() + seed++);
            y = rand() % map_size_y;
            if(this->costmap_->getCost(x, y) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) break;
        }
        this->costmap_->mapToWorld(x, y, mx, my);
        random_point.first = mx;
        random_point.second = my;
        return random_point;
    }

    //欧式距离
    double MultiPRMPlannerROS::getEuDistance(double x1, double x2, double y1, double y2){
        return std::sqrt(std::pow((x1, x2), 2) + std::pow((y1, y2), 2));
    }

    //计算邻居节点
    void MultiPRMPlannerROS::getNeighbors(std::vector<Node>& nodes){
        for(auto& i:nodes){
            std::map<float, Node*> distanceMap;
            for(auto& j:nodes){
                if(i.node_id != j.node_id && obstacleFree(i, j)){
                    distanceMap[getEuDistance(i.x, j.x, i.y, j.y)] = &j;
                }
            }

            int count = 0;
            for(auto iter:distanceMap){
                if(count > max_neighbors_num) break;
                // if(obstacleFree(i, iter.second)){
                    i.neighbors.push_back(iter.second);
                // }
                count++;
            }
        }
    }

    //两点之间是否有障碍物
    bool MultiPRMPlannerROS::obstacleFree(Node node1, Node node2){
        double dist = getEuDistance(node1.x, node2.x, node1.y, node2.y);
        if(dist < resolution_){
            if(collision(node1.x, node1.y) || collision(node2.x, node2.y)) return false;
            return true;
        }
        else{
            int value = int(floor(dist/resolution_));
            double angle = atan2(node2.y - node1.y, node2.x - node1.x);
            double x, y;
            for(int i = 1; i <= value; i++){
                x = node1.x + i * resolution_ * cos(angle);
                y = node1.y + i * resolution_ * sin(angle);
                if(collision(x, y)) return false;
            }
        }
        return true;
    }

    std::vector<int> MultiPRMPlannerROS::solveShortestPath(){
        double dist[max_nodes_num] = {10000.0};
        bool vset[max_nodes_num] = {true};
        int prev[max_nodes_num] = {-2};
        
        dist[0] = 0;
        prev[0] = -1;

        while(true){
            int sum = 0;
            for(auto i : vset){
                if(sum == 0) break;
                sum += vset[i];
            }

            int u = -1;
            double low = 10000.0;
            for(int i = 0; i < max_nodes_num; i++){
                if(vset[i]){
                    if(u == -1 || dist[i] < low){
                        low = dist[i];
                        u = i;
                    }
                }
            }

            vset[u] = false;

            for(Node* v : nodes[u].neighbors){
                auto alt = dist[u] + getEuDistance(v->x, nodes[u].x, v->y, nodes[u].y);
                if(alt < dist[v->node_id]){
                    dist[v->node_id] = alt;
                    prev[v->node_id] = u;
                }
            }
        }
        std::vector<int> path;
        int node = 1;
        while(node != -1){
            path.push_back(node);
            node = prev[node];
        }
        return path;
    }


};
