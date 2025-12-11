#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

// define the Nodes
struct AStarNode
{
    AStarNode *parent;
    double f;
    double g;
    double h;
    int r;
    int c;
    bool expanded;

    // constructor
    AStarNode(int c, int r)
        : parent(nullptr), f(INFINITY), g(INFINITY), h(INFINITY), r(r), c(c), expanded(false)
    {
    }
};

// Define the Open list and comparators using std::priority_queue
template <typename T> // T must be pointer type
struct OpenListComparator
{
    bool operator()(const T &l, const T &r) const { return l->f > r->f; };
};

template <typename T> // T must be pointer type
using OpenList = std::priority_queue<T, std::deque<T>, OpenListComparator<T>>;

class Planner : public rclcpp::Node
{

private:
    // Node Instance Properties =============================================================

    // Handles
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_request_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data from Subscriptions
    std::vector<int8_t> costmap_; // read only; written only by subscriber(s)
    double costmap_resolution_;   // read only; written only by subscriber(s)
    double costmap_origin_x_;     // read only; written only by subscriber(s)
    double costmap_origin_y_;     // read only; written only by subscriber(s)
    double costmap_rows_;         // read only; written only by subscriber(s)
    double costmap_cols_;         // read only; written only by subscriber(s)
    double rbt_x_;                // read only; written only by subscriber(s)
    double rbt_y_;                // read only; written only by subscriber(s)
    double goal_x_;               // read only; written only by subscriber(s)
    double goal_y_;               // read only; written only by subscriber(s)

    // Parameters
    double max_access_cost_; // read only; written only in constructor

    // Other Instance Variables
    bool has_new_request_;
    bool received_map_;

public:
    explicit Planner(std::string node_name = "tb3_yolo_planner")
        : rclcpp::Node(node_name)
    {
        // Node Constructor =============================================================

        // Declare Parameters
        this->declare_parameter<int>("max_access_cost", 100);

        // Get Parameters
        this->max_access_cost_ = this->get_parameter("max_access_cost").get_value<int>();

        // Global costmap subscriber
        auto qos = rclcpp::ServicesQoS();
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal); // latch
        this->sub_global_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "custom_costmap",
            qos, // latching qos. Only one message is published, and at the start of launch.
            std::bind(&Planner::callbackSubGlobalCostmap_, this, std::placeholders::_1));

        // Path request subscriber
        this->sub_path_request_ = this->create_subscription<nav_msgs::msg::Path>(
            "path_request",
            10,
            std::bind(&Planner::callbackSubPathRequest_, this, std::placeholders::_1));

        // Path publisher
        this->pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
            "path",
            10);

        // Timers:
        this->timer_ = this->create_timer(
            0.1s,
            std::bind(&Planner::callbackTimer_, this));

        // Instance Variables
        this->has_new_request_ = false;
        this->received_map_ = false;
    }

private:
    // Callbacks =============================================================

    // Path request subscriber callback
    void callbackSubPathRequest_(nav_msgs::msg::Path::SharedPtr msg)
    {
        // !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_
        this->rbt_x_ = msg->poses[0].pose.position.x;
        this->rbt_y_ = msg->poses[0].pose.position.y;
        this->goal_x_ = msg->poses[1].pose.position.x;
        this->goal_y_ = msg->poses[1].pose.position.y;

        this->has_new_request_ = true;
    }

    // Global costmap subscriber callback
    // This is only run once because the costmap is only published once, at the start of the launch.
    void callbackSubGlobalCostmap_(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
        this->costmap_ = msg->data;
        this->costmap_resolution_ = msg->info.resolution;
        this->costmap_origin_x_ = msg->info.origin.position.x;
        this->costmap_origin_y_ = msg->info.origin.position.y;
        this->costmap_rows_ = msg->info.height;
        this->costmap_cols_ = msg->info.width;
        this->received_map_ = true;
    }

    // runs the path planner at regular intervals as long as there is a new path request.
    void callbackTimer_()
    {
        if (!this->received_map_ || !this->has_new_request_)
            return; // silently return if no new request or map is not received.

        // run the path planner
        aStar_(this->rbt_x_, this->rbt_y_, this->goal_x_, this->goal_y_);

        // reset the request
        this->has_new_request_ = false;
    }

    // Converts world coordinates to cell column and cell row.
    std::pair<int, int> XYToCR_(double x, double y)
    {
        double dx = x - this->costmap_origin_x_;
        double dy = y - this->costmap_origin_y_;

        int c = std::floor(dx / this->costmap_resolution_);
        int r = std::floor(dy / this->costmap_resolution_);

        return {c, r};
    }

    // Converts cell column and cell row to world coordinates.
    std::pair<double, double> CRToXY_(int c, int r)
    {
        double dc = 0.5 + c;
        double dr = 0.5 + r;

        double x = (dc * costmap_resolution_) + this->costmap_origin_x_;
        double y = (dr * costmap_resolution_) + this->costmap_origin_y_;

        return {x, y};
    }

    // Converts cell column and cell row to flattened array index.
    int CRToIndex_(int c, int r)
    {
        return r * this->costmap_cols_ + c;
    }

    // Returns true if the cell column and cell row is outside the costmap.
    bool outOfMap_(int c, int r)
    {
        return c < 0 || r < 0 || c >= this->costmap_cols_ || r >= this->costmap_rows_;
    }

    // Runs the path planning algorithm based on the world coordinates.
    void aStar_(double start_x, double start_y, double goal_x, double goal_y)
    {
        // Initializations ---------------------------------

        // nodes
        std::vector<AStarNode> nodes;
        for (int r = 0; r < this->costmap_rows_; ++r)
            for (int c = 0; c < this->costmap_cols_; ++c)
                nodes.emplace_back(c, r);

        // initialize start and goal
        auto [rbt_c, rbt_r] = this->XYToCR_(start_x, start_y);
        auto [goal_c, goal_r] = this->XYToCR_(goal_x, goal_y);
        int rbt_idx = this->CRToIndex_(rbt_c, rbt_r);
        AStarNode *start_node = &nodes[rbt_idx];
        start_node->g = 0;
        start_node->h = std::hypot(goal_c - rbt_c, goal_r - rbt_r);
        start_node->f = start_node->h;
        start_node->expanded = false;

        // open list
        OpenList<AStarNode *> open_list;
        open_list.push(start_node);

        // Expansion Loop ---------------------------------------------
        while (!open_list.empty())
        {
            // Poll the cheapest node
            AStarNode *node = open_list.top();
            open_list.pop();

            // Skip if visited
            if (node->expanded)
                continue;
            node->expanded = true;

            // Return path if reached goal
            if (node->c == goal_c && node->r == goal_r)
            {
                // obtain the path from the nodes
                nav_msgs::msg::Path msg_path;
                msg_path.header.stamp = this->get_clock()->now();
                msg_path.header.frame_id = "map";

                while (node != nullptr)
                {
                    // get the coordinates of current node
                    auto [x, y] = this->CRToXY_(node->c, node->r);

                    // write the path point
                    geometry_msgs::msg::PoseStamped path_pose;
                    path_pose.pose.position.x = x;
                    path_pose.pose.position.y = y;

                    // push path point to the array in the message
                    msg_path.poses.push_back(path_pose);

                    // go to parent
                    node = node->parent;
                }

                // reverse path so that the front of the path is where the robot is, instead of where the goal is.
                std::reverse(msg_path.poses.begin(), msg_path.poses.end());

                // publish path
                this->pub_path_->publish(msg_path);

                // RCLCPP_INFO(this->get_logger(),
                            // "Path Found from Rbt @ (%7.3f, %7.3f) to Goal @ (%7.3f, %7.3f)!",
                            // start_x, start_y, goal_x, goal_y);

                return;
            }

            // Neighbor Loop --------------------------------------------------
            for (auto [dc, dr] : std::vector<std::pair<int, int>>{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}})
            {
                // Get neighbor coordinates and neighbor
                int nb_c = node->c + dc;
                int nb_r = node->r + dr;

                // Continue if out of map
                if (this->outOfMap_(nb_c, nb_r))
                    continue;

                // Extract pointer to neighboring node
                int nb_idx = this->CRToIndex_(nb_c, nb_r);
                AStarNode *nb_node = &nodes[nb_idx];

                // Continue if neighbor is expanded
                if (nb_node->expanded)
                    continue;

                // Ignore if the cell cost exceeds max_access_cost_ (to avoid passing through obstacles)
                int nb_cell_cost = this->costmap_[nb_idx];
                if (nb_cell_cost > this->max_access_cost_)
                    continue;

                // Get the relative g-cost and push to open-list if cheaper to get from parent
                double new_nb_g = node->g + std::hypot(dc, dr) * (nb_cell_cost + 1);
                if (nb_node->g > new_nb_g)
                {
                    nb_node->g = new_nb_g;
                    nb_node->h = std::hypot(goal_c - nb_c, goal_r - nb_r); // assumes smallest costs are 1.
                    nb_node->f = nb_node->g + nb_node->h;
                    nb_node->parent = node;
                    open_list.push(nb_node);
                }
            }
        }
        RCLCPP_WARN(this->get_logger(), "No Path Found! Check that Robot and Goal points are not in inflated zones (too close to wall)!"); // should publish an empty path.
    }
};

// Main Boiler Plate =============================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
    return 0;
}