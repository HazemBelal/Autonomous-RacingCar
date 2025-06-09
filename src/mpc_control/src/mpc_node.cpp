#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ma_rrt_path_plan/WaypointsArray.h>
#include <cppad/cppad.hpp>
#include <vector>
#include <algorithm>
#include <mpc_control/MPC.h>
#include <Eigen/Core>
#include <Eigen/Dense>

// Evaluate a polynomial at x
static double polyeval(const Eigen::VectorXd& coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * std::pow(x, i);
  }
  return result;
}

// Fit a polynomial of given order
static Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals,
                               const Eigen::VectorXd& yvals,
                               int order) {
  assert(xvals.size() == yvals.size());
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals[j];
    }
  }
  auto Q = A.householderQr();
  return Q.solve(yvals);
}

static constexpr double Lf            = 1.53;   // wheelbase [m]
static constexpr double MAX_STEER     = 0.52;   // ±30° in radians

class MPCNode {
public:
  MPCNode()
    : nh_("~"), x_(0), y_(0), yaw_(0), v_(0) {
    nh_.param("control_rate", control_rate_, 20.0);

    state_sub_ = nh_.subscribe("/ground_truth/state_raw", 1,
                               &MPCNode::stateCB, this);
    path_sub_  = nh_.subscribe("/waypoints",             1,
                               &MPCNode::pathCB,  this);
    cmd_pub_   = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
                  "/robot_control/command", 1);

    timer_ = nh_.createTimer(
      ros::Duration(1.0 / control_rate_),
      &MPCNode::controlLoop, this);
  }

private:
  void stateCB(const nav_msgs::Odometry::ConstPtr& odom) {
    x_   = odom->pose.pose.position.x;
    y_   = odom->pose.pose.position.y;
    yaw_ = tf::getYaw(odom->pose.pose.orientation);
    v_   = odom->twist.twist.linear.x;
  }

  void pathCB(const ma_rrt_path_plan::WaypointsArray::ConstPtr& msg) {
    waypoints_ = *msg;
  }

  void controlLoop(const ros::TimerEvent&) {
    if (waypoints_.waypoints.empty()) return;

    // Transform to vehicle coordinates and collect points
    size_t pts_count = waypoints_.waypoints.size();
    Eigen::VectorXd ptsx(pts_count), ptsy(pts_count);
    for (size_t i = 0; i < pts_count; ++i) {
      double dx = waypoints_.waypoints[i].x - x_;
      double dy = waypoints_.waypoints[i].y - y_;
      ptsx[i] = dx * std::cos(-yaw_) - dy * std::sin(-yaw_);
      ptsy[i] = dx * std::sin(-yaw_) + dy * std::cos(-yaw_);
    }

    // Fit polynomial
    Eigen::VectorXd coeffs = polyfit(ptsx, ptsy, 3);

    // Initial state in vehicle coordinates
    double cte  = polyeval(coeffs, 0.0);
    double epsi = -std::atan(coeffs[1]);
    Eigen::VectorXd state(6);
    state << 0.0, 0.0, 0.0, v_, cte, epsi;

    // Solve MPC
    std::vector<double> actuators = mpc_.Solve(state, coeffs);

    // Publish command
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.header.stamp          = ros::Time::now();
    cmd.drive.steering_angle = std::max(-MAX_STEER,
                               std::min(MAX_STEER, actuators[0]));
    cmd.drive.speed          = actuators[1];
    cmd_pub_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber  state_sub_, path_sub_;
  ros::Publisher   cmd_pub_;
  ros::Timer       timer_;
  double x_, y_, yaw_, v_, control_rate_;
  ma_rrt_path_plan::WaypointsArray waypoints_;
  MPC mpc_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mpc_node");
  MPCNode node;
  ros::spin();
  return 0;
}
