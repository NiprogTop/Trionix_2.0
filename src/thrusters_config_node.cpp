#include <ros/ros.h>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>

#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>

using Eigen::MatrixXd;
typedef Eigen::Matrix<double,6,1> RowVector6d;

class ThrusterController
{
public:
  ThrusterController(ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh("~");

    if (pnh.hasParam("thrusters"))
    {
        XmlRpc::XmlRpcValue thrusters;
        pnh.getParam("thrusters", thrusters);
        // ROS_INFO(thrusters);
        parseThrusters_(nh, thrusters);
    }
    else
      throw "Doesn't found parameters";

    wrench_sub_ = nh.subscribe<geometry_msgs::Wrench>("command", 1, &ThrusterController::wrenchCallback, this);
  }

  ~ThrusterController()
  {
    std_msgs::Float64 msg;
    msg.data = 0.0;
    for(auto p : thrusters_publishers_)
      p.publish(msg);
  }

  void wrenchCallback(const geometry_msgs::WrenchConstPtr& msg)
  {
    wrenchMsgToEigen_(*msg, wrench_);
    Eigen::VectorXd effort = tam_ * wrench_;
    std_msgs::Float64 thrust_msg;
    for(unsigned i = 0; i < tam_.rows(); ++i)
    {
      thrust_msg.data = effort[i] > 0 ? k_forward_[i] * effort[i] : k_backward_[i] * effort[i];
      thrusters_publishers_[i].publish(thrust_msg);
    }
  }

private:
  void parseThrusters_(ros::NodeHandle& nh, XmlRpc::XmlRpcValue &thrusters)
  {
    MatrixXd T(6, thrusters.size());
    thrusters_publishers_.resize(static_cast<unsigned>(thrusters.size()));
    k_forward_.resize(static_cast<unsigned long>(thrusters.size()));
    k_backward_.resize(static_cast<unsigned long>(thrusters.size()));
    num_thruster_.resize(static_cast<unsigned long>(thrusters.size()));

    unsigned current_col = 0;
    for(auto thruster: thrusters)
    {
      
      Eigen::Vector3d f, r;
      for(int i = 0; i < 3; ++i)
      {
        f[i] = static_cast<double>(thruster.second["f"][i]);
        r[i] = static_cast<double>(thruster.second["r"][i]);
      }
      T.col(current_col) << f, r.cross(f);
      if(thruster.second.hasMember("k_forward"))
        k_forward_[current_col] = static_cast<double>(thruster.second["k_forward"]);
      else
        k_forward_[current_col] = 1.0;

      if(thruster.second.hasMember("k_backward"))
        k_backward_[current_col] = static_cast<double>(thruster.second["k_backward"]);
      else
        k_backward_[current_col] = 1.0;
      
      num_thruster_[current_col] = static_cast<int>(thruster.second["thruster_number"]);
      

      thrusters_publishers_[current_col] = nh.advertise<std_msgs::Float64>("thrusters/" + thruster.first, 1);

      ROS_INFO("Thruster: %s", thruster.first.c_str());
      ROS_INFO("f: [%.2f %.2f %.2f]", f[0], f[1], f[2]);
      ROS_INFO("r: [%.2f %.2f %.2f]", r[0], r[1], r[2]);
      ROS_INFO("K_f: %.2f; K_b: %.2f", k_forward_[current_col], k_backward_[current_col]);
      ROS_INFO("N_thruster: %2i", num_thruster_[current_col]);

      current_col++;
    }

    tam_ = pseudoInverse(T);

    ROS_INFO_STREAM("T\n" << T);
    ROS_INFO_STREAM("Inverse T:\n" << tam_);
  }

  void wrenchMsgToEigen_(const geometry_msgs::Wrench& msg, RowVector6d& wrench)
  {
    wrench[0] = msg.force.x;
    wrench[1] = msg.force.y;
    wrench[2] = msg.force.z;
    wrench[3] = msg.torque.x;
    wrench[4] = msg.torque.y;
    wrench[5] = msg.torque.z;
  }

  template<typename _Matrix_Type_>
  _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
  {
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
  }

  ros::Subscriber wrench_sub_;

  std::vector<ros::Publisher> thrusters_publishers_;

  RowVector6d wrench_;
  MatrixXd tam_;

  std::vector<double> k_forward_;
  std::vector<double> k_backward_;
  std::vector<int> num_thruster_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrusters_config_node");
  ros::NodeHandle nh;
  ThrusterController t(nh);
  ros::spin();
}
