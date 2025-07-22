#include "ur3_trajectory_senders/interpolation_utils.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ur3_trajectory_senders {

using Pose = geometry_msgs::msg::PoseStamped; // all poses are stamped poses

// converts ros2 point to eigen 3d vector
static Eigen::Vector3d toEigen(const geometry_msgs::msg::Point& p){
  return {p.x, p.y, p.z};
}


bool fitSphereLS(const std::vector<Pose>& samples,
                 Eigen::Vector3d& centre,
                 double& radius)
{
  const size_t N = samples.size();
  if (N < 4) return false;  // need ≥4 points for unique sphere LS fit

  // set up linear system Ax = b
  Eigen::MatrixXd A(N-1,3);
  Eigen::VectorXd b(N-1);

  
  Eigen::Vector3d p0 = toEigen(samples[0].pose.position); // choose first point as reference
  
  for(size_t i=1;i<N;++i){
    Eigen::Vector3d pi = toEigen(samples[i].pose.position);
    A.row(i-1) = 2.0*(pi - p0);
    b(i-1)    = pi.squaredNorm() - p0.squaredNorm();
  }
  centre = A.colPivHouseholderQr().solve(b);
  radius = (p0 - centre).norm();
  return true;
}

std::vector<Pose> sampleSphere(const Eigen::Vector3d& centre,
                               double radius,
                               const std::string& frame_id,
                               rclcpp::Clock::SharedPtr clock,
                               int M)
{
  // initialzie stamped pose vector and reserve space
  std::vector<Pose> out;
  out.reserve(M);

  const double golden = M_PI * (3.0 - std::sqrt(5.0)); // golden angle ~2.39996
  
  for(int k = 0; k < M; ++k) {
    double i = k + 0.5;                // offset to avoid poles
    double y   = 1.0 - (2.0 * i) / M;            // map k to (‑1,1)
    double r   = std::sqrt(1 - y*y);    
    double phi = golden * i;

    Eigen::Vector3d dir(r*std::cos(phi), r*std::sin(phi), y); // unit vector pointing from center to surface point
    Eigen::Vector3d pos = centre + radius * dir;              // final position on surface by scaling dir

    // orientation: Z -> centre, choose X from world +Y
    Eigen::Vector3d z = (centre - pos).normalized();
    Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitY());
    
    if (x.norm() < 1e-6) x = z.cross(Eigen::Vector3d::UnitX());
    x.normalize();
    Eigen::Vector3d y_vec = z.cross(x).normalized();
    Eigen::Matrix3d R;
    R.col(0)=x; R.col(1)=y_vec; R.col(2)=z;
    Eigen::Quaterniond q(R);

    Pose P;
    P.header.frame_id = frame_id;
    P.header.stamp = clock->now();
    P.pose.position.x = pos.x();
    P.pose.position.y = pos.y();
    P.pose.position.z = pos.z();
    P.pose.orientation.w = q.w();
    P.pose.orientation.x = q.x();
    P.pose.orientation.y = q.y();
    P.pose.orientation.z = q.z();
    out.push_back(P);
  }
  return out;
}

} // namespace