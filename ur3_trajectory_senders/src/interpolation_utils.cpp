#include "ur3_trajectory_senders/interpolation_utils.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace ur3_trajectory_senders
{

struct Circle3D
{
  Eigen::Vector3d center;
  double radius;
  Eigen::Vector3d u;
  Eigen::Vector3d v;
};

static Circle3D circleThroughThreePoints(const Eigen::Vector3d &p1,
                                         const Eigen::Vector3d &p2,
                                         const Eigen::Vector3d &p3)
{
  Eigen::Vector3d u_vec = p2 - p1;
  Eigen::Vector3d v_vec = p3 - p1;
  Eigen::Vector3d n = u_vec.cross(v_vec);
  double n_norm = n.norm();
  if (n_norm < 1e-6)
    throw std::runtime_error("Points are colinear – circle undefined");

  Eigen::Vector3d u = u_vec.normalized();
  Eigen::Vector3d v_tmp = v_vec - v_vec.dot(u) * u;
  Eigen::Vector3d v = v_tmp.normalized();

  double d = u_vec.norm();
  double x3 = v_vec.dot(u);
  double y3 = v_vec.dot(v);

  double denom = 2 * (x3 * 0 - y3 * d);
  if (std::abs(denom) < 1e-8)
    throw std::runtime_error("Degenerate circle");

  double cx = ( (d*d) * y3 ) / denom;
  double cy = ( (x3*x3 + y3*y3) * (-d) ) / denom;
  double radius = std::sqrt(cx*cx + cy*cy);

  Eigen::Vector3d center3d = p1 + cx * u + cy * v;
  return {center3d, radius, u, v};
}

static double angleOfPoint(const Eigen::Vector3d &p, const Circle3D &c)
{
  Eigen::Vector3d d = p - c.center;
  double x = d.dot(c.u);
  double y = d.dot(c.v);
  return std::atan2(y, x);
}

std::vector<geometry_msgs::msg::PoseStamped>
interpolateArc(const geometry_msgs::msg::PoseStamped &start,
               const geometry_msgs::msg::PoseStamped &mid,
               const geometry_msgs::msg::PoseStamped &end,
               int samples)
{
  if (samples < 2) samples = 2;

  auto toVec = [](const geometry_msgs::msg::Point &p){
    return Eigen::Vector3d(p.x,p.y,p.z);
  };
  Eigen::Vector3d p1 = toVec(start.pose.position);
  Eigen::Vector3d p2 = toVec(mid.pose.position);
  Eigen::Vector3d p3 = toVec(end.pose.position);

  Circle3D circ = circleThroughThreePoints(p1,p2,p3);

  double a1 = angleOfPoint(p1,circ);
  double a3 = angleOfPoint(p3,circ);

  if (a3 < a1) a3 += 2*M_PI;
  double a2 = angleOfPoint(p2,circ);
  if (a2 < a1) a2 += 2*M_PI;
  if (!(a1 < a2 && a2 < a3)) {
    a3 -= 2*M_PI;
    if (a2 < a1) a2 += 2*M_PI;
  }

  Eigen::Quaterniond q1(start.pose.orientation.w, start.pose.orientation.x,
                        start.pose.orientation.y, start.pose.orientation.z);
  Eigen::Quaterniond q3(end.pose.orientation.w, end.pose.orientation.x,
                        end.pose.orientation.y, end.pose.orientation.z);

  std::vector<geometry_msgs::msg::PoseStamped> out;
  out.reserve(samples);

  for (int i=0;i<samples;++i) {
    double t = static_cast<double>(i)/(samples-1);
    double ang = (1.0 - t) * a1 + t * a3;
    Eigen::Vector3d pos = circ.center + circ.radius * (std::cos(ang)*circ.u + std::sin(ang)*circ.v);

    Eigen::Quaterniond q = q1.slerp(t, q3);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = start.header;
    ps.pose.position.x = pos.x();
    ps.pose.position.y = pos.y();
    ps.pose.position.z = pos.z();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    out.push_back(ps);
  }

  return out;
}

} // namespace ur3_trajectory_senders