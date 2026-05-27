#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/pose.hpp>

struct Vec3
{
  double x{};
  double y{};
  double z{};
};

inline double lerp(double a, double b, double t)
{
  return (1.0 - t) * a + t * b;
}

inline Vec3 lerpVec3(const Vec3& a, const Vec3& b, double t)
{
  return {
    lerp(a.x, b.x, t),
    lerp(a.y, b.y, t),
    lerp(a.z, b.z, t)
  };
}

inline Vec3 cubicBezierVec3(
  const Vec3& p0,
  const Vec3& p1,
  const Vec3& p2,
  const Vec3& p3,
  double t)
{
  const Vec3 a = lerpVec3(p0, p1, t);
  const Vec3 b = lerpVec3(p1, p2, t);
  const Vec3 c = lerpVec3(p2, p3, t);

  const Vec3 d = lerpVec3(a, b, t);
  const Vec3 e = lerpVec3(b, c, t);

  return lerpVec3(d, e, t);
}

inline Vec3 poseToVec3(const geometry_msgs::msg::Pose& p)
{
  return {p.position.x, p.position.y, p.position.z};
}

inline geometry_msgs::msg::Pose vec3ToPose(
  const Vec3& v,
  const geometry_msgs::msg::Pose& reference_pose)
{
  auto p = reference_pose;

  p.position.x = v.x;
  p.position.y = v.y;
  p.position.z = v.z;

  return p;
}

inline void appendSegment(
  std::vector<geometry_msgs::msg::Pose>& full_path,
  const std::vector<geometry_msgs::msg::Pose>& segment)
{
  if (segment.empty())
    return;

  if (full_path.empty())
  {
    full_path.insert(full_path.end(), segment.begin(), segment.end());
  }
  else
  {
    full_path.insert(full_path.end(), segment.begin() + 1, segment.end());
  }
}

inline std::vector<geometry_msgs::msg::Pose> sampleCubicBezier(
  const geometry_msgs::msg::Pose& p0,
  const geometry_msgs::msg::Pose& p1,
  const geometry_msgs::msg::Pose& p2,
  const geometry_msgs::msg::Pose& p3,
  int num_points)
{
  num_points = std::max(num_points, 2);

  std::vector<geometry_msgs::msg::Pose> output;
  output.reserve(num_points);

  const Vec3 v0 = poseToVec3(p0);
  const Vec3 v1 = poseToVec3(p1);
  const Vec3 v2 = poseToVec3(p2);
  const Vec3 v3 = poseToVec3(p3);

  for (int i = 0; i < num_points; ++i)
  {
    const double t =
      static_cast<double>(i) / static_cast<double>(num_points - 1);

    output.push_back(
      vec3ToPose(cubicBezierVec3(v0, v1, v2, v3, t), p3));
  }

  return output;
}

inline std::vector<geometry_msgs::msg::Pose> buildVerticalBezier(
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& end_pose,
  int num_points,
  double handle_ratio)
{
  auto p0 = start_pose;
  auto p1 = start_pose;
  auto p2 = end_pose;
  auto p3 = end_pose;

  const double dz = end_pose.position.z - start_pose.position.z;
  const double handle = std::fabs(dz) * handle_ratio;

  p1.position.z = start_pose.position.z + (dz >= 0.0 ? handle : -handle);
  p2.position.z = end_pose.position.z - (dz >= 0.0 ? handle : -handle);

  p1.orientation = end_pose.orientation;
  p2.orientation = end_pose.orientation;
  p3.orientation = end_pose.orientation;

  return sampleCubicBezier(p0, p1, p2, p3, num_points);
}

inline std::vector<geometry_msgs::msg::Pose> buildAboveToAboveBezier(
  const geometry_msgs::msg::Pose& start_above,
  const geometry_msgs::msg::Pose& end_above,
  int num_points,
  double extra_peak_z)
{
  auto p0 = start_above;
  auto p1 = start_above;
  auto p2 = end_above;
  auto p3 = end_above;

  const double dx = end_above.position.x - start_above.position.x;
  const double dy = end_above.position.y - start_above.position.y;

  const double peak_z =
    std::max(start_above.position.z, end_above.position.z) + extra_peak_z;

  p1.position.x = start_above.position.x + 0.25 * dx;
  p1.position.y = start_above.position.y + 0.25 * dy;
  p1.position.z = peak_z;

  p2.position.x = start_above.position.x + 0.75 * dx;
  p2.position.y = start_above.position.y + 0.75 * dy;
  p2.position.z = peak_z;

  p1.orientation = end_above.orientation;
  p2.orientation = end_above.orientation;
  p3.orientation = end_above.orientation;

  return sampleCubicBezier(p0, p1, p2, p3, num_points);
}