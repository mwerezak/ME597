#ifndef POSE_UTIL_HPP
#define POSE_UTIL_HPP

// Convenience macros
#define POSEST_X(pose_stamped) (pose_stamped.pose.position.x)
#define POSEST_Y(pose_stamped) (pose_stamped.pose.position.y)
#define POSEST_Z(pose_stamped) (pose_stamped.pose.position.z)
#define POSEST_O(pose_stamped) (pose_stamped.pose.orientation)

#endif