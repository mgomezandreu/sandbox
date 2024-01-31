#include "angles.h"
#include <cmath>

namespace angles{
    EulerAngles quaternionToEulerAngles(const Quaternion& q){
        EulerAngles angles;


        double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        angles.roll = std::atan2(sinr_cosp, cosr_cosp);


        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1.0)
            angles.pitch = std::copysign(M_PI / 2.0, sinp); // Use 90 degrees if out of range
        else
            angles.pitch = std::asin(sinp);


        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        angles.yaw = std::atan2(siny_cosp, cosy_cosp);


        if (angles.yaw < 0.0)
            angles.yaw += 2.0 * M_PI;

        return angles;

    }

    double signed_angular_distance(double angle1, double angle2){
  
        angle1 = std::fmod(angle1, 2.0 * M_PI);
        angle2 = std::fmod(angle2, 2.0 * M_PI);


        double distance = angle1 - angle2;

        if (distance > M_PI)
            distance -= 2.0 * M_PI;
        else if (distance < -M_PI)
            distance += 2.0 * M_PI;

        return distance;
    }
}