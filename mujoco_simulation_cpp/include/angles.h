#ifndef ANGLES_H
#define ANGLES_H

namespace angles{
    struct Quaternion {
        double w, x, y, z;
    };
    
    struct EulerAngles {
        double yaw, pitch, roll;
    };

    EulerAngles quaternionToEulerAngles(const Quaternion& q);
    double signed_angular_distance(double angle1, double angle2); 
}

#endif // ANGLES_H