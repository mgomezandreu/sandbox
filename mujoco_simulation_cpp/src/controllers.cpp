#include "mujoco/mujoco.h"
#include "controllers.h"

#include <iostream>
#include <algorithm>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>





namespace InvertedPendulum{
    mjtNum kp;
    mjtNum kd;
    mjtNum ki;
    mjtNum joint_pos_des;
    mjtNum integral;

    void control(const mjModel *m, mjData *d){

        std::cout << "kp: " << kp << std::endl;
        // Get the current joint position
        mjtNum joint_pos = d->qpos[0];

        // Calculate the error
        mjtNum error = joint_pos_des - joint_pos;

        // Calculate the integral
        integral += error;

        // Calculate the control signal
        mjtNum control_signal = kp*error - kd*d->qvel[0] + ki*integral;

        // Apply the control signal
        d->ctrl[0] = control_signal;
    }
}

namespace DroneStabilization{
    // Drone Model:
    // qpos = [x, y, z, ...]

    mjtNum z_des = 0.1;
    mjtNum x_des = 0.1;
    mjtNum y_des = 0.1;
    mjtNum kpx = 1;
    mjtNum kpz = 10;
    mjtNum kd = 1;

    mjtNum mass = 0.28;
    mjtNum g = 9.81;

    mjtNum min_thrust = 0;
    mjtNum max_thrust = 2*mass*g;


    // Allocation Matrix U = A*F
    // A = [1, 1, 1, 1;
    //      0, 1, 0, -1;
    //      -1, 0, 1, 0;
    //      1, -1, 1, -1]
    // F = [F1, F2, F3, F4]

    // Define allocation matrix as eigen matrix
    Eigen::Matrix4d A;

    void init(){
        A << 1, 1, 1, 1,
          0, .12,0,-.12,
         -.12, 0, .12, 0,
         -.1, .1, -.1, .1;
    }

    Eigen::Vector3d pos_des(double t){
        Eigen::Vector3d pos;
        pos << 0,0, z_des + 0.1 * sin(t);
        return pos;
    }
    
    void control(const mjModel *m, mjData *d){
        //clear console
        system("clear");

        // Define system state
        // Position and velocity
        mjtNum time = d->time;

        Eigen::Vector3d pos;
        pos << d->qpos[0], d->qpos[1], d->qpos[2];
        Eigen::Vector3d v;
        v << d->qvel[0], d->qvel[1], d->qvel[2];

        // Orientation and angular velocity
        Eigen::Quaterniond q(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
        Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

        // Extract roll, pitch, and yaw
        double roll = std::fmod(euler_angles[0] + M_PI, 2 * M_PI) - M_PI;
        double pitch = std::fmod(euler_angles[1] + M_PI, 2 * M_PI) - M_PI;
        double yaw = std::fmod(euler_angles[2] + M_PI, 2 * M_PI) - M_PI;

        // Print the results
        std::cout << "Quaternion: " << q.coeffs().transpose() << std::endl;
        std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;

        Eigen::Vector3d ang_vel;
        ang_vel << d->qvel[3], d->qvel[4], d->qvel[5];

        Eigen::Vector3d v_des;
        std::cout << "pos: " << pos_des(time).transpose() -pos.transpose() << std::endl;
        v_des = pos_des(time) - pos;
        // // Calculate the desired orientation
        mjtNum roll_des = 0;//std::clamp(v_des[1] - v[1], -0.0872665, 0.0872665);
        mjtNum pitch_des =  0;//std::clamp(v_des[0] - v[0], -0.0872665 , 0.0872665);
        mjtNum yaw_des = 0.5;
        


        
        Eigen::Vector4d U;
        U << mass *g + kpz*(v_des[2] - v[2]), 
            0,//0,//0.0001*(theta_des - euler[0]) - ang_vel[0],
            0,//0.1*(roll_des - roll) - ang_vel[0],// 0.0001*(theta_des - euler[1]) - ang_vel[1],
            (yaw_des - yaw) - ang_vel[2];

        
        // std::cout << "A:" << A.inverse() << std::endl;

    
        Eigen::Vector4d F = A.inverse()*U;
        // set F to sqrt(F) to get thrust
        for (int i = 0; i < 4; i++){
            F[i] = std::sqrt(std::clamp(F[i], min_thrust, max_thrust));
        }

        // set constant trust to all rotors
        for (int i = 0; i < 4; i++){
            d->ctrl[i] = std::clamp(F[i], min_thrust, max_thrust);
        }


    }
}
