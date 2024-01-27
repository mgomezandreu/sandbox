#include "mujoco/mujoco.h"
#include "controllers.h"

#include <iostream>
#include <algorithm>

#include <eigen3/Eigen/Dense>



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

    mjtNum z_des = 0.2;
    mjtNum x_des = 0.1;
    mjtNum y_des = 0.1;
    mjtNum kp = 100;
    mjtNum kd = 10;

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
         0, 1, 0, -1,
         -1, 0, 1, 0,
         1, -1, 1, -1;
    }

    Eigen::Vector3d pos_des(double t){
        Eigen::Vector3d pos;
        pos << 0,0, z_des + 0.1 * sin(t);
        //pos << x_des + 0.1* sin(t),y_des + 0.1 * sin(t), z_des + 0.1 * sin(t);
        return pos;
    }


    
    void control(const mjModel *m, mjData *d){
        // Define system state
        // Position and velocity
        mjtNum time = d->time;

        Eigen::Vector3d pos;
        pos << d->qpos[0], d->qpos[1], d->qpos[2];
        Eigen::Vector3d pos_dot;
        pos_dot << d->qvel[0], d->qvel[1], d->qvel[2];

        // Orientation and angular velocity
        Eigen::Vector3d euler;
        euler << d->qpos[3], d->qpos[4], d->qpos[5];
        Eigen::Vector3d euler_dot;
        euler_dot << d->qvel[3], d->qvel[4], d->qvel[5];

        Eigen::Vector3d v_des;
        v_des = kp*(pos_des(time) - pos);

        // Calculate the desired orientation
        mjtNum phi_des = kp*(v_des[0] - pos_dot[0]);
        mjtNum theta_des = kp*(v_des[1] - pos_dot[1]);
        mjtNum psi_des = 0;
        

        Eigen::Vector4d U;
        U << mass *g + kp*(v_des[2] - pos_dot[2]),
            kp*(phi_des - euler[0]) - kd*euler_dot[0],
            kp*(theta_des - euler[1]- kd*euler_dot[1]),
            kp*(psi_des - euler[2] - kd*euler_dot[2]);

        Eigen::Vector4d F = A.inverse()*U;

        
        std::cout << "F: " << F << std::endl;   


        // set constant trust to all rotors
        for (int i = 0; i < 4; i++){
            d->ctrl[i] = std::clamp(F[i], min_thrust, max_thrust);
        }

    
    }
}
