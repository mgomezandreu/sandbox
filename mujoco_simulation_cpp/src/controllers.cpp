#include "mujoco/mujoco.h"
#include "controllers.h"
#include "angles.h"

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
    mjtNum kpx = 10;
    mjtNum kpy = 10;
    mjtNum kpz = 10;
    mjtNum kd = 1;

    mjtNum mass = 0.28;
    mjtNum g = 9.81;

    mjtNum min_thrust = 0;
    mjtNum max_thrust = 4*mass*g;


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
        pos << 0 + 0.5*sin(t), 0.5*sin(0.5*t) , z_des + 0.5 + sin(t);
        return pos;
    }

    void printLidar(const mjModel *m, mjData *d){
        for (int i = 0; i < 8; ++i) {
            std::string sensorName = "lidar" + std::to_string(i);
            int sensorIndex = mj_name2id(m, mjOBJ_SENSOR, sensorName.c_str());
            double rangeReading = d->sensordata[sensorIndex];

            std::cout << i << ": " << rangeReading << " | ";
            // Use rangeReading as needed
        }
        std::cout << std::endl;
    }



    
    void control(const mjModel *m, mjData *d){
        system("clear");
        printLidar(m, d);



        mjtNum time = d->time;

        Eigen::Vector3d pos;
        pos << d->qpos[0], d->qpos[1], d->qpos[2];
        Eigen::Vector3d v;
        v << d->qvel[0], d->qvel[1], d->qvel[2];

        // Orientation and angular velocity
        angles::Quaternion q = {d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]};
        angles::EulerAngles euler = quaternionToEulerAngles(q);
        mjtNum roll = euler.roll;
        mjtNum pitch = euler.pitch;
        mjtNum yaw = euler.yaw;
        


        Eigen::Vector3d ang_vel;
        ang_vel << d->qvel[3], d->qvel[4], d->qvel[5];

        Eigen::Vector3d v_des;
        v_des = (pos_des(time) - pos);

        // // Calculate the desired orientation
        mjtNum roll_des = -std::clamp(v_des[1] - v[1], -0.0872665, 0.0872665);
        mjtNum pitch_des =  std::clamp(v_des[0] - v[0], -0.0872665 , 0.0872665);
        mjtNum yaw_des = 0.0;


        mjtNum yaw_err = angles::signed_angular_distance(yaw_des, yaw);
        mjtNum pitch_err = angles::signed_angular_distance(pitch_des, pitch);
        mjtNum roll_err = angles::signed_angular_distance(roll_des, roll);
        
        Eigen::Vector4d U;
        U << mass *g + kpz*(v_des[2] - v[2]), 
            roll_err*kpx - kd*ang_vel[0],
            pitch_err*kpy - kd*ang_vel[1],
            yaw_err* kpz - ang_vel[2];

        

    
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
