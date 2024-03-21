#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/Cartesian.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <istream>
#include <streambuf>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>

std::string file_name = "legKalmanState.csv";
std::ofstream outfile;
double firstTime = 1684320000;
//double firstTime = 1688570000;




/*  ************************************************************************************************/
#define NUM_LEG 4 
#define STATE_SIZE 18  
#define MEAS_SIZE 28  
#define PROCESS_NOISE_PIMU 0.01 
#define PROCESS_NOISE_VIMU 0.01     
#define PROCESS_NOISE_PFOOT 0.01   
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001    
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1     
#define SENSOR_NOISE_ZFOOT 0.001  


class Custom
{
    public:
     /*  ************************************************************************************************/
        Eigen::Matrix<double, 3, 3>  root_rot_mat;  
        Eigen::Matrix<double, 3, NUM_LEG>  foot_pos_rel; 
        Eigen::Vector3d  imu_acc;  
        Eigen::Vector3d  imu_ang_vel; 
        Eigen::Matrix<double, 1, NUM_LEG>  foot_force;  
        Eigen::Matrix<double, 3, NUM_LEG>  foot_vel_rel;  
        Eigen::Vector3d root_pos;
        Eigen::Vector3d root_vel;
         uint8_t movement_mode;

        Eigen::Vector3d estimated_root_pos;
        Eigen::Vector3d estimated_root_vel;
       


        /*  ************************************************************************************************/
        // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR 
        Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
        Eigen::Matrix<double, STATE_SIZE, 1> xbar; //estimation state after process update
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; //estimation state covariance after process update
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; //estimation state transition
        Eigen::Matrix<double, STATE_SIZE, 3> B; //
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; //estimation state transition noise

        /*  ************************************************************************************************/

        // observation
        // 0 1 2   FL pos residual  
        // 3 4 5   FR pos residual
        // 6 7 8   RL pos residual
        // 9 10 11 RR pos residual
        // 12 13 14 vel residual from FL    
        // 15 16 17 vel residual from FR
        // 18 19 20 vel residual from RL
        // 21 22 23 vel residual from RR
        // 24 25 26 27 foot height
        Eigen::Matrix<double, MEAS_SIZE, 1> y; //observation
        Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
        Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // y-yhat estimated observation
        Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; //S^-1*error_y
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
        Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; //
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; //estimation state observation noise

        /*  ************************************************************************************************/
        // helper matrices 
        Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
        Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; //? Innovation (or pre-fit residual) covariance
        Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // kalman gain

        bool assume_flat_ground = true;    

        Eigen::Matrix<double,3,3> last_rot;


        // variables to process foot force
        double estimated_contacts[4];



        /*  ************************************************************************************************/
        ros::NodeHandle nh;
        ros::Subscriber subHighState;
        ros::Publisher  pubLegKalmanFilter;

        ros::Time currStamp;
        double currTime;
        double lastTime;

    public:
        Custom()
        {
            // constructor
                    subHighState = nh.subscribe("high_state", 100000,&Custom::highStateHandler, this, ros::TransportHints().tcpNoDelay());
                    pubLegKalmanFilter = nh.advertise<nav_msgs::Odometry>("legKF", 100000);

                    currTime = 0;
                    lastTime = 0;
                     // constructor
                    eye3.setIdentity();
                    // C is fixed
                    C.setZero();    
                    for (int i=0; i<NUM_LEG; ++i) {
                        C.block<3,3>(i*3,0) = -eye3;  //-pos
                        C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
                        C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
                        C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot
                    }

                    // Q R are fixed
                    //
                    Q.setIdentity();
                    Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU*eye3;               // position transition
                    Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU*eye3;               // velocity transition
                    for (int i=0; i<NUM_LEG; ++i) {
                        Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  // foot position transition
                    }

                    //! 测量协防差矩阵R 固定
                    R.setIdentity();
                    for (int i=0; i<NUM_LEG; ++i) {
                        R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                        // fk estimation
                        R.block<3,3>(NUM_LEG*3+i*3,NUM_LEG*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;      // vel estimation
                        R(NUM_LEG*6+i,NUM_LEG*6+i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
                    }

                    // set A to identity
                    A.setIdentity();

                    // set B to zero
                    B.setZero();

                    assume_flat_ground = true; 


                    P.setIdentity();
                    P = P * 3;

                    //set initial value of x
                    x.setZero();

                    root_rot_mat.setIdentity();
                    // x.segment<3>(0) = Eigen::Vector3d(0, 0, 0); 
                    // for (int i = 0; i < NUM_LEG; ++i) {
                    //     Eigen::Vector3d fk_pos = foot_pos_rel.block<3, 1>(0, i);
                    //     x.segment<3>(6 + i * 3) =root_rot_mat * fk_pos + x.segment<3>(0);
                    // }
        }


void highStateHandler(const unitree_legged_msgs::HighState::ConstPtr &msg){
    getInfoFromMsg(msg);

    //std::cout << foot_force[0]<< std::endl;

    if (foot_force[0] == 0.0) return;

    //std::cout << 1 << std::endl;




    currTime = currStamp.toSec();
    double deltaT = lastTime == 0 ? 0.002 : currTime-lastTime;
    //deltaT = 0.0021;
    static bool is_firstone = true;
    if(is_firstone){
        last_rot = root_rot_mat;
        is_firstone = false;
    }

    root_rot_mat = last_rot.inverse() * root_rot_mat;

    //root_rot_mat = root_rot_mat * Exp(imu_ang_vel * deltaT);

    // imu_ang_vel = Log(last_rot.transpose() * root_rot_mat)/deltaT;
    // last_rot = root_rot_mat;

    //printf("angular  velocity x = %f   y=  %f   z = %f \n",imu_ang_vel[0], imu_ang_vel[1],imu_ang_vel[2]);


    update_estimation(deltaT);
    
    printf("position x = %f   y=  %f   z = %f \n",estimated_root_pos[0], estimated_root_pos[1],estimated_root_pos[2]);


    //Eigen::Vector3d rotation_vector = x.rot.eulerAngles(2,1,0);
    // Eigen::Quaterniond quat(x.rot);
    // quat.normalize();

    // outfile << currTime-firstTime <<","<< x.pos[0] <<","<<x.pos[1] <<","<<x.pos[2]  <<","
    //                     <<quat.x()<<","<< quat.y()<<","<<quat.z()  << ","<<quat.w() << std::endl;

     outfile << currTime-firstTime <<","<< estimated_root_pos[0] <<","<<estimated_root_pos[1] <<","<<estimated_root_pos[2]  <<","
                        <<root_rot_mat(0,0) <<"," <<root_rot_mat(0,1) <<","<<root_rot_mat(0,2) <<","
                        <<root_rot_mat(1,0) <<","<<root_rot_mat(1,1) <<","<<root_rot_mat(1,2) <<","
                        <<root_rot_mat(2,0) <<","<<root_rot_mat(2,1) <<","<<root_rot_mat(2,2) <<std::endl;

    lastTime = currTime;


}


void getInfoFromMsg(const unitree_legged_msgs::HighState::ConstPtr &msg){
        Eigen::Vector3d euler(msg->imu.rpy[2],msg->imu.rpy[1],msg->imu.rpy[0]);;  // 对应 z y x
        root_rot_mat = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());


        foot_pos_rel << msg->footPosition2Body[1].x, msg->footPosition2Body[0].x, msg->footPosition2Body[3].x, msg->footPosition2Body[2].x, 
                                            msg->footPosition2Body[1].y, msg->footPosition2Body[0].y, msg->footPosition2Body[3].y, msg->footPosition2Body[2].y, 
                                            msg->footPosition2Body[1].z, msg->footPosition2Body[0].z, msg->footPosition2Body[3].z, msg->footPosition2Body[2].z;

        foot_vel_rel << msg->footSpeed2Body[1].x, msg->footSpeed2Body[0].x, msg->footSpeed2Body[3].x, msg->footSpeed2Body[2].x, 
                                            msg->footSpeed2Body[1].y, msg->footSpeed2Body[0].y, msg->footSpeed2Body[3].y, msg->footSpeed2Body[2].y, 
                                            msg->footSpeed2Body[1].z, msg->footSpeed2Body[0].z, msg->footSpeed2Body[3].z, msg->footSpeed2Body[2].z;
        
        imu_acc << msg->imu.accelerometer[0], msg->imu.accelerometer[1], msg->imu.accelerometer[2];
        imu_ang_vel << msg->imu.gyroscope[0],  msg->imu.gyroscope[1],  msg->imu.gyroscope[2];

        foot_force << msg->footForce[1],  msg->footForce[0], msg->footForce[3], msg->footForce[2];

        root_pos << msg->position[0], msg->position[1], msg->position[2];
        root_vel << msg->velocity[0],msg->velocity[1],msg->velocity[2];

        currStamp = msg->stamp;


}
void update_estimation(double dt)
    {
    //update A B using latest dt
    A.block<3, 3>(0, 3) = dt * eye3;
    B.block<3, 3>(3, 0) = dt * eye3;

    //control input u is Ra + ag
    Eigen::Vector3d u = root_rot_mat * imu_acc + Eigen::Vector3d(0, 0, -9.81);


    for(int i=0; i< NUM_LEG; i++){
        if(foot_force(i) > 200) estimated_contacts[i] = 1.0;
        else estimated_contacts[i] = 0;
    }

    //update Q
    Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
    Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
    // update Q R for legs not in contact
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        Q.block<3, 3>(6 + i * 3, 6 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3;  // foot position transition
        // for estimated_contacts[i] == 1, Q = 0.002
        // for estimated_contacts[i] == 0, Q = 1001*Q
         
        R.block<3, 3>(i * 3, i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT * eye3;                       // fk estimation

        R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;      // vel estimation
        if (assume_flat_ground) 
        {
            R(NUM_LEG * 6 + i, NUM_LEG * 6 + i) = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;       // height z estimation
        }
    }


    //process update
    xbar = A*x + B*u ;
    Pbar = A * P * A.transpose() + Q;

    // measurement construction
    yhat = C*xbar;

    //! actual measurement
    for (int i=0; i<NUM_LEG; ++i) 
    {
        Eigen::Vector3d fk_pos = foot_pos_rel.block<3,1>(0,i);
        y.block<3,1>(i*3,0) = root_rot_mat*fk_pos;  
        Eigen::Vector3d leg_v = -foot_vel_rel.block<3,1>(0,i) - skew(imu_ang_vel)*fk_pos;    
        y.block<3,1>(NUM_LEG*3+i*3,0) = (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*root_rot_mat*leg_v;      // vel estimation

        y(NUM_LEG*6+i) = (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;                               // height z estimation
    }


    S = C * Pbar *C.transpose() + R;
    S = 0.5*(S+S.transpose());

    error_y = y - yhat; 
    Serror_y = S.fullPivHouseholderQr().solve(error_y);     

    x = xbar + Pbar * C.transpose() * Serror_y; 

    SC = S.fullPivHouseholderQr().solve(C);
    P = Pbar - Pbar * C.transpose() * SC * Pbar;
    P = 0.5 * (P + P.transpose());  

    //! tricks  reduce position drift
    if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
        P.block<2, 16>(0, 2).setZero();
        P.block<16, 2>(2, 0).setZero();
        P.block<2, 2>(0, 0) /= 10.0;
    }

//    std::cout << x.transpose() <<std::endl;
    estimated_root_pos = x.segment<3>(0); 
    estimated_root_vel = x.segment<3>(3);     

    }
  

    Eigen::Matrix3d skew(Eigen::Vector3d imu_ang_vel_)
    {
        Eigen::Matrix3d skewresult;
        skewresult << 0, -imu_ang_vel_[2], imu_ang_vel_[1],
                                    imu_ang_vel_[2], 0, -imu_ang_vel_[0],
                                     -imu_ang_vel_[1], imu_ang_vel_[0], 0;
        return skewresult;
    }


Eigen::Matrix3d Exp(const Eigen::Vector3d& r){
    Eigen::Matrix3d expr;
    double theta = r.norm();
    if(theta < 1e-12){
        expr = Eigen::Matrix3d::Identity();
    }
    else{
        Eigen::Matrix3d skew_ = skew(r / theta);
        expr = Eigen::Matrix3d::Identity() + sin(theta) * skew_ + (1 - cos(theta)) * skew_ * skew_;
    }
    return expr;
}


Eigen::Vector3d Log(const Eigen::Matrix3d& R){
    double theta = (R.trace() > 3 - 1e-6) ? 0 : acos((R.trace() - 1) / 2);
    Eigen::Vector3d r(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return fabs(theta) < 0.001 ? (0.5 * r) : (0.5 * theta / sin(theta) * r);

}   

Eigen::Matrix3d A_(const Eigen::Vector3d &u){
    Eigen::Matrix3d result;
    Eigen::Matrix3d skew_u = skew(u);
    double u_norm = u.norm();
    double a = 0.5 * u_norm * cos(0.5*u_norm) / sin(0.5 * u_norm);
    result = eye3  - 0.5 * skew_u + ((1-a)/(u_norm * u_norm)) *skew_u * skew_u ;

    return result;
}

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_kalmanState");
    Custom custom;
    outfile.open(file_name,std::ios::out | std::ios::trunc);
    ROS_INFO("\033[1;32m----> leg Kalman Filter Started.\033[0m");
    ros::spin();

    return 0;
}