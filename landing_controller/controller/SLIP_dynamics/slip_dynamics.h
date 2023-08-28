#ifndef SLIP_DYNAMICS_H
#define SLIP_DYNAMICS_H

#include <ctime>
#include <chrono>
#include <sys/time.h>
typedef std::chrono::system_clock Clock;

#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <vector>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <eigenpy/eigenpy.hpp>

//#define PREALLOC_MEM_SIZE 1000

class  SLIP_dynamics
{
public:
    SLIP_dynamics(double dt,
                  double L,
                  double max_spring_compression,
                  double m,
                  double g_mag,
                  double w_v,
                  double w_p,
                  double w_u,
                  double max_settling_time);
    ~SLIP_dynamics();

    double dt;
    int window_size;
    double L;
    double max_spring_compression;
    double g_mag;
    double m;
    double w_v;
    double w_p;
    double w_u;

    int ctrl_horz;

    // these data could be passed in class initialization
    double max_settling_time;
    double limit_z_vel;


    double K = 0.;
    double D = 0.;
    double eig_z = 0.;


    // data for z dynamics
    double K_limit;
    double settling_time = 0.;



    // matrices for the cost
    Eigen::Matrix<double, 2, 2> Q_xy;
    Eigen::Matrix<double, 1, 2> Q_xyu;
    Eigen::Matrix<double, 1, 1> Q_u;



    // reference trajectories
    Eigen::Matrix<double, 1, Eigen::Dynamic> ctrl_time;

    Eigen::Matrix<double, 3, Eigen::Dynamic> T_p_com_ref;
    Eigen::Matrix<double, 3, Eigen::Dynamic> T_v_com_ref;
    Eigen::Matrix<double, 3, Eigen::Dynamic> T_a_com_ref;
    Eigen::Matrix<double, 1, Eigen::Dynamic> omega_sq_ref;

    Eigen::Matrix<double, 2, 1> scaling_factor0;
    Eigen::Matrix<double, 2, 1> scaling_factor1;




    Eigen::Matrix<double, 1, Eigen::Dynamic> G;
    Eigen::Matrix<double, 1, Eigen::Dynamic> L_vec;

    // initial states
    Eigen::Matrix<double, 2, 1> T_state_x_init;
    Eigen::Matrix<double, 2, 1> T_state_y_init;
    Eigen::Matrix<double, 2, 1> T_state_z_init;


    // methods
    void run(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init);
    void runVerbose(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init);
    void def_and_solveOCP(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init);
    void def_and_solveOCPVerbose(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init);
    void xy_dynamics();
    void z_dynamics();
    void cost_matrices();
    void compute_zmp();
    void propagation_matrices();
    void set_init(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init);



    Eigen::Matrix<double, 2, 2> A_z;

    // explicit form dynamics of x y
    Eigen::Matrix<double, Eigen::Dynamic, 2> P_xy_stack;
    Eigen::Matrix<double, Eigen::Dynamic, 1>  P_u_stack;


    Eigen::Matrix<double, 2, 2> state_xy;
    Eigen::Matrix<double, 2, 2> state_xy_init;

    Eigen::Matrix<double, 1, 2> zmp_xy;


    Eigen::Matrix<double, 2, 2> P_xy;
    Eigen::Matrix<double, 2, 1> P_u;




private:
    // matrices for com x y z dynamics

    // implicite form dynamcis of x y
    Eigen::Matrix<double, 2, 2> A_xy;
    Eigen::Matrix<double, 2, 1> B_xy;

    //discretized version
    Eigen::Matrix<double, 2, 2> A_xy_d;
    Eigen::Matrix<double, 2, 1> B_xy_d;


    // z dynamics
    double t = 0.;
    double exp_eig_t = 0.;
    Eigen::Array<double, 1, Eigen::Dynamic> ctrl_time_arr;
    Eigen::Array<double, 1, Eigen::Dynamic> exp_eig_t_arr;
    Eigen::Array<double, 1, Eigen::Dynamic> eig_t_arr;

    Eigen::Matrix<double, 1, Eigen::Dynamic> p;
    Eigen::Matrix<double, 1, Eigen::Dynamic> v;



    // implicite form dynamics of x y
    Eigen::Matrix<double, 2, 2> A_zt;
    Eigen::Matrix<double, 2, 2> expAt;

    // selection matrices
    Eigen::Matrix<double, 1, 2> Cv;
    Eigen::Matrix<double, 1, 2> Cp;

    Eigen::Matrix<double, 1, 2> CvPxy;
    Eigen::Matrix<double, 1, 2> CpPxy;
    Eigen::Matrix<double, 1, 1> CvPu;
    Eigen::Matrix<double, 1, 1> CpPu;

    Eigen::Matrix<double, 1, 2> Q_hash;

    // for computations
    Eigen::Matrix<double, 2, 1> state_x;
    Eigen::Matrix<double, 2, 1> state_y;
    Eigen::Matrix<double, 2, 1> state_z;


    Eigen::Matrix<double, 2, 2> P_xy_last;
    Eigen::Matrix<double, 2, 1> P_u_last;
    Eigen::Matrix<double, 1, 1> I1;



    time_t tstart, tend;


};

#endif // SLIP_DYNAMICS_H
