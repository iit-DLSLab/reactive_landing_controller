#include "slip_dynamics.h"
#include <typeinfo>
using namespace boost::python;
SLIP_dynamics::SLIP_dynamics(double dt,
                             double L,
                             double max_spring_compression,
                             double m,
                             double g_mag,
                             double w_v,
                             double w_p,
                             double w_u,
                             double max_settling_time)
{

    assert(dt>0 && "Sampling time dt must be a positive number");
    assert(L>0 && "Desired final height L must be a positive number");
    assert(max_spring_compression>0 && "max_spring_compression must be a positive number");
    assert(m>0 && "Robot mass m must be a positive number");
    assert(g_mag>0 && "Gravity vector magintude g_mag must be a positive number");
    assert(w_v>0 && w_p>0 && "State weights w_v and w_p must be positive numbers");
    assert(w_u>=0 && "Input weight w_u must be a positive number");
    assert(max_settling_time>0 && "max_settling_time must be a positive number");

    this->dt = dt;
    this->L = L;
    this->max_spring_compression = max_spring_compression;
    this->m = m;
    this->g_mag = g_mag;
    this->w_v = w_v;
    this->w_p = w_p;
    this->w_u = w_u;

    this->max_settling_time = max_settling_time;



    K_limit = m*pow(8/max_settling_time, 2);
    K = K_limit;
    D = 2 * sqrt(this->m * K);
    eig_z = -sqrt(K/this->m);

    this->limit_z_vel = exp(1)*(this->max_spring_compression)*eig_z;

    A_z(0,0) = - D/this->m;
    A_z(0,1) = - K/this->m;
    settling_time = -8/eig_z;
    ctrl_horz = int(settling_time/dt)+1;

    int prealloc_mem_size = ctrl_horz;

    ctrl_time_arr.resize(1, prealloc_mem_size);
    for(int k=0; k<prealloc_mem_size; k++) { ctrl_time_arr[k] = k*this->dt; }

    eig_t_arr.resize(1, prealloc_mem_size);
    eig_t_arr = eig_z*ctrl_time_arr;

    exp_eig_t_arr.resize(1, prealloc_mem_size);
    exp_eig_t_arr = exp(eig_t_arr);

    p.resize(1, prealloc_mem_size);
    v.resize(1, prealloc_mem_size);

    p.setZero();
    v.setZero();

    // resize and initialize references trajectories to zero
    ctrl_time.resize(1, prealloc_mem_size);
    for(int k=0; k<prealloc_mem_size; k++) { ctrl_time[k] = k*this->dt; }

    T_p_com_ref.resize(3, prealloc_mem_size);
    T_v_com_ref.resize(3, prealloc_mem_size);
    T_a_com_ref.resize(3, prealloc_mem_size);
    omega_sq_ref.resize(1, prealloc_mem_size);


    T_p_com_ref.setZero();
    T_v_com_ref.setZero();
    T_a_com_ref.setZero();

    omega_sq_ref.setZero();

    // initial states
    T_state_x_init.setZero();
    T_state_y_init.setZero();
    T_state_z_init.setZero();


    // initialize matrices
    A_xy.setZero();
    A_xy(1,0) = 1.;

    B_xy.setZero();

    A_xy_d.setIdentity();
    A_xy_d(1,0) = this->dt;
    B_xy_d.setZero();


    P_xy_stack.resize(2*(prealloc_mem_size), 2);
    P_xy_stack.setZero();
    P_xy_stack.block<2,2>(0,0).setIdentity();

    P_u_stack.resize(2*(prealloc_mem_size), 1);
    P_u_stack.setZero();


    A_z.setZero();
    A_z(1,0) = 1.;

    A_zt.setZero();
    expAt.setZero();

    Q_xy.setZero();
    Q_xyu.setZero();
    Q_u.setZero();

    Cv << 1.0, 0.0;
    Cp << 0.0, 1.0;

    CvPxy.setZero();
    CpPxy.setZero();
    CvPu.setZero();
    CpPu.setZero();

    state_x.setZero();
    state_y.setZero();
    state_z.setZero();

    I1 << 1.0;

    state_xy.setZero();
    state_xy_init.setZero();
    zmp_xy.setZero();

    // std::cout<<"LC initializad"<<std::endl;
}

// compute explicit form of the dynamics
void SLIP_dynamics::z_dynamics()
{
    // T_state_z_init is the initial state of com z:
    // T_state_z_init = [velocity_init    position_init].T

    // The dynamics is computed using the explicit form! (not recursive)


    for(int k = 0; k < ctrl_horz; k++)
    {

            t = ctrl_time[k];
            exp_eig_t = exp_eig_t_arr[k];

            T_p_com_ref(2, k) = exp_eig_t * t * T_state_z_init[0] + L;
            T_v_com_ref(2, k) = exp_eig_t * (eig_z * t + 1) * T_state_z_init[0];

            T_a_com_ref(2, k) = -D/m * T_v_com_ref(2, k) - K/m * (T_p_com_ref(2, k)-L);
            omega_sq_ref(0,k) = ( g_mag + T_a_com_ref(2, k) ) / T_p_com_ref(2, k);

    }
}

void SLIP_dynamics::propagation_matrices()
{

    double omega_sq_dt;

    for(int k = 0; k < ctrl_horz-1; k++)
    {

        // the loop starts from 1 because P_xy(0) = I and P_u(0) = 0
        //A_xy(0,1) =-omega_sq_ref(0,k);
        //A_xy_d = dt * A_xy;
        //A_xy_d(0,0) += 1;
        //A_xy_d(1,1) += 1;
        omega_sq_dt = dt *omega_sq_ref(0,k);
        A_xy_d(0,1) = omega_sq_dt;

        //B_xy(0,0) = omega_sq_ref(0,k);
        B_xy_d(0,0) = -omega_sq_dt;


        P_xy_stack.block<2,2>(2*(k+1), 0).noalias() = A_xy_d * P_xy_stack.block<2,2>(2*k, 0);
        P_u_stack.block<2,1>(2*(k+1), 0).noalias() = A_xy_d * P_u_stack.block<2,1>(2*k, 0);
        P_u_stack.block<2,1>(2*(k+1), 0) += B_xy_d;
    }

}

void SLIP_dynamics::xy_dynamics()
{
    // state = [velocity_x  velocity_y
    //          position_x  position_y]


    for(int k = 0; k < ctrl_horz; k++)
    {
        state_xy.noalias()  = P_xy_stack.block<2,2>(2*k, 0) * state_xy_init;
        state_xy.noalias() += P_u_stack.block<2,1>(2*k, 0) * zmp_xy;
        
        T_p_com_ref(0, k) = state_xy(1,0);
        T_v_com_ref(0, k) = state_xy(0,0);
        T_a_com_ref(0, k) = omega_sq_ref(0, k) * (T_p_com_ref(0, k) - zmp_xy(0));

        T_p_com_ref(1, k) = state_xy(1,1);
        T_v_com_ref(1, k) = state_xy(0,1);
        T_a_com_ref(1, k) = omega_sq_ref(0, k) * (T_p_com_ref(1, k) - zmp_xy(1));

    }

}

void SLIP_dynamics::cost_matrices()
{
    // The cost is written in the form
    // J_x(x0, u_x) = x0^T * Q_x * x0 + 2 * u^T * Q_xu * x0 + u^T * Q_u * u
    P_xy_last = P_xy_stack.block<2,2>(2*(ctrl_horz-1), 0);
    P_u_last = P_u_stack.block<2,1>(2*(ctrl_horz-1), 0);

    CvPxy = Cv * P_xy_last;
    CpPxy = Cp * P_xy_last;
    CvPu  = Cv * P_u_last;
    CpPu  = Cp * P_u_last;


    Q_xy  = w_v * CvPxy.transpose() * CvPxy + w_p * CpPxy.transpose() * CpPxy;
    Q_xyu = w_v *  CvPu.transpose() * CvPxy + w_p * (CpPu - I1).transpose() * CpPxy;
    Q_u   = w_v *  CvPu.transpose() * CvPu  + w_p * (CpPu - I1).transpose() * (CpPu - I1) + w_u * I1;

}



void SLIP_dynamics::set_init(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init)
{
    T_state_x_init << T_vel_init(0,0), T_pos_init(0,0);
    T_state_y_init << T_vel_init(1,0), T_pos_init(1,0);
    T_state_z_init << T_vel_init(2,0), T_pos_init(2,0)-L;
   
    state_xy_init(0,0) = T_state_x_init(0);
    state_xy_init(1,0) = T_state_x_init(1);
    state_xy_init(0,1) = T_state_y_init(0);
    state_xy_init(1,1) = T_state_y_init(1);
    
    // to avoid that com reference goes below max_spring_compression K is increased with the square of the velocity at touch down
    if (T_state_z_init[0]<limit_z_vel)
    {
       K = m * pow(T_state_z_init[0]/(exp(1) * max_spring_compression), 2);
       D = 2 * sqrt(m * K);
       eig_z = -sqrt(K/m);

       A_z(0,0) = - D/m;
       A_z(0,1) = - K/m;
       settling_time = -8/eig_z;
       ctrl_horz = int(settling_time/dt)+1;

       eig_t_arr = eig_z*ctrl_time_arr;
       exp_eig_t_arr = exp(eig_t_arr);
    }
    /*
        K = pow(T_vel_init(0,0), 2)+pow(T_vel_init(1,0), 2)+pow(T_vel_init(2,0), 2);
        K = m*K/pow(0.15, 2);
        D = 2 * sqrt(m * K);
        eig_z = -sqrt(K/m);

        A_z(0,0) = - D/m;
        A_z(0,1) = - K/m;
        settling_time = -8/eig_z;
        ctrl_horz = int(settling_time/dt)+1;
        
        eig_t_arr = eig_z*ctrl_time_arr;
        exp_eig_t_arr = exp(eig_t_arr);
    */

}


void SLIP_dynamics::compute_zmp()
{
     Q_hash = -Q_u.cwiseInverse() * Q_xyu;
     zmp_xy = Q_hash * state_xy_init;
}

void SLIP_dynamics::def_and_solveOCP(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init)
{
    set_init(T_pos_init, T_vel_init);
    z_dynamics();
    propagation_matrices();
    cost_matrices();
    compute_zmp();
}

void SLIP_dynamics::def_and_solveOCPVerbose(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init)
{   
    auto t0 = std::chrono::high_resolution_clock::now();
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms;

    for(int i=0; i<20;i++) {std::cout << "-";}
    std::cout << std::endl;

    t0 = std::chrono::high_resolution_clock::now();
    t1 = std::chrono::high_resolution_clock::now();
    set_init(T_pos_init, T_vel_init);
    t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;

    std::cout << "K: " << K << ", v0: " << T_vel_init(2,0) << ", ctrl_horz: " << ctrl_horz << ", settling time: " << settling_time << std::endl;
    std::cout << "set_init() took " << fp_ms.count() << " ms "<< std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    z_dynamics();
    t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;
    std::cout << "z_dynamics() took " << fp_ms.count() << " ms "<< std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    propagation_matrices();
    t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;
    std::cout << "propagation_matrices() took " << fp_ms.count() << " ms "<< std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    cost_matrices();
    t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;
    std::cout << "cost_matrices() took " << fp_ms.count() << " ms "<< std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    compute_zmp();
    t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;
    std::cout << "compute_zmp() took " << fp_ms.count() << " ms "<< std::endl;
}


void SLIP_dynamics::run(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init)
{
      set_init(T_pos_init, T_vel_init);
      z_dynamics();
      propagation_matrices();
      cost_matrices();
      compute_zmp();
      xy_dynamics();
}


void SLIP_dynamics::runVerbose(Eigen::Matrix<double, 3, 1> T_pos_init, Eigen::Matrix<double, 3, 1> T_vel_init)
{

      auto t0 = std::chrono::high_resolution_clock::now();
      auto t1 = std::chrono::high_resolution_clock::now();
      auto t2 = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> fp_ms;


      // SET INIT
      t0 = std::chrono::high_resolution_clock::now();
      t1 = std::chrono::high_resolution_clock::now();
      set_init(T_pos_init, T_vel_init);
      t2 = std::chrono::high_resolution_clock::now();
      fp_ms = t2 - t1;
      for(int i=0; i<20;i++) {std::cout << "-";}
      std::cout << std::endl;
      std::cout << "K: " << K << ", v0: " << T_vel_init(2,0) << ", ctrl_horz: " << ctrl_horz << ", settling time: " << settling_time << std::endl;
      std::cout << "set_init() took " << fp_ms.count() << " ms "<< std::endl;

      // Z DYNAMICS

      t1 = std::chrono::high_resolution_clock::now();
      z_dynamics();
      t2 = std::chrono::high_resolution_clock::now();
      fp_ms = t2 - t1;
      std::cout << "z_dynamics() took " << fp_ms.count() << " ms "<< std::endl;


      // PROPAGATION MATRICES
      t1 = std::chrono::high_resolution_clock::now();
      propagation_matrices();
      t2 = std::chrono::high_resolution_clock::now();
      fp_ms = t2 - t1;
      std::cout << "propagation_matrices() took " << fp_ms.count() << " ms "<< std::endl;

      // COST MATRICES
      t1 = std::chrono::high_resolution_clock::now();
      cost_matrices();
      t2 = std::chrono::high_resolution_clock::now();
      fp_ms = t2 - t1;
      std::cout << "cost_matrices() took " << fp_ms.count() << " ms "<< std::endl;

      // COMPUTE ZMP
      t1 = std::chrono::high_resolution_clock::now();
      compute_zmp();
      t2 = std::chrono::high_resolution_clock::now();
      fp_ms = t2 - t1;
      std::cout << "compute_zmp() took " << fp_ms.count() << " ms "<< std::endl;


      // XY DYANAMICS
      t1 = std::chrono::high_resolution_clock::now();
      xy_dynamics();
      t2 = std::chrono::high_resolution_clock::now();
      fp_ms = t2 - t1;
      std::cout << "xy_dynamics() took " << fp_ms.count() << " ms "<< std::endl;


      t1 = std::chrono::high_resolution_clock::now();
      fp_ms = t1 - t0;
      std::cout << "slip dynamics run took " << fp_ms.count() << " ms "<< std::endl;
}


SLIP_dynamics::~SLIP_dynamics()
{
}
;

BOOST_PYTHON_MODULE(SLIP_dynamics_lib)
{
    // convert Eigen::Matrix to np.ndarray
    eigenpy::enableEigenPy();
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatrixXd;
    typedef Eigen::Matrix<double,2,2> Matrix22d;
    typedef Eigen::Matrix<double,2,1> Matrix21d;
    eigenpy::enableEigenPySpecific<MatrixXd>();
    eigenpy::enableEigenPySpecific<Matrix22d>();
    eigenpy::enableEigenPySpecific<Matrix21d>();


    class_< std::vector<Matrix22d> >("vec_of_matrices22")
        .def(vector_indexing_suite< std::vector<Matrix22d> >())
    ;

    class_< std::vector<Matrix21d> >("vec_of_matrices21")
        .def(vector_indexing_suite< std::vector<Matrix21d> >())
    ;


    // wrapper function
    class_< SLIP_dynamics >("SLIP_dynamics", init<double, double, double, double, double, double, double, double, double>())
            // attributes
            .def_readwrite("dt", &SLIP_dynamics::dt)
            .def_readonly("ctrl_horz", &SLIP_dynamics::ctrl_horz)
            .def_readwrite("L", &SLIP_dynamics::L)
            .def_readwrite("max_spring_compression", &SLIP_dynamics::max_spring_compression)
            .def_readonly("m", &SLIP_dynamics::m)
            .def_readonly("K", &SLIP_dynamics::K)
            .def_readonly("D", &SLIP_dynamics::D)
            .def_readonly("eig_z", &SLIP_dynamics::eig_z)
            .def_readonly("g_mag", &SLIP_dynamics::g_mag)
            .def_readonly("w_v", &SLIP_dynamics::w_v)
            .def_readonly("w_p", &SLIP_dynamics::w_p)
            .def_readonly("w_u", &SLIP_dynamics::w_u)
            .def_readonly("Q_xy", &SLIP_dynamics::Q_xy)
            .def_readonly("Q_xyu", &SLIP_dynamics::Q_xyu)
            .def_readonly("Q_u", &SLIP_dynamics::Q_u)
            .def_readonly("A_z", &SLIP_dynamics::A_z)
            .def_readonly("settling_time", &SLIP_dynamics::settling_time)

            .def_readonly("P_xy_stack", &SLIP_dynamics::P_xy_stack)
            .def_readonly("P_u_stack", &SLIP_dynamics::P_u_stack)

            .def_readonly("ctrl_time", &SLIP_dynamics::ctrl_time)
            .def_readonly("T_p_com_ref", &SLIP_dynamics::T_p_com_ref)
            .def_readonly("T_v_com_ref", &SLIP_dynamics::T_v_com_ref)
            .def_readonly("T_a_com_ref", &SLIP_dynamics::T_a_com_ref)
            .def_readonly("omega_sq_ref", &SLIP_dynamics::omega_sq_ref)

            .def_readonly("T_state_x_init", &SLIP_dynamics::T_state_x_init)
            .def_readonly("T_state_y_init", &SLIP_dynamics::T_state_y_init)
            .def_readonly("T_state_z_init", &SLIP_dynamics::T_state_z_init)
            .def_readonly("zmp_xy", &SLIP_dynamics::zmp_xy)




            // methods

            .def("run", &SLIP_dynamics::run, args("T_pos_init", "T_vel_init"),
                 "Compute optimal zmp and reference trajectory. It is equivalent to call in sequence: "
                 "set_init(), z_dynamics(), propagation_matrices(), compute_zmp(), xy_dynamics()."
                 "It does side effect on class attributes.")
            .def("runVerbose", &SLIP_dynamics::runVerbose, args("T_pos_init", "T_vel_init"),
                 "Compute optimal zmp and reference trajectory. It is equivalent to call in sequence: "
                 "set_init(), z_dynamics(), propagation_matrices(), compute_zmp(), xy_dynamics()."
                 "It does side effect on class attributes.")
            .def("def_and_solveOCP", &SLIP_dynamics::def_and_solveOCP, args("T_pos_init", "T_vel_init"),
                 "Compute optimal zmp and ONLY z reference trajectory. It is equivalent to call in sequence: "
                 "set_init(), z_dynamics(), propagation_matrices(), compute_zmp(). After touch down one should call xy_dynamics()."
                 "It does side effect on class attributes.")
            .def("def_and_solveOCPVerbose", &SLIP_dynamics::def_and_solveOCPVerbose, args("T_pos_init", "T_vel_init"),
                 "Compute optimal zmp and ONLY z reference trajectory. It is equivalent to call in sequence: "
                 "set_init(), z_dynamics(), propagation_matrices(), compute_zmp(). After touch down one should call xy_dynamics()."
                 "It does side effect on class attributes.") 
            .def("xy_dynamics", &SLIP_dynamics::xy_dynamics)
            .def("propagation_matrices", &SLIP_dynamics::propagation_matrices)
            .def("cost_matrices", &SLIP_dynamics::cost_matrices)
            .def("set_init", &SLIP_dynamics::set_init)
            .def("compute_zmp", &SLIP_dynamics::compute_zmp)

    ;

}


