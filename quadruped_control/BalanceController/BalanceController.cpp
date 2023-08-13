/*

References:
   [R1] M. Focchi, A. del Prete, I. Havoutis, R. Featherstone, D. G. Caldwell,
and C. Semini. High-slope terrain locomotion for torque-controlled quadruped
robots. Autonomous Robots, 2016.

   [R2] R. M. Murray, S. S. Sastry, and L. Zexiang. A Mathematical Introduction
to Robotic Manipulation. CRC Press, Inc., Boca Raton, FL, USA, 1st edition,
1994.

Cheetah-3-Documentation-Control:
   [C1] balanceController.pdf

   qpOASES variables are terminated with _qpOASES
*/

#include "BalanceController.hpp"
#include <iostream>
// #include "Sim_Utils.h"

using namespace std;

BalanceController::BalanceController()
    : QProblemObj_qpOASES(NUM_VARIABLES_QP, NUM_CONSTRAINTS_QP) {
  // *!! It seems like this next block of 5 lines does not actually do anything,
  // I had to set print level in the OptimizationThread class
  Options options;
  options.printLevel = PL_NONE;
  QProblemObj_qpOASES.setOptions(options);
  QProblemObj_qpOASES.setPrintLevel(PL_NONE);

  QPFinished = false;

  //lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
  //if (lcm->good()) {
  //  printf("LCM IN BALANCE CONTROL INITIALIZED\n");
  //} else {
  //  printf("LCM IN BALANCE CONTROLLER FAILED\n");
  //  exit(-1);
  //}

  // Eigen QP matrices
  H_eigen.resize(12, 12);
  A_eigen.resize(NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
  g_eigen.resize(12, 1);
  xOpt_eigen.resize(12, 1);
  yOpt_eigen.resize(NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP, 1);

  mass = 13.856;
  inertia = 0.01;

  /* Model and World parameters and force limits */
  Ig.resize(3, 3);
  Ig.setIdentity();
  Ig = inertia * Ig;
  gravity.resize(3, 1);

  direction_normal_flatGround.resize(3, 1);
  direction_tangential_flatGround.resize(3, 1);

  /* Initialize to all feet on the ground */
  contact_state.resize(2, 1);
  contact_state << 1, 1;

  minNormalForces_feet.resize(2, 1);
  maxNormalForces_feet.resize(2, 1);

  /* Actual Kinematics*/
  x_COM_world.resize(3, 1);
  xdot_COM_world.resize(3, 1);
  omega_b_world.resize(3, 1);
  quat_b_world.resize(4, 1);
  R_b_world.resize(3, 3);
  p_feet.resize(3, 2);

  R_yaw_act.resize(3, 3);
  R_yaw_act.setIdentity();

  /* Desired Kinematics */
  x_COM_world_desired.resize(3, 1);
  xdot_COM_world_desired.resize(3, 1);
  xddot_COM_world_desired.resize(3, 1);
  omega_b_world_desired.resize(3, 1);
  omegadot_b_world_desired.resize(3, 1);
  R_b_world_desired.resize(3, 3);
  orientation_error.resize(3, 1);

  error_x_rotated.setZero(3);
  error_dx_rotated.setZero(3);
  error_theta_rotated.setZero(3);
  error_dtheta_rotated.setZero(3);

  vbd_command_eigen.resize(3, 1);
  vbd_command_eigen.setZero();

  /* Temporary, Internal Matrices */
  omegaHat.resize(3, 3);
  tempSkewMatrix3.resize(3, 3);
  tempVector3.resize(3, 1);

  tempSkewMatrix3.setZero();
  tempVector3.setZero();
  // A_control.setZero();

  A_control.resize(6, 12);
  A_control.setZero();
  b_control.resize(6, 1);
  b_control_Opt.resize(6, 1);
  S_control.resize(6, 6);
  W_control.resize(12,12);
  C_control.resize(NUM_CONSTRAINTS_QP, 12);

  C_times_f_control.resize(NUM_CONSTRAINTS_QP, 1);

  C_control.setZero();
  xOptPrev.setZero(12);
  yOptPrev.setZero(NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP);

  for (int i = 0; i < NUM_VARIABLES_QP; i++) {
    xOpt_qpOASES[i] = 0.0;
    xOpt_initialGuess[i] = 0.0;
  }

  xOpt_initialGuess[2] = 100;
  xOpt_initialGuess[5] = 100;
  xOpt_initialGuess[8] = 0;
  xOpt_initialGuess[11] = 0;

  for (int i = 0; i < NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP; i++) {
    yOpt_qpOASES[i] = 0.0;
  }

  //    FullStateData.p 1.000000 -0.131507
  // FullStateData.p 2.000000 -0.000000
  // FullStateData.p 3.000000 0.571957

  set_QPWeights();
  set_RobotLimits();
  set_worldData();

  x_COM_world_desired << 0.0, 0.0, 0.59;
  xdot_COM_world_desired << 0, 0, 0;
  omega_b_world_desired << 0, 0, 0;
  R_b_world_desired.setIdentity();

  Kp_COMx = 50;
  Kp_COMy = 50;
  Kp_COMz = 150;

  Kd_COMx = 20;
  Kd_COMy = 20;
  Kd_COMz = 60;

  Kp_Base_roll = 150;
  Kp_Base_pitch = 100;
  Kp_Base_yaw = 50;

  Kd_Base_roll = 60;
  Kd_Base_pitch = 10;
  Kd_Base_yaw = 10;

  yaw_act = 0;

  cpu_time = 0.001;
  cpu_time_fixed = 0.001;
  qp_exit_flag = -1.0;

  qp_not_init = 1.0;
}

void BalanceController::testFunction() { printf("testfun "); }

/*
void BalanceController::set_base_support_flag(double sflag)
{
   double support_flag[] = {sflag};
   command.command_number = 4;
   command.data_size = sizeof(support_flag) / sizeof(double);
   command.data.resize(command.data_size);
   memcpy(command.data.data(), support_flag, sizeof(support_flag) );
   lcm.publish("cheetah:sim_command", &command);
}
*/

/* --------------- Primary Interface ------------------- */

void BalanceController::updateProblemData(double* xfb_in, double* p_feet_in,
                                          double* p_des, double* p_act,
                                          double* v_des, double* v_act,
                                          double* O_err, double yaw_act_in) {
  // Unpack inputs
  copy_Array_to_Eigen(quat_b_world, xfb_in, 4, 0);
  copy_Array_to_Eigen(x_COM_world, xfb_in, 3, 4);
  copy_Array_to_Eigen(omega_b_world, xfb_in, 3, 7);
  copy_Array_to_Eigen(xdot_COM_world, xfb_in, 3, 10);
  copy_Array_to_Eigen(p_feet, p_feet_in, 6, 0);
  // std::cout << "omega_b_world:" <<omega_b_world << std::endl;
  
  yaw_act = yaw_act_in;

  R_yaw_act.setZero();

  R_yaw_act(0, 0) = cos(yaw_act);
  R_yaw_act(0, 1) = -sin(yaw_act);
  R_yaw_act(1, 0) = sin(yaw_act);
  R_yaw_act(1, 1) = cos(yaw_act);
  R_yaw_act(2, 2) = 1;

  // std::cout << "pfeet = " << p_feet_world << "\n";

  quaternion_to_rotationMatrix(R_b_world, quat_b_world);
  

  //std::cout << "quat to R: " << std::endl;
  // std::cout << "R_b_world:" <<R_b_world << std::endl;
  //std::cout << R_b_world << std::endl;

  // Compute controller matrices. Must call these before calculating H_qp and
  // g_qp
  calc_PDcontrol();
  update_A_control();

  // Compute QP Problem data
  calc_H_qpOASES();
  calc_A_qpOASES();
  calc_g_qpOASES();

  update_log_variables(p_des, p_act, v_des, v_act, O_err);

  cpu_time = cpu_time_fixed;
  nWSR_qpOASES = nWSR_fixed;
}

void BalanceController::SetContactData(double* contact_state_in,
                                       double* min_forces_in,
                                       double* max_forces_in) {
  // Unpack inputs
  copy_Array_to_Eigen(contact_state, contact_state_in, 2, 0);
  copy_Array_to_Eigen(minNormalForces_feet, min_forces_in, 2, 0);
  copy_Array_to_Eigen(maxNormalForces_feet, max_forces_in, 2, 0);

  calc_lb_ub_qpOASES();
  calc_lbA_ubA_qpOASES();
}

void BalanceController::set_desired_swing_pos(double* pFeet_des_in) {
  for (int i = 0; i < 12; i++) {
    qp_controller_data.pfeet_des[i] = pFeet_des_in[i];
  }
}

void BalanceController::set_actual_swing_pos(double* pFeet_act_in) {
  for (int i = 0; i < 12; i++) {
    qp_controller_data.pfeet_act[i] = pFeet_act_in[i];
  }
}

void BalanceController::solveQP_nonThreaded(double* xOpt) {
  // &cpu_time
  if (qp_not_init == 1.0) {
    qp_exit_flag = QProblemObj_qpOASES.init(
        H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, lbA_qpOASES,
        ubA_qpOASES, nWSR_qpOASES, &cpu_time, xOpt_initialGuess);
    qp_not_init = 0.0;

    nWSR_initial = nWSR_qpOASES;
    cpu_time_initial = cpu_time;
  } else {
    qp_exit_flag = QProblemObj_qpOASES.init(
        H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, lbA_qpOASES,
        ubA_qpOASES, nWSR_qpOASES, &cpu_time, xOpt_qpOASES, yOpt_qpOASES,
        &guessedBounds, &guessedConstraints);
  }

  QProblemObj_qpOASES.getPrimalSolution(xOpt_qpOASES);
  QProblemObj_qpOASES.getDualSolution(yOpt_qpOASES);

  QProblemObj_qpOASES.getBounds(guessedBounds);
  QProblemObj_qpOASES.getConstraints(guessedConstraints);

  // std::cout << "cpu_time_initial = " << cpu_time_initial << "\n";
  // std::cout << "qp exit flag = " << qp_exit_flag << "\n";
  // std::cout << "nWSR_initial = " << nWSR_initial << "\n";
  // std::cout << "max NWSR = " << RET_MAX_NWSR_REACHED << "\n"; // 64
  // std::cout << "qp failed = " << RET_INIT_FAILED << "\n"; // 33

  // std::cout <<  "Kp_COMx = " << Kp_COMx << "\n";
  // std::cout <<  "Kp_COMy = " << Kp_COMy << "\n";
  // std::cout <<  "Kp_COMz = " << Kp_COMz << "\n";
  // std::cout <<  "Kd_COMx = " << Kd_COMx << "\n";
  // std::cout <<  "Kd_COMy = " << Kd_COMy << "\n";
  // std::cout <<  "Kd_COMz = " << Kd_COMz << "\n";

  // std::cout <<  "Kp_Base_roll = " << Kp_Base_roll << "\n";
  // std::cout <<  "Kp_Base_pitch = " << Kp_Base_pitch << "\n";
  // std::cout <<  "Kp_Base_yaw = " << Kp_Base_yaw << "\n";
  // std::cout <<  "Kd_Base_roll = " << Kd_Base_roll << "\n";
  // std::cout <<  "Kd_Base_pitch = " << Kd_Base_pitch << "\n";
  // std::cout <<  "Kd_Base_yaw = " << Kd_Base_yaw << "\n";

  // std::cout << "S_control = " << S_control << "\n";

  qp_controller_data.exit_flag = qp_exit_flag;
  qp_controller_data.nWSR = nWSR_qpOASES;
  qp_controller_data.cpu_time_microseconds = cpu_time * 1.0e6;

  copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, 12);
  //std::cout << "xopt_pri= " << xOpt_qpOASES << "\n";
  //std::cout << "xopt= " << xOpt_eigen << "\n";
  // copy_real_t_to_Eigen(yOptPrev, yOpt_qpOASES,
  // NUM_VARIABLES_QP+NUM_CONSTRAINTS_QP);

  b_control_Opt = A_control * xOpt_eigen;

  for (int i = 0; i < 6; i++) {
    qp_controller_data.b_control_Opt[i] = b_control_Opt(i);
  }

  // xOptPrev = xOpt_eigen;

  // for (int i = 0; i < 4; i++) {
  //   tempVector3 = -R_b_world.transpose() * xOpt_eigen.segment(3 * i, 3);
  //   xOpt[3 * i] = tempVector3(0);
  //   xOpt[3 * i + 1] = tempVector3(1);
  //   xOpt[3 * i + 2] = tempVector3(2);
  // }
  

  xOpt[0] = -xOpt_eigen(0);
  xOpt[1] = -xOpt_eigen(1);
  xOpt[2] = -xOpt_eigen(2);
  xOpt[3] = -xOpt_eigen(3);
  xOpt[4] = -xOpt_eigen(4);
  xOpt[5] = -xOpt_eigen(5);
  xOpt[6] = -xOpt_eigen(6);
  xOpt[7] = -xOpt_eigen(7);
  xOpt[8] = -xOpt_eigen(8);
  xOpt[9] = -xOpt_eigen(9);
  xOpt[10] = -xOpt_eigen(10);
  xOpt[11] = -xOpt_eigen(11);

  
  QProblemObj_qpOASES.reset();

  //calc_constraint_check();
  qp_controller_data_publish = qp_controller_data;
}

void BalanceController::verifyModel(double* vbd_command) {
  vbd_command_eigen =
      (1.0 / mass) * A_control.block<3, 12>(0, 0) * xOpt_eigen - gravity;
  copy_Eigen_to_double(vbd_command, vbd_command_eigen, 3);
}

/* --------------- Control Math ------------ */

void BalanceController::calc_PDcontrol() {
  // calculate error in yaw rotated coordinates
  error_x_rotated =  (x_COM_world_desired - x_COM_world);
  error_dx_rotated = (xdot_COM_world_desired - xdot_COM_world);
  matrixLogRot(R_b_world_desired *R_b_world.transpose(), orientation_error);
  //std:: cout << "x_COM_world_desired =" << x_COM_world_desired << "\n";
  //std:: cout << "x_COM_world =" << x_COM_world << "\n";
  //std:: cout << "xdot_COM_world_desired =" << xdot_COM_world_desired << "\n";
  //std:: cout << "xdot_COM_world =" << xdot_COM_world << "\n";
  //std:: cout << "orientation_error =" << orientation_error << "\n";
  error_dtheta_rotated =(omega_b_world_desired - omega_b_world);

  xddot_COM_world_desired(0) =
      Kp_COMx * error_x_rotated(0) + Kd_COMx * error_dx_rotated(0);
  xddot_COM_world_desired(1) =
      Kp_COMy * error_x_rotated(1) + Kd_COMy * error_dx_rotated(1);
  xddot_COM_world_desired(2) =
      Kp_COMz * error_x_rotated(2) + Kd_COMz * error_dx_rotated(2);

  omegadot_b_world_desired(0) = Kp_Base_roll * orientation_error(0) +
                                Kd_Base_roll * error_dtheta_rotated(0);
  omegadot_b_world_desired(1) = Kp_Base_pitch * orientation_error(1) +
                                Kd_Base_pitch * error_dtheta_rotated(1);
  omegadot_b_world_desired(2) = Kp_Base_yaw * orientation_error(2) +
                                Kd_Base_yaw * error_dtheta_rotated(2);

  // Compute orientation error using (4) of [R1] and Proposition 2.5 of [R2]

  // Ig << .0168, 0, 0, 0, 0.0565, 0, 0, 0, 0.064; // A1
  Ig << 0.0463619, 0, 0, 0, 0.056636, 0, 0, 0, 0.024569; // biped
  //std:: cout << "Ig=" << Ig << "\n";
  MatrixXd II =  Ig;

  // See RHS of Equation (5), [R1]
  b_control << mass * (xddot_COM_world_desired + gravity),
      II * omegadot_b_world_desired;
  //std::cout << "error_dz =" << error_dx_rotated(2) << "\n";
  //std:: cout << "error_z =" << error_x_rotated(2) << "\n";
  //std::cout << "b_control: " << b_control << std::endl;

  //std::cout << "orientation_error = " << orientation_error << "\n";
}


void BalanceController::calc_constraint_check() {
  C_times_f_control = C_control * xOpt_eigen;

  for (int i = 0; i < NUM_VARIABLES_QP; i++) {
    qp_controller_data.xOpt[i] = xOpt_eigen(i);
  }

  for (int i = 0; i < NUM_CONSTRAINTS_QP; i++) {
    qp_controller_data.lbA[i] = lbA_qpOASES[i];
    qp_controller_data.ubA[i] = ubA_qpOASES[i];
    qp_controller_data.C_times_f[i] = C_times_f_control(i);
  }

  for (int i = 0; i < 6; i++) {
    qp_controller_data.b_control[i] = b_control(i);
  }
}
/* ---------------- Utility for b_control ------------ */
void BalanceController::get_b_matrix(double* b){
  for (int i =0; i <6; i++){
    b[i] = b_control(i);
  }
}

/* --------------- QP Matrices and Problem Data ------------ */

void BalanceController::update_A_control() {
  // Update the A matrix in the controller notation A*f = b
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    A_control.block<3, 3>(0, 3 * i) << R_yaw_act.transpose();

    // std::cout << "R_yaw_act is " << std::endl;
    // for (int i = 0; i < 3; i++){
      // std::cout << R_yaw_act << "  ";
    // }
    // std::cout << "\n";

    tempVector3 <<  p_feet.col(i);
    crossMatrix(tempSkewMatrix3, tempVector3);
    A_control.block<3, 3>(3, 3 * i) <<  tempSkewMatrix3;
    A_control.block<1, 3>(4, 6+3 * i) << 0,1,0;
    A_control.block<1, 3>(5, 6+3 * i) << 0,0,1;
  }
  // std::cout << "A_control: " << A_control << std::endl;

}

void BalanceController::calc_H_qpOASES() {
  // Use the A matrix to compute the QP cost matrix H
  H_eigen = (A_control.transpose() * S_control * A_control +
                 (alpha_control) * W_control);

  // Copy to real_t array (qpOASES data type)
  copy_Eigen_to_real_t(H_qpOASES, H_eigen, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  //std::cout << "H_control: " << H_eigen << std::endl;
}

void BalanceController::calc_A_qpOASES() {
  Eigen::Vector3d t1x;
  t1x << 1, 0, 0;
  Eigen::Vector3d t2y;
  t2y << 0, 1, 0;

  
    C_control.block<1, 3>(0, 0)
        << -mu_friction * direction_normal_flatGround.transpose() +
               t1x.transpose();
    C_control.block<1, 3>(1, 0)
        << -mu_friction * direction_normal_flatGround.transpose() -
               t1x.transpose();
    C_control.block<1, 3>(2, 0)
        << -mu_friction * direction_normal_flatGround.transpose() +
               t2y.transpose();
    C_control.block<1, 3>(3, 0)
        << -mu_friction * direction_normal_flatGround.transpose() -
               t2y.transpose();
    //
    C_control.block<1, 3>(4, 3)
        << -mu_friction * direction_normal_flatGround.transpose() +
               t1x.transpose();
    C_control.block<1, 3>(5, 3)
        << -mu_friction * direction_normal_flatGround.transpose() -
               t1x.transpose();
    C_control.block<1, 3>(6, 3)
        << -mu_friction * direction_normal_flatGround.transpose() +
               t2y.transpose();
    C_control.block<1, 3>(7, 3)
        << -mu_friction * direction_normal_flatGround.transpose() -
               t2y.transpose();
    //force saturation:
    C_control.block<1, 3>(8, 0)
        << 0,0,1;
    C_control.block<1, 3>(9, 3)
        << 0,0,1;

    //line-foot constraint:
    double lt = 0.09-0.02; // toe length
    double lh = 0.06-0.02; // heel length
    C_control.block<1, 12>(10, 0)
        << 0,0,-lt, 0,0,0, 0,-1,0, 0,0,0;
    C_control.block<1, 12>(11, 0)
        << 0,0,0, 0,0,-lt, 0,0,0, 0,-1,0;
    C_control.block<1, 12>(12, 0)
        << 0,0,-lh, 0,0,0, 0,1,0, 0,0,0;
    C_control.block<1, 12>(13, 0)
        << 0,0,0, 0,0,-lh, 0,0,0, 0,1,0;
    C_control.block<1, 12>(14, 0)
        << 0,lt,-mu_friction*lt, 0,0,0, 0,-mu_friction,-1, 0,0,0;
    C_control.block<1, 12>(15, 0)
        << 0,0,0,0,lt,-mu_friction*lt, 0,0,0, 0,-mu_friction,-1;
    C_control.block<1, 12>(16, 0)
        << 0,-lt,-mu_friction*lt, 0,0,0, 0,-mu_friction,1, 0,0,0;
    C_control.block<1, 12>(17, 0)
        << 0,0,0,0,-lt,-mu_friction*lt, 0,0,0, 0,-mu_friction,1;
    C_control.block<1, 12>(18, 0)
        << 0,-lh,mu_friction*lh, 0,0,0, 0,-mu_friction,-1, 0,0,0;
    C_control.block<1, 12>(19, 0)
        << 0,0,0, 0,-lh,mu_friction*lh, 0,0,0,  0,-mu_friction,-1;
    C_control.block<1, 12>(20, 0)
        << 0,-lh,-mu_friction*lh, 0,0,0, 0,mu_friction,-1, 0,0,0;
    C_control.block<1, 12>(21, 0)
        << 0,0,0, 0,-lh,-mu_friction*lh, 0,0,0,  0,mu_friction,-1;


  copy_Eigen_to_real_t(A_qpOASES, C_control, NUM_CONSTRAINTS_QP,
                       NUM_VARIABLES_QP);

  //std::cout << "c_control: " << C_control << std::endl;
}

void BalanceController::calc_g_qpOASES() {
  g_eigen = -b_control.transpose() * S_control * A_control;
  //g_eigen += -2 * xOptPrev.transpose() * alpha_control;
  // Copy to real_t array (qpOASES data type)
  copy_Eigen_to_real_t(g_qpOASES, g_eigen, NUM_VARIABLES_QP, 1);

  //std::cout << "g_control: " << g_eigen << std::endl;
}

void BalanceController::calc_lb_ub_qpOASES() {
  for (int i = 0; i < 12; i++) {
    
    lb_qpOASES[i] =
          NEGATIVE_NUMBER;
    ub_qpOASES[i] =
          POSITIVE_NUMBER;
    
  }

}

void BalanceController::calc_lbA_ubA_qpOASES() {
  for (int i = 0; i < 8; i++) {
    lbA_qpOASES[i] = -1000;
    ubA_qpOASES[i] = 0;
  }
  // for (int i = 8; i < 10; i++) {
    lbA_qpOASES[8] = minNormalForces_feet[0];
    ubA_qpOASES[8] = maxNormalForces_feet[0];
    lbA_qpOASES[9] = minNormalForces_feet[1];
    ubA_qpOASES[9] = maxNormalForces_feet[1];
  // }
  for (int i = 10; i < 22; i++) {
    lbA_qpOASES[i] = -10000;
    ubA_qpOASES[i] = 0;
  }
}

void BalanceController::publish_data_lcm() {
 // lcm->publish("CONTROLLER_qp_controller_data", &qp_controller_data_publish);
    //qp_pub.publish(qp_controller_data_publish);
}

void BalanceController::update_log_variables(double* p_des, double* p_act,
                                             double* v_des, double* v_act,
                                             double* O_err) {
  copy_Eigen_to_double(p_des, x_COM_world_desired, 3);
  copy_Eigen_to_double(p_act, x_COM_world, 3);
  copy_Eigen_to_double(v_des, xdot_COM_world_desired, 3);
  copy_Eigen_to_double(v_act, xdot_COM_world, 3);
  copy_Eigen_to_double(O_err, orientation_error, 3);

  for (int i = 0; i < 3; i++) {
    qp_controller_data.p_des[i] = x_COM_world_desired(i);
    qp_controller_data.p_act[i] = x_COM_world(i);
    qp_controller_data.v_des[i] = xdot_COM_world_desired(i);
    qp_controller_data.v_act[i] = xdot_COM_world(i);
    qp_controller_data.O_err[i] = orientation_error(i);
    qp_controller_data.omegab_des[i] = omega_b_world_desired(i);
    qp_controller_data.omegab_act[i] = omega_b_world(i);
  }
}
/* ------------ Set Parameter Values -------------- */

void BalanceController::set_PDgains(double* Kp_COM_in, double* Kd_COM_in,
                                    double* Kp_Base_in, double* Kd_Base_in) {
  Kp_COMx = Kp_COM_in[0];
  Kp_COMy = Kp_COM_in[1];
  Kp_COMz = Kp_COM_in[2];

  Kd_COMx = Kd_COM_in[0];
  Kd_COMy = Kd_COM_in[1];
  Kd_COMz = Kd_COM_in[2];

  Kp_Base_roll = Kp_Base_in[0];
  Kp_Base_pitch = Kp_Base_in[1];
  Kp_Base_yaw = Kp_Base_in[2];

  Kd_Base_roll = Kd_Base_in[0];
  Kd_Base_pitch = Kd_Base_in[1];
  Kd_Base_yaw = Kd_Base_in[2];
}

void BalanceController::set_desiredTrajectoryData(
    double* rpy_des_in, double* p_des_in, double* omegab_des_in,
    double* v_des_in)  //, double* vdot_des_in)
{
  x_COM_world_desired << p_des_in[0], p_des_in[1], p_des_in[2];
  rpyToR(R_b_world_desired, rpy_des_in);
  omega_b_world_desired << omegab_des_in[0], omegab_des_in[1], omegab_des_in[2];
  xdot_COM_world_desired << v_des_in[0], v_des_in[1], v_des_in[2];
  //std::cout << "rpy_des: " << std::endl;
  //std::cout << rpy_des_in[0] << " " <<  rpy_des_in[1] << " " << rpy_des_in[2] << std::endl;
  //std::cout << "R_b_world_des: " << std::endl;
  //std::cout << "R_b_world_desired: " << R_b_world_desired << std::endl;
  //std::cout << R_b_world_desired << std::endl;
  // xddot_COM_world_desired << vdot_des_in[0], vdot_des_in[1], vdot_des_in[2];
}

void BalanceController::set_wrench_weights(double* COM_weights_in,
                                           double* Base_weights_in) {
  S_control(0, 0) = COM_weights_in[0];
  S_control(1, 1) = COM_weights_in[1];
  S_control(2, 2) = COM_weights_in[2];

  S_control(3, 3) = Base_weights_in[0];
  S_control(4, 4) = Base_weights_in[1];
  S_control(5, 5) = Base_weights_in[2];
}

void BalanceController::set_QP_options(double use_hard_constraint_pitch_in) {
  use_hard_constraint_pitch = use_hard_constraint_pitch_in;
}

void BalanceController::set_QPWeights() {
  S_control.setIdentity();
  W_control.setIdentity();
  alpha_control =0.001;
}

void BalanceController::set_worldData() {
  direction_normal_flatGround << 0, 0, 1;
  gravity << 0, 0, 9.81;
  direction_tangential_flatGround << 0.7071, 0.7071, 0;
  mu_friction = 0.6;
}

void BalanceController::set_friction(double mu_in) { mu_friction = mu_in; }

void BalanceController::set_alpha_control(double alpha_control_in) {
  alpha_control = alpha_control_in;
}

void BalanceController::set_mass(double mass_in) { mass = mass_in; }

void BalanceController::set_RobotLimits() {
  minNormalForces_feet << 10, 10;
  maxNormalForces_feet << 160, 160;
}

/* ------------ Utilities -------------- */

bool BalanceController::getQPFinished() { return QPFinished; }

void BalanceController::print_QPData() {
  std::cout << "\n\n";
 std::cout << "\n\nH = ";
 print_real_t(H_qpOASES, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  std::cout << "\n\nA = ";
  print_real_t(A_qpOASES, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
 std::cout << "\n\ng = ";
 print_real_t(g_qpOASES, NUM_VARIABLES_QP, 1);
 std::cout << "\n\nlb = ";
 print_real_t(lb_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nub = ";
  print_real_t(ub_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nlbA = ";
  print_real_t(lbA_qpOASES, NUM_CONSTRAINTS_QP, 1);
  std::cout << "\n\nubA = ";
  print_real_t(ubA_qpOASES, NUM_CONSTRAINTS_QP, 1);
}

void BalanceController::print_real_t(real_t* matrix, int nRows, int nCols) {
  int count = 0;
  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      std::cout << matrix[count] << "\t";
      count++;
    }
    std::cout << "\n";
  }
}

void BalanceController::copy_Eigen_to_real_t(real_t* target,
                                             Eigen::MatrixXd& source, int nRows,
                                             int nCols) {
  int count = 0;

  // Strange Behavior: Eigen matrix matrix(count) is stored by columns (not
  // rows)
  for (int i = 0; i < nRows; i++) {
    for (int j = 0; j < nCols; j++) {
      target[count] = source(i, j);
      count++;
    }
  }
}

void BalanceController::copy_Eigen_to_double(double* target,
                                             Eigen::VectorXd& source,
                                             int length) {
  for (int i = 0; i < length; i++) {
    target[i] = source(i);
  }
}

void BalanceController::copy_Array_to_Eigen(Eigen::VectorXd& target,
                                            double* source, int len,
                                            int startIndex) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i + startIndex];
  }
}

void BalanceController::copy_Array_to_Eigen(Eigen::MatrixXd& target,
                                            double* source, int len,
                                            int startIndex) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i + startIndex];
  }
}

void BalanceController::copy_real_t_to_Eigen(Eigen::VectorXd& target,
                                             real_t* source, int len) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i];
  }
}

void BalanceController::matrixLogRot(const Eigen::MatrixXd& R,
                                     Eigen::VectorXd& omega) {
  // theta = acos( (Trace(R) - 1)/2 )
  double theta;
  double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
  
  theta = acos(tmp);
  

  // Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
  // crossExtract(omegaHat,omega);
  //omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  omega << R(2, 1), R(0, 2) , R(1, 0) ;
  omega *= theta / (2 * sin(theta));
  
}

void BalanceController::crossMatrix(Eigen::MatrixXd& R,
                                    const Eigen::VectorXd& omega) {
  R(0, 1) = -omega(2);
  R(0, 2) = omega(1);
  R(1, 0) = omega(2);
  R(1, 2) = -omega(0);
  R(2, 0) = -omega(1);
  R(2, 1) = omega(0);
}

void BalanceController::matrixExpOmegaCross(const Eigen::VectorXd& omega,
                                            Eigen::MatrixXd& R) {
  double theta = omega.norm();
  R.setIdentity();

  if (theta > 1e-9) {
    omegaHat.setZero();
    crossMatrix(omegaHat, omega / theta);
    // R = I + omegaHat sin(theta) + omegaHat^2 (1-cos(theta))
    R += omegaHat * sin(theta) + omegaHat * omegaHat * (1 - cos(theta));
  }
}


void BalanceController::quaternion_to_rotationMatrix(Eigen::MatrixXd& R,
                                                     Eigen::VectorXd& quat) {
  // wikipedia
  R(0, 0) = 1 - 2 * quat(2) * quat(2) - 2 * quat(3) * quat(3);
  R(0, 1) = 2 * quat(1) * quat(2) - 2 * quat(0) * quat(3);
  R(0, 2) = 2 * quat(1) * quat(3) + 2 * quat(0) * quat(2);
  R(1, 0) = 2 * quat(1) * quat(2) + 2 * quat(0) * quat(3);
  R(1, 1) = 1 - 2 * quat(1) * quat(1) - 2 * quat(3) * quat(3);
  R(1, 2) = 2 * quat(2) * quat(3) - 2 * quat(1) * quat(0);
  R(2, 0) = 2 * quat(1) * quat(3) - 2 * quat(2) * quat(0);
  R(2, 1) = 2 * quat(2) * quat(3) + 2 * quat(1) * quat(0);
  R(2, 2) = 1 - 2 * quat(1) * quat(1) - 2 * quat(2) * quat(2);
}

void BalanceController::rpyToR(Eigen::MatrixXd& R, double* rpy_in) {
  Eigen::Matrix3d Rz, Ry, Rx;

  Rz.setIdentity();
  Ry.setIdentity();
  Rx.setIdentity();

  Rz(0, 0) = cos(rpy_in[2]);
  Rz(0, 1) = -sin(rpy_in[2]);
  Rz(1, 0) = sin(rpy_in[2]);
  Rz(1, 1) = cos(rpy_in[2]);

  Ry(0, 0) = cos(rpy_in[1]);
  Ry(0, 2) = sin(rpy_in[1]);
  Ry(2, 0) = -sin(rpy_in[1]);
  Ry(2, 2) = cos(rpy_in[1]);

  Rx(1, 1) = cos(rpy_in[0]);
  Rx(1, 2) = -sin(rpy_in[0]);
  Rx(2, 1) = sin(rpy_in[0]);
  Rx(2, 2) = cos(rpy_in[0]);

  R = Rz * Ry * Rx;
}