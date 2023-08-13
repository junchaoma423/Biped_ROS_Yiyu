#include "../../include/common/PositionVelocityEstimator.h"
#include "../../include/FSM/FSMState.h"

void LinearKFPositionVelocityEstimator::setup() {
  double dt = 0.001;
  _xhat.setZero();
  //_xhat[2] = 0.06;
  _ps.setZero();
  _vs.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  _A.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
  _B.setZero();
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
  C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
  C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 6) = C1;
  _C.block(3, 0, 3, 6) = C1;
  // _C.block(6, 0, 3, 6) = C1;
  // _C.block(9, 0, 3, 6) = C1;
  _C.block(0, 6, 6, 6) = -1 * Eigen::Matrix<double, 6, 6>::Identity();
  _C.block(6, 0, 3, 6) = C2;
  _C.block(9, 0, 3, 6) = C2;
  // _C.block(18, 0, 3, 6) = C2;
  // _C.block(21, 0, 3, 6) = C2;
  // _C(27, 17) = 1;
  // _C(26, 14) = 1;
  _C(13, 11) = 1;
  _C(12, 8) = 1;
  _P.setIdentity();
  _P = 100 * _P;
  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) = (dt*1.f ) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(3, 3, 3, 3) =
      (dt * 9.8f * 1.f ) * Eigen::Matrix<double, 3, 3>::Identity();
  _Q0.block(6, 6, 6, 6) =  dt * Eigen::Matrix<double, 6, 6>::Identity();
  _R0.setIdentity();
  std::cout << "PosVel setup" << std::endl;
}

LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator() {}
 
// run estimator
void LinearKFPositionVelocityEstimator::run() {
  //std::cout << "run LinearKFPosVlEstimate" << std::endl;

  std::cout << "T265_pose data: " << std::endl;
  for (int i = 0; i < 6; i++) {
    std::cout << FSMState::T265_pose[i] << "     ";
  }
  std::cout << std::endl;

  double process_noise_pimu = 0.02;
  double process_noise_vimu = 0.01;
  double process_noise_pfoot = 0.01;
  double sensor_noise_pimu_rel_foot = 0.01;
  double sensor_noise_vimu_rel_foot = 0.01;
  double sensor_noise_zfoot = 0.01;

  Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = //50000* 
  _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q(5,5) = _Q0(5,5) * process_noise_vimu;
  Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 6, 6) * process_noise_pfoot;
  // std::cout << "Q0 is " << _Q0 << std::endl;

  Eigen::Matrix<double, 14, 14> R = Eigen::Matrix<double, 14, 14>::Identity();
  R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
  R.block(6, 6, 6, 6) =
     // 5000000*
      _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;

  // std::cout << "Q is " << Q << std::endl;
  // std::cout << "R is " << R << std::endl; 

  // R(12,12) = 2.5 * R(12,12);
  // R(15,15) = 2.5 * R(15,15);
  // R(18,18) = 2.5 * R(18,18);
  // R(21,21) = 2.5 * R(21,21);

  // R(14,14) =  sensor_noise_vimu_rel_foot;
  // R(17,17) =  sensor_noise_vimu_rel_foot;
  // R(20,20) =  sensor_noise_vimu_rel_foot;
  // R(23,23) =  sensor_noise_vimu_rel_foot;

  R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  // std::cout << "a_x " << this->_stateEstimatorData.result->aWorld << std::endl;

  Vec3<double> g(0, 0, -9.81);
  Mat3<double> Rbod = this->_stateEstimatorData.result->rBody.transpose();
  //  std::cout << "Rbody"<< Rbod << "\n";
  Vec3<double> a = this->_stateEstimatorData.result->aWorld + g;  // in old code, Rbod * se_acc + g
  // std::cout << "A WORLD\n" << a << "\n";
  Vec2<double> pzs = Vec2<double>::Zero();
  Vec2<double> trusts = Vec2<double>::Zero();
  Vec3<double> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];


  for (int i = 0; i < 2; i++) {
    int i1 = 3 * i;
    //std::cout << "loop: " << i << std::endl;
    //std::cout << "find hip location" << std::endl;
     Quadruped& quadruped =
        *(this->_stateEstimatorData.legControllerData->A1);

    //std::cout << "dynamics defined" << std::endl;
    Vec3<double> ph = quadruped.getHip2Location(i);  // hip positions relative to CoM
    // std::cout<<"ph: "<< ph <<std::endl;
    // std::cout<<"p: "<< this->_stateEstimatorData.legControllerData[i].p <<std::endl;

    //std::cout << ph << std::endl;

    Vec3<double> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
    // std::cout << ".p is " << this->_stateEstimatorData.legControllerData[i].p << std::endl;
    Vec3<double> dp_rel = this->_stateEstimatorData.legControllerData[i].v; 
        // std::cout<<"p_rel: "<< p_rel <<std::endl;

    Vec3<double> p_f = Rbod * p_rel;

    // std::cout<<"p_f: "<< p_f <<std::endl;
    Vec3<double> dp_f =
        Rbod *
        (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);


    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 6 + i1;
    rindex3 = 12 + i;

  //  std::cout << "Check 1" << std::endl;

    double trust = 0.5;
    double phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), 1);
      //  std::cout << "phase: " << phase << std::endl;

    double trust_window = 0.3;

    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (1 - trust_window)) {
      trust = (1 - phase) / trust_window;
    }

    //  std::cout << "Check 2" << std::endl;

    //printf("Trust %d: %.3f\n", i, trust);
    Q.block(qindex, qindex, 3, 3) =
        (1 + (1 - trust) * 100.0) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 0.1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (1 + (1 - trust) * 100.0) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (1 + (1 - trust) * 100.0) * R(rindex3, rindex3);

    // std::cout << "Q1 is " << std::endl;
    // std::cout << Q << std::endl;
    // std::cout << "R1 is " << std::endl;
    // std::cout << R << std::endl;
    

    trusts(i) = trust;
    // std::cout << "trust: " << trust << std::endl;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    //  std::cout << "Check 3" << std::endl;
  }

  // std::cout << "Ps is " << _ps << std::endl;
  // std::cout << "Vs is " << _vs << std::endl;
  // std::cout << "pzs is " << pzs << std::endl;
  Eigen::Matrix<double, 14, 1> y;
  y << _ps, _vs, pzs;
    // std::cout << "Xhat is " << _xhat << std::endl;
  _xhat = _A * _xhat + _B * a;
  // std::cout << "Xhat is " << _xhat << std::endl;
  // std::cout << "Dimension of A is " << size(A) << std::endl;
  //  std::cout << "Check 3.0" << std::endl;
  Eigen::Matrix<double, 12, 12> At = _A.transpose();
  //  std::cout << "Check 3.1" << std::endl;
  Eigen::Matrix<double, 12, 12> Pm = _A * _P * At + Q;
  //  std::cout << "Check 3.2" << std::endl;
  Eigen::Matrix<double, 12, 14> Ct = _C.transpose();
  //  std::cout << "Check 3.3" << std::endl;
  Eigen::Matrix<double, 14, 1> yModel = _C * _xhat;
  //  std::cout << "Check 3.4" << std::endl;
  Eigen::Matrix<double, 14, 1> ey = y - yModel;
  //  std::cout << "Check 3.5" << std::endl;
  Eigen::Matrix<double, 14, 14> S = _C * Pm * Ct + R;

  //  std::cout << "Check 4" << std::endl;

  // todo compute LU only once
  Eigen::Matrix<double, 14, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<double, 14, 12> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<double, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<double, 12, 12> Pt = _P.transpose();
  _P = (_P + Pt) / 2;

  if (_P.block(0, 0, 2, 2).determinant() > 0.000001) {
    _P.block(0, 2, 2, 10).setZero();
    _P.block(2, 0, 10, 2).setZero();
    _P.block(0, 0, 2, 2) /= 10;
  }

  //  std::cout << "Check 5" << std::endl;
  
  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
  this->_stateEstimatorData.result->vBody =
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;

  std::cout << "T265 reading is" << std::endl;
  for (int i = 0; i < 6; i++){
    std::cout << FSMState::T265_pose[i] << std::endl;
  }

  this->_stateEstimatorData.result->position(0) = -FSMState::T265_pose[2];
  this->_stateEstimatorData.result->position(1) = -FSMState::T265_pose[0];
  this->_stateEstimatorData.result->position(2) = 0.5 + FSMState::T265_pose[1];

  this->_stateEstimatorData.result->position = 
    this->_stateEstimatorData.result->rBody.transpose()*
    this->_stateEstimatorData.result->position;

  // this->_stateEstimatorData.result->vBody(0) = -T265_pose[5];
  // this->_stateEstimatorData.result->vBody(1) = -T265_pose[3];
  // this->_stateEstimatorData.result->vBody(2) = T265_pose[4];

  // this->_stateEstimatorData.result->vWorld = 
  //   this->_stateEstimatorData.result->rBody.transpose()*
  //   this->_stateEstimatorData.result->vBody;

  


}

// void CheaterPositionVelocityEstimator::run() {
//  // std::cout << "run StateEstimator" << std::endl;
//   this->_stateEstimatorData.result->position[0] = lowState.cheat.position[0];
//   this->_stateEstimatorData.result->position[1] = lowState.cheat.position[1];
//   this->_stateEstimatorData.result->position[2] = lowState.cheat.position[2];

//   this->_stateEstimatorData.result->vBody[0] = lowState.cheat.vBody[0];
//   this->_stateEstimatorData.result->vBody[1] = lowState.cheat.vBody[1];
//   this->_stateEstimatorData.result->vBody[2] = lowState.cheat.vBody[2];

//   this->_stateEstimatorData.result->vWorld =
//     this->_stateEstimatorData.result->rBody.transpose() * this->_stateEstimatorData.result->vBody;
// }
