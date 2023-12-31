#include <iostream>
#include "../include/common/Utilities/Timer.h"
#include "../include/common/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"

using namespace ori;

/* ========================= GAIT ========================= */
Gait::Gait(int nMPC_segments, Vec2<int> offsets, Vec2<int> durations, const std::string &name) : _offsets(offsets.array()), // 0 5
                                                                                                 _durations(durations.array()),
                                                                                                 _nIterations(nMPC_segments)
{
  _mpc_table = new int[nMPC_segments * 2];

  _offsetsPhase = offsets.cast<double>() / (double)nMPC_segments;     // 0 0.5
  _durationsPhase = durations.cast<double>() / (double)nMPC_segments; // 0.5 0.5

  _stance = durations[0];                // 5
  // std::cout << "stance: " << _stance << std::endl;

  _swing = nMPC_segments - durations[0]; // 5
}

Gait::~Gait()
{
  delete[] _mpc_table;
}

Vec2<double> Gait::getContactSubPhase()
{
  Array2d progress = _phase - _offsetsPhase; // 0,0.5------

  for (int i = 0; i < 2; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > _durationsPhase[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsPhase[i];
    }
  }

  return progress.matrix();
}

Vec2<double> Gait::getSwingSubPhase()
{
  Array2d swing_offset = _offsetsPhase + _durationsPhase; //(0.5 1)
  for (int i = 0; i < 2; i++)
    if (swing_offset[i] > 1)
      swing_offset[i] -= 1.;
  Array2d swing_duration = 1. - _durationsPhase; //(0.5 0.5)

  Array2d progress = _phase - swing_offset;

  for (int i = 0; i < 2; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;
    if (progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

int *Gait::mpc_gait()
{
  for (int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration) % _nIterations;
    Array2i progress = iter - _offsets; // 0 5
    for (int j = 0; j < 2; j++)
    {
      if (progress[j] < 0)
        progress[j] += _nIterations;
      if (progress[j] < _durations[j])
        _mpc_table[i * 2 + j] = 1;
      else
        _mpc_table[i * 2 + j] = 0;
    }
  }
  // // 20 horizons:
  // for (int i = 0; i < _nIterations; i++)
  // {
  //   int inner_iter =  (i + _iteration) % 20 ;
  //   std::cout << "inner_iter: " << inner_iter << std::endl;
  //   if (0 <= inner_iter && inner_iter <= 4)
  //   {
  //     _mpc_table[2*i] = 1;
  //     _mpc_table[2*i+1] = 0;
  //   }
  //   else if (10 <= inner_iter && inner_iter <= 14)
  //   {
  //     _mpc_table[2*i] = 0;
  //     _mpc_table[2*i+1] = 1;
  //   }
  //   else{
  //     _mpc_table[2*i] = 1;
  //     _mpc_table[2*i+1] = 1;
  //   }

  // }

  // std::cout << "horizon: " << _iteration << std::endl;
  // std::cout << "nIterations: " << _nIterations << std::endl;

  return _mpc_table;
  // std::cout << "MPC Table: " << *_mpc_table << std::endl;
}

void Gait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  // _iteration = (currentIteration / iterationsPerMPC);
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double)(iterationsPerMPC * _nIterations);
}

/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) : iterationsBetweenMPC(_iterations_between_mpc),
                                                                                    horizonLength(10),
                                                                                    dt(_dt),
                                                                                    galloping(horizonLength, Vec2<int>(0, 2), Vec2<int>(5, 5), "Galloping"),
                                                                                    pronking(horizonLength, Vec2<int>(0, 0), Vec2<int>(4, 4), "Pronking"),
                                                                                    trotting(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Trotting"),
                                                                                    bounding(horizonLength, Vec2<int>(0, 0), Vec2<int>(5, 5), "Bounding"),
                                                                                    walking(horizonLength, Vec2<int>(0, 5), Vec2<int>(5, 5), "Walking"),
                                                                                    pacing(horizonLength, Vec2<int>(0, 9), Vec2<int>(5, 5), "Pacing"),
                                                                                    standing(horizonLength, Vec2<int>(0, 0), Vec2<int>(10, 10), "Standing")
{
  // gaitNumber = 7;
  dtMPC = dt * iterationsBetweenMPC;
  // std::cout << "dtMPC: " << dtMPC << std::endl;
  rpy_int[2] = 0;
  for (int i = 0; i < 2; i++)
    firstSwing[i] = true;

  foot_position.open("foot_pos.txt");
}

void ConvexMPCLocomotion::run(ControlFSMData &data)
{
  bool omniMode = false;

  //  auto* debugSphere = data.visualizationData->addSphere();
  //  debugSphere->color = {1,1,1,0.5};
  //  debugSphere->radius = 1;
  // data._legController->updateData();
  // data._stateEstimator->run();
  auto &seResult = data._stateEstimator->getResult();
  auto &stateCommand = data._desiredStateCommand;
  std::cout << "in side mpc" << seResult.rBody << std::endl;;
  std::cout << "T265 Reading: " << std::endl;
  std::cout << "x: " << seResult.position(0) << std::endl;
  std::cout << "y: " << seResult.position(1) << std::endl;
  std::cout << "z: " << seResult.position(2) << std::endl;



  // pick gait
  Gait *gait = &trotting;
  if (gaitNumber == 1)
    gait = &bounding;
  else if (gaitNumber == 2)
    gait = &trotting;
  else if (gaitNumber == 3)
    gait = &walking;
  else if (gaitNumber == 4)
    gait = &pacing;
  else if (gaitNumber == 5)
    gait = &galloping;
  else if (gaitNumber == 6)
    gait = &pronking;
  else if (gaitNumber == 7)
    gait = &standing;
  current_gait = gaitNumber;
  // integrate position setpoint
  // Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_robot;
  Vec3<double> v_des_world;

  // std::cout << "v_des_robot is " << v_des_robot << std::endl;
  // std::cout << "v_des_world is " << v_des_world << std::endl;

  // v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
  // Vec3<double> v_robot = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2])*seResult.vWorld;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.55; //.5;;;

  // printf("p_des \t%.6f\n", dt * v_des_world[0]);
  // Integral-esque pitch and roll compensation
  // if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  // {
  //   rpy_int[1] += dt*(stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/ - seResult.rpy[1])/v_robot[0];
  // }
  // if(fabs(v_robot[1]) > 0.1)
  // {
  //   rpy_int[0] += dt*(stateCommand->data.stateDes[3] /*-hw_i->state_estimator->se_ground_pitch*/ - seResult.rpy[0])/v_robot[1];
  // }

  // rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  // rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  // rpy_comp[1] = v_robot[0] * rpy_int[1];
  // rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=6);  //turn off for pronking

  // get then foot location in world frame
  for (int i = 0; i < 2; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data._quadruped->getHip2Location(i) + data._legController->data[i].p);
    // Vec3<double> PactFoot[i] =  
    // pFoot[i] = data._legController->data[i].p;
    // std::cout << "pFoot" << i << "\n" << pFoot[i] << std::endl;
  }

  // if(climb){
  //   bool allContact = true;
  //   for(int i = 0; i < 4; i++){
  //     if(lowState.footForce[i] < 3 ){
  //       allContact = false;
  //     }
  //   }
  // if(iterationCounter % (iterationsBetweenMPC * horizonLength) == 0){
  //   for(int i = 0; i < 2; i++)
  //       {
  //         W.row(i) << 1, pFoot[i][0], pFoot[i][1];
  //         pz[i] = pFoot[i][2];
  //        // if(i != 0) W.row(i) << 0, pFoot[i][0], pFoot[i][1];
  //       }
  //     a = W.transpose() * W * (W.transpose()* W * W.transpose()*W).inverse()*  W.transpose() * pz;
  //     ground_pitch = acos(-1/sqrt(a[1]*a[1] + a[2]*a[2] +1)) - 3.14;
  //     //std::cout << "ground pitch: " << ground_pitch << std::endl;
  //     if(pz[0] < pz[2]){
  //       ground_pitch = -ground_pitch;
  //     }
  //     if(abs(pz[0] - pz[2]) < 0.01){
  //       ground_pitch = 0;
  //     }
  //   }
  // }

  // if(climb){
  //   if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  // {
  //   rpy_int[1] += dt*(stateCommand->data.stateDes[4] - seResult.rpy[1])/v_robot[0];
  // }
  // if(fabs(v_robot[1]) > 0.1)
  // {
  //   rpy_int[0] += dt*(stateCommand->data.stateDes[3]
  //       - seResult.rpy[0])/v_robot[1];
  // }
  // }

  // some first time initialization
  if (firstRun)
  {
    // std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0, 0, 0); // connect to desired state command later
    Vec3<double> v_des_world(0, 0, 0); // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    //
    if (gaitNumber == 7)
    {
      pBody_des[0] = seResult.position[0];
      pBody_des[1] = seResult.position[1];
      pBody_des[2] = 0.55;

      vBody_des[0] = 0;
      vBody_des[0] = 0;
    }

    for (int i = 0; i < 2; i++)
    {
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      // std::cout << "orig foot pos " << i << pFoot[i] << std::endl;
    }
    firstRun = false;
  }

  // Update For WBC//
  // pBody_des[0] = world_position_desired[0];
  // pBody_des[1] = world_position_desired[1];
  // pBody_des[2] = 0.4;

  // vBody_des[0] = v_des_world[0];
  // vBody_des[1] = v_des_world[1];
  // vBody_des[2] = 0.;

  // pBody_RPY_des[0] = 0.;
  // pBody_RPY_des[1] = 0.;
  // pBody_RPY_des[2] = 0.; // stateCommand->data.stateDes[5];

  // vBody_Ori_des[0] = 0.;
  // vBody_Ori_des[1] = 0.;
  // vBody_Ori_des[2] = 0.; // stateCommand->data.stateDes[11];

  // contact_state = gait->getContactSubPhase();

  // std::cout << "foot force 0 :" << lowState.footForce[0] << std::endl;
  // std::cout << "contact 0 : " << contact_state[0] << std::endl;

  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;

  // std::cout << "Swing Time: " << swingTimes[0] << std::endl;

  // std::cout << "Swing Time" << swingTimes << std::endl;
  double side_sign[2] = {1, -1};
  double interleave_y[2] = {-0.1, 0.1};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for (int i = 0; i < 2; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    // if (firstSwing[i])
    // {

      footSwingTrajectories[i].setHeight(.1);
      Vec3<double> offset(0, side_sign[i] * data._quadruped->hipLinkLength, 0);
      // simple heuristic function
      // std::cout << "swing time" << swingTimeRemaining[i] << std::endl;

      Vec3<double> pRobotFrame = (data._quadruped->getHip2Location(i) + offset);
      // Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -stateCommand->data.stateDes[11] * gait->_stance * dtMPC / 2) * pRobotFrame;

      Vec3<double> des_vel;

      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = stateCommand->data.stateDes(8);
      // std:: cout << "des_vel0 =" << des_vel[0] << "\n";
      // std:: cout << "des_vel1 =" << des_vel[1] << "\n";
      // std:: cout << "des_vel2 =" << des_vel[2] << "\n";

      Vec3<double> Pf = seResult.position +
                        seResult.rBody.transpose() * pRobotFrame 
                        + seResult.vWorld * swingTimeRemaining[i];
      // std::cout << "P =" << seResult.position << "\n";
      // std::cout << "i =" << i << "\n";
      // std::cout << "Pf =" << Pf << "\n";
      //+ seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.4;
      double pfx_rel = seResult.vWorld[0] * 0.5 * gait->_stance * dtMPC +
                       0 * (seResult.vWorld[0] - v_des_world[0]);

      double pfy_rel = seResult.vWorld[1] * 0.5 * gait->_stance * dtMPC +
                       0 * (seResult.vWorld[1] - v_des_world[1]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      // std:: cout << "pfy_rel =" << pfy_rel << "\n";
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel; //+ interleave_y[i] * v_abs * interleave_gain;
      Pf[2] = -0.01;

      footSwingTrajectories[i].setFinalPosition(Pf);
    // }
  }

  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter); 
  // std::cout<< "iteration_counter; "<< iterationCounter << std::endl;

  // load LCM leg swing gains
  Kp << 150, 0, 0,
      0, 150, 0,
      0, 0, 300;
  Kp_stance = 0* Kp;

  Kd << 3, 0, 0,
      0, 3, 0,
      0, 0, 10;
  Kd_stance = 0*Kd;
  // gait
  Vec2<double> contactStates = gait->getContactSubPhase();
  Vec2<double> swingStates = gait->getSwingSubPhase();

  // // contact and swing states for 20horizon slow walking gait:
  // int outer_iter = iterationCounter % 600;
  // if (0 <= outer_iter && outer_iter <= 149){
  //   contactStates(0) = outer_iter/150.0;
  //   contactStates(1) = 0;
  //   swingStates(0) = 0;
  //   swingStates(1) = outer_iter/150.0;
  // }
  // else if (300 <= outer_iter && outer_iter <= 449){
  //   contactStates(1) = (outer_iter-300)/150.0;
  //   contactStates(0) = 0;
  //   swingStates(1) = 0;
  //   swingStates(0) = (outer_iter-300)/150.0;
  // }
  // else {
  //   contactStates(1) = 1.0;
  //   contactStates(0) = 1.0;
  //   swingStates(1) = 0;
  //   swingStates(0) = 0;
  // }
  int *mpcTable = gait->mpc_gait();
  
    // std::cout << "/////////  MPC Table:  /////////" <<std::endl;
    // for(int i = 0; i<20; i++)
    //   {
    //   std::cout << mpcTable[i] <<std::endl;
    //   }
  
  
  updateMPCIfNeeded(mpcTable, data, omniMode);

  // for(int foot = 0; foot < 2; foot++){
  //   data._legController->commands[foot].feedforwardForce = f_ff[foot];
  //   std::cout << "ff:" << f_ff[foot] << std::endl;
  // }
  iterationCounter++;

  //  StateEstimator* se = hw_i->state_estimator;
  Vec2<double> se_contactState(0, 0);
  // for(int i = 0; i < 2; i++){
  //   footSwingTrajectories[i].setHeight(0.1);
  //   footSwingTrajectories[i].setInitialPosition(pFoot[i]);
  //   footSwingTrajectories[i].setFinalPosition(pFoot[i]);
  // }

  for (int foot = 0; foot < 2; foot++)
  {

    double contactState = contactStates(foot);
    double swingState = swingStates(foot); 
    // std::cout << "swing" << foot << ": " << swingState << std::endl;
    // std::cout << "Contact" << foot << ": " << contactState << std::endl;
    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        // std::cout << "check 1" << std::endl;
        firstSwing[foot] = false;
        
        footSwingTrajectories[foot].setHeight(0.1);
      }

      // std::cout << "foot" << foot << ": " << foot << std::endl;
      // footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      // Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      // Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      // Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHip2Location(foot);
      // Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // if (vDesLeg.hasNaN())
      // {
      //   vDesLeg << 0, 0, 0;
      // }
      Vec3<double> pDesFootWorld = {0, 0, 0};
      Vec3<double> vDesFootWorld = {0, 0, 0};

      // // new trajectory
      double side = 1.0 ;
      if (foot == 0){
        side = -1.0;
      }
      double footHeight = 0.06;
      pDesFootWorld[0] = seResult.position[0] +
                      seResult.vWorld[0] * 0.50 * gait->_stance * dtMPC +
                       0.05 * (seResult.vWorld[0] - v_des_world[0]);
      pDesFootWorld[1] = seResult.position[1] + 
                      seResult.vWorld[1] * 0.50 * gait->_stance * dtMPC +
                       0.05 * (seResult.vWorld[1] - v_des_world[1]);
      pDesFootWorld[2] = -footHeight/2.0 * std::cos(2.0*3.1415*swingState)+footHeight/2 - 0.0;
      vDesFootWorld[2] = footHeight * 3.1415 * std::sin(2.0*3.1415*swingState);
      // pDesFootWorld[2] = footHeight/2.0 * std::cos(3.1415*swingState) + footHeight/2.0 - 0.015;
      // vDesFootWorld[2] = -3.1415*footHeight/2.0 * std::sin(3.1415*swingState);
      // vDesFootWorld[2] = 0;

      // vDesFootWorld[0] = seResult.vWorld[0];
      // vDesFootWorld[1] = seResult.vWorld[1];

      Vec3<double> hipHeightOffSet = {0,0,-0.126};
      Vec3<double> hipWidthOffSet = {-0.025,side*0.07,0};
      Vec3<double> dummyPos = {0,0,0.55};
      
      Vec3<double> pDesLeg =  (pDesFootWorld - seResult.position) - hipHeightOffSet + hipWidthOffSet;
      Vec3<double> vDesLeg =   (vDesFootWorld - seResult.vWorld);
//seResult.rBody.transpose() *
      std::cout << "Rotation: " << seResult.rBody.transpose() << std::endl;

      // pDesLeg = seResult.rBody.transpose() * pDesLeg;

      // Vec3<double> footForce =
      //       Kp * (pDesLeg - data._legController->data[foot].p) + 
      //       Kd * (vDesLeg - data._legController->data[foot].v);
      // footForce =  seResult.rBody.transpose() * footForce;
      // footForce = footForce*0;
      // std::cout << "check 2" << std::endl;

      if (vDesLeg.hasNaN())
      {
        vDesLeg << 0, 0, 0;
      }
      if (pDesLeg.hasNaN())
      {
        pDesLeg << 0, 0, -0.4;
      }

      // std::cout << "footz" << foot << ": " << pDesLeg << std::endl;

      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
      // cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      // std::cout << "check 1" << std::endl;
      data._legController->commands[foot].feedforwardForce << 0, 0, 0 , 0 , 0 , 0;
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp;
      data._legController->commands[foot].kdCartesian = Kd;
      // std::cout << "check 3" << std::endl;
      data._legController->commands[foot].kptoe = 10; // 0
      data._legController->commands[foot].kdtoe = 0.2;
      // std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(0) << std::endl;
      // std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(1) << std::endl;
      // std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(2) << std::endl;
      // std::cout << "foot Des " << foot << ": \n " << pDesLeg(2) << std::endl;
      // singularity barrier
      // data._legController->commands[foot].tau[2] =
      //  50*(data._legController->data[foot].q(2)<.1)*data._legController->data[foot].q(2);
      // std::cout << "contat " << foot << ": " << contactState << std::endl;
      // if (foot == 0)
      // {
      //   foot_position << pDesFootWorld[0] << " " << pDesFootWorld[1] << " " << pDesFootWorld[2] << "\n";
      // }
      // if(climb){
      //   if(lowState.footForce[foot] > 10 && swingState>0.5){
      //     //std::cout << "force changed for leg " << foot << std::endl;
      //     data._legController->commands[foot].kpCartesian = Kp_stance;
      //     data._legController->commands[foot].kdCartesian = 0 * Kd;
      //     data._legController->commands[foot].feedforwardForce << 0, 0, -10;
      //     contactState = 0;
      //     firstSwing[foot] = true;
      //   }
      // }
      se_contactState[foot] = contactState;
    }

    else if (contactState > 0) // foot is in stance
    { 
      //firstSwing[foot] = true;
      // footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      // Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
      // Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
      // Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHip2Location(foot);
      // Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // if (vDesLeg.hasNaN())
      //   {
      //    vDesLeg << 0, 0, 0;
      //    }

      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
      // cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
      // std::cout << "robot pos: \n" << seResult.position << std::endl;
      // foot_swing << pDesFootWorld[2] << " ";
      Vec3<double> pDesLeg = {0, 0, 0};
      Vec3<double> vDesLeg = {0, 0, 0};
      data._legController->commands[foot].pDes = pDesLeg;
      data._legController->commands[foot].vDes = vDesLeg;
      data._legController->commands[foot].kpCartesian = Kp_stance; // 0
      data._legController->commands[foot].kdCartesian = Kd_stance;

      data._legController->commands[foot].kptoe = 0; // 0
      data._legController->commands[foot].kdtoe = 0;

      data._legController->commands[foot].feedforwardForce = f_ff[foot];
      std::cout << "feedforwardforce saved" << std::endl;
      
      // std::cout << "y: " << pDesLeg[1] << std::endl;
      // std::cout << "contact " << foot << " : \n" << contactState << std::endl;
      // std::cout << "foot force " << foot << " : \n " << data._legController->commands[foot].feedforwardForce(2) << std::endl;
      // data._legController->commands[foot].kdJoint = Mat3<double>::odyIdentity() * 0.2

      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      // Fr_des[foot] = -f_ff[foot];

      // foot_swing << "\n";
      // std::cout << "foot Des" << foot << " \n " << data._legController->commands[foot].pDes << std::endl;
    }

    // se->set_contact_state(se_contactState); todo removed
    data._stateEstimator->setContactPhase(se_contactState);
    // data._legController->updateCommand();
  }
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData &data, bool omniMode)
{
  // iterationsBetweenMPC = 30;
  //  std::cout << "MPC iteration: ";
  //  std::cout << iterationCounter % iterationsBetweenMPC << std::endl;
  // data._legController->updateData();
  if ((iterationCounter % 10) == 0)
  {
    // std::cout << "state est: ";

    // for(int i = 0; i < 3; i++){
    //   std::cout << data._stateEstimator->getResult().rpy(i) << " " ;
    //  }
    //  std::cout << endl;
    // std::cout << "runMPC" << std::endl;
    auto seResult = data._stateEstimator->getResult();
    auto &stateCommand = data._desiredStateCommand;

    double *p = seResult.position.data();
    double *v = seResult.vWorld.data();
    double *w = seResult.omegaWorld.data();
    double *q = seResult.orientation.data();

    // float* pf = (float*) seResult.position.data();
    // float* vf = (float*) seResult.vWorld.data();
    // float* wf = (float*) seResult.omegaWorld.data();
    // float* qf = (float*) seResult.orientation.data();

    double r[6];
    for (int i = 0; i < 6; i++)
    {
      r[i] = pFoot[i % 2][i / 2] - seResult.position[i / 2];
    }
    
    double Q[12] = {80, 80, 40,   500, 500, 150,   .1, .1, 1,   .1, .1, .2}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    
    // if (gaitNumber != 7) // with gait
    // double Q[12] = {90, 60, 40,   220, 270, 100,   .1, .1, .1,   1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    // double Q[12] = {90, 60, 40,   350, 350, 100,   .1, .1, .1,   0.1, 0.5, 3}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz

    // std::cout << "gait " << gaitNumber <<std::endl;
    // double Q[12] = {150, 50, 100,  700, 250, 350,  .5, .5, .5,  .5, .5, .5}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    //  double Q[12] = {1000, 1000, 1000,   1000, 1000, 5000,   1, 1, 1,   1, 1, 1};
    // double Q[12] = {3.5*1000, 3.5*1000, 1000,   1000, 1000, 0.25*10000,   1000, 2*1000, 1000,   1000, 1000, 3*1000};
    //double Q[12] = {2000, 2000, 10, 35, 35, 35, 1, 1, 1, 1, 1, 1};

    double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
    // double Alpha[12] = {1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4,   5e-4, 5e-4, 5e-4, 5e-4, 5e-4, 5e-4};

    double *weights = Q;
    double *Alpha_K = Alpha;
    // double alpha = 1e-4; //1e-5 make setting eventually // make it a vector that contains all control inputs

    double yaw = seResult.rpy[2];
    // double alpha = 1e-3; // make setting eventually // make it a vector that contains all control inputs

    // printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    // if(alpha > 1e-4)
    // {

    //   std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    //   alpha = 1e-5;
    // }
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7], 0);
    // Vec3<double> v_des_robot(0.2, 0, 0);

    // Vec3<double> v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;

    std::cout << "v_des_robot is " << v_des_robot << std::endl;
    std::cout << "v_des_world is " << v_des_world << std::endl;

    
    // v_des_world[0] = 0.1;
    // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // if(current_gait == 4)
    // {
    //   double trajInitial[12] = {0, //(double)stateCommand->data.stateDes[3],
    //                             0, // (double)stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/,
    //                             seResult.rpy(2), // (double)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
    //                             0, //(double)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
    //                             0, //(double)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
    //                            0.4/*fsm->main_control_settings.p_des[2]*/,
    //                            0,0,0,0,0,0};

    //   for(int i = 0; i < horizonLength; i++)
    //     for(int j = 0; j < 12; j++)
    //       trajAll[12*i+j] = trajInitial[j];
    // }

    // else
    //{
    // world_position_desired = seResult.position;
    const double max_pos_error = 0.2;
    double xStart = world_position_desired[0];
    double yStart = world_position_desired[1];
    // std::cout << "orig " << xStart << "  " << yStart << std::endl;
    // printf("orig \t%.6f\t%.6f\n", xStart, yStart);
    // printf("ref: \t%.6f\t%.6f\n", p[0], p[1]);

    double yaw_des = v_des_robot[1] * 10;
    double height_add_des = v_des_robot[0] * 0.75;

    if (gaitNumber != 7) //with gait
    {
    if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
    if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

    if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
    if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;
    
    // yaw_des = 0;
    // height_add_des = 0;
    }
    // printf("xys: \t%.3f\t%3.f\n", xStart, yStart);
    // printf("perr \t%.3f\t%.3f\n", p[0] - world_position_desired[0], p[1] - world_position_desired[1]);

    // double trajInitial[12] = {/*rpy_comp[0] + */stateCommand->data.stateDes[3],  // 0
    //                           /*rpy_comp[1] + */stateCommand->data.stateDes[4],    // 1
    //                           stateCommand->data.stateDes[5] + yaw_des,    // 2
    //                           xStart,                                   // 3
    //                           yStart,                                   // 4
    //                           0.55 + height_add_des,   // 5
    //                           0,                                        // 6
    //                           0,                                        // 7
    //                           stateCommand->data.stateDes[11],  // 8
    //                           v_des_world[0],                           // 9
    //                           v_des_world[1],                           // 10
    //                           0};   // 11
    
    // if (gaitNumber == 7){ //standing
    double trajInitial[12] = {0,  // 0
                              0,    // 1
                              0 + yaw_des,    // 2
                              0,                                   // 3
                              0,                                   // 4
                              0.55 + height_add_des,   // 5
                              0,                                        // 6
                              0,                                        // 7
                              0,  // 8
                              0,                           // 9
                              0,                           // 10
                              0};                                       // 11
    // }
    // double trajInitial[12] = {0,   // 0
    //                           0 + v_des_world[1]*5,   // 1
    //                           0 ,   // 2
    //                           0,   // 3
    //                           0,   // 4
    //                           0.55 + v_des_world[0]*1 , // 5
    //                           0,   // 6
    //                           0,   // 7
    //                           0,   // 8
    //                           0,   // 9
    //                           0,   // 10
    //                           0};

    // if(climb){
    //   trajInitial[1] = ground_pitch;
    //   trajInitial[5] += a[0] + a[1] * xStart + a[2] * yStart;
    // }
    // double trajInitial[12] = {rpy_comp[0],  // 0
    //                           rpy_comp[1],    // 1
    //                           stateCommand->data.stateDes[5],    // 2
    //                           xStart,                                   // 3
    //                           yStart,                                   // 4
    //                           0.4,      // 5
    //                           0,                                        // 6
    //                           0,                                        // 7
    //                           stateCommand->data.stateDes[11],  // 8
    //                           v_des_world[0],                           // 9
    //                           v_des_world[1],                           // 10
    //                           0};                                       // 11

    // float trajInitial[12] = {(float)0., // 0
    //                          (float)0., // 1
    //                          (float)stateCommand->data.stateDes[5],    // 2
    //                          (float)xStart,                                   // 3
    //                          (float)yStart,                                   // 4f
    //                          (float)0.4,      // 5
    //                          0,                                        // 6
    //                          0,                                        // 7
    //                          (float)stateCommand->data.stateDes[11],  // 8
    //                          (float)v_des_world[0],                           // 9
    //                          (float)v_des_world[1],                           // 10
    //                          0};

    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
        trajAll[12 * i + j] = trajInitial[j];

      // if (gaitNumber != 7){
      // if(i == 0) // start at current position  TODO consider not doing this
      // {
      //   trajAll[0] = seResult.rpy[0];
      //   trajAll[1] = seResult.rpy[1];
      //   trajAll[2] = seResult.rpy[2];
      //   trajAll[3] = seResult.position[0];
      //   trajAll[4] = seResult.position[1];
      //   trajAll[5] = seResult.position[2];
      // }
      // else{
      // if (v_des_world[0] < 0.001 && v_des_world[0] > -0.001) {
      //   trajAll[12*i + 3] = trajInitial[3] + i * dtMPC * v_des_world[0];
      //   }
      //   else{
      //    trajAll[12*i + 3] = seResult.position[0] + i * dtMPC * v_des_world[0]; 
      //   }
      //   if (v_des_world[1] < 0.001 && v_des_world[1] > -0.001) {
      //   trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
      //   }
      //   else{
      //    trajAll[12*i + 4] = seResult.position[1] + i * dtMPC * v_des_world[1]; 
      //   }
      //   // trajAll[12*i + 4] = trajInitial[4] + i * dtMPC * v_des_world[1];
      //   trajAll[12*i + 2] = stateCommand->data.stateDes[5] + i * dtMPC * stateCommand->data.stateDes[11];
      //   //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
      // }
      // }
      std::cout << "traj " << i << std::endl;
      for (int j = 0; j < 12; j++) {
        std::cout << trajAll[12 * i + j] << "  ";
      }
          std::cout<< " " <<std::endl;
    }
    // }

    // for(int i = 0; i < 12; i++)
    //     printf("%.4f, ", trajAll[i]);

    // printf("\n\n");

    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC, horizonLength, 0.25, 500);
    Timer t2;
    t2.start();
    // cout << "dtMPC: " << dtMPC << "\n";
    //  update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
    update_problem_data(p, v, q, w, r, yaw, weights, trajAll, Alpha_K, mpcTable);
    // t2.stopPrint("Run MPC");
    printf("MPC Solve time %f ms\n", t2.getMs());
    // std::cout << t2.getSeconds() << std::endl;
    for (int leg = 0; leg < 2; leg++)
    {
      Vec3<double> GRF;
      Vec3<double> GRF_R;
      Vec3<double> GRM;
      Vec3<double> GRM_R;
      Vec6<double> f;
      for (int axis = 0; axis < 3; axis++)
      {

        GRF[axis] = get_solution(leg * 3 + axis);
        GRM[axis] = get_solution(leg * 3 + axis + 6);
      }
      GRF_R = -coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]) * GRF;
    
      GRM_R = -coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]) * GRM;

      for (int i = 0; i < 3; i++){
        f(i) = GRF_R(i);
        f(i+3) = GRM_R(i);
      }
      f_ff[leg] = f;
      // f_ff[leg].setZero();

      // std::cout << f_ff[leg] << std::endl;
      std::cout << "mpc solution" << leg << "\n" << f << std::endl;
      //  Update for WBC
      //  Fr_des[leg] = f;
    }

    // printf("update time: %.3f\n", t1.getMs());
  }
}