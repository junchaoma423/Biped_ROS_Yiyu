#include "../../include/FSM/FSMState_QPStand.h"

FSMState_QPStand::FSMState_QPStand(ControlFSMData *data)
                 :FSMState(data, FSMStateName::QPSTAND, "QPStand")
{}

void FSMState_QPStand::enter()
{   
    _data->_interface->zeroCmdPanel();
    _data->_legController->zeroCommand();
    _data->_legController->updateData(_data->_lowState, offset);
    _data->_stateEstimator->run();

    init_yaw = _data->_stateEstimator->getResult().rpy(2);

    for(int i = 0; i < 3; i++){
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }


    //set rpy command
    rpy[2] = init_yaw;

    //set COM command
    p_des[0] = 0;
    p_des[1] = 0;
    p_des[2] = 0.55;

    // Set the Translational Gains
    kpCOM[0] = 30;
    kpCOM[1] = 30;
    kpCOM[2] = 50;
    kdCOM[0] = 0.1;
    kdCOM[1] = 0.1;
    kdCOM[2] = 0.1;

    // Set the Orientational Gains
    kpBase[0] = 25;
    kpBase[1] = 25;
    kpBase[2] = 10;
    kdBase[0] = 0.1;
    kdBase[1] = 0.1;
    kdBase[2] = 0.1;
    // abort();
}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}   //I don't think we are using it


void FSMState_QPStand::run()
{
    motionTime++;
    std::cout << "Current state is qpstand state" << std::endl;
    try {
        pd = get_pose_data(pipe);
    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return; // or handle the error appropriately
    }
    T265_pose[0] = pd.x;
    T265_pose[1] = pd.y;
    T265_pose[2] = pd.z;
    T265_pose[3] = pd.x_vel;
    T265_pose[4] = pd.y_vel;
    T265_pose[5] = pd.z_vel;
    
    T265_pos << -T265_pose[2] << "  " << -T265_pose[0] << " " << T265_pose[1] << "  " << -T265_pose[5] << " " << -T265_pose[3] << " " << T265_pose[4];

    joystick.pollEvents();
    state=joystick.getState();
    xAxis = state.axes[0];
    yAxis = state.axes[1];

    buttonA = state.buttons[0];
    buttonB = state.buttons[1];
    left_shoulder = state.buttons[4];

    if (left_shoulder) {
        abort();
    }

    _data->_legController->updateData(_data->_lowState, offset);
    _data->_stateEstimator->setContactPhase(contactphase);
    _data->_stateEstimator->run();

    if (motionTime > 5){
    //Thigh Constraint
    for (int leg = 0; leg < 2; leg++){
        if ((_data->_legController->data[leg].q(2) < Thigh_Constraint[0]) || 
        (_data->_legController->data[leg].q(2) > Thigh_Constraint[1])) {
            std::cout << "Thigh Angle Exceeded" << std::endl;
            abort();
        }
    }

    //Calf Constraint
    for (int leg = 0; leg < 2; leg++){
        if ((_data->_legController->data[leg].q(3) < Calf_Constraint[0]) || 
        (_data->_legController->data[leg].q(3) > Calf_Constraint[1])) {
            std::cout << "Calf Angle Exceeded" << std::endl;
            abort();
        }
    }

    //Ankle Constraint
    for (int leg = 0; leg < 2; leg++){
        if ((_data->_legController->data[leg].q(4) < Ankle_Constraint[0]) || 
        (_data->_legController->data[leg].q(4) > Ankle_Constraint[1])) {
            std::cout << "Ankle Angle Exceeded" << std::endl;
            abort();
        }
    }

    //Pitch Constraint
    if ((_data->_stateEstimator->getResult().rpy(1)) < -0.3){
        std::cout << "Pitch Angle Exceeded" << std::endl;
        abort();
    }
    }
    
        
    //QP Parameters
    for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
    }
    for (int i = 0; i < 3; i++) {
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
    }

    // Get the foot locations relative to COM
    for (int leg = 0; leg < 2; leg++) {
        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose()*(_data->_quadruped->getHip2Location(leg) + _data->_legController->data[leg].p); //getResult().rBody *
        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
    }

    //QP Controller Calculation
    balanceController.set_alpha_control(0.001);
    balanceController.set_friction(0.4);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);

    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                        O_err, _data->_stateEstimator->getResult().rpy(2));
    // balanceController.print_QPData();
    balanceController.solveQP_nonThreaded(fOpt);

    // std::cout << "fopt is " << std::endl;
    // for (int i = 0; i < 12; i++){
    //     std::cout << fOpt[i] << "  ";
    // }
    // std::cout << "\n";


    for (int i = 0; i < 13; i++){
        b_des << se_xfb[i] << " ";
    }

    for (int leg = 0; leg < 2; leg++){
        fOptO[0] = fOpt[leg * 3];
        fOptO[1] = fOpt[leg * 3 + 1];
        fOptO[2] = fOpt[leg * 3 + 2];
        fOptR = fOptO;
 
        mOptO[0] = fOpt[6 + leg * 3];
        mOptO[1] = fOpt[6 + leg * 3 + 1];
        mOptO[2] = fOpt[6 + leg * 3 + 2];
        mOptR = mOptO;

        footFeedForwardForces.col(leg) << fOptR[0], fOptR[1], fOptR[2], mOptR[0], mOptR[1], mOptR[2];

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);

    }

        //Push the Command to Leg Controller
    _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);
        
    //Data Recording

    for (int i = 0; i <3; i++){
        com_pos << _data->_stateEstimator->getResult().position(i) << "  ";
        rpy_input << _data->_stateEstimator->getResult().rpy(i) << " ";
        omega << _data->_stateEstimator->getResult().omegaWorld(i) << " ";
        acceleration << _data->_stateEstimator->getResult().aBody(i) << " ";
    }

    for (int i = 0; i < 3; i++){
    com_pos << v_act[i] << "  ";
    }
    for (int i = 0; i < 3; i++){
    rpy_input << rpy[i] << "  ";
    }
    for (int i = 0; i < 13; i++){
    myfile << se_xfb[i] << "  ";
    }
    for (int i = 0; i <6; i++){
    footposition << pFeet[i] << "  ";
    }
    for (int i = 0; i < 12; i++){
    force << fOpt[i] << " ";
    angle << _lowState->motorState[i].q << "  ";
    QP << fOpt[i] << "  ";
    }

    for (int leg = 0; leg < 2; leg++)
    {
    for (int i = 0; i< 5; i++){
        corrected_angle << _data->_legController->data[leg].q(i) << " ";
    }
    }

    for (int leg = 0; leg < 2; leg++)
    {
    for (int i = 0; i< 5; i++){
        corrected_angle << _data->_legController->data[leg].qd(i) << " ";
    }
    }

    for (int i = 0; i < 12; i++){
    tau_est << _lowCmd->motorCmd[i].tau << "  ";
    }


    for (int i = 0; i < 12; i++){
    tau_est << _lowState->motorState[i].tauEst << "  ";
    }

    angle << std::endl;
    torque << std::endl;
    com_pos << std::endl;
    footposition << std::endl;
    QP << std::endl;
    myfile << std::endl;
    force << std::endl;
    rpy_input << std::endl;
    b_des << std::endl;
    tau_est << std::endl;
    temperature<<std::endl;
    corrected_angle<<std::endl;
    T265_pos<<std::endl;

    // control.PowerProtect(_lowCmd, _lowState, 10);
}

void FSMState_QPStand::exit()
{
    _data->_interface->zeroCmdPanel();
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_QPStand::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_X){ // TODO: Change this to the corrent UserCommand
        std::cout << "transition from QP stand to MPC stand" << std::endl;
        return FSMStateName::MPCSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::QPSTAND;
    }
}
