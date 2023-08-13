#include "../../include/FSM/FSMState_MPCStand.h"

FSMState_MPCStand::FSMState_MPCStand(ControlFSMData *data)
                 :FSMState(data, FSMStateName::MPCSTAND, "MPCStand"),
                 Cmpc(0.001, 30){}

void FSMState_MPCStand::enter()
{   
    _data->_interface->zeroCmdPanel();
    _data->_legController->zeroCommand();
    _data->_legController->updateData(_data->_lowState, offset);
    _data->_stateEstimator->run();

    init_yaw = _data->_stateEstimator->getResult().rpy(2);
    motionTime = 5000;

}

template<typename T0, typename T1, typename T2>
T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}


void FSMState_MPCStand::run()
{
    motionTime++;

    std::cout << "Current state is MPC " << std::endl;

    pd = get_pose_data(pipe);

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
    vx_command = -yAxis * 0.1;
    vy_command = -xAxis * 0.1;

    buttonA = state.buttons[0];
    buttonB = state.buttons[1];
    left_shoulder = state.buttons[4];

    if (left_shoulder) {
        abort();
    }

    // _data->_stateEstimator->setContactPhase(contactphase);

    v_des[0] = vx_command;
    v_des[1] = vy_command;
    v_command = {v_des[0], v_des[1], v_des[2]};


    for (int i = 0; i < 12; i++){
        angle << _lowState->motorState[i].q << "  ";
    }


    _data->_legController->updateData(_data->_lowState, offset);

        //Angle Constraints
    if (motionTime > 5){
        // Hip Constraint
        if ((_data->_legController->data[0].q(0) < Abad_Leg1_Constraint[0]) || 
          (_data->_legController->data[0].q(0) > Abad_Leg1_Constraint[1])) {
            std::cout << "Abad R Angle Exceeded" << _data->_legController->data[0].q(0) << std::endl;
            abort();
          }
        if ((_data->_legController->data[1].q(0) < Abad_Leg2_Constraint[0]) || 
            (_data->_legController->data[1].q(0) > Abad_Leg2_Constraint[1])) {
            std::cout << "Abad L Angle Exceeded" << _data->_legController->data[1].q(0) << std::endl;
            abort();
            }

        // AbAd Constraint
        if ((_data->_legController->data[0].q(1) < Hip_Leg1_Constraint[0]) ||
            (_data->_legController->data[0].q(1) > Hip_Leg1_Constraint[1])) {
            std::cout << "Hip R Angle Exceeded" << std::endl;
            abort();
            }
        if ((_data->_legController->data[1].q(1) < Hip_Leg2_Constraint[0]) ||
            (_data->_legController->data[1].q(1) > Hip_Leg2_Constraint[1])) {
            std::cout << "Hip L Angle Exceeded" << std::endl;
            abort();
            }

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

    _data->_stateEstimator->run();

    //////////////////// MPC ///////////////////
    if(motionTime >= 0){
        Cmpc.setGaitNum(7);

        _data->_desiredStateCommand->setStateCommands(roll, pitch, v_command, yaw_rate);
        Cmpc.run(*_data);

        std::cout << "MPC sol.:" << std::endl;
        for (int i = 0; i < 12; i++) {
          std::cout << get_solution(i) << std::endl;
        }

        //Push the Command to Leg Controller
        _data->_legController->updateCommand(_data->_lowCmd, offset, motionTime);

    }

    // //Data Recording

    for (int i = 0; i <3; i++){
        com_pos << _data->_stateEstimator->getResult().position(i) << "  ";
        com_pos << _data->_stateEstimator->getResult().vWorld(i) << "  ";
        rpy_input << _data->_stateEstimator->getResult().rpy(i) << " ";
    }

 

    for (int leg = 0; leg < 2; leg++){
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].q(i) << " ";
        }
    }
    for (int leg = 0; leg < 2; leg++){
        for (int i = 0; i< 5; i++){
            corrected_angle << _data->_legController->data[leg].qd(i) << " ";
        }
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
    corrected_angle<<std::endl;
    T265_pos << std::endl;
}

void FSMState_MPCStand::exit()
{
    _data->_interface->zeroCmdPanel();
    _data->_interface->cmdPanel->setCmdNone();
}

FSMStateName FSMState_MPCStand::checkTransition()
{
    if(_lowState->userCmd == UserCommand::L2_X){ // TODO: Change this to the corrent UserCommand
        std::cout << "transition from MPC stand to QP stand" << std::endl;
        return FSMStateName::QPSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::MPCSTAND;
    }
}
