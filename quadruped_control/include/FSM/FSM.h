#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_QPStand.h"
#include "FSMState_MPCStand.h"
#include "FSMState_Passive.h"
#include "../common/enumClass.h"
#include "../common/PoseData.h"

struct FSMStateList{
    FSMState *invalid;
    FSMState_QPStand *qpstand;
    FSMState_MPCStand *mpcstand;
    FSMState_Passive *passive;
   
    void deletePtr(){
        delete qpstand;
        delete mpcstand;
        delete passive;
    }  
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();

        Mat62<double> footFeedForwardForces;    // feedforward forces at the feet
        Mat32<double> hiptoeforces;

        // PoseData pd;
        // rs2::pipeline pipe;
        // rs2::config cfg;
    private:
        FSMState* getNextState(FSMStateName stateName);
        bool checkSafty();
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;

        UNITREE_LEGGED_SDK::LowCmd lowCmd;
        UNITREE_LEGGED_SDK::LowState lowState;
        xRockerBtnDataStruct _gamepad;

        // double *Angle_Caliberation();
        // double *rpy_Caliberation();
};

#endif