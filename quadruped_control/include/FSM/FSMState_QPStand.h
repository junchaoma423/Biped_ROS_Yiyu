#ifndef QPSTAND_H
#define QPSTAND_H

#include "FSMState.h"
#include <fstream>
// #include "../../ConvexMPC/ConvexMPCLocomotion.h""
#include "../../BalanceController/BalanceController.hpp"

class FSMState_QPStand: public FSMState
{
    public:
        FSMState_QPStand(ControlFSMData *data);
        ~FSMState_QPStand(){}
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();

    private:
        BalanceController balanceController;

        // QP Data
        double init_yaw;
        double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
        Mat62<double> footFeedForwardForces = Mat62<double>::Zero();    // feedforward forces at the feet
        double minForce = 1;
        double maxForce = 150;
        double contactStateScheduled[2] = {1, 1};
        double minForces[2] = {minForce, minForce};
        double maxForces[2] = {maxForce, maxForce};

        double COM_weights_stance[3] = {1, 1, 1};
        double Base_weights_stance[3] = {1, 1, 1};
        double pFeet[6] = {0};
        double p_act[3] = {0};
        double v_act[3] = {0};
        double O_err[3] = {0};
        double rpy[3] = {0};
        double v_des[3] = {0};
        double p_des[3] = {0.0, 0.0, 0.0};
        double omegaDes[3] = {0};
        double se_xfb[13] = {0};
        double b_control[6] = {0};

        Vec3<double> pFeetVecCOM;
        double fOpt[12];
        Vec3<double> fOptR;
        Vec3<double> fOptO;
        Vec3<double> mOptR;
        Vec3<double> mOptO;

};

#endif