/*!
 * @file LegController.h
 * @brief Comman Leg Control Interface
 * 
 * Implement low-level leg control for Aliengo Robot
 * Leg 0: FR; Leg 1: FL;
 * Leg 2: RR ; Leg 3: RL;
 */ 

#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "cppTypes.h"
#include "../messages/LowlevelState.h"
#include "../messages/LowLevelCmd.h"
#include "Quadruped.h"

/*!
 * Data sent from control algorithm to legs
 */ 
    struct LegControllerCommand{
        LegControllerCommand() {zero();}

        void zero();

        Vec5<double> qDes, qdDes, tau;
        Vec3<double> pDes, vDes;
        Mat5<double> kpJoint, kdJoint;
        Vec6<double> feedforwardForce;
        Vec3<double> hiptoeforce;
        Mat3<double> kpCartesian;
        Mat3<double> kdCartesian;
        double kptoe;
        double kdtoe;
    };

/*!
 * Data returned from legs to control code
 */ 
    struct LegControllerData{
        LegControllerData() {zero();}
        void setQuadruped(Quadruped& quad) { A1 = &quad; }

        void zero();
        Vec5<double> q, qd, qDes;
        Vec3<double> p, v, pDes;
        Mat65<double> J;
        Mat35<double> J2;
        Vec5<double> tau;
        Quadruped* A1;
    };

/*!
 * Controller for 4 legs of quadruped
 */ 
    class LegController {
      public:
        LegController(Quadruped& quad) : _quadruped(quad) {
            for (auto& dat : data) dat.setQuadruped(_quadruped);
            for(int i = 0; i<2; i++){
                commands[i].zero();
                data[i].zero();
            //    commands[i].kdCartesian << 100, 0, 0, 0, 100, 0, 0 ,0, 100;
            //    commands[i].kpCartesian << 100, 0, 0, 0, 100, 0, 0 ,0, 100;
            }
        };
        
        void stand();
        void zeroCommand();
        void edampCommand(double gain);
        void updateData(const LowlevelState* state, double* offset);
        void updateCommand(LowlevelCmd* cmd, double* offset, int motionTime);
        void setEnabled(bool enabled) {_legsEnabled = enabled;};    // Don't know what does this do


        LegControllerCommand commands[2];
        LegControllerData data[2];
        bool _legsEnabled = false;
        /*!
        * compute foot jacobian and position in leg frame
        */ 
        Quadruped& _quadruped;
        //CurrentState& curr;
        //ros::NodeHandle n;
    };

    void computeLegJacobianAndPosition(Quadruped& _quad, Vec5<double>& q, Mat65<double>* J, Mat35<double>* J2,
                                       Vec3<double>* p, int leg);

    // void computeInverseKinematics(Quadruped& _quad, Vec3<double>& pDes, int leg, Vec3<double>* qDes);

#endif