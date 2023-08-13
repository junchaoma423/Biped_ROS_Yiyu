#include "../../include/common/LegController.h"

#include <iostream>

// upper level of joint controller 
// send data to joint controller

void LegControllerCommand::zero(){
    tau = Vec5<double>::Zero();
    qDes = Vec5<double>::Zero();
    qdDes = Vec5<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec6<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat5<double>::Zero();
    kdJoint = Mat5<double>::Zero();
}

/*!
 * Zero leg data
 */ 
void LegControllerData::zero(){
    q = Vec5<double>::Zero();
    qd = Vec5<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J = Mat65<double>::Zero();
    J2 = Mat35<double>::Zero();
    tau = Vec5<double>::Zero();
}

void LegController::zeroCommand(){
    for (int i = 0; i<2; i++){
        commands[i].zero();
    }
}

void LegController::updateData(const LowlevelState* state, double* offset){
    int motor_sequence[10] = {1, 2, 9, 10, 11, 4, 5, 6, 7, 8};
    double gear_ratio = 1.5;

    std::cout << "Raw data is " << std::endl;
    for (int i = 0; i < 12; i++){
        std::cout << state->motorState[i].q << "    ";
    }
    std::cout << "\n";

    std::cout << "Checkpoint start updateData" << std::endl;
    // Joint sequence reassignment
    for (int leg = 0; leg < 2; leg++){
        for(int j = 0; j<5; j++){
            data[leg].q(j) = state->motorState[motor_sequence[j + leg*5]].q;
            data[leg].qd(j) = state->motorState[motor_sequence[j + leg*5]].dq;
            data[leg].tau(j) = state->motorState[motor_sequence[j + leg*5]].tauEst;
        }

        if (leg == 1){
            data[leg].qd(0) = -data[leg].qd(0);
            data[leg].qd(1) = -data[leg].qd(1);
            data[leg].qd(2) = -data[leg].qd(2);
        }
    }

    for (int leg = 0; leg < 2; leg++){
        for(int i = 0; i < 5; i++){
            data[leg].q(i) = data[leg].q(i) + offset[i+leg*5];
        }
        if (leg == 1){
            data[leg].q(2) = -data[leg].q(2);
            data[leg].q(1) = -data[leg].q(1);
            data[leg].q(0) = -data[leg].q(0);
        }
        data[leg].q(3) = -data[leg].q(3);


        // Implement Gear Ratio on Knee
        data[leg].q(3) = data[leg].q(3) / gear_ratio;
        data[leg].qd(3) = data[leg].qd(3) / gear_ratio;
        data[leg].tau(3) = data[leg].tau(3) * gear_ratio;

        data[leg].q(4) = (data[leg].q(4)-data[leg].q(3));

        std::cout << "leg "<< leg<< " angle data: " << std::endl;
        for(int j = 0; j<5; j++){
            std::cout << data[leg].q(j)*180/3.1415 << "    ";
        }


        computeLegJacobianAndPosition(_quadruped, data[leg].q,&(data[leg].J), &(data[leg].J2) ,&(data[leg].p),leg);

         // v
        data[leg].v = data[leg].J2 * data[leg].qd;
    }
}

void LegController::updateCommand(LowlevelCmd* cmd, double* offset, int motionTime){
    int motor_sequence[10] = {1, 2, 9, 10, 11, 4, 5, 6, 7, 8};
    double gear_ratio = 1.5;

    int zeroForceSwitch = 1;    // 0 = No input stance force
    int zeroSwingForceSwitch = 0;   // 0 = No input swing

    for (int leg = 0; leg < 2; leg++){
        computeLegJacobianAndPosition(_quadruped, data[leg].q, &(data[leg].J), &(data[leg].J2), 
        &(data[leg].p), leg);

        // tauFF
        //commands[i].tau = Vec3<double>::Zero();
        Vec5<double> legTorque = commands[leg].tau;
        //std::cout << "commmand" << commands[i].tau << std::endl;
        // forceFF

        Vec6<double> footForce = commands[leg].feedforwardForce;

        std::cout << "foot force is " << commands[leg].feedforwardForce << std::endl;
        
        legTorque = (data[leg].J.transpose() * footForce)*zeroForceSwitch;

        //   std::cout << "Leg torque "<<leg<<" is " << std::endl;
        // for (int i = 0; i < 5; i++){
        //     std::cout << legTorque(i) << "  ";
        // }
        
        Vec3<double> footForce3 = commands[leg].kpCartesian * (commands[leg].pDes - data[leg].p);
        footForce3 += commands[leg].kdCartesian * (commands[leg].vDes - data[leg].v);

        // Torque
        Vec5<double> swingtau = data[leg].J2.transpose() * footForce3;

        legTorque(0) += swingtau(0) * zeroSwingForceSwitch;
        legTorque(1) += swingtau(1) * zeroSwingForceSwitch;
        legTorque(2) += swingtau(2) * zeroSwingForceSwitch;
        legTorque(3) += swingtau(3) * zeroSwingForceSwitch;



        // for parallel control
        double toetau = commands[leg].kptoe * (-data[leg].q(3) - data[leg].q(2) - data[leg].q(4))
                + commands[leg].kdtoe * (0 - data[leg].qd(4));

        legTorque(4) += toetau;

        double kphip1 = 20;
        double kdhip1 = 1;
        double hip1tau = kphip1*(0-data[leg].q(0)) + kdhip1*(0-data[leg].qd(0));
        legTorque(0) += hip1tau;


        commands[leg].tau = legTorque;

        //PD control
        double desired_pos[5] = {0.0, 0.0, 0.785298, -1.5708, 0.785398};    // Desired Standup Pose
        double percent;

        double duration = 4000;
        percent = (double)motionTime/duration;

        if (motionTime > 4000){
            percent = 1.0;
        }

        commands[leg].tau(0) = legTorque[0] * percent;
        commands[leg].tau(1) = legTorque[1] * percent;
        commands[leg].tau(2) = legTorque[2] * percent;
        commands[leg].tau(3) = legTorque[3] * percent;
        commands[leg].tau(4) = legTorque[4] * percent;

        std::cout << "Percent is " << percent << std::endl;

        std::cout << "Leg torque "<<leg<<" is " << std::endl;
        for (int i = 0; i < 5; i++){
            std::cout << commands[leg].tau(i) << "  ";
        }


        // Reassigning motor torque sequence (Only have torque control)
        commands[leg].tau(3) = -commands[leg].tau(3) / gear_ratio;
        commands[leg].tau(4) = -commands[leg].tau(4);

        std::cout << "Leg torque command "<<leg<<" is " << std::endl;
        for (int i = 0; i < 5; i++){
            std::cout << commands[leg].tau(i) << "  ";
        }
        std::cout << "\n";


        if (leg == 1){
            commands[leg].tau(0) = -commands[leg].tau(0);
            commands[leg].tau(1) = -commands[leg].tau(1);
            commands[leg].tau(2) = -commands[leg].tau(2);
            commands[leg].tau(4) = -commands[leg].tau(4);
        }

        if (leg == 0){
            commands[leg].tau(4) = -commands[leg].tau(4);
        }

        for (int k = 0; k < 5; k++){
            cmd->motorCmd[motor_sequence[k + leg*5]].tau = commands[leg].tau(k);
            cmd->motorCmd[motor_sequence[k + leg*5]].q = commands[leg].qDes(k);
        }

        cmd->motorCmd[0].tau = 0;
        cmd->motorCmd[3].tau = 0;

        for (int i = 0; i < 12; i++){
            cmd->motorCmd[i].Kd = 0.75;
            cmd->motorCmd[0].Kp = 0;
            cmd->motorCmd[3].Kp = 0;

            // commands[i].tau << 0, 0, 0, 0, 0; // zero torque command to prevent interference
        }

        // std::cout << "This doesn't have output " << std::endl;
        // No Output (Comment out if running)
        // for (int i = 0; i < 12; i++){
        //     cmd->motorCmd[i].tau = 0;
        //     cmd->motorCmd[i].Kp = 0;
        //     cmd->motorCmd[i].Kd = 0;
        // }

    }

    for (int i = 0; i< 2; i++){
        commands[i].tau << 0,0,0,0,0;
    }
    //std::cout << "cmd sent" << std::endl;
   
}

void computeLegJacobianAndPosition(Quadruped& _quad, Vec5<double>& q, Mat65<double>* J, Mat35<double>* J2, Vec3<double>* p, int leg)
{

    double q0 = q(0);
    double q1 = q(1);
    double q2 = q(2);
    double q3 = q(3);
    double q4 = q(4);

    
    double sideSign; // 1 for Left legs; -1 for right legs
    if (leg == 1){
        // std::cout<< "Leg Sign checked" << std::endl;
        sideSign = 1.0;}
    if (leg == 0){
        // std::cout<< "Leg Sign checked" << std::endl;
        sideSign = -1.0;
    }
    // std::cout<< "Leg Sign" << sideSign << std::endl;
    

    if(J){
    J->operator()(0, 0) =  (3*sin(q0))/200 - (cos(q0)*  (sideSign))/50 + (11*sin(q0)*sin(q2))/50 - (23*cos(q0)*cos(q1)*  (sideSign))/1000 - (11*cos(q0)*cos(q2)*sin(q1))/50 + (11*cos(q2)*sin(q0)*sin(q3))/50 + (11*cos(q3)*sin(q0)*sin(q2))/50 - (11*cos(q0)*cos(q2)*cos(q3)*sin(q1))/50 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q2))/250 + (11*cos(q0)*sin(q1)*sin(q2)*sin(q3))/50 - (9*sin(q0)*sin(q2)*sin(q3)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4)*sin(q1))/250 + (9*cos(q0)*cos(q2)*sin(q1)*sin(q3)*sin(q4))/250 + (9*cos(q0)*cos(q3)*sin(q1)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q1)*sin(q2)*sin(q3))/250;
    J->operator()(1, 0) =  (9*cos(q0)*sin(q2)*sin(q3)*sin(q4))/250 - (11*cos(q0)*sin(q2))/50 - (  (sideSign)*sin(q0))/50 - (11*cos(q0)*cos(q2)*sin(q3))/50 - (11*cos(q0)*cos(q3)*sin(q2))/50 - (23*cos(q1)*  (sideSign)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q2))/250 - (11*cos(q2)*cos(q3)*sin(q0)*sin(q1))/50 - (3*cos(q0))/200 + (11*sin(q0)*sin(q1)*sin(q2)*sin(q3))/50 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0)*sin(q1))/250 + (9*cos(q2)*sin(q0)*sin(q1)*sin(q3)*sin(q4))/250 + (9*cos(q3)*sin(q0)*sin(q1)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q1)*sin(q2)*sin(q3))/250;
    J->operator()(2, 0) =  0;
    J->operator()(3, 0) = 0;
    J->operator()(4, 0) = 0;
    J->operator()(5, 0) = 1;

    J->operator()(0, 1) =  (sin(q0)*(23*  (sideSign)*sin(q1) - 220*cos(q1)*cos(q2) - 220*cos(q1)*cos(q2)*cos(q3) + 220*cos(q1)*sin(q2)*sin(q3) - 36*cos(q1)*cos(q2)*cos(q3)*cos(q4) + 36*cos(q1)*cos(q2)*sin(q3)*sin(q4) + 36*cos(q1)*cos(q3)*sin(q2)*sin(q4) + 36*cos(q1)*cos(q4)*sin(q2)*sin(q3)))/1000;
    J->operator()(1, 1) =  -(cos(q0)*(23*  (sideSign)*sin(q1) - 220*cos(q1)*cos(q2) - 220*cos(q1)*cos(q2)*cos(q3) + 220*cos(q1)*sin(q2)*sin(q3) - 36*cos(q1)*cos(q2)*cos(q3)*cos(q4) + 36*cos(q1)*cos(q2)*sin(q3)*sin(q4) + 36*cos(q1)*cos(q3)*sin(q2)*sin(q4) + 36*cos(q1)*cos(q4)*sin(q2)*sin(q3)))/1000;
    J->operator()(2, 1) =  (23*cos(q1)*  (sideSign))/1000 + (11*cos(q2)*sin(q1))/50 - (11*sin(q1)*sin(q2)*sin(q3))/50 + (11*cos(q2)*cos(q3)*sin(q1))/50 + (9*cos(q2)*cos(q3)*cos(q4)*sin(q1))/250 - (9*cos(q2)*sin(q1)*sin(q3)*sin(q4))/250 - (9*cos(q3)*sin(q1)*sin(q2)*sin(q4))/250 - (9*cos(q4)*sin(q1)*sin(q2)*sin(q3))/250;
    J->operator()(3, 1) = 1;
    J->operator()(4, 1) = 0;
    J->operator()(5, 1) = 0;

    J->operator()(0, 2) =  (11*sin(q0)*sin(q1)*sin(q2))/50 - (11*cos(q0)*cos(q2))/50 - (11*cos(q0)*cos(q2)*cos(q3))/50 + (11*cos(q0)*sin(q2)*sin(q3))/50 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4))/250 + (9*cos(q0)*cos(q2)*sin(q3)*sin(q4))/250 + (9*cos(q0)*cos(q3)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q2)*sin(q3))/250 + (11*cos(q2)*sin(q0)*sin(q1)*sin(q3))/50 + (11*cos(q3)*sin(q0)*sin(q1)*sin(q2))/50 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q1)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q1)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q1)*sin(q2))/250 - (9*sin(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J->operator()(1, 2) =  (11*sin(q0)*sin(q2)*sin(q3))/50 - (11*cos(q2)*sin(q0))/50 - (11*cos(q2)*cos(q3)*sin(q0))/50 - (11*cos(q0)*sin(q1)*sin(q2))/50 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0))/250 - (11*cos(q0)*cos(q2)*sin(q1)*sin(q3))/50 - (11*cos(q0)*cos(q3)*sin(q1)*sin(q2))/50 + (9*cos(q2)*sin(q0)*sin(q3)*sin(q4))/250 + (9*cos(q3)*sin(q0)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q2)*sin(q3))/250 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q1)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q1)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q1)*sin(q2))/250 + (9*cos(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J->operator()(2, 2) =  (cos(q1)*(9*sin(q2 + q3 + q4) + 55*sin(q2 + q3) + 55*sin(q2)))/250;
    J->operator()(3, 2) = 0;
    J->operator()(4, 2) = 1;
    J->operator()(5, 2) = 0;

    J->operator()(0, 3) =  (11*cos(q0)*sin(q2)*sin(q3))/50 - (11*cos(q0)*cos(q2)*cos(q3))/50 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4))/250 + (9*cos(q0)*cos(q2)*sin(q3)*sin(q4))/250 + (9*cos(q0)*cos(q3)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q2)*sin(q3))/250 + (11*cos(q2)*sin(q0)*sin(q1)*sin(q3))/50 + (11*cos(q3)*sin(q0)*sin(q1)*sin(q2))/50 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q1)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q1)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q1)*sin(q2))/250 - (9*sin(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J->operator()(1, 3) =  (11*sin(q0)*sin(q2)*sin(q3))/50 - (11*cos(q2)*cos(q3)*sin(q0))/50 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0))/250 - (11*cos(q0)*cos(q2)*sin(q1)*sin(q3))/50 - (11*cos(q0)*cos(q3)*sin(q1)*sin(q2))/50 + (9*cos(q2)*sin(q0)*sin(q3)*sin(q4))/250 + (9*cos(q3)*sin(q0)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q2)*sin(q3))/250 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q1)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q1)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q1)*sin(q2))/250 + (9*cos(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J->operator()(2, 3) =  (11*sin(q1 + q2 + q3))/100 + (11*sin(q2 - q1 + q3))/100 + (9*sin(q1 + q2 + q3 + q4))/500 + (9*sin(q2 - q1 + q3 + q4))/500;
    J->operator()(3, 3) = 0;
    J->operator()(4, 3) = 1;
    J->operator()(5, 3) = 0;

    J->operator()(0, 4) =  (9*cos(q0)*cos(q2)*sin(q3)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4))/250 + (9*cos(q0)*cos(q3)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q2)*sin(q3))/250 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q1)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q1)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q1)*sin(q2))/250 - (9*sin(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J->operator()(1, 4) =  (9*cos(q2)*sin(q0)*sin(q3)*sin(q4))/250 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0))/250 + (9*cos(q3)*sin(q0)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q2)*sin(q3))/250 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q1)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q1)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q1)*sin(q2))/250 + (9*cos(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J->operator()(2, 4) =  (9*sin(q1 + q2 + q3 + q4))/500 + (9*sin(q2 - q1 + q3 + q4))/500;
    J->operator()(3, 4) = 0;
    J->operator()(4, 4) = 1;
    J->operator()(5, 4) = 0;
   }

   if(J2){
    J2->operator()(0, 0) =  (3*sin(q0))/200 - (cos(q0)*  (sideSign))/50 + (11*sin(q0)*sin(q2))/50 - (23*cos(q0)*cos(q1)*  (sideSign))/1000 - (11*cos(q0)*cos(q2)*sin(q1))/50 + (11*cos(q2)*sin(q0)*sin(q3))/50 + (11*cos(q3)*sin(q0)*sin(q2))/50 - (11*cos(q0)*cos(q2)*cos(q3)*sin(q1))/50 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q2))/250 + (11*cos(q0)*sin(q1)*sin(q2)*sin(q3))/50 - (9*sin(q0)*sin(q2)*sin(q3)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4)*sin(q1))/250 + (9*cos(q0)*cos(q2)*sin(q1)*sin(q3)*sin(q4))/250 + (9*cos(q0)*cos(q3)*sin(q1)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q1)*sin(q2)*sin(q3))/250;
    J2->operator()(1, 0) =  (9*cos(q0)*sin(q2)*sin(q3)*sin(q4))/250 - (11*cos(q0)*sin(q2))/50 - (  (sideSign)*sin(q0))/50 - (11*cos(q0)*cos(q2)*sin(q3))/50 - (11*cos(q0)*cos(q3)*sin(q2))/50 - (23*cos(q1)*  (sideSign)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q2))/250 - (11*cos(q2)*cos(q3)*sin(q0)*sin(q1))/50 - (3*cos(q0))/200 + (11*sin(q0)*sin(q1)*sin(q2)*sin(q3))/50 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0)*sin(q1))/250 + (9*cos(q2)*sin(q0)*sin(q1)*sin(q3)*sin(q4))/250 + (9*cos(q3)*sin(q0)*sin(q1)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q1)*sin(q2)*sin(q3))/250;
    J2->operator()(2, 0) =  0;

    J2->operator()(0, 1) =  (sin(q0)*(23*  (sideSign)*sin(q1) - 220*cos(q1)*cos(q2) - 220*cos(q1)*cos(q2)*cos(q3) + 220*cos(q1)*sin(q2)*sin(q3) - 36*cos(q1)*cos(q2)*cos(q3)*cos(q4) + 36*cos(q1)*cos(q2)*sin(q3)*sin(q4) + 36*cos(q1)*cos(q3)*sin(q2)*sin(q4) + 36*cos(q1)*cos(q4)*sin(q2)*sin(q3)))/1000;
    J2->operator()(1, 1) =  -(cos(q0)*(23*  (sideSign)*sin(q1) - 220*cos(q1)*cos(q2) - 220*cos(q1)*cos(q2)*cos(q3) + 220*cos(q1)*sin(q2)*sin(q3) - 36*cos(q1)*cos(q2)*cos(q3)*cos(q4) + 36*cos(q1)*cos(q2)*sin(q3)*sin(q4) + 36*cos(q1)*cos(q3)*sin(q2)*sin(q4) + 36*cos(q1)*cos(q4)*sin(q2)*sin(q3)))/1000;
    J2->operator()(2, 1) =  (23*cos(q1)*  (sideSign))/1000 + (11*cos(q2)*sin(q1))/50 - (11*sin(q1)*sin(q2)*sin(q3))/50 + (11*cos(q2)*cos(q3)*sin(q1))/50 + (9*cos(q2)*cos(q3)*cos(q4)*sin(q1))/250 - (9*cos(q2)*sin(q1)*sin(q3)*sin(q4))/250 - (9*cos(q3)*sin(q1)*sin(q2)*sin(q4))/250 - (9*cos(q4)*sin(q1)*sin(q2)*sin(q3))/250;

    J2->operator()(0, 2) =  (11*sin(q0)*sin(q1)*sin(q2))/50 - (11*cos(q0)*cos(q2))/50 - (11*cos(q0)*cos(q2)*cos(q3))/50 + (11*cos(q0)*sin(q2)*sin(q3))/50 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4))/250 + (9*cos(q0)*cos(q2)*sin(q3)*sin(q4))/250 + (9*cos(q0)*cos(q3)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q2)*sin(q3))/250 + (11*cos(q2)*sin(q0)*sin(q1)*sin(q3))/50 + (11*cos(q3)*sin(q0)*sin(q1)*sin(q2))/50 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q1)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q1)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q1)*sin(q2))/250 - (9*sin(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J2->operator()(1, 2) =  (11*sin(q0)*sin(q2)*sin(q3))/50 - (11*cos(q2)*sin(q0))/50 - (11*cos(q2)*cos(q3)*sin(q0))/50 - (11*cos(q0)*sin(q1)*sin(q2))/50 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0))/250 - (11*cos(q0)*cos(q2)*sin(q1)*sin(q3))/50 - (11*cos(q0)*cos(q3)*sin(q1)*sin(q2))/50 + (9*cos(q2)*sin(q0)*sin(q3)*sin(q4))/250 + (9*cos(q3)*sin(q0)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q2)*sin(q3))/250 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q1)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q1)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q1)*sin(q2))/250 + (9*cos(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J2->operator()(2, 2) =  (cos(q1)*(9*sin(q2 + q3 + q4) + 55*sin(q2 + q3) + 55*sin(q2)))/250;

    J2->operator()(0, 3) =  (11*cos(q0)*sin(q2)*sin(q3))/50 - (11*cos(q0)*cos(q2)*cos(q3))/50 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4))/250 + (9*cos(q0)*cos(q2)*sin(q3)*sin(q4))/250 + (9*cos(q0)*cos(q3)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q2)*sin(q3))/250 + (11*cos(q2)*sin(q0)*sin(q1)*sin(q3))/50 + (11*cos(q3)*sin(q0)*sin(q1)*sin(q2))/50 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q1)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q1)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q1)*sin(q2))/250 - (9*sin(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J2->operator()(1, 3) =  (11*sin(q0)*sin(q2)*sin(q3))/50 - (11*cos(q2)*cos(q3)*sin(q0))/50 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0))/250 - (11*cos(q0)*cos(q2)*sin(q1)*sin(q3))/50 - (11*cos(q0)*cos(q3)*sin(q1)*sin(q2))/50 + (9*cos(q2)*sin(q0)*sin(q3)*sin(q4))/250 + (9*cos(q3)*sin(q0)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q2)*sin(q3))/250 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q1)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q1)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q1)*sin(q2))/250 + (9*cos(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J2->operator()(2, 3) =  (11*sin(q1 + q2 + q3))/100 + (11*sin(q2 - q1 + q3))/100 + (9*sin(q1 + q2 + q3 + q4))/500 + (9*sin(q2 - q1 + q3 + q4))/500;

    J2->operator()(0, 4) =  (9*cos(q0)*cos(q2)*sin(q3)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q3)*cos(q4))/250 + (9*cos(q0)*cos(q3)*sin(q2)*sin(q4))/250 + (9*cos(q0)*cos(q4)*sin(q2)*sin(q3))/250 + (9*cos(q2)*cos(q3)*sin(q0)*sin(q1)*sin(q4))/250 + (9*cos(q2)*cos(q4)*sin(q0)*sin(q1)*sin(q3))/250 + (9*cos(q3)*cos(q4)*sin(q0)*sin(q1)*sin(q2))/250 - (9*sin(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J2->operator()(1, 4) =  (9*cos(q2)*sin(q0)*sin(q3)*sin(q4))/250 - (9*cos(q2)*cos(q3)*cos(q4)*sin(q0))/250 + (9*cos(q3)*sin(q0)*sin(q2)*sin(q4))/250 + (9*cos(q4)*sin(q0)*sin(q2)*sin(q3))/250 - (9*cos(q0)*cos(q2)*cos(q3)*sin(q1)*sin(q4))/250 - (9*cos(q0)*cos(q2)*cos(q4)*sin(q1)*sin(q3))/250 - (9*cos(q0)*cos(q3)*cos(q4)*sin(q1)*sin(q2))/250 + (9*cos(q0)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/250;
    J2->operator()(2, 4) =  (9*sin(q1 + q2 + q3 + q4))/500 + (9*sin(q2 - q1 + q3 + q4))/500;
    
    // J2->operator()(0, 4) = 0;
    // J2->operator()(1, 4) = 0;
    // J2->operator()(2, 4) = 0;
    }

   if(p){
    p->operator()(0) = - (3*cos(q0))/200 - (9*sin(q4)*(cos(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)) - sin(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1))))/250 - (11*cos(q0)*sin(q2))/50 - ( (sideSign)*sin(q0))/50 - (11*cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)))/50 - (11*sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(cos(q0)*sin(q2) + cos(q2)*sin(q0)*sin(q1)) + sin(q3)*(cos(q0)*cos(q2) - sin(q0)*sin(q1)*sin(q2))))/250 - (23*cos(q1)* (sideSign)*sin(q0))/1000 - (11*cos(q2)*sin(q0)*sin(q1))/50;
    p->operator()(1) = (cos(q0)* (sideSign))/50 - (9*sin(q4)*(cos(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)) - sin(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1))))/250 - (3*sin(q0))/200 - (11*sin(q0)*sin(q2))/50 - (11*cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)))/50 - (11*sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2)))/50 - (9*cos(q4)*(cos(q3)*(sin(q0)*sin(q2) - cos(q0)*cos(q2)*sin(q1)) + sin(q3)*(cos(q2)*sin(q0) + cos(q0)*sin(q1)*sin(q2))))/250 + (23*cos(q0)*cos(q1)* (sideSign))/1000 + (11*cos(q0)*cos(q2)*sin(q1))/50;
    p->operator()(2) = (23*(sideSign)*sin(q1))/1000 - (11*cos(q1)*cos(q2))/50 - (9*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/250 + (9*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/250 - (11*cos(q1)*cos(q2)*cos(q3))/50 + (11*cos(q1)*sin(q2)*sin(q3))/50 - 3.0/50.0;
   }
  
}

// double q1_ik(double py, double pz, double l1)
// {
//     double q1;
//     double L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
//     q1 = atan2(pz*l1+py*L, py*l1-pz*L);

//     return q1;
// }

// double q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z){
//     double q2, a1, a2, m1, m2;
    
//     a1 = py*sin(q1) - pz*cos(q1);
//     a2 = px;
//     m1 = b4z*sin(q3);
//     m2 = b3z + b4z*cos(q3);
//     q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
//     return q2;
// }

// double q3_ik(double b3z, double b4z, double b){
//     double q3, temp;
//     temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
//     if(temp>1) temp = 1;
//     if(temp<-1) temp = -1;
//     q3 = acos(temp);
//     q3 = -(M_PI - q3); //0~180
//     return q3;
// }

// void computeInverseKinematics(Quadruped& _quad, Vec3<double>& pDes, int leg, Vec3<double>* qDes)
// {
//     double l1 = _quad.hipLinkLength; // ab_ad
//     double l2 = _quad.thighLinkLength;
//     double l3 = _quad.calfLinkLength;
    
//     double q1, q2, q3;
//     double b2y, b3z, b4z, a, b, c;
//     int sideSign = 1; // 1 for Left legs; -1 for right legs
//     if (leg == 0 || leg == 2){
//         sideSign = -1;
//     }

//     b2y = l1 * sideSign;
//     b3z = -l2;
//     b4z = -l3;
//     a = l1;
//     c = sqrt(pow(pDes(0), 2) + pow(pDes(1), 2) + pow(pDes(2), 2)); // whole length
//     b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

//     q1 = q1_ik(pDes(1), pDes(2), b2y);
//     q3 = q3_ik(b3z, b4z, b);
//     q2 = q2_ik(q1, q3, pDes(0), pDes(1), pDes(2), b3z, b4z);

//     qDes->operator()(0) = q1;
//     qDes->operator()(1) = q2;
//     qDes->operator()(2) = q3;
// }

