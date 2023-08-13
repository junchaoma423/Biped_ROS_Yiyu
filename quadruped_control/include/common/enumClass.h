#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO_A1,
    REAL_A1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // walking
    L2_B,       // passive
    L2_X,       // QP stand
    L2_A,       // MPC stand  
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    QPSTAND,
    MPCSTAND,
    PASSIVE,
    INVALID,
};


#endif  // ENUMCLASS_H