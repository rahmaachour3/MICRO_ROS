#pragma once

#include <cstdint>

enum class CommandType : int32_t
{
    FORWARD = 1,
    BACKWARD = 2,
    ROTATE = 3,
    PIGNON = 4,
    STOP = 5,
   
};

enum class CommandState : int32_t
{
    FAILED = -1,
    ACCEPTED = 1,
    COMPLETED = 2
};

enum class IdleState : int32_t
{

    FAULT = -1,
    BUSY = 0,
    READY = 1
};

struct RobotCommand
{
    int32_t id;
    CommandType type;
    int32_t amount;
};