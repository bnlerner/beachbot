#ifndef IPC_MESSAGES_H
#define IPC_MESSAGES_H

#include "core.h"

struct TestMessage : BaseMessage {
    double value;
    bool passed;
};

#endif // IPC_MESSAGES_H
