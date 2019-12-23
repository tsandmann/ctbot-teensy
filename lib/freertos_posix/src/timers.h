#ifndef TIMERS_H
#define TIMERS_H

#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t xTimer);

void* pvTimerGetTimerID(const TimerHandle_t) {
    return nullptr;
}

BaseType_t xTimerGenericCommand(TimerHandle_t, const BaseType_t, const TickType_t, BaseType_t* const, const TickType_t) {
    return 0;
}

BaseType_t xTimerStart(TimerHandle_t xTimer, const TickType_t xTicksToWait) {
    return xTimerGenericCommand(xTimer, 1, 0, nullptr, xTicksToWait);
}

TimerHandle_t xTimerCreate(const char* const, const TickType_t, const UBaseType_t, void* const, TimerCallbackFunction_t) {
    return nullptr;
}

BaseType_t xTimerStop(TimerHandle_t xTimer, const TickType_t xTicksToWait) {
    return xTimerGenericCommand(xTimer, 3, 0U, nullptr, xTicksToWait);
}

BaseType_t xTimerDelete(TimerHandle_t xTimer, const TickType_t xTicksToWait) {
    return xTimerGenericCommand(xTimer, 5, 0U, nullptr, xTicksToWait);
}

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H */
