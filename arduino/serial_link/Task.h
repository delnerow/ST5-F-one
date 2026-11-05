#ifndef Task_h
#define Task_h


extern void TaskOn(int task) ;
extern void TaskOff(int task) ;
extern void TaskOffAll() ;
extern bool TaskState(int task) ;
extern bool watchdogTripped ;

extern void launch_tasks() ;

#endif