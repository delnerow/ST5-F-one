#ifndef Task_h
#define Task_h


extern void TaskOn(int task) ;
extern void TaskOff(int task) ;
extern void TaskOffAll() ;
extern bool TaskState(int task) ;
extern bool watchdogTripped ;

extern void launch_tasks() ;

// [PI] régulation de vitesse
extern void initVitesse() ;              // remise à zéro (consignes, PWM, PI, compteurs)
extern int  getVitesseMesuree(int i) ;   // vitesse filtrée de la roue i, en ticks/s

#endif
