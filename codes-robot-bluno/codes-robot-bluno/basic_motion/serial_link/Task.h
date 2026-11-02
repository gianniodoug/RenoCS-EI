#ifndef Task_h
#define Task_h


extern void TaskOn(int task) ;
extern void TaskOff(int task) ;
extern void TaskOffAll() ;
extern bool TaskState(int task) ;

extern void launch_tasks() ;

#endif