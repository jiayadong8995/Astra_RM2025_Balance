#ifndef __PSTWO_TASK_H
#define __PSTWO_TASK_H

#include "main.h"
#include "chassisR_task.h"
#include "remote_control.h"

extern void PS2_data_process(RC_ctrl_t *rc_ctrl,chassis_t *chassis,float dt);

extern void pstwo_task(void);
	



#endif



