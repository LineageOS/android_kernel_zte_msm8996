#ifndef _NDT_FORCE_TOUCH_H
#define _NDT_FORCE_TOUCH_H

int ndt_stress_resume(void);
int ndt_stress_suspend(void);
int ndt_stress_set_vlaue(unsigned short x1,  unsigned short y1, unsigned char status);
int ndt_stress_get_vlaue(unsigned int *value);
int ndt_get_max_coord(unsigned short max_x,  unsigned short max_y);
void ndt_i2c_comm_lock(void);
void ndt_i2c_comm_unlock(void);

#endif