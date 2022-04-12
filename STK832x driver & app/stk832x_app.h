#ifndef _STK832X_APP_H
#define _STK832X_APP_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define INT1_IO 28

void STK832x_Init(void);
void read_data_3dh(void);

void STK832x_stop(void);
void STK832x_start(void);

void task_read_3dh(void);

#endif
