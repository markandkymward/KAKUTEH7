#ifndef DSHOT_H
#define DSHOT_H

#include <stdint.h>

uint8_t dshot_init(void);
void dshot_write_motor(uint8_t motor_index, uint16_t value);
void dshot_send_idle(void);
void dshot_stop_all(void);

uint8_t dshot_is_ready(void);
uint8_t dshot_dma_running(void);
uint16_t dshot_last_value(uint8_t motor_index);

#endif /* DSHOT_H */
