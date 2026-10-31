#ifndef DRIVERS_I2C_RECOVERY_H
#define DRIVERS_I2C_RECOVERY_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Performs a 9-clock I2C bus recovery sequence on I2C3 (PA8=SCL, PB8=SDA).
 *
 * If a slave device (e.g. BNO085) hung while transmitting and is holding SDA low,
 * clocking SCL up to 9 times forces the slave to complete the byte transfer and
 * release SDA. A manual STOP condition is then generated and the hardware I2C3
 * peripheral is re-initialized.
 *
 * @return true if SDA was successfully released (high), false if line remained held low.
 */
bool i2c_bus_recovery_perform(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_I2C_RECOVERY_H */
