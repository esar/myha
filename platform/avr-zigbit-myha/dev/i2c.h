#ifndef I2C_H
#define I2C_H

void i2c_enable();
void i2c_disable();
void i2c_start();
void i2c_stop();
int i2c_write(uint8_t value);
uint8_t i2c_read(int ack);

#endif /* I2C_H */
