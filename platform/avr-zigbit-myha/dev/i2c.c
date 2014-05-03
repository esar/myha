#include <contiki.h>
#include <dev/i2c.h>

void i2c_enable()
{
  // Set SCL to 400kHz
  TWSR = 0x00;
  TWBR = 0x0C;

  // Enable TWI
  TWCR = 1 << TWEN;
}

void i2c_disable()
{
  TWCR &= ~(1 << TWEN);
}

void i2c_start()
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while((TWCR & (1 << TWINT)) == 0)
    ;
}

void i2c_stop()
{
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
  // TODO: Wait?
}

int i2c_write(uint8_t value)
{
  TWDR = value;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while((TWCR & (1 << TWINT)) == 0)
    ;
  if((TWSR & 0xF8) == 0x40)
    return 1;
  else
    return 0;
}

uint8_t i2c_read(int ack)
{
  if(ack)
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  else
    TWCR = (1 << TWINT) | (1 << TWEN); 
  while((TWCR & (1 << TWINT)) == 0)
    ;
  return TWDR;
}
