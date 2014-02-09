#include "I2C.h"

I2C i2c = I2C();

void I2C::mem_write(uint8 devID, uint8 reg_addr, uint8 val) 
{
  Wire.beginTransmission(devID);  // start transmission to device 
  Wire.write(reg_addr);		  // send register address
  Wire.write(val);		  // send value to write
  Wire.endTransmission();	  // end transmission
}

void I2C::writeToRegister(uint8 devID, uint8 reg_addr, uint8 val)
{
  mem_write(devID, reg_addr, val) ;
}

void I2C::mem_read(uint8 devID, uint8 reg_addr, uint8 bytesToRead, uint8 *buf)
{
  Wire.beginTransmission(devID);  // start transmission to device 
  Wire.write(reg_addr);			  // send register address
  Wire.endTransmission();

  Wire.beginTransmission(devID);
  Wire.requestFrom(devID, bytesToRead);

  int i = 0;
  while(Wire.available())
  {
    buf[i++] = Wire.read();
  }
  
  Wire.endTransmission();
}

/* Results of the read are stored in ptr */
void I2C::readRegister(uint8 devID, uint8 reg_addr, uint8 *ptr)
{
  mem_read(devID, reg_addr, 1, ptr);
}

/* bitNumber is from 0 to N-1, right to left. That is, bit 0 is the 
 * least significant bit. bit 1 the 2nd least significant, and so on. 
 */
bool I2C::readBitInRegister(uint8 devID, uint8 reg_addr, uint8 bitNumber)
{
  uint8 byte_ptr;
  readRegister(devID, reg_addr, &byte_ptr); /* Populate byte_ptr with the contents of reg_addr */
  return ((byte_ptr >> bitNumber) & 1);
}

void I2C::writeBitToRegister(uint8 devID, uint8 reg_addr, uint8 bitNumber, bool val)
{
  uint8 byte_ptr;
  readRegister(devID, reg_addr, &byte_ptr); /* First we read the contents of reg_addr */

  /* Modify the bit in the register */
  if(val)
    byte_ptr |= (val << bitNumber);
  else
    byte_ptr &= ~(val << bitNumber);

  mem_write(devID, reg_addr, byte_ptr);	/* Write new value to reg_addr */
}
