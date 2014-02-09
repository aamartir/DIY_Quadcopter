#ifndef I2C_H
#define I2C_H

#include <Wire.h>
#include "Quad.h"

class I2C
{
  public:
    /* Write functions */
    void mem_write(uint8 devID, uint8 reg_addr, uint8 val);
    void writeToRegister(uint8 devID, uint8 reg_addr, uint8 val);
    void writeBitToRegister(uint8 devID, uint8 reg_addr, uint8 bitNumber, bool val);
	
    /* Read functions */
    void mem_read(uint8 devID, uint8 reg_addr, uint8 bytesToRead, uint8 *buf);
    void readRegister(uint8 devID, uint8 reg_addr, uint8 *ptr);
    bool readBitInRegister(uint8 devID, uint8 reg_addr, uint8 bitNumber);
    
    /* Configuration */
};

extern I2C i2c;

#endif
