#ifndef __MYI2C_H
#define __MYI2C_H

void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
uint8_t Soft_I2C_WriteByte(uint8_t data);
uint8_t Soft_I2C_ReadByte(uint8_t ack);

#endif
