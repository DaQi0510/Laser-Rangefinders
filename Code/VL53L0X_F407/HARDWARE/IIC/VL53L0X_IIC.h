#ifndef __VL53L0X_IIC_H
#define __VL53L0X_IIC_H
#include "sys.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define VL53L0X_SCL_H  PCout(7)=1
#define VL53L0X_SCL_L  PCout(7)=0

#define VL53L0X_SDA_H  PCout(9)=1
#define VL53L0X_SDA_L  PCout(9)=0

#define VL53L0X_SDA    PCin(9)

void VL53L0X_IIC_GpioInt(void);
void VL53L0X_SDA_OUT(void);
void VL53L0X_SDA_IN(void);
void VL53L0X_SCL_OUT(void);
void VL53L0X_SCL_IN(void);
void VL53L0X_IIC_Start(void);
void VL53L0X_IIC_Stop(void);
u8 VL53L0X_IIC_Tx_Byte(u8 Data);
u8 VL53L0X_IIC_Rx_Byte(u8 Ack);
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data);
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data);
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data);
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data);
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data);
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data);
u8 VL53L0X_RByte(u8 Address);
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData);
void VL53L0X_delay(void);
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev);
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count);
void VL53L0X_Init();
VL53L0X_Error RangingInit(VL53L0X_Dev_t *pMyDevice);
u16 GetDistance(void);



#endif

