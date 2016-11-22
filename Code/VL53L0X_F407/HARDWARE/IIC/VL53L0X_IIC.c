#include "VL53L0X_IIC.h" 
extern VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
extern VL53L0X_Error Status;
extern VL53L0X_Dev_t MyDevice;
extern VL53L0X_Dev_t *pMyDevice ;
extern VL53L0X_Version_t                   Version;
extern VL53L0X_Version_t                  *pVersion ;
extern VL53L0X_DeviceInfo_t                DeviceInfo;
#define USE_I2C_2V8

//IIC引脚初始化
void VL53L0X_IIC_GpioInt(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
}
//SDA信号线输出模式
void VL53L0X_SDA_OUT(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOC, &GPIO_InitStructure);	
}
//SDA信号线输入模式
void VL53L0X_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);	
}
//SCL信号线输出模式
void VL53L0X_SCL_OUT(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
   GPIO_Init(GPIOC, &GPIO_InitStructure);	
}
//SCL信号线输入模式
void VL53L0X_SCL_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);	
}
//开始位
void VL53L0X_IIC_Start(void)
{
	VL53L0X_SDA_OUT();
	VL53L0X_SCL_OUT();
	VL53L0X_SDA_H;
	VL53L0X_SCL_H;
	VL53L0X_delay();
	VL53L0X_SDA_L;
	VL53L0X_delay();
	VL53L0X_delay();
	VL53L0X_SCL_L;
	VL53L0X_delay();
}
//停止位
void VL53L0X_IIC_Stop(void)
{
	VL53L0X_SDA_L;
	VL53L0X_SCL_L;
	VL53L0X_delay();
	VL53L0X_SCL_H;
	VL53L0X_delay();
	VL53L0X_SDA_H;
	VL53L0X_delay();
}
//写入一个字节
u8 VL53L0X_IIC_Tx_Byte(u8 Data)
{
	u8 i,dat; 
	u8 Ack;
	dat=Data;
	for(i=0;i<8;i++)
	{
		if(dat&0x80)
			VL53L0X_SDA_H;
		else
			VL53L0X_SDA_L;
		VL53L0X_delay();
		VL53L0X_SCL_H;
		VL53L0X_delay();
		VL53L0X_SCL_L;
		VL53L0X_delay();
		dat=dat<<1;
	}
	VL53L0X_SDA_IN();
	VL53L0X_SCL_H;
	VL53L0X_delay();
	Ack=VL53L0X_SDA;
	VL53L0X_SCL_L;
	VL53L0X_delay();
	VL53L0X_SDA_OUT();
	return Ack;
}
//读取一个字节
u8 VL53L0X_IIC_Rx_Byte(u8 Ack)
{
	u8 i,Data=0;
	VL53L0X_SDA_IN();
	for(i=0;i<8;i++)
	{
		Data=Data<<1;
		VL53L0X_SCL_H;
		VL53L0X_delay();
		if(VL53L0X_SDA==1)
			Data=Data+1;
		VL53L0X_SCL_L;
		VL53L0X_delay();
	}
	VL53L0X_SDA_OUT();
	if(Ack==1)
		VL53L0X_SDA_H;
	else
		VL53L0X_SDA_L;
	VL53L0X_delay();
	VL53L0X_SCL_H;
	VL53L0X_delay();
	VL53L0X_SCL_L;
	VL53L0X_delay();
  return Data;
}
//向寄存器写入一个数据
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_IIC_Start();
	VL53L0X_IIC_Tx_Byte(0x52);
	VL53L0X_IIC_Tx_Byte(index);
	VL53L0X_IIC_Tx_Byte(data);
	VL53L0X_IIC_Stop();
	return Status;
}
/**
 * Write word register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      16 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
*/
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	u8 DataTem;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	DataTem=(data&0xff00)>>8;
	VL53L0X_WrByte(Dev,index,DataTem);
	index=index+1;
	DataTem=(data&0xff);
	VL53L0X_WrByte(Dev,index,DataTem);
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x52);
//	VL53L0X_IIC_Tx_Byte(index);
//	DataTem=(data&0xff00)>>8;
//	VL53L0X_IIC_Tx_Byte(DataTem);
//	DataTem=(data&0xff);
//	VL53L0X_IIC_Tx_Byte(DataTem);
	VL53L0X_IIC_Stop();
	return Status;
}
/**
 * Write double word (4 byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      32 bit register data
 * @return  VL53L0X_ERROR_NONE        Success
*/
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	u8 DataTem;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	DataTem=(data&0xff000000)>>24;
	VL53L0X_WrByte(Dev,index,DataTem);
	index=index+1;
	DataTem=(data&0xff0000)>>16;
	VL53L0X_WrByte(Dev,index,DataTem);
	index=index+1;
	DataTem=(data&0xff00)>>8;
	VL53L0X_WrByte(Dev,index,DataTem);
	index=index+1;
	DataTem=data&0xff;
	VL53L0X_WrByte(Dev,index,DataTem);
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x52);
//	VL53L0X_IIC_Tx_Byte(index);
//	DataTem=(data&0xff000000)>>24;
//	VL53L0X_IIC_Tx_Byte(DataTem);
//	DataTem=(data&0xff0000)>>16;
//	VL53L0X_IIC_Tx_Byte(DataTem);
//	DataTem=(data&0xff00)>>8;
//	VL53L0X_IIC_Tx_Byte(DataTem);
//	DataTem=data&0xff;
//	VL53L0X_IIC_Tx_Byte(DataTem);
//	VL53L0X_IIC_Stop();
	return Status;
}
//向寄存器读取一个数据
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_IIC_Start();
	VL53L0X_IIC_Tx_Byte(0x52);
	VL53L0X_IIC_Tx_Byte(index);
	VL53L0X_IIC_Stop();
	VL53L0X_IIC_Start();
	VL53L0X_IIC_Tx_Byte(0x53);
	*data=VL53L0X_IIC_Rx_Byte(1);
	VL53L0X_IIC_Stop();
	return Status;
}


/**
 * Read word (2byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 16 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	u8 DataTem[2],i;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	for(i=0;i<2;i++)
	{
		VL53L0X_RdByte(Dev,index,&DataTem[i]);
		index++;
	}
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x52);
//	VL53L0X_IIC_Tx_Byte(index);
//	VL53L0X_IIC_Stop();
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x53);
//	DataTem[0]=VL53L0X_IIC_Rx_Byte(1);
//	DataTem[1]=VL53L0X_IIC_Rx_Byte(1);
	*data=(((u16)DataTem[0])<<8)+DataTem[1];
//	VL53L0X_IIC_Stop();
	return Status;
}
/**
 * Read dword (4byte) register
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   data      pointer to 32 bit data
 * @return  VL53L0X_ERROR_NONE        Success
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	u8 DataTem[4],i;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	for(i=0;i<4;i++)
	{
		VL53L0X_RdByte(Dev,index,&DataTem[i]);
		index++;
	}
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x52);
//	VL53L0X_IIC_Tx_Byte(index);
//	VL53L0X_IIC_Stop();
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x53);
//	DataTem[0]=VL53L0X_IIC_Rx_Byte(1);
//	DataTem[1]=VL53L0X_IIC_Rx_Byte(1);
//	DataTem[2]=VL53L0X_IIC_Rx_Byte(1);
//	DataTem[3]=VL53L0X_IIC_Rx_Byte(1);
	*data=(((u32)DataTem[0])<<24)+(((u32)DataTem[1])<<16)+(((u32)DataTem[2])<<8)+DataTem[3];
//	VL53L0X_IIC_Stop();
	return Status;
}
/**
 * Threat safe Update (read/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param   Dev        Device Handle
 * @param   index      The register index
 * @param   AndData    8 bit and data
 * @param   OrData     8 bit or data
 * @return  VL53L0X_ERROR_NONE        Success
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	u8 data;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_RdByte(Dev, index, &data);
	data = (data & AndData) | OrData;
	VL53L0X_WrByte(Dev,index,data);
	return Status;
}
//延时
void VL53L0X_delay(void)
{
	u8 i;
	for(i=0;i<250;i++);
}
/**
 * @brief execute delay in all polling API call
 *
 * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
 * if nothing specific is need you can define it as an empty/void macro
 * @code
 * #define VL53L0X_PollingDelay(...) (void)0
 * @endcode
 * @param Dev       Device Handle
 * @return  VL53L0X_ERROR_NONE        Success
 */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	u16 i;
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
  for(i=0;i<20000;i++) ;
  return status;
}
/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	u8 i;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	for(i=0;i<count;i++)
	{
		VL53L0X_WrByte(Dev, index, *pdata);
		index++;
		pdata++;
	}
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x52);
//	VL53L0X_IIC_Tx_Byte(index);
//	for(i=0;i<count;i++)
//	{
//		VL53L0X_IIC_Tx_Byte(*pdata);
//		pdata++;
//	}
//	VL53L0X_IIC_Stop();
	return Status;
}
/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	u8 i;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	for(i=0;i<count;i++)
	{
		VL53L0X_RdByte(Dev,index,pdata);
		index++;
		pdata++;
	}
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x52);
//	VL53L0X_IIC_Tx_Byte(index);
//	VL53L0X_IIC_Stop();
//	VL53L0X_IIC_Start();
//	VL53L0X_IIC_Tx_Byte(0x53);
//	for(i=0;i<count;i++)
//	{
//		*pdata=VL53L0X_IIC_Rx_Byte(1);
//		pdata++;
//	}
//	VL53L0X_IIC_Stop();
	return Status;
}
/**
 * 初始化VL53L0X
 */
void VL53L0X_Init()
{
	VL53L0X_IIC_GpioInt();
	VL53L0X_delay();
	VL53L0X_GetVersion(pVersion);
	VL53L0X_DataInit(&MyDevice);
	VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
	RangingInit(&MyDevice);
	
}
VL53L0X_Error RangingInit(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
  //      printf ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
    //    print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
   //     printf ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
     //   print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE) // needed if a coverglass is used and no calibration has been performed
    {
    //    printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
      //  printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
     //   print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
   //     printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
     //   print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
				
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        		(FixPoint1616_t)(0.25*65536));
		}			
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
        		(FixPoint1616_t)(18*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
        		200000);
    }
    return Status;
}
u16 GetDistance(void)
{
	FixPoint1616_t LimitCheckCurrent;
	Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,&RangingMeasurementData);
	VL53L0X_GetLimitCheckCurrent(pMyDevice,VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);
	return RangingMeasurementData.RangeMilliMeter ;
}

