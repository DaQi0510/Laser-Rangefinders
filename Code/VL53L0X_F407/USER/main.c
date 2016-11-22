#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "VL53L0X_IIC.h" 
#include "lcd.h"

//ALIENTEK 探索者STM32F407开发板 实验1
//跑马灯实验 -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK
VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice);
VL53L0X_Error Status = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t *pMyDevice = &MyDevice;
VL53L0X_Version_t                   Version;
VL53L0X_Version_t                  *pVersion   = &Version;
VL53L0X_DeviceInfo_t                DeviceInfo;
u8 i,j,k;
u8 data[5];
u16 m;
int main(void)
{ 
  
	delay_init(168);		  //初始化延时函数
	delay_ms(100);
	LED_Init();
	VL53L0X_Init();
//	VL53L0X_IIC_GpioInt();
	delay_ms(100);
//	VL53L0X_GetVersion(pVersion);
//	VL53L0X_DataInit(&MyDevice);
//	VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
	LCD_Init ();
//	rangingTest(&MyDevice);
//	i=15;
//	VL53L0X_WrByte(&MyDevice, 0x80, 0x01);
//	VL53L0X_RdByte(&MyDevice, 0x80,&i);
//	VL53L0X_WrWord(pMyDevice,0xC0,0x3344);
//  VL53L0X_ReadMulti(pMyDevice,0xC0,data,3);
	while(1)
	{
		GetDistance();
		LCD_ShowNum (1,1,RangingMeasurementData.RangeMilliMeter);
		LCD_ShowNum (1,3,RangingMeasurementData.RangeMilliMeter-45);
		LED0=1;
		delay_ms(1000);
		LED0=0;
		delay_ms(1000);
	      //延时300ms
	}
}

VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int i;
    FixPoint1616_t LimitCheckCurrent;
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
    /*
     *  Step  4 : Test ranging mode
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<10;i++){
        //    printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

//            print_pal_error(Status);
//            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
            		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

 //           printf("RANGE IGNORE THRESHOLD: %f\n\n", (float)LimitCheckCurrent/65536.0);


            if (Status != VL53L0X_ERROR_NONE) break;

//            printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);


        }
    }
    return Status;
}




