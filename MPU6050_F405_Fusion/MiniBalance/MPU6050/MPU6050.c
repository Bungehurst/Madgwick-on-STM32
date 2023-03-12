#include "MPU6050.h"
#include "IOI2C.h"
#include "usart.h"

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define MAX_MPU_HZ      (1000)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f

static short gyro[3]={0,0,0}, accel[3]={0,0,0}, sensors;
static float gyro_bias[3]={0,0,0}, accel_bias[3]={0,0,0};
static float syn_gyro[3]={0,0,0}, syn_accel[3]={0,0,0};
unsigned char sensors1;
static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

volatile int32_t send_data[13];

volatile FusionVector3 calibratedGyroscope= {
    .axis.x = 0.0f, /* replace this value with actual gyroscope x axis measurement in lsb */
    .axis.y = 0.0f, /* replace this value with actual gyroscope y axis measurement in lsb */
    .axis.z = 0.0f, /* replace this value with actual gyroscope z axis measurement in lsb */
};
volatile FusionVector3 calibratedAccelerometer = {
    .axis.x = 0.0f, /* replace this value with actual gyroscope x axis measurement in lsb */
    .axis.y = 0.0f, /* replace this value with actual gyroscope y axis measurement in lsb */
    .axis.z = 1.0f, /* replace this value with actual gyroscope z axis measurement in lsb */
};
volatile FusionVector3 uncalibratedGyroscope = {
    .axis.x = 0.0f, /* replace this value with actual gyroscope x axis measurement in lsb */
    .axis.y = 0.0f, /* replace this value with actual gyroscope y axis measurement in lsb */
    .axis.z = 0.0f, /* replace this value with actual gyroscope z axis measurement in lsb */
        };
volatile FusionVector3 uncalibratedAccelerometer = {
    .axis.x = 0.0f, /* replace this value with actual accelerometer x axis measurement in lsb */
    .axis.y = 0.0f, /* replace this value with actual accelerometer y axis measurement in lsb */
    .axis.z = 1.0f, /* replace this value with actual accelerometer z axis measurement in lsb */
};

static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
    }
}

uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];

void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
    unsigned char i ;
    int32_t sum=0;
    for(i=1;i<10;i++){	
        MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
        MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
        MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
        MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
        MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
        MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[0][9]=ax;
    MPU6050_FIFO[1][9]=ay;
    MPU6050_FIFO[2][9]=az;
    MPU6050_FIFO[3][9]=gx;
    MPU6050_FIFO[4][9]=gy;
    MPU6050_FIFO[5][9]=gz;

    sum=0;
    for(i=0;i<10;i++){	
        sum+=MPU6050_FIFO[0][i];
    }
    MPU6050_FIFO[0][10]=sum/10;

    sum=0;
    for(i=0;i<10;i++){
        sum+=MPU6050_FIFO[1][i];
    }
    MPU6050_FIFO[1][10]=sum/10;

    sum=0;
    for(i=0;i<10;i++){
        sum+=MPU6050_FIFO[2][i];
    }
    MPU6050_FIFO[2][10]=sum/10;

    sum=0;
    for(i=0;i<10;i++){
        sum+=MPU6050_FIFO[3][i];
    }
    MPU6050_FIFO[3][10]=sum/10;

    sum=0;
    for(i=0;i<10;i++){
        sum+=MPU6050_FIFO[4][i];
    }
    MPU6050_FIFO[4][10]=sum/10;

    sum=0;
    for(i=0;i<10;i++){
        sum+=MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[5][10]=sum/10;
}

/*******************************************************************************
* void MPU6050_setClockSource(uint8_t source)
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_initialize(void) {
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); 
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    MPU6050_setSleepEnabled(0); 
    MPU6050_setI2CMasterModeEnabled(0);	 
    MPU6050_setI2CBypassEnabled(0);	 
}

/**************************************************************************

**************************************************************************/
#define Data_Num 100.0f
#define Calib_Threshold 20.0f

FusionBias fusionBias;
FusionAhrs fusionAhrs;

const float samplePeriod = 0.001f; // replace this value with actual sample period in seconds

// FusionVector3 gyroscopeSensitivity = {
//     .axis.x = 0.015267175572519f,
//     .axis.y = 0.015267175572519f,
//     .axis.z = 0.015267175572519f,
// }; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet (1/65.5) -> 500 deg/s
FusionVector3 gyroscopeSensitivity = {
    .axis.x = 0.03048780487804878f,
    .axis.y = 0.03048780487804878f,
    .axis.z = 0.03048780487804878f,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet (1/32.8) -> 1000 deg/s

// FusionVector3 accelerometerSensitivity = {
//     .axis.x = 0.00006103515625f,
//     .axis.y = 0.00006103515625f,
//     .axis.z = 0.00006103515625f,
// }; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet (1/16384) -> 2g 
// FusionVector3 accelerometerSensitivity = {
//     .axis.x = 0.00048828125f,
//     .axis.y = 0.00048828125f,
//     .axis.z = 0.00048828125f,
// }; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet (1/2048) -> 16g 
FusionVector3 accelerometerSensitivity = {
    .axis.x = 0.0001220703125f,
    .axis.y = 0.0001220703125f,
    .axis.z = 0.0001220703125f,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet (1/8192) -> 4g 
void MPU_Init(void)
{ 
		unsigned char sensors;
		unsigned long sensor_timestamp;
		unsigned char more;
        double avrgyro[3], sumgyro[3], vargyro[3], avrgyro2[3], vargyro2[3];
		short gyro[3], accel[3], gyro_data[100][3];
		u8 temp[1]={0};
		i2cRead(0x68,0x75,1,temp);
		printf("mpu_set_sensor complete ......\r\n");
		if(temp[0]!=0x68)	NVIC_SystemReset();
		
		if(!mpu_init())
		{
			if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
				 printf("mpu_set_sensor complete ......\r\n");
			if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
				 printf("mpu_configure_fifo complete ......\r\n");
			if(!mpu_set_sample_rate(MAX_MPU_HZ))
				 printf("mpu_set_sample_rate complete ......\r\n");
            
    #ifdef Gyro_PowerOn_Calib_Bias
            memset(&avrgyro2,0,sizeof(double)*3);
            memset(&vargyro2,0,sizeof(double)*3);
            for (int i=0; i < 10;i++)
            {
                memset(&avrgyro,0,sizeof(double)*3);
                memset(&vargyro,0,sizeof(double)*3);
                memset(&sumgyro,0,sizeof(double)*3);
                memset(&gyro_data,0,sizeof(short)*3*(uint16_t)Data_Num);   
                for (int i=0; i < (uint16_t)Data_Num;i++)
                {
                    mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
                    for(int j=0;j<3;j++)
                        gyro_data[i][j] = gyro[j];
                    for(int j=0;j<3;j++)
                        sumgyro[j] += gyro_data[i][j];
                }
                for(int i=0; i<3;i++)
                    avrgyro[i] = sumgyro[i] / Data_Num ;
                for (int i=0; i < (uint16_t)Data_Num; i++)
                    for(int j=0;j<3;j++)
                        vargyro[j] += pow(gyro_data[i][j] - avrgyro[j],2);
                for(int i=0; i<3;i++)
                    vargyro[i] /= Data_Num;
                for(int i=0;i<3;i++){
                    avrgyro2[i] += avrgyro[i];
                    vargyro2[i] += vargyro[i];
                }
            }
            for(int i=0;i<3;i++){
                avrgyro2[i] /= 10.0f;
                vargyro2[i] /= 10.0f;
            }
            for(int i=0;i<3;i++)
                gyro_bias[i] = avrgyro2[i]*gyroscopeSensitivity.axis.x;
    #endif
    #ifndef IMU_Calib_Mode
        #ifdef Use_Complementary_Filter
            // Initialise gyroscope bias correction algorithm
            FusionBiasInitialise(&fusionBias, 0.3f, samplePeriod);
            // Initialise AHRS algorithm
            FusionAhrsInitialise(&fusionAhrs, 0.5f); // gain = 0.5
        #endif
    #endif
		}
}

void Read_DMP(void)
{	
	    unsigned long sensor_timestamp;
		unsigned char more;
		long quat[4];

        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
        if (sensors & INV_WXYZ_QUAT )
        {    
                q0=quat[0] / q30;
                q1=quat[1] / q30;
                q2=quat[2] / q30;
                q3=quat[3] / q30;
                Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
                Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
                Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;//yaw
        }

}

// int Temp;
void Read_Data_Directly(void)
{
    unsigned long sensor_timestamp;
    unsigned char more;
    static float result[3];
    mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors1, &more);
    // Temp = Read_Temperature();
    if (sensors1 & 0x78) // Check if gyro and accel data are synchronized
    {  
    #ifdef IMU_Gyro_Calibrated
        for(int i = 0; i < 3; i++)
            #ifdef Gyro_PowerOn_Calib_Bias
            result[i] = param_Gyro_A[i]*(gyro[i]*gyroscopeSensitivity.axis.x-gyro_bias[i]);
            #elif
            result[i] = param_Gyro_A[i]*(gyro[i]*gyroscopeSensitivity.axis.x-param_Gyro_Bias[i]);
            #endif
        
        syn_gyro[0] = param_Gyro_T[0]*result[0]+param_Gyro_T[1]*result[1]+param_Gyro_T[2]*result[2];
        syn_gyro[1] = param_Gyro_T[3]*result[0]+param_Gyro_T[4]*result[1]+param_Gyro_T[5]*result[2];
        syn_gyro[2] = param_Gyro_T[6]*result[0]+param_Gyro_T[7]*result[1]+param_Gyro_T[8]*result[2];
        
        calibratedGyroscope.axis.x = syn_gyro[0];
        calibratedGyroscope.axis.y = syn_gyro[1];
        calibratedGyroscope.axis.z = syn_gyro[2];
        
    #endif
    #ifdef  IMU_Calib_Mode
        send_data[0] = gyro[0]/32.8*1e+3; 
        send_data[1] = gyro[1]/32.8*1e+3;
        send_data[2] = gyro[2]/32.8*1e+3;

		send_data[3] = accel[0]/8192.0f*9.8015f*1e+3;
        send_data[4] = accel[1]/8192.0f*9.8015f*1e+3;
        send_data[5] = accel[2]/8192.0f*9.8015f*1e+3;
        
    #endif
    #ifdef IMU_Gyro_Calibrated_Only_Bias
        for(int i = 0; i < 3; i++)
            result[i] = (gyro[i]*gyroscopeSensitivity.axis.x-param_Gyro_Bias[i]);

        calibratedGyroscope.axis.x = result[0];
        calibratedGyroscope.axis.y = result[1];
        calibratedGyroscope.axis.z = result[2];
    #endif
    #ifdef IMU_Acc_Calibrated
        for(int i = 0; i < 3; i++)
            result[i] = param_Acc_A[i]*(accel[i]*accelerometerSensitivity.axis.x*G2MPS2-param_Acc_Bias[i]);
        
        syn_accel[0] = param_Acc_T[0]*result[0]+param_Acc_T[1]*result[1]+param_Acc_T[2]*result[2];
        syn_accel[1] = param_Acc_T[3]*result[0]+param_Acc_T[4]*result[1]+param_Acc_T[5]*result[2];
        syn_accel[2] = param_Acc_T[6]*result[0]+param_Acc_T[7]*result[1]+param_Acc_T[8]*result[2];

        calibratedAccelerometer.axis.x = syn_accel[0]/G2MPS2; 
        calibratedAccelerometer.axis.y = syn_accel[1]/G2MPS2;
        calibratedAccelerometer.axis.z = syn_accel[2]/G2MPS2;

    #endif
    #ifdef Use_Madgwick_Filter
        imu_filter(calibratedAccelerometer.axis.x,calibratedAccelerometer.axis.y,calibratedAccelerometer.axis.z,calibratedGyroscope.axis.x/57.3f,calibratedGyroscope.axis.y/57.3f,calibratedGyroscope.axis.z/57.3f);
    #endif
    }
}

int Read_Temperature(void)
{	   
    float Temp;
    Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
    if(Temp>32768) Temp-=65536;
    Temp=(36.53f+(double)Temp/340)*100.0f;
    return (int)Temp;
}

volatile FusionQuaternion Quaternion_Data;
FusionEulerAngles Run_AHRS_Without_Mag(void){
	// calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);
    // calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);
	// Update gyroscope bias correction algorithm
	calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);
	// Update AHRS algorithm
	FusionAhrsUpdateWithoutMagnetometer(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, samplePeriod);
    Quaternion_Data = FusionAhrsGetQuaternion(&fusionAhrs);
    return FusionQuaternionToEulerAngles(Quaternion_Data);
}

void Get_Pure_Acc(float *data,float *result){
		static float alpha = 0.5;
		static float gravity[3];
		for(int i=0;i<3;i++) 
			gravity[i] = alpha*gravity[i]+(1-alpha)*data[i];
		for(int i=0;i<3;i++) 
			result[i] = data[i]-gravity[i];
}

