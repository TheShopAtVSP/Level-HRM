/*
 * CFile1.c
 *
 * Created: 7/30/2014 3:37:25 PM
 *  Author: RichKl
 */ 

#include <asf.h>
#include <math.h>
#include "mpu9150.h"
#include "hal_twim.h"
#include "inv_mpu.h"

extern bool inv_debug;
uint8_t uichip;

status_code_t mpu9150_init(inv_pins_t *mpu9150stat, bool debug)
{
	status_code_t error_stat;
	
	inv_debug = debug;
	uichip = mpu9150stat->address;
	
	if (inv_debug) puts("mpu9150_init: start\r");
	
	// init interface
	//error_stat = hal_twim_init(mpu9150stat);
	error_stat = hal_twim_init();
	
	if ( error_stat != STATUS_OK )	return error_stat;
	
	// init mpu_9150
	error_stat = mpu9150_board_init();
	
	if ( error_stat == STATUS_OK ) 
	{
		if (inv_debug) puts("mpu9150_init: done\r");
	}
	else 
	{
		if (inv_debug) puts("mpu9150_init: failed\r");		
	}
	
	return error_stat;
}

inv_error_code_t mpu9150_board_init(void)
{
	uint8_t uidata;
	inv_error_code_t op_stat;
	
	if (inv_debug) puts("mpu91501_board_init: start\r");	
	
	// first check of the part is responsive...
	op_stat = mpu_who_am_i( &uidata );
	if( op_stat != ERROR_NONE) return op_stat;
	
		////Recommend reset sequence
		//mpu6000SetPower1( BIT_DEV_RESET );
		//lpWaitMS(100);    //Datasheet recommends waiting 100 ms after issuing reset
		//mpu6000ReadWriteByte( MPUREG_SIG_PATH_RST, BIT_GYRO_RESET|BIT_ACCEL_RESET|BIT_TEMP_RESET );
		//lpWaitMS(100);    //Datasheet recommends waiting 100 ms after issuing reset
		//
		//// Auxiliary I2C Supply Selection, set to zero for MPU-6000
		//mpu6000ReadWriteByte( MPUREG_SIG_PATH_RST, 0 );
		//
		//// Wake up device (better performance)
		//mpu6000SetPower1( BIT_TEMP_DIS|BIT_OSC_8MHZ );
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_PWR_MGMT_1|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_CLK_SRC) != BIT_OSC_8MHZ )
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}
//
		//// Disable I2C bus (recommended on datasheet)
		//mpu6000ReadWriteByte( MPUREG_USER_CTRL, BIT_I2C_IF_DIS );
//
		////Lower Power Mode where Gyro is Off for now
		//mpu6000SetPower2( BIT_LP40HZ|MASK_XYZ_G );
//
		////Digital Low Pass Cutoff at 20 Hz. (Gyro Fsample is 1 kHz with this DLPF setting)
		//mpu6000SetDLP( Status.Imu.ucDLPFilter );
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_CONFIG|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_DLPF_CFG) != Status.Imu.ucDLPFilter )
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}
//
		////Sensor Register Output, FIFO Output, and DMP sampling Rate
		//mpu6000SetSampleRate( Status.Imu.uiSampleRate );
//
		//// Gyro scale 250º/s, 500º/s, 1000º/s, 2000º/s
		//mpu6000SetGyroRes( Status.Imu.Gyro.ucHwSenSel );
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_GYRO_CONFIG|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_GYRO_FS) != Status.Imu.Gyro.ucHwSenSel)
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}
//
		//// Accel scale +-2g, +-4g, +-8g, +-16g
		//mpu6000SetAccelRes( Status.Imu.Accel.ucHwSenSel );
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_ACCEL_CONFIG|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_ACCEL_FS) != Status.Imu.Accel.ucHwSenSel )
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}
		//
		//// Turns FIFO On if FIFO data has been Defined
		//mpu6000FifoOnOff( Status.Imu.ucFifoData );
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_FIFO_EN|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_FIFO_EN) != Status.Imu.ucFifoData )
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}
//
		//// Init Interrupt Source
		//mpu6000ReadWriteByte( MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR );    // INT: Clear on any read
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_INT_PIN_CFG|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_INT_PIN) != BIT_INT_RD_CLEAR )
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}
		//mpu6000ReadWriteByte( MPUREG_INT_ENABLE, Status.Imu.ucIntSources );
		//ucReadCheck = mpu6000ReadWriteByte( MPUREG_INT_ENABLE|MPU_READ, 0 );
		//if( (ucReadCheck&MASK_INT_EN) != Status.Imu.ucIntSources )
		//{   //Accel Write Failed!!!
			//error = ER_INIT;
		//}

	
	if (inv_debug) puts("mpu91501_board_init: done\r");
	return ERROR_NONE;
}

//
//
/////
/////
/////
/////
//void mpu6000SetPower1( unsigned char ucPwrManage )
//{
	//mpu6000ReadWriteByte( MPUREG_PWR_MGMT_1, ucPwrManage );
//}
//
/////
/////
/////
/////
//void mpu6000SetPower2( unsigned char ucPwr2 )
//{
	//mpu6000ReadWriteByte( MPUREG_PWR_MGMT_2, ucPwr2 );
//}
//
/////
/////
/////
/////
//void mpu6000SetDLP( unsigned char ucCutoff )
//{
	//mpu6000ReadWriteByte( MPUREG_CONFIG, EXT_SYNC_SET0|ucCutoff );
//}
//
/////
/////
/////
/////
//TERRORS mpu6000SetSampleRate( unsigned int uiSampleRate )
//{
	//unsigned int uiSampRateDiv;
//
	//if( uiSampleRate == 0 ) return ER_DATA_INVALID;     //Avoid div0
//
	//if( Status.Imu.ucDLPFilter == BITS_DLPF_CFG_256HZ_NOLPF2 ||
	//Status.Imu.ucDLPFilter == BITS_DLPF_CFG_2100HZ_NOLPF    )
	//{   //Gyro samples at 8KHz when the digital filter is set to these values
		//uiSampRateDiv = 8000/uiSampleRate - 1;
	//}
	//else
	//{   //Gyro samples at 1Khz for all other DLP Filter settings
		//uiSampRateDiv = 1000/uiSampleRate - 1;
	//}
//
	//if( uiSampRateDiv > MASK_SAMP_RATE_DIV ) return ER_DATA_INVALID;    //result too big!
//
	//mpu6000ReadWriteByte( MPUREG_SMPLRT_DIV, (unsigned char)uiSampRateDiv );
//
	//return ER_NONE;
//}
//
/////
/////
/////
/////
//void mpu6000FifoOnOff( unsigned char fifo_data )
//{
	//unsigned char ucUC_Reg;
//
	//mpu6000ReadWriteByte( MPUREG_FIFO_EN, fifo_data );
//
	//ucUC_Reg = mpu6000ReadWriteByte( MPUREG_USER_CTRL|MPU_READ, 0 );
//
	//if( fifo_data > 0 )
	//mpu6000ReadWriteByte( MPUREG_USER_CTRL, BIT_FIFO_EN|ucUC_Reg );     //Turn FIFO On
	//else
	//mpu6000ReadWriteByte( MPUREG_USER_CTRL, (~BIT_FIFO_EN)&ucUC_Reg );  //FIFO Off
//}
//
/////
///// Gyro Resolution
/////
/////
//void mpu6000SetGyroRes( unsigned char ucGyroRes )
//{
	//mpu6000ReadWriteByte( MPUREG_GYRO_CONFIG, ucGyroRes );
//}
//
/////
///// Accel Resolution
/////
/////
//void mpu6000SetAccelRes( unsigned char ucAccelRes )
//{
	//mpu6000ReadWriteByte( MPUREG_ACCEL_CONFIG, ucAccelRes );
//}
//
/////
/////
/////
/////
//TERRORS mpu6000IntStatus( void )
//{
	//TERRORS res;
//
	//res = mpu6000BurstRead( (MPUREG_INT_STATUS|MPU_READ), (unsigned char *)&Status.Imu.uiIntSrcReg, 2 );
	//return( res);
//}
//
/////
/////
/////
/////
//TERRORS read_sensors( void )
//{
	//TERRORS res;
//
	//Status.Imu.measIndex = (Status.Imu.measIndex+1)&SAMPLE_CNT;
//
	//res = mpu6000BurstRead( (MPUREG_ACCEL_XOUT_H|MPU_READ), (unsigned char *)&Status.Imu.Meas[Status.Imu.measIndex], 14 );
//
	////Save a seqential copy of data for storage
	//if( Status.Log.dataType == IMU_DATA )
	//{
		//Status.Log.Sample.AxRev = Status.Imu.Meas[Status.Imu.measIndex].Ax.iVal;
		//Status.Log.Sample.AyRev = Status.Imu.Meas[Status.Imu.measIndex].Ay.iVal;
		//Status.Log.Sample.AzRev = Status.Imu.Meas[Status.Imu.measIndex].Az.iVal;
		//Status.Log.Sample.GxRev = Status.Imu.Meas[Status.Imu.measIndex].Gx.iVal;
		//Status.Log.Sample.GyRev = Status.Imu.Meas[Status.Imu.measIndex].Gy.iVal;
		//Status.Log.Sample.GzRev = Status.Imu.Meas[Status.Imu.measIndex].Gz.iVal;
	//}
//
	////ENDIANness is reversed from MPU6000, swap bytes to correct
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Ax.iVal);
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Ay.iVal);
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Az.iVal);
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Temperature.iVal);
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Gx.iVal);
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Gy.iVal);
	//SWAP(Status.Imu.Meas[Status.Imu.measIndex].Gz.iVal);
//
	////Indicate that a new sample is ready
	//Status.Log.ucImuDataRdy = BOOL_TRUE;
//
	//return res;
//}
//
/////
/////
/////
/////
//unsigned int mpu6000FifoLen( void )
//{
	//uintbytes temp;
//
	//mpu6000BurstRead( ( MPUREG_FIFO_COUNTH|MPU_READ), (unsigned char *)&temp, 2 );
	//return( temp.data );
//}
//
/////
/////
/////
/////
//unsigned char mpu6000ReadWriteByte(unsigned char ucAddr, unsigned char ucData)
//{
	//unsigned char ucTemp[2];
	//
	//ucTemp[0] = ucAddr;
	//ucTemp[1] = ucData;
	//
	//spi1SwitchModes(SPIDEV_MPU6000);
	//HW_ACCEL_CS_ON;
	//spi1BuffReadWrite( ucTemp, 2 );
	//HW_ACCEL_CS_OFF;
	//
	//return ucTemp[1];
//}
//
/////
///// Function executes in ~600 uSec at 1MHz SPI and 4MHz Fcy
///// param ucData - pointer to array to store sensor data
/////
//TERRORS mpu6000BurstRead( unsigned char addr, unsigned char * ucData, unsigned char len )
//{
	//TERRORS res;
//
	//spi1SwitchModes(SPIDEV_MPU6000);
	//HW_ACCEL_CS_ON;
	//spi1BuffReadWrite( &addr, 1);
	//res = spi1BuffReadWrite( ucData, len ); //Send the data
	//HW_ACCEL_CS_OFF;
//
	//return res;
//}
