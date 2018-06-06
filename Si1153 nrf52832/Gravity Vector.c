/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    Gravity Vector.c                                  			         */
/*                                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
#include "cviogl.h"
#include <ansi_c.h>
#include <math.h>
#include <cvirte.h>
#include <userint.h>
#include <rs232.h>
#include <utility.h>
#include <formatio.h>
#include <string.h>
#include <toolbox.h>
#include "Activity Discriminator vars.h" 
#include "Activity Discriminator.h"


void normalize3DVec(float * vector);
float squared(float x);
void getInclination();


// offsets are chip specific. 
int g_offx = 120;
int g_offy = 20;
int g_offz = 93;

char str[512]; 


/// boolean firstSample = true;

float RwAcc[3];    //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float Gyro_ds[3];  //Gyro readings         
float RwGyro[3];   //Rw obtained from last estimated value and gyro movement
float Awz[2];      //angles between projection of R on XZ/YZ plane and Z axis (deg)
float RwEst[3];

int lastTime = 0;
int interval = 0;
float wGyro = 10.0;


int CVICALLBACK Gravity_Vector_CB (int panel, int control, int event,
								   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:

			break;
	}
	return 0;
}


int CVICALLBACK Apply_Gravity_Vector_CB (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:

			break;
	}
	return 0;
}



void normalize3DVec(float * vector) 
{
  float R;
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  if(R == 0) R = 0.000000001; // div by 0 correction
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}


float squared(float x)
{
  return x*x;
}



/***** Gravity Vector *****/
void Get_Gravity_Vector(void)
{
//getInclination 

  int 		w = 0, clk_tck;
  char 		out_str[128];
  float 	tmpf = 0.0;
  int 		signRzGyro;
  double	rg0, rg1, rgsum;
  
  if(ser_input_size == 0) return;
	
  gv_firstSample = 1;	
//GetGraphCursor (mainpnl, MAINPNL_SIG1GRAPH, 1,&acc_plot_index ,&throwaway_y );
////  currentTime = millis();
///  interval = acc_plot_index - gv_acc_plot_index_old;  /// <-----------------------------
  interval = 1;
	for(clk_tck=0; clk_tck<(int)acc_plot_index; clk_tck++)
	{
	////  lastTime = currentTime;
	
		/// normalize3DVec(RwAcc);
  
	  if (gv_firstSample) 
	  { 
			// the NaN check is used to wait for good data from the Arduino
		    //for(w=0;w<=2;w++) 
			//{
		    //  RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
		    //}
			 //initialize with accelerometer readings 
			RwEst[0] = filt_acl_x[0];
			RwEst[1] = filt_acl_y[0];
			RwEst[2] = filt_acl_z[0];
		
	  }
	  else
	  {
		    //evaluate RwGyro vector
		    /// if(fabs(RwEst[2]) < 0.01)
			if(0)
			{
			      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
			      //in this case skip the gyro data and just use previous estimate
			      for(w=0;w<=2;w++) 
				  {
			        RwGyro[w] = RwEst[w];
			      }
		    }
		    else 
			{
			      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
				  RwAcc[0] = filt_acl_x[(int)acc_plot_index];
				  RwAcc[1] = filt_acl_y[(int)acc_plot_index];
				  RwAcc[2] = filt_acl_z[(int)acc_plot_index];
				  normalize3DVec(RwAcc); /// <-----------------------------
			  	  Gyro_ds[0] = raw_gyr_p[(int)acc_plot_index];
				  Gyro_ds[1] = raw_gyr_y[(int)acc_plot_index];
				  Gyro_ds[2] = raw_gyr_r[(int)acc_plot_index];
			  
			      for(w=0;w<=1;w++)
				  {
			        tmpf = Gyro_ds[w];                        //get current gyro rate in deg/s
			        tmpf *= interval / 100.0f;                     //get angle change in deg
			        Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees 
			        Awz[w] += tmpf;             //get updated angle according to gyro movement
			      }
      
			      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
			      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			      signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
      
			      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			      for(w=0;w<=1;w++)
				  {
			        RwGyro[0] = sin(Awz[0] * PI / 180);
			        /// RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
					RwGyro[0] /= sqrt( 1 + (cos(Awz[0] * PI / 180) * cos(Awz[0] * PI / 180)) * (tan(Awz[1] * PI / 180) * (tan(Awz[1] * PI / 180))) ); 
			        RwGyro[1] = sin(Awz[1] * PI / 180);
			        ///RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );
					RwGyro[1] /= sqrt( 1 + (cos(Awz[1] * PI / 180) * cos(Awz[1] * PI / 180)) * (tan(Awz[0] * PI / 180) * (tan(Awz[0] * PI / 180))) );
			      }
			      /// RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
				  rg0 = RwGyro[0] * RwGyro[0]; rg1 = RwGyro[1] * RwGyro[1]; rgsum = 1 - squared(RwGyro[0]) - squared(RwGyro[1]);
				  RwGyro[2] = signRzGyro * sqrt( fabs(1 - rg0 - rg1) );
			}
    
		    //combine Accelerometer and gyro readings
		    for(w=0;w<=2;w++) RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

		    normalize3DVec(RwEst);
	  }
	  gv_firstSample = 0;
	}
	if(gv_firstSample == 0)
	{
	sprintf(out_str, "acc_plot_index = %d\n   RwEstx = %f\n   RwEsty = %f\n   RwEstz = %f\n", (int)acc_plot_index, RwEst[0], RwEst[1], RwEst[2]);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	sprintf(out_str, "Awz0 = %f,    Awz1 = %f\n\n", Awz[0], Awz[1]);
	SetCtrlVal(mainpnl, MAINPNL_TEXTBOX, out_str);
	theta_x = Awz[0]; theta_y = Awz[1];
	RenderGVImage(1); 
	OGLRefreshGraph(mainpnl, OGLControlPanel);
	}

}




//=================================================================================================================================
/*
#include <Wire.h> // I2C library, gyroscope

// Accelerometer ADXL345
#define ACC (0x53)    //ADXL345 ACC address
#define A_TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)


// Gyroscope ITG3200 
#define GYRO 0x69 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E

#define G_TO_READ 8 // 2 bytes for each axis x, y, z

// offsets are chip specific. 
int g_offx = 120;
int g_offy = 20;
int g_offz = 93;

char str[512]; 


boolean firstSample = true;

float RwAcc[3];  //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float Gyro_ds[3];  //Gyro readings         
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)
float RwEst[3];

int lastTime = 0;
int interval = 0;
float wGyro = 10.0;

void initAcc() {
  //Turning on the ADXL345
  writeTo(ACC, 0x2D, 0);      
  writeTo(ACC, 0x2D, 16);
  writeTo(ACC, 0x2D, 8);
  //by default the device is in +-2g range reading
}

void getAccelerometerData(int * result) {
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  byte buff[A_TO_READ];
  
  readFrom(ACC, regAddress, A_TO_READ, buff); //read the acceleration data from the ADXL345
  
  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  result[0] = (((int)buff[1]) << 8) | buff[0];   
  result[1] = (((int)buff[3])<< 8) | buff[2];
  result[2] = (((int)buff[5]) << 8) | buff[4];
}

void rawAccToG(int * raw, float * RwAcc) {
  RwAcc[0] = ((float) raw[0]) / 256.0;
  RwAcc[1] = ((float) raw[1]) / 256.0;
  RwAcc[2] = ((float) raw[2]) / 256.0;
}


void getGyroscopeData(int * result)
{
  /**************************************
  Gyro ITG-3200 I2C
  registers:
  temp MSB = 1B, temp LSB = 1C
  x axis MSB = 1D, x axis LSB = 1E
  y axis MSB = 1F, y axis LSB = 20
  z axis MSB = 21, z axis LSB = 22
  ************************************

  int regAddress = 0x1B;
  int temp, x, y, z;
  byte buff[G_TO_READ];
  
  readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
  
  result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
  result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
  result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
  result[3] = (buff[0] << 8) | buff[1]; // temperature 
}


// convert raw readings to degrees/sec
void rawGyroToDegsec(int * raw, float * gyro_ds) {
  gyro_ds[0] = ((float) raw[0]) / 14.375;
  gyro_ds[1] = ((float) raw[1]) / 14.375;
  gyro_ds[2] = ((float) raw[2]) / 14.375;
}


void normalize3DVec(float * vector) {
  float R;
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}


float squared(float x){
  return x*x;
}


void getInclination() {
  int w = 0;
  float tmpf = 0.0;
  int currentTime, signRzGyro;
  
  
  currentTime = millis();
  interval = currentTime - lastTime;
  lastTime = currentTime;
  
  if (firstSample) { // the NaN check is used to wait for good data from the Arduino
    for(w=0;w<=2;w++) {
      RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
    }
  }
  else{
    //evaluate RwGyro vector
    if(abs(RwEst[2]) < 0.1) {
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for(w=0;w<=2;w++) {
        RwGyro[w] = RwEst[w];
      }
    }
    else {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
      for(w=0;w<=1;w++){
        tmpf = Gyro_ds[w];                        //get current gyro rate in deg/s
        tmpf *= interval / 1000.0f;                     //get angle change in deg
        Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees 
        Awz[w] += tmpf;             //get updated angle according to gyro movement
      }
      
      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
      signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
      
      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
      for(w=0;w<=1;w++){
        RwGyro[0] = sin(Awz[0] * PI / 180);
        RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
        RwGyro[1] = sin(Awz[1] * PI / 180);
        RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );        
      }
      RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
    }
    
    //combine Accelerometer and gyro readings
    for(w=0;w<=2;w++) RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

    normalize3DVec(RwEst);
  }
  
  firstSample = false;
}



void setup()
{
  Serial.begin(19200);
  Wire.begin();
  initAcc();
  initGyro();
}


void loop()
{
  if(!Serial.available()) {
    int acc[3];
    int gyro[4];
    
    
    getAccelerometerData(acc);
    rawAccToG(acc, RwAcc);
    normalize3DVec(RwAcc);
    
    getGyroscopeData(gyro);
    rawGyroToDegsec(gyro, Gyro_ds);
    
    getInclination();
    
    //sprintf(str, "%d,%d,%d,%d,%d,%d,%d,", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], gyro[3]);  
    //Serial.print(str);
    //Serial.print(10, BYTE);
    
    
    serialPrintFloatArr(RwAcc, 3);
    serialPrintFloatArr(Gyro_ds, 3);
    serialPrintFloatArr(RwGyro, 3);
    serialPrintFloatArr(Awz, 2);
    serialPrintFloatArr(RwEst, 3);
    Serial.println();
    //Serial.print(10, BYTE);
    
    //delay(50);
  }
}

void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  Serial.print("f:");
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}


//---------------- Functions
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
   Wire.beginTransmission(DEVICE); //start transmission to ACC 
   Wire.send(address);        // send register address
   Wire.send(val);        // send value to write
   Wire.endTransmission(); //end transmission
}


//reads num bytes starting from address register on ACC in to buff array
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.send(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
  
  int i = 0;
  while(Wire.available())    //ACC may send less than requested (abnormal)
  { 
    buff[i] = Wire.receive(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
*/
