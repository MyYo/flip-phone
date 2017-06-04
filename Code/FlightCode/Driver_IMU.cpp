//This is higher level driver wrapper of Driver_IMU_MPU9250
#include "Driver_IMU.h"
#include "quaternionFilters.h"

//This varible stores the current IMU State
MPU9250 myIMU;


void IMU_Init ()
{
	myIMU.initMPU9250();
	myIMU.initAK8963(myIMU.magCalibration);
}

void IMU_Measure()
{
	if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	{
		myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
        myIMU.getAres();

        // Now we'll calculate the accleration value into actual g's
        // This depends on scale being set
        myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
        myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
        myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

        myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
        myIMU.getGres();

        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

        myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
        myIMU.getMres();
        // User environmental x-axis correction in milliGauss, should be
        // automatically calculated
        myIMU.magbias[0] = +470.;
        // User environmental x-axis correction in milliGauss TODO axis??
        myIMU.magbias[1] = +120.;
        // User environmental x-axis correction in milliGauss
        myIMU.magbias[2] = +125.;

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental
        // corrections
        // Get actual magnetometer value, this depends on scale being set
        myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
                myIMU.magbias[0];
        myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
                myIMU.magbias[1];
        myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
                myIMU.magbias[2];
    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    myIMU.updateTime();
}

float  IMU_GetAccMag ()
{
	return sqrt(pow(myIMU.ax,2)+pow(myIMU.ay,2)+pow(myIMU.az,2));
}

void   IMU_GetOrientation  (float &q1,float &q2,float &q3, float &q4)
{
	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                    myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                    myIMU.mx, myIMU.mz, myIMU.deltat);

	float *q = getQ ();
	q1 = q[1-1];
	q2 = q[2-1];
	q3 = q[3-1];
	q4 = q[4-1];
}

float IMU_GetZenitAngle ()
{
	float q1,q2,q3,q4;
	IMU_GetOrientation(q1,q2,q3,q4);
	return QtAngleToZenit(q1,q2,q3,q4);
}

void IMU_GetRotationRate (float &omegaX,float &omegaY,float &omegaZ)
{
	omegaX = myIMU.gx*3.141/180.0;
	omegaY = myIMU.gy*3.141/180.0;
	omegaZ = myIMU.gz*3.141/180.0;
}

void IMU_ExportData(float dataArray[])
{
	int i=0;
	dataArray[i] = myIMU.ax; i++;
	dataArray[i] = myIMU.ay; i++;
	dataArray[i] = myIMU.az; i++;
	dataArray[i] = IMU_GetAccMag(); i++;
	dataArray[i] = myIMU.gx; i++;
	dataArray[i] = myIMU.gy; i++;
	dataArray[i] = myIMU.gz; i++;
	dataArray[i] = myIMU.mx; i++;
	dataArray[i] = myIMU.my; i++;
	dataArray[i] = myIMU.mz; i++;
	dataArray[i] = IMU_GetZenitAngle(); i++;
}

//This function implements the IMU tester configuration
//The idea is we will run the IMU_Test function instead of the logic, it will spit out data while we rotate the IMU to different orientations and see data matches our estimates
#include "Logger.h"
void IMU_TestLogInit ()
{
	//Initiate
	Log_Init();
	Log_DefineNextField("a_x","g");
	Log_DefineNextField("a_y","g");
	Log_DefineNextField("a_z","g");
	Log_DefineNextField("a_mag","g");
	Log_DefineNextField("g_x","deg/s");
	Log_DefineNextField("g_y","deg/s");
	Log_DefineNextField("g_z","deg/s");
	Log_DefineNextField("m_x","mGauss");
	Log_DefineNextField("m_y","mGauss");
	Log_DefineNextField("m_z","mGauss");
	Log_DefineNextField("z_ang","deg");

	Log_WriteLogHeader();
}
void IMU_Test ()
{
	IMU_TestLogInit();
	IMU_Init();

	float dataArray[11];
	while (true) //Loop forever
	{
		//Read IMU and log values
		Log_SetTime(millis());
		IMU_Measure();
	 	
		IMU_ExportData(dataArray);
		for(int i=0;i<11;i++)
			Log_SetData(i,dataArray[i]);
	
		//Log
		Log_WriteLine();
	}
}
