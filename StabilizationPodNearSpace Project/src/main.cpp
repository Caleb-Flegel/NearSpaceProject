#include <mbed.h>
#include <BNO055.h>
#include <DSHOT150.h>

//Making the threads
Thread nineDoFSensor;
Thread motorController;
Thread dataReporting;

#define ANGULAR_VELOCITY_TO_SPEED .000001

void getGyroData();
void setSpeed();
void reportData();

//9 Axis Sensor Address stuff 
I2C i2c(p9,p10);
BNO055 nineAxisSensor (&i2c, 0x28 << 1); 

//Motor Controller Address stuff 
DSHOT150 motor(p19); 

//Global Variables to share between threads
float headings [3]; //The basic data from the sensor (heading, roll, pitch)
float gyros[3]; //Array for the readouts of the angular velocities from the sensor in the x y and z direction
float currSpeed; //VAriable used to send the requested speed to the motor and report the current speed to the command module

int main() {
  //Wake up the motor
  wait_us(100000); //Buffer for everything to wake up

  motor.arm_3d(); //Arming the motor

  //Setting up the sensor
  char id; 
  id = nineAxisSensor.checkID();
  if (id != 0xA0) {
    printf("\nSensor communication failure\n");
  }

  //Setting the sensor settings
  nineAxisSensor.setMode(BNO055_MODE_NDOF);
  nineAxisSensor.setAngleUnits(BNO055_ANGLE_UNITS_DEGREE);
  nineAxisSensor.setAccelerationUnits(BNO055_ACCEL_UNITS_SI);

  //Starting the threads
    nineDoFSensor.start(getGyroData);
    motorController.start(setSpeed);
    dataReporting.start(reportData); 

  /*
  Timer s;
  s.start(); 
  while(1) {
    if (s.elapsed_time() > 30s)
    {
      nineDoFSensor.terminate();
      motorController.terminate();
      dataReporting.terminate();
    }
  */
    
    
}


void getGyroData() {
  while (true) {
    //Getting the heading variables
    headings[0] = nineAxisSensor.readHeading();
    //printf("\n%.1f\n", headings[0]);
    headings[1] = nineAxisSensor.readRoll(); 
    headings[2] = nineAxisSensor.readPitch(); 

    //Getting the gyroData
    nineAxisSensor.getGyroData(gyros); 

    ThisThread::sleep_for(10ms);
  } 
}

void setSpeed(){
  while (true) {
    //converting the most recent angular velocity to speed
    //currSpeed = gyros[1] * ANGULAR_VELOCITY_TO_SPEED;

    Timer t;
    t.start();

    while (t.elapsed_time() < 50ms)
    {
      if (gyros[1] > 0) {
        currSpeed -= (gyros[1] * ANGULAR_VELOCITY_TO_SPEED);
      }
      else if (gyros[1] == 0) {
        currSpeed = 0;
      }
      else {
        currSpeed += (gyros[1] * ANGULAR_VELOCITY_TO_SPEED);
      }
      //Send the speed to the motor controller
      motor.throttle_3d(currSpeed); 
    }
    //ThisThread::sleep_for(5ms); 
  }
}

void reportData(){
  while (true) {
    printf("\nReported data:\n");

    printf("\tHeading:\t%.1f\n", headings[0]);
    printf("\tPitch:\t%.1f\n", headings[1]);
    printf("\tHeading Angular Velocity:\t%.1f\n", gyros[0]); 
    printf("\tMotor Speed:\t%.6f\n", currSpeed);

    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n");

    ThisThread::sleep_for(2s); 
  }
}