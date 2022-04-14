#include <mbed.h>
#include <BNO055.h>
#include <DSHOT150.h>

//Making the threads
Thread nineDoFSensor;
Thread motorController;
Thread dataReporting;

#define ERROR_CONVERSION .000003

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
float prevSpeed; //Variable used to calculate the new speed based on the change in error (angular velocity)

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

  //Define prevspeed so there is something to compare against
  prevSpeed = 0; 

  //Setting the sensor settings
  nineAxisSensor.setMode(BNO055_MODE_NDOF);
  nineAxisSensor.setAngleUnits(BNO055_ANGLE_UNITS_DEGREE);
  nineAxisSensor.setAccelerationUnits(BNO055_ACCEL_UNITS_SI);
  
  Timer s;

  s.start();
  while (s.elapsed_time() < 10s)
  {
    motor.throttle_3d(.1);
  }
  while (s.elapsed_time() < 30s)
  {
    motor.throttle_3d(0);
  }
  while (s.elapsed_time() < 40s)
  {
    motor.throttle_3d(-.1);
  }
  while (s.elapsed_time() < 60s)
  {
    motor.throttle_3d(0);
  }
  s.reset(); 

  //Starting the threads
    nineDoFSensor.start(getGyroData);
    motorController.start(setSpeed);
    dataReporting.start(reportData); 

  
  
  s.start(); 
  while(1) {
    if (s.elapsed_time() > 60s)
    {
      nineDoFSensor.terminate();
      motorController.terminate();
      dataReporting.terminate();
    }
  }
    
    
}


void getGyroData() {
  while (true) {
    //Getting the heading variables
    headings[0] = nineAxisSensor.readHeading();
    headings[1] = nineAxisSensor.readRoll(); 
    headings[2] = nineAxisSensor.readPitch(); 

    //Getting the gyroData
    nineAxisSensor.getGyroData(gyros); 

    ThisThread::sleep_for(10ms);
  } 
}

void setSpeed(){
  Timer t;
  while (true) {
    t.start();
    //converting the most recent angular velocity to speed
    //currSpeed = gyros[1] * ANGULAR_VELOCITY_TO_SPEED;

  /*
    if (gyros[1] > 0) {
      currSpeed -= (gyros[1] * ANGULAR_VELOCITY_TO_SPEED);
    }
    else if (gyros[1] < 0) {
      currSpeed += (gyros[1] * ANGULAR_VELOCITY_TO_SPEED);
    }
  */

    //currSpeed = prevSpeed + (gyros[1] * ERROR_CONVERSION);

    /*
    if ((currSpeed < .03) && (currSpeed >= .01 )) {
      currSpeed = .03; 
    }
    else if ((currSpeed < .01) && (currSpeed > -.01)) {
      currSpeed = 0;
    }
    else if ((currSpeed <= -.01) && (currSpeed > -.03)) {
      currSpeed = -.03;
    }
    */
    float throttleChange = gyros[1] * ERROR_CONVERSION; 

    if (throttleChange > .10) {
      currSpeed = prevSpeed + .10;
    }
    else if (throttleChange < -.10) {
      currSpeed = prevSpeed - .10;
    }
    else {
      currSpeed = prevSpeed + throttleChange; 
    }

    while (t.elapsed_time() < 52ms)
    {
      //Send the speed to the motor controller
      motor.throttle_3d(currSpeed); 
    }
    //ThisThread::sleep_for(5ms); 
    t.reset(); 

    prevSpeed = currSpeed; 
  }
}

void reportData(){
  while (true) {
    printf("\nReported data:\n");

    printf("\tHeading:\t%.1f\n", headings[0]);
    printf("\tPitch:\t%.1f\n", headings[1]);
    printf("\tHeading Angular Velocity:\t%.1f\n", gyros[1]); 
    printf("\tMotor Speed:\t%.6f\n", currSpeed);

    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n");

    ThisThread::sleep_for(2s); 
  }
}
