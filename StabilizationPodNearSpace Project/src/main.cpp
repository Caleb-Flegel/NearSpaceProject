#include <mbed.h>
#include <BNO055.h> //Commands to read the data from the 9axis sensor
#include <DSHOT150.h> //Commands for the motor controller
#include <FC_to_CM.h> //Commands for sending data to the command module
#include <FATFileSystem.h> //Objects used to send data to the MBED SD card 
#include <SDBlockDevice.h>

//Making the threads
Thread motorController(osPriorityNormal, 2000);
Thread nineDoFSensor(osPriorityNormal, 2000);

//Definitions for the functions that will be ran by the threads
void setSpeed();
void getData();

//9 Axis Sensor Address stuff 
I2C i2c(p9,p10);
BNO055 nineAxisSensor (&i2c, 0x28 << 1); 

//Motor Controller Address stuff 
DSHOT150 motor(p19); 

//Class for reporting the data to the command module
FC_to_CM radio(p28, p27);

//Classes for the SD card output
SDBlockDevice sd(p5, p6, p7, p8);
FATFileSystem fs("fs"); 

//Three indicator lights
DigitalOut errorLED(p23); //Signals there was an error
DigitalOut sdLED(p21); //Signals there is data being written to the SD card
DigitalOut compLED(p22); //Signals the program has been sucessfully completed

//Value that converts angular velocity into a speed setting that won't make the motor spin too fast
#define ERROR_CONVERSION .000005
//Definitions for the motor speed stabilization
#define mint 0.05 //Absolute value of minimim stable throttle
#define ming 600 //Minimum acceptable gyro[1] value 

//Global Variables to share between threads
float headings [3]; //The basic data from the sensor (heading, roll, pitch)
float gyros[3]; //Array for the readouts of the angular velocities from the sensor in the x y and z direction
float currSpeed; //VAriable used to send the requested speed to the motor and report the current speed to the command module
float prevSpeed; //Variable used to calculate the new speed based on the change in error (angular velocity)

//Values that will be used to send data to the command module
int16_t motorSpeed;
int16_t angularVelocity; 

//Global variable to access the SD card log file
FILE* logFile;

int main() {
  //Turning the lights on for startup
  errorLED = 1;
  sdLED = 1;
  compLED = 1;

  //Wake up the motor
  wait_us(100000); //Buffer for everything to wake up

  //Resetting the LEDs
  errorLED = 0;
  sdLED = 0;
  compLED = 0;

  //Getting ready to send the data to the command module
  radio.setResponseState(RESPONSE_DATA);
  radio.setDataTransmitSize(10); //Prep it for 16 bytes

  //Getting the SD card ready
  int errors = fs.mount(&sd); 
  if (errors) {
    //Turning on the error light
    errorLED = 1;
  } else {
    logFile = fopen("/fs/log.txt", "a");
    sdLED = 1;
  }

  //Verifying that the SD card file opened
  if (logFile == NULL) {
    errorLED = 1;
  }

  //Logging the headers to the file
  fprintf(logFile, "\n\nPitch\tRoll\tHeading  Motor_Speed  Angular_Velocity  Time\n");

  motor.arm_3d(); //Arming the motor

  //Setting up the sensor
  char id; 
  id = nineAxisSensor.checkID();
  if (id != 0xA0) {
    errorLED = 1;
  }

  //Define prevspeed so there is something to compare against
  prevSpeed = 0; 
  currSpeed = 0;

  //Setting the sensor settings
  nineAxisSensor.setMode(BNO055_MODE_NDOF);
  nineAxisSensor.setAngleUnits(BNO055_ANGLE_UNITS_DEGREE);
  nineAxisSensor.setAccelerationUnits(BNO055_ACCEL_UNITS_SI);

  //Starting the threads
  nineDoFSensor.start(getData);
  motorController.start(setSpeed);

  //Timer variables to help keep track of flight times
  Timer masterTimer;
  Timer fileTimer;

  //Starting the master file timer
  masterTimer.start(); 
  fileTimer.start();

  while (masterTimer.elapsed_time() < 220min) {
    //Report data
    motorSpeed = currSpeed; 
    angularVelocity = gyros[1]; 

    //Send the data to the command module
    //Headings (pitch, roll, then heading)
    radio.saveFloatAsInt16(headings[2], 2); //Pitch
    radio.saveFloatAsInt16(headings[1], 2); //Roll
    radio.saveFloatAsInt16(headings[0], 2); //Heading
    //Sending speed data
    radio.saveInt16(motorSpeed); 
    radio.saveInt16(angularVelocity);  

    //Send the data to the SD card
    fprintf(logFile, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%li\n", headings[2], headings[1], headings[0], currSpeed, gyros[1], (unsigned long)radio.getTime()); 

    //Check to see if the data needs to be flushed to the SD card
    if (fileTimer.elapsed_time() > 30s) {
      //Turn the sdled off so the user knows there is not data being written to it
      sdLED = 1;

      //Close the file to make sure that the data is transferred to it
      fclose(logFile); 

      //Reopen the file so that data can be written to it again
      logFile = fopen("/fs/log.txt", "a");

      //Turn the SD led on to let the user know that data is being logged to the SD card
      sdLED = 0;

      //Reset the file timer so the data flushes in another 30s
      fileTimer.reset();
    }

    ThisThread::sleep_for(5s); 
  }

  //Terminate the threads
  nineDoFSensor.terminate();
  motorController.terminate();

  //Close the SD file and turn off its indicator light
  fclose(logFile);
  sdLED = 0;

  //Mrking the program complete by the led
  compLED = 1;
}

void setSpeed(){
  Timer t;
  while (true) {
    t.start();
  
    //Use the previous speed to set the current speed based on the angular velocity so the motor thrust isn't too sudden 
    currSpeed = prevSpeed + (gyros[1] * ERROR_CONVERSION); 

    //Using this to make sure a certain number of packets have been sent to the motor controller
    while (t.elapsed_time() < 52ms)
    {
      //Send the speed to the motor controller
      motor.throttle_3d(currSpeed); 
    }
    //ThisThread::sleep_for(5ms); 
    t.reset(); 

    //Set the prev speed to current speed so it can be compared against later
    prevSpeed = currSpeed;  
  }
}

void getData() {
  while (true)
  {
    //Getting the heading variables
    headings[0] = nineAxisSensor.readHeading();
    headings[1] = nineAxisSensor.readRoll(); 
    headings[2] = nineAxisSensor.readPitch(); 

    //Getting the gyroData
    nineAxisSensor.getGyroData(gyros); 

    ThisThread::sleep_for(60ms);
  }
}