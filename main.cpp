#include <mbed.h>
#include <BNO055.h>
#include <DSHOT150.h>

//Making the threads
Thread nineDoFSensor;
Thread motorController;
Thread dataReporting;

#define ERROR_CONVERSION_CONSTANT 0.0005
#define p (ERROR_CONVERSION_CONSTANT / refreshn)                 //p term (error conversion, error gets multiplied by this then added to currspeed)
//#define DECEL_CAP 0.005
//#define ACCEL_CAP 0.005
#define mint 0.07                   //abs value of minimum stable throttle (7%)
//#define med_gyro 420
//#define small_gyro 210
#define refreshn 1 / 100 
#define refresh 100ms
#define ming 300


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
    motor.throttle_3d(.07);
  }
  s.reset(); 

  while (s.elapsed_time() < 10s)
  {
    motor.throttle_3d(.0);
  }
  s.reset(); 

  //Starting the threads
    nineDoFSensor.start(getGyroData);
    motorController.start(setSpeed);
    dataReporting.start(reportData);

  

  s.start(); 

  while(1) {
    if (s.elapsed_time() > 10s)
    {
      nineDoFSensor.terminate();
      motorController.terminate();
      dataReporting.terminate();
    }
  }
    printf("done\n");
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

    float throttleChange = gyros[1] * p; 


    //converting the most recent angular velocity to speed
    //currSpeed = gyros[1] * ANGULAR_VELOCITY_TO_SPEED;

/*
    if (gyros[1] > 0) {                                             //V1 
      currSpeed -= (gyros[1] * ANGULAR_VELOCITY_TO_SPEED);          //V1
    }                                                               //V1
    else if (gyros[1] < 0) {                                        //V1
      currSpeed += (gyros[1] * ANGULAR_VELOCITY_TO_SPEED);          //V1
    }
*/

    //currSpeed = prevSpeed + (gyros[1] * ERROR_CONVERSION);

/*
    if ((currSpeed < .03) && (currSpeed >= .01 )) {               //V2
      currSpeed = .03;                                            //V2
    }                                                             //V2
    else if ((currSpeed < .01) && (currSpeed > -.01)) {           //V2
      currSpeed = 0;                                              //V2 
    }                                                             //V2
    else if ((currSpeed <= -.01) && (currSpeed > -.03)) {         //V2
      currSpeed = -.03;                                           //V2
    }                                                             //V2
*/




//checks and adjusts the throttle change if it is over the decel or accel caps. 
//Decel is defined as reducing the absolute value of currSpeed, which is just coasting. it is not powered or controlled
/*
    if (currSpeed == 0 ) {                  //absolute acceleration is always acceleration when starting from 0 rpm; you can't decelerate from 0
      if (throttleChange > ACCEL_CAP) {     //checking if outside positive cap
        throttleChange = ACCEL_CAP;
        }
      else if (throttleChange < -1*(ACCEL_CAP)) { //checking if outside negative cap
        throttleChange = -1*ACCEL_CAP;
        }
    }
    else if (currSpeed > 0 ) {              //if curr speed is positive 
      if (throttleChange > ACCEL_CAP) {     //checking if outside positive cap
        throttleChange = ACCEL_CAP;
        }
      else if (throttleChange < -1*DECEL_CAP) { //checking if outside negative cap
        throttleChange = -1*DECEL_CAP;
        }
    }
    else {                                //currSpeed is negative; currSpeed < 0
      if (throttleChange < -1*ACCEL_CAP) {     //checking if outside positive cap
        throttleChange = -1*ACCEL_CAP;
        }
      else if (throttleChange > DECEL_CAP) { //checking if outside negative cap
        throttleChange = DECEL_CAP;
        }
    }
*/




//Low throttle adjustment 
/*
//VERSION 1
if (mint > currSpeed && currSpeed > -mint) {                  //if currSpeed is in low range, we should not directly use the P function     
    
    if(gyros[1] > med_gyro) {
      currSpeed = mint + 0.005;
      while (t.elapsed_time() < refresh + 3000ms){
      motor.throttle_3d(currSpeed);}
    }
    else if (med_gyro > gyros[1] && gyros[1] > small_gyro) {                //min throttle
      while (t.elapsed_time() < refresh + 3000ms){                          //V1
      motor.throttle_3d(mint+0.005);} 
    }
    else if (small_gyro > gyros[1] && gyros[1] > -small_gyro) {
      while (t.elapsed_time() < refresh + 3000ms){
      motor.throttle_3d(0);}
      currSpeed = 0; 
    }
    else if (-small_gyro > gyros[1] && gyros[1] > -med_gyro) {
      while (t.elapsed_time() < refresh + 3000ms){
      motor.throttle_3d(-mint -0.005);}
    }
    else if (-med_gyro > gyros[1]) {
      while (t.elapsed_time() < refresh + 3000ms){
      currSpeed = (-mint -0.005);}
      
    }
}

//changes the currspeed based on the error it should correct
else if(currSpeed >= mint || -mint >= currSpeed) {
      currSpeed = prevSpeed + throttleChange;
      while (t.elapsed_time() < refresh){  
      motor.throttle_3d(currSpeed);    
    }
}
*/




if (mint < abs(currSpeed)){
  currSpeed = prevSpeed + throttleChange;
}

else if (abs(gyros[1]) < ming) {
//don't do anything becuase our error is acceptable
}

else if (ming < gyros[1] && abs(currSpeed) < 0.01) {
  currSpeed = -mint;
}

else if (gyros[1] < -ming && abs(currSpeed) < 0.01) {
  currSpeed = mint;
}

else if (ming < gyros[1] && (currSpeed < (mint + 0.01)) && (currSpeed > (mint - 0.01)) ) {
  currSpeed = 0;
}

else if (-ming > gyros[1] && (currSpeed < (-mint + 0.01)) && (currSpeed > (-mint - 0.01)) ) {
  currSpeed = 0;
}


while (t.elapsed_time() < refresh){  
      motor.throttle_3d(currSpeed);

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

    ThisThread::sleep_for(1s); 
  }
}