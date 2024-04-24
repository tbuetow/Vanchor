//Re-write of TrollingMotorController to use Servo-style PWM outputs instead of direct stepper control. Remove all calibration too.

#include <Servo.h>
#include <math.h>
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>

const String version = "v0.0.5";

///////////////////////////////
// Options
///////////////////////////////
// Communication
const int baudRate = 19200;

///////////////////////////////
// RC Servo Pulse Widths
///////////////////////////////

const int stoppedPulseWidth = 1500; //us
const int fullForwardPulseWidth = 2000; //us
const int fullReversePulseWidth = 1000; //us 

///////////////////////////////
// Steering
///////////////////////////////

const int stepsPerRevolution = 200;
const int stepsFullLeft = -60*36;
const int stepsFullRight = 60*36;

// Stepper controller pinout
const int stepperRcPin = 9;
const int stepperDefaultSpeed = 180;

///////////////////////////////
// Trolling motor controller
///////////////////////////////
const int trollingMotorFwReverseDelay = 500; // ms (delay between reverse and forwarding, to not destroy the engine)
const int rampTime = 800; //ms

// DC Motor controller pinout
const int trollingMotorRcPin = 10; // Pin for enabling trolling motor

// Init vars
double long lastOutput = millis();
bool chokeMotor;
bool output = true;
int currentMotorSpeed;
int motorSpeed;
int lastMotorSpeed;
long double rampStart;
int rampValue;
bool ramping;
bool rampBlock;
bool motor_forward;

int calibBegin;
int calibEnd;

int set_stepperPosition;
int set_stepperSpeed;
int set_stepperAcceleration;
int set_motorSpeed;
bool set_motorRev;

///////////////////////////////
// INIT
///////////////////////////////

// Servos
Servo steeringServo;
Servo motorServo;

// CMD
CmdCallback<3> cmdCallback;
CmdBuffer<32> myBuffer;
CmdParser     myParser;


// Define commands

char strUpdate[] = "UPD";
char strCalib[] = "CAL";
char strPing[] = "PING";


// Init functions



void setup() {

  
  Serial.begin(baudRate);

  // Set Servos
  steeringServo.attach(stepperRcPin);
  motorServo.attach(trollingMotorRcPin);

  // CMD
  lastOutput = millis();

  Serial.println("SETUP: CMD");
  myBuffer.setEcho(true);
  
  
  cmdCallback.addCmd(strPing, &replyPing);
  cmdCallback.addCmd(strCalib, &calibCmd);
  cmdCallback.addCmd(strUpdate,&updateCmd);
}


void loop()
{

  if(!Serial) {  //check if Serial is available... if not,
      Serial.end();      // close serial port
      delay(100);        //wait 100 millis
      Serial.begin(baudRate); // reenable serial again
  }
  else{

    rampMotor();

    cmdCallback.updateCmdProcessing(&myParser, &myBuffer, &Serial);
  }
}

int motorTargetPulseWidth(int speed, bool forward=true){
  int targetPulseWidth;
  if(forward){
        targetPulseWidth = speed/100 * (fullForwardPulseWidth - stoppedPulseWidth) + stoppedPulseWidth;
      }
      else {
        targetPulseWidth = speed/100 * (fullReversePulseWidth - stoppedPulseWidth) + stoppedPulseWidth;
      }
}

void setMotorSpeed(int speed, bool reverse=false){
  int targetPulseWidth;
  
  if(motorSpeed < speed && (reverse && motor_forward == true) != true && (reverse == false && motor_forward == false) != true){
    
    motorServo.writeMicroseconds(motorTargetPulseWidth(motorSpeed,motor_forward));
  }

  lastMotorSpeed = motorSpeed;
  motorSpeed = speed;
  
  if(reverse && motor_forward == true){
    motor_forward = false;
    ramping = false;
    motorServo.writeMicroseconds(stoppedPulseWidth);
    
    rampStart = millis() + trollingMotorFwReverseDelay;
  }
  else if(reverse == false && motor_forward == false){
    motor_forward = true;
    ramping = false;

    motorServo.writeMicroseconds(stoppedPulseWidth);

    rampStart = millis() + trollingMotorFwReverseDelay;
  }
  else {
    motor_forward = true;
    rampStart = millis();
  }

  Serial.print("rampMotor | Starting motor ramp at ");
  Serial.println(millis());
  
}

void rampMotor(){
  int targetPulseWidth;
  
  if(millis() < (rampStart + rampTime)){
    ramping = true;
    
    int rampValue = round((100.0 / rampTime) * (millis() - rampStart));
    int newSpeed = motorSpeed;
    

    if(lastMotorSpeed < motorSpeed){
      newSpeed = lastMotorSpeed + rampValue;
      if(newSpeed > motorSpeed){
        newSpeed = motorSpeed;
      }
    }
    else if(lastMotorSpeed == motorSpeed){
      newSpeed = motorSpeed;
    }
    else{
      newSpeed = lastMotorSpeed - rampValue;
      if(newSpeed < motorSpeed){
        newSpeed = motorSpeed;
      }
    }

    motorServo.writeMicroseconds(motorTargetPulseWidth(motorSpeed,motor_forward));
    currentMotorSpeed = newSpeed;

  }
  else {
    if(ramping){
      ramping = false;
      currentMotorSpeed = motorSpeed;
      motorServo.writeMicroseconds(motorTargetPulseWidth(motorSpeed,motor_forward));
      Serial.print("rampMotor | Ending motor ramp at ");
      Serial.println(millis());
    }
  }
}

void replyPing(CmdParser *myParser){
  Serial.println("Pong");
  Serial.println("OpenTrollingMotor Controller "+version);
}

void calibCmd(CmdParser *myParser){
  // stepper.setCurrentPosition(atoi(myParser->getCmdParam(1)));
}

int cstepperPosition;
int cstepperSpeed;
int cstepperAcceleration;
int cmotorSpeed;
bool cmotorRev;
int steeringTargetPulseWidth;

void updateCmd(CmdParser *myParser){
  cstepperPosition = atoi(myParser->getCmdParam(1));
  cstepperSpeed = atoi(myParser->getCmdParam(2));
  cstepperAcceleration = atoi(myParser->getCmdParam(3));
  cmotorSpeed = atoi(myParser->getCmdParam(4));
  cmotorRev = atoi(myParser->getCmdParam(5)) == 1;
  
  
  if(set_stepperPosition != cstepperPosition){
    steeringTargetPulseWidth = (cstepperPosition/(stepsFullRight - stepsFullLeft)) * 500 + stoppedPulseWidth;
    steeringServo.writeMicroseconds(steeringTargetPulseWidth);
    set_stepperPosition = cstepperPosition;
  }
  
  
  if(set_motorSpeed != cmotorSpeed){
    setMotorSpeed(cmotorSpeed, cmotorRev);
    set_motorSpeed = cmotorSpeed;
    set_motorRev = cmotorRev;
  }

  Serial.print("STATUS ");
  Serial.print("SSP:");
  Serial.print(set_stepperPosition);
  Serial.print(" SDTG:");
  Serial.print(steeringTargetPulseWidth);
  Serial.print(" MS:");
  Serial.print(set_motorSpeed);
}
