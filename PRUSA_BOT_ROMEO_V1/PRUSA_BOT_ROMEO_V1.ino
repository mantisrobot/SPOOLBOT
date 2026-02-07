// *****************************************************
//  Prusa Spool Bot (Spooly) by Matt Denton 05/02/2026
//  Written for DFRobot Romeo Mini ESP32.
//  Used "ESP32C3 Dev Module" as the device selection.
//  Enable "USB CDC On Boot" for USB debug messages.
//  Requires Adafruit BNO085 IMU.
//
//  Removeable / Magenetc side is the left, Fixed side is right
//  
//  PWM Inputs will require calibration, use the USBC connector to access the serial terminal
//  type 'C' and press enter in the console and follow instructions.
//
//  Some know issues with BNO085 and ESP32 i2c. if you are having problems try adding a 2K2 to 4k7 
//  resistor on the BNO085 from VCC to SDA line
//
// *****************************************************

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <PID_v1.h>
#include <EEPROM.h>

//  Loop Frequency
#define LOOP_TIME 10 //ms

// *****************************************************
// FEATURE DEFINITIONS

//#define ENABLE_OTA        // Over The Air Updates, switch off once tuning/development is done
//#define ENABLE_UDP_DEBUG  // send / receive UDP debug data for tuning, switch off once tuning is done.
#if defined(ENABLE_OTA) | defined(ENABLE_UDP_DEBUG)
#include <WiFi.h>
#include <WiFiUdp.h>
#ifdef ENABLE_OTA
#include <ArduinoOTA.h>  // For enabling over the air updates
#endif
#endif
IPAddress pcIP(10,1,0,123);   // your PC IP address for SerialPlot

#define USE_LEDC          // ledcWrite routines rather than analogWrite for Motor PWM
#define PWM_FREQUENCY_SWITCHING // without this only MOTOR_PWM_LO_FREQUENCY is used
#define PWM_HI_FREQ_DUTY         (0.1f) // NORMALISED SO 0.1 = 10%
#define PWM_LO_FREQ_DUTY         (0.05f) // NORMALISED SO 0.05 = 5%
#define MOTOR_PWM_HI_FREQUENCY   8000 // Above PWM_HI_FREQ_DUTY dutr this frequency is used
#define MOTOR_PWM_LO_FREQUENCY   2000 // Belwo PWM_LO_FREQ_DUTY duty this frequency is used

// *****************************************************
// MOTOR SPEED / CONTROL VARIABLES

#define ENC_CPR (48.0f)
#define MOTOR_GEAR_RATIO  (9.7f) // motor attached gearbox
#define PRINTED_GEAR_RATIO  (80.0f/18.0f)

#define MOTOR_MAX_RPM       (8000) // before gearbox, this determines max forward speed
#define RPM_DEADBAND        (100) // RPM
#define MIN_RPM             (0) // provides kick if needed
#define MAX_TURN_SPEED      (540.0f) // degreees /sec

#define TURN_FORWARD_SPEED_REDUCTION 0.35f // reduction in forward speed when turning, this is a sliding scale from 100% to this value.
#define PITCH_CURVE 0.35f                 // IMU Pitch sigmoid curve, lower values = higher brakeing, 1 = linear 
#define HALF_SPEED_REDUCTION      (0.6f)  // user control speed reduction for forward / reverse
#define HALF_SPEED_TURN_REDUCTION (0.6f)  // user control speed reduction for turn

// *****************************************************
// IMU / CONTROL VARIABLES USED FOR NORMALISING IMU DATA

#define IMU_PITCH_MAX (75.0f) // at this angle the motor is in full reverse to prevent looping
#define IMU_ROLL_MAX (50.0f)  // fall/tilt angle motors off 
#define IMU_YAW_MAX (50.0f)
#define IMU_PITCH_RATE_MAX (360.0f)
#define IMU_ROLL_RATE_MAX (360.0f)
#define IMU_YAW_RATE_MAX (MAX_TURN_SPEED) // determins max turn speed

// *****************************************************
//  PINS

#define MOTA_ENCA  6
#define MOTA_ENCB  7
#define MOTB_ENCA  20
#define MOTB_ENCB  21

#define PWM_PITCH  4
#define PWM_YAW    3
#define PWM_AUX    5

#define MOTA_DIR   1
#define MOTA_PWM   0
#define MOTB_DIR   10
#define MOTB_PWM   2

// *****************************************************
//  PID VARIABLES & INSTANCES

// left motor / magnetic removable side
double  lDesiredRpm=0, lMeasuredRpm=0, lDriveOutput=0;
PID lPid(&lMeasuredRpm, &lDriveOutput, &lDesiredRpm,3.2f,6.8f,0.12f, DIRECT); // 0.00012, 0.0005
// right motor / fixed side
double  rDesiredRpm=0, rMeasuredRpm=0, rDriveOutput=0;
PID rPid(&rMeasuredRpm, &rDriveOutput, &rDesiredRpm,3.2f,6.8f,0.12f, DIRECT); // 0.2, 3.5, 0.001
// turn PID
double  measuredYaw=0, setYaw=0, yawOutput=0;
PID yawPid(&measuredYaw, &yawOutput, &setYaw,0.35f,2.0f,0.005f, DIRECT); // 0.2, 3.5, 0.001

// variables sued in control caluculations and in experimental speed PID
float pitchOutput=0,measuredSpeed=0;


// *****************************************************
// trim variables used during remote tuning of PID loops
float trimTemp = PITCH_CURVE;
float pTrim = 0.001;
float iTrim = 0.012f;
float dTrim = 0.0f;

#define PTRIM_INC 0.0001f  // 0.05
#define ITRIM_INC 0.001f  // 0.05
#define DTRIM_INC 0.00001f // 0.001

// *****************************************************
//  IMU

Adafruit_BNO08x  bno08x(-1);
sh2_SensorValue_t sensorValue;


// *****************************************************
// global variables

unsigned long previousMillis = 0;
bool  initialBoot = true; // must be cleared by user input before motors can be enabled on initial boot.
bool  motorsEnabled = false;  // user motor kill switch
bool  headingHoldAlways = false;  // hold heading even when no user input
bool  positionHoldAlways = true;  // hold position rather than drift if nudged
bool  userControl = false; // is there a desired move from user controls
bool  halfSpeed = false;  // finer speed control
bool  halfSpeedTurn = true;  // finer speed control

float  normPitch=0,normRoll=0,normPitchRate=0,normYawRate=0,pitchCurve=0;
float  measuredSpeedFiltered=0;

// encoder outputs
float EncA_SpeedFiltered = 0; // filtered encoder speeds
float EncB_SpeedFiltered = 0;
float WheelA_OutputSpeed = 0; // calculated wheel speeds
float WheelB_OutputSpeed = 0;

// loop delta variables
uint32_t  loopLastUs = 0;
uint32_t  loopDeltaUs = 0;
float loopDelta = 0;

// user controls
uint16_t  pitch_PWM = 1500;
uint16_t  yaw_PWM = 1500;
uint16_t  aux_PWM = 1500;
float pitch_controlFiltered = 0;  // filtered user controls
float yaw_controlFiltered = 0;
float aux_controlFiltered = 0;
float pitch_controlFinal = 0;     // final user controls after all filters applied
float yaw_controlFinal = 0;

// Gyro Structure
typedef struct {
  float x;
  float y;
  float z;
} gyro_s;

// gyro filtered data
gyro_s gyroF;

// *****************************************************
// Calibration data

struct __attribute__((packed)) PWMCal {
  uint16_t min_raw;   // 
  uint16_t max_raw;   // 
  uint16_t mid_raw;   // 
};

struct __attribute__((packed)) CalStore {
  uint32_t magic;     // 0x50454431 
  PWMCal pitch;
  PWMCal yaw;
  uint16_t crc;       // simple sum or 0xFFFF if unused
};

#define PWM_MAX (2000)  // uS
#define PWM_MID (1500)
#define PWM_MIN (1000)  // uS
#define CAL_MAGIC 0x50454431
CalStore calData;

// *****************************************************
void setup() {
  
  pinMode(MOTA_ENCA, INPUT_PULLUP);
  pinMode(MOTA_ENCB, INPUT_PULLUP);
  pinMode(MOTB_ENCA, INPUT_PULLUP);
  pinMode(MOTB_ENCB, INPUT_PULLUP);
  pinMode(PWM_PITCH, INPUT_PULLDOWN);
  pinMode(PWM_YAW, INPUT_PULLDOWN);
  pinMode(PWM_AUX, INPUT_PULLDOWN);
  setupMotorPins();
  
  //  Start Serial and wait for connection 2 seconds
  Serial.begin(115200);
  int t = 20;
  while (!Serial && t--) delay(100);
  Serial.println("\r\nPrusa Spool Bot (Spooly) by Matt Denton\r\n");

  // configure WiFi
  #if defined(ENABLE_OTA) || defined(ENABLE_UDP_DEBUG)
  setupWiFi();
  #endif

  // Try to initialize IMU
  while (!bno08x.begin_I2C()) 
  {
    Serial.println("Failed to find BNO08x chip");
    delay(200);
  }
  Serial.println("BNO08x Found!");

  // report IMU data
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) 
  {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  // set IMU events
  BNOSetEvents();

  // configure Motor Encoders
  initEncoders();

  // configure interupt pins
  attachInterrupt(digitalPinToInterrupt(MOTA_ENCA), EncA_isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTA_ENCB), EncA_isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTB_ENCA), EncB_isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTB_ENCB), EncB_isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_PITCH), pwm_pitch_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_YAW), pwm_yaw_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_AUX), pwm_aux_isr, CHANGE);

  // load PWM calibration data
  loadCal();
  Serial.println("Cal after load:");
  Serial.printf("  magic=%x\r\n",calData.magic);
  Serial.printf("  pitch[min,mid,max]=[%d,%d,%d]\r\n",calData.pitch.min_raw,calData.pitch.mid_raw,calData.pitch.max_raw);
  Serial.printf("  yaw[min,mid,max]=[%d,%d,%d]\r\n",calData.yaw.min_raw,calData.yaw.mid_raw,calData.yaw.max_raw);
  CalStore chk=calData; 
  uint16_t expect; 
  chk.crc=0; 
  expect=simple_crc(chk);
  Serial.printf("  crc stored=%x expect=%x\r\n",calData.crc,expect);

  // configure left motor PID
  lPid.SetOutputLimits(-1, 1);
  lPid.SetSampleTime(LOOP_TIME);
  lPid.SetMode(AUTOMATIC);

  // configure right motor PID
  rPid.SetOutputLimits(-1, 1);
  rPid.SetSampleTime(LOOP_TIME);
  rPid.SetMode(AUTOMATIC);

  // configure turn rate/yaw PID
  yawPid.SetOutputLimits(-1, 1);
  yawPid.SetSampleTime(LOOP_TIME);
  yawPid.SetMode(AUTOMATIC);

  // initialise encoder variables
  noInterrupts();
  resetEncoders();
  initRPMCDF();
  initSpeedCDF();
  interrupts();

  // start
  loopLastUs = micros();
  Serial.println("Loop..");
}

// *****************************************************
void loop() 
{

  #if defined(ENABLE_OTA) || defined(ENABLE_UDP_DEBUG)
  wifiService();
  #endif

  BNOService();  
  checkUserCommands();

  if (millis() - previousMillis >= LOOP_TIME) 
  {    
    previousMillis = millis();
    
    // read volatile encoder values
    updateEncoders();
    // read new PWM user control values
    updatePWMControls();
    // update loop delta variables
    updateLoopVariables();
    // fitler encoders using FOF
    //FOF_EncoderFilter();
    // filter encoders using critical damped filter
    RPMCdf();
    // filter PWM user controls
    filterPWM();
    // Check user control inputs & Configuration mode
    checkControlInputs();
    // Filter & Normailse IMU date to +/-1
    normailseIMUData();
    // Compute the forward and turn speed using control inputs and IMU data
    computeDriveVariables();
    // update motors
    updateDriveMotors(pitchOutput ,yawOutput );
    // turn off motors on tip over
    rollOverProtection();

    #ifdef ENABLE_UDP_DEBUG
    udpPacketReceive();
    sendWiFiUDPDataStream();
    #endif

    // Serial Debug Messages
    //debugEncoder();
    //debugAHRS();
    //debugPWM();
    //debugOutput();
  
  }

}


void computeDriveVariables()
{ 

  // Setup initial drive speed, All normalised values +/-1.0
  float bodyPitchSpeed = pitch_controlFinal;
  // reduce speed if required
  if( halfSpeed ) bodyPitchSpeed *= HALF_SPEED_REDUCTION;
  // reduce body speed when turning to prevent roll over, this is a graded
  // reduction from 100% down to TURN_FORWARD_SPEED_REDUCTION %
  bodyPitchSpeed *= 1.0-( fabs(yaw_controlFiltered) * TURN_FORWARD_SPEED_REDUCTION );

  // calucalte measured speed of wheels using motor RPM and offset gyro rotations
  // to remove body climbing the wheel gear. Sort of works
  measuredSpeed = ((-WheelA_OutputSpeed + WheelB_OutputSpeed)/2.0f) + (gyroF.y/6.0f);
  measuredSpeedFiltered = speedCdf();

  // calculate curved pitch, this gives much better braking control rather than a linear pitch output
  // effectively makes IMY less responsive when alligneds with ground, Spooly stationary.
  pitchCurve = sigmoidCurve(normPitch,PITCH_CURVE);

  // Calculate the pitchOutput depending on what mode we are in
  // if we dont want to adjust position when nudged or on a hill
  if( positionHoldAlways == true ) 
  {
    // higher rate gain when holding position and no pitch gain
    if( userControl == false ) pitchOutput = normPitchRate * 0.5f;
    else pitchOutput = bodyPitchSpeed + pitchCurve + normPitchRate * 0.05f;
  }
  // otherwise always allow drift
  else
  {
    pitchOutput = bodyPitchSpeed + pitchCurve + normPitchRate * 0.05f;
  }

  // Update YAW PID variables
  measuredYaw = normYawRate;
  setYaw = yaw_controlFinal;//yaw_controlFiltered;
  // reduce for half speed turn
  if( halfSpeedTurn ) setYaw *= HALF_SPEED_TURN_REDUCTION;
  // if heading hold is off and there is no user input, do not adjust yaw control
  // we dont want the integrator to wind up so we disable PID updates when not in use.
  if( headingHoldAlways == false ) 
  {
    if( (userControl == false || motorsEnabled == false ) && yawPid.GetMode() == AUTOMATIC )
    {
      Serial.println("YAW PID = Off");
      yawPid.SetMode(MANUAL);
      yawOutput = 0;
    }      
    else if( userControl == true && yawPid.GetMode() == MANUAL )
    {
      Serial.println("YAW PID = On");
      yawPid.SetMode(motorsEnabled);
    }
  }
  else
  {
    yawPid.SetMode(motorsEnabled);
  }

  
  // Compute YAW PID
  yawPid.Compute();

}


void  updateDriveMotors( float pForwardSpeed, float pYawSpeed )
{
  float leftMotorRpm,rightMotorRpm;

  // tank drive mix
  leftMotorRpm = (pForwardSpeed + pYawSpeed);// * MOTOR_MAX_RPM;
  rightMotorRpm = (pForwardSpeed - pYawSpeed);// * MOTOR_MAX_RPM;
   
  //clamp drive to MAX
  leftMotorRpm = constrain(leftMotorRpm,-1.0f,1.0f);
  rightMotorRpm = constrain(rightMotorRpm,-1.0f,1.0f);

  // normalise encoder speed
  lMeasuredRpm = EncA_SpeedFiltered/MOTOR_MAX_RPM;
  rMeasuredRpm = EncB_SpeedFiltered/MOTOR_MAX_RPM;

  // if motors are off, set speed to 0 and disable PID to prevent windup
  if( motorsEnabled == false )
  {
    leftMotorRpm = 0;
    rightMotorRpm = 0;
    lDriveOutput = 0;
    rDriveOutput = 0;
    lPid.SetMode(MANUAL);
    rPid.SetMode(MANUAL);
  }
  else
  {
    lPid.SetMode(AUTOMATIC);
    rPid.SetMode(AUTOMATIC);
  }

  // LEFT MOTOR
  // invert left side drive here
  lDesiredRpm = -leftMotorRpm;

  // deadband and minimum punch rpm
  if( fabs(lDesiredRpm) <= (RPM_DEADBAND/MOTOR_MAX_RPM) ) lDesiredRpm = 0;
  #if MIN_RPM > 0
  else if( lDesiredRpm > (RPM_DEADBAND/MOTOR_MAX_RPM) && lDesiredRpm < (MIN_RPM/MOTOR_MAX_RPM) ) lDesiredRpm = (MIN_RPM/MOTOR_MAX_RPM);
  else if( lDesiredRpm < (-RPM_DEADBAND/MOTOR_MAX_RPM) && lDesiredRpm > (-MIN_RPM/MOTOR_MAX_RPM) ) lDesiredRpm = (-MIN_RPM/MOTOR_MAX_RPM);
  #endif


  if( lPid.Compute() )
  {
    // apply max duty
    lDriveOutput = constrain(lDriveOutput,-1.0f,1.0f);
  }


  // RIGHT MOTOR
  // invert right side drive here
  rDesiredRpm = rightMotorRpm;

    // deadband and minimum punch rpm
  if( fabs(rDesiredRpm) <= (RPM_DEADBAND/MOTOR_MAX_RPM) ) rDesiredRpm = 0;
  #if MIN_RPM > 0
  else if( rDesiredRpm > (RPM_DEADBAND/MOTOR_MAX_RPM)  && rDesiredRpm < (MIN_RPM/MOTOR_MAX_RPM) ) rDesiredRpm = (MIN_RPM/MOTOR_MAX_RPM);
  else if( rDesiredRpm < (-RPM_DEADBAND/MOTOR_MAX_RPM)  && rDesiredRpm > (-MIN_RPM/MOTOR_MAX_RPM) ) rDesiredRpm = (-MIN_RPM/MOTOR_MAX_RPM);
  #endif


  if( rPid.Compute() )
  {
    // apply max duty
    rDriveOutput = constrain(rDriveOutput,-1.0f,1.0f);
  }

  updateMotorAPWM(lDriveOutput);
  updateMotorBPWM(rDriveOutput);
  

}




void rollOverProtection()
{
  // roll over protection
  if( fabs(normRoll) >= 0.99f && motorsEnabled == true )
  {
    motorsEnabled = false;
    initialBoot = true;
    userControl = false;
    Serial.println("Roll Over Motors Off!");
  }
}


void checkControlInputs()
{
  // check aux input for enable / disable
  // must be disabled first if initialBoot flag is true
  if( aux_controlFiltered < -0.5 )
  {
    initialBoot = false;
    if( motorsEnabled == true ) 
    {
      motorsEnabled = false;
      Serial.println("Motors Disabled!");
      userControl = false;
      Serial.println("Moveing Stop.");
    }
  }
  else if( aux_controlFiltered > 0.5 )
  {      
    if( motorsEnabled == false && initialBoot == false )
    {
      motorsEnabled = true;
      Serial.println("Motors Enabled!");
    }
  }

  // set moving / stopped flag using user controls and current motor speeds
  // this is used for switching modes / braking etc
  if( motorsEnabled == true )
  {
    if( fabs(pitch_controlFiltered) > 0.005f || fabs(yaw_controlFiltered) > 0.005f )
    {
      if( userControl == false )
      {
        userControl = true;
        Serial.println("Moveing..");
      }
    }
    else if( fabs(lMeasuredRpm) < (50.0f/MOTOR_MAX_RPM) && fabs(rMeasuredRpm) < (50.0f/MOTOR_MAX_RPM) )
    {
      if( userControl == true )
      {
        userControl = false;
        Serial.println("Moveing Stop.");
      }
    }
  }

  // config mode enabled when motors off
  #define PROGRAM_MODE_THRESHOLD (0.95f)
  if( motorsEnabled == false && initialBoot == false  )
  {
    pitch_controlFinal = 0;
    yaw_controlFinal = 0;

    // Toggle position hold mnode / brake when no user input
    int t = 25;
    // user must hold positon fo 1 second to set mode
    while( pitch_controlFiltered > PROGRAM_MODE_THRESHOLD && t-- ) {keepAlive();}
    if( pitch_controlFiltered > PROGRAM_MODE_THRESHOLD )
    {
      positionHoldAlways = !positionHoldAlways;
      Serial.print("Forward Position Hold = ");
      Serial.println(positionHoldAlways);
      while(fabs(pitch_controlFiltered) > 0.5f ) keepAlive();      
      Serial.println("OK");
    }

    // Toggle half forward speed
    t = 25;
    while( pitch_controlFiltered < -PROGRAM_MODE_THRESHOLD && t-- ) {keepAlive();}
    if( pitch_controlFiltered < -PROGRAM_MODE_THRESHOLD )
    {
      halfSpeed = !halfSpeed;
      Serial.print("Half Speed = ");
      Serial.println(halfSpeed);
      while(fabs(pitch_controlFiltered) > 0.5f ) keepAlive();     
      Serial.println("OK");
    }      

    // Toggle Heading Hold mode
    t = 25;
    while( yaw_controlFiltered > PROGRAM_MODE_THRESHOLD && t-- ) {keepAlive();}
    if( yaw_controlFiltered > PROGRAM_MODE_THRESHOLD )
    {
      headingHoldAlways = !headingHoldAlways;
      Serial.print("Heading Hold = ");
      Serial.println(headingHoldAlways);
      while(fabs(yaw_controlFiltered) > 0.5f ) keepAlive();     
      Serial.println("OK");
    }

    // Toggle half turn speed
    t = 25;
    while( yaw_controlFiltered < -PROGRAM_MODE_THRESHOLD && t-- ) {keepAlive();}      
    if( yaw_controlFiltered < -PROGRAM_MODE_THRESHOLD )
    {
      halfSpeedTurn = !halfSpeedTurn;
      Serial.print("Half Speed Turn = ");
      Serial.println(halfSpeedTurn);
      while(fabs(yaw_controlFiltered) > 0.5f ) keepAlive();    
      Serial.println("OK");
    }      

  }

}

// polls critical routines while in config mode
void keepAlive()
{
  BNOService();
  #if defined(ENABLE_OTA) || defined(ENABLE_UDP_DEBUG)
  wifiService();
  #endif
  updateEncoders();
  updatePWMControls();
  updateLoopVariables();
  filterPWM();
  #ifdef ENABLE_UDP_DEBUG
  sendWiFiUDPDataStream();
  #endif  
  delay(20);
}


// user inputs via serial used for trimming and calibration
void checkUserCommands()
{  
  while( Serial.available() )
  {
    char c = Serial.read();
    switch(c)
    {
      case 'P':
        pTrim += PTRIM_INC;
        Serial.print("P = ");
        Serial.println(pTrim);
        break;
      case 'p':
        pTrim -= PTRIM_INC;
        if( pTrim < 0 ) pTrim = 0;
        Serial.print("P = ");
        Serial.println(pTrim);
        break;
      case 'I':
        iTrim += ITRIM_INC;
        Serial.print("I = ");
        Serial.println(iTrim);
        break;
      case 'i':
        iTrim -= ITRIM_INC;
        if( iTrim < 0 ) iTrim = 0;
        Serial.print("I = ");
        Serial.println(iTrim);
        break;
      case 'D':
        dTrim += DTRIM_INC;
        Serial.print("D = ");
        Serial.println(dTrim);
        break;
      case 'd':
        dTrim -= DTRIM_INC;
        if( dTrim < 0 ) dTrim = 0;
        Serial.print("D = ");
        Serial.println(dTrim);
        break;              
      case '>':
        trimTemp += 0.01;
        Serial.print("Trim = ");
        Serial.println(trimTemp);
        break;
      case '<':
        trimTemp -= 0.01;
        Serial.print("Trim = ");
        Serial.println(trimTemp);
        break;
      case 'C':
        runCalibration();
        break;
    }
  }
}


void updateLoopVariables()
{
  // calculate loop delta variables
  uint32_t usNow = micros();
  loopDeltaUs = usNow - loopLastUs;   // OK across wrap-around for unsigned
  loopLastUs = usNow;
  if (loopDeltaUs < 500 ) return; // ignore silly-small dt
  loopDelta = loopDeltaUs * 1e-6f;
}

/*
 * interpolation routine
 */


float interpolate( float x, float inlo, float inhi, float outlo, float outhi )
{
  
  if( x > inhi ) return outhi;
  else if( x < inlo ) return outlo;
  
  return (   outlo + ( ( ( x - inlo ) * (outhi - outlo) ) / (inhi-inlo) ) );
}



void debugOutput()
{
    Serial.print("A MEAS:");
    Serial.print(lMeasuredRpm);
    Serial.print(", DES:");
    Serial.print(lDesiredRpm);  
    Serial.print(", OUT:");
    Serial.println(lDriveOutput);  
    //Serial.print(", HZ:");
    //Serial.print(ledcReadFreq(MOTA_PWM));
    //Serial.print(", PWM");
    //Serial.println(ledcRead(MOTA_PWM));    

    Serial.print(" B MEAS:");
    Serial.print(rMeasuredRpm);
    Serial.print(", DES:");
    Serial.print(rDesiredRpm);  
    Serial.print(", OUT:");
    Serial.println(rDriveOutput);  
    //Serial.print(", HZ:");
    //Serial.print(ledcReadFreq(MOTB_PWM));
    //Serial.print(", PWM:");
    //Serial.println(ledcRead(MOTB_PWM));    
}

void debugPWM()
{
    Serial.print("PWM P:");
    Serial.print(pitch_controlFiltered,3);
    Serial.print(",Y:");
    Serial.print(yaw_controlFiltered,3);  
    Serial.print(",A:");
    Serial.println(aux_controlFiltered,3);  
}

void debugEncoder()
{
    Serial.print("ENC A:");
    Serial.print(EncA_SpeedFiltered);
    Serial.print(", ENC B:");
    Serial.println(EncB_SpeedFiltered);  
}

// Sum over bytes excluding the crc field (last 2 bytes)
uint16_t simple_crc(const CalStore& c){
  const uint8_t* p = (const uint8_t*)&c;
  size_t n = sizeof(CalStore) - sizeof(uint16_t);
  uint32_t s = 0;
  for (size_t i=0;i<n;i++) s += p[i];
  return (uint16_t)(s & 0xFFFF);
}

void loadCal(){
  Serial.println("LOAD CALIBRATION DATA");
  EEPROM.begin(sizeof(CalStore));
  EEPROM.get(0, calData);

  // Validate
  CalStore check = calData;
  uint16_t expect = 0;
  check.crc = 0;
  expect = simple_crc(check);

  if (calData.magic != CAL_MAGIC || calData.crc != expect){
    Serial.println("\nNO PWM CALIBRATION DATA, Using defaults!");
    // defaults (deterministic)
    memset(&calData, 0, sizeof(calData));
    calData.magic = CAL_MAGIC;
    calData.pitch = { PWM_MIN, PWM_MAX, PWM_MID };
    calData.yaw = { PWM_MIN, PWM_MAX, PWM_MID };
    saveCal();
  }
}

void saveCal(){
  // ensure deterministic content
  // (not strictly needed with packed + assigned fields, but belt & braces)
  CalStore tmp;
  memset(&tmp, 0, sizeof(tmp));
  tmp.magic = calData.magic;
  tmp.pitch = calData.pitch;
  tmp.yaw = calData.yaw;
  tmp.crc   = 0;
  tmp.crc   = simple_crc(tmp);
  EEPROM.put(0, tmp);
  EEPROM.commit();
  calData = tmp; // keep in RAM too
}

void runCalibration(){
  Serial.println("\n=== PWM CALIBRATION ===");
  Serial.println("Instructions:");
  Serial.println("Move control sticks through full range of motion.\r\nwhen done return to center and press any key. Press a key to start..");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) { delay(10); }
  while (Serial.available()) Serial.read();

  calData.pitch.max_raw = 0;
  calData.pitch.min_raw = 65535;
  calData.pitch.mid_raw = PWM_MID;
  calData.yaw.max_raw = 0;
  calData.yaw.mid_raw = PWM_MID;

  while(1)
  {
    if(Serial.available()) break;

    Serial.print("PWM: P = ");
    Serial.print(pitch_PWM);
    Serial.print(" MIN = ");
    Serial.print(calData.pitch.min_raw);
    Serial.print(" MAX = ");
    Serial.print(calData.pitch.max_raw);
    Serial.print(" OUT = ");
    Serial.print(interpolate(pitch_PWM,calData.pitch.min_raw,calData.pitch.max_raw,-100,100));

    Serial.print("  Y = ");
    Serial.print(yaw_PWM);
    Serial.print(" MIN = ");
    Serial.print(calData.yaw.min_raw);
    Serial.print(" MAX = ");
    Serial.print(calData.yaw.max_raw);
    Serial.print(" OUT = ");
    Serial.println(interpolate(yaw_PWM,calData.yaw.min_raw,calData.yaw.max_raw,-100,100));
  
    delay(100);
    updatePWMControls();  

    if( pitch_PWM > calData.pitch.max_raw ) calData.pitch.max_raw = pitch_PWM;
    if( pitch_PWM < calData.pitch.min_raw ) calData.pitch.min_raw = pitch_PWM;
    if( yaw_PWM > calData.yaw.max_raw ) calData.yaw.max_raw = yaw_PWM;
    if( yaw_PWM < calData.yaw.min_raw ) calData.yaw.min_raw = yaw_PWM;
  }

  calData.pitch.mid_raw = pitch_PWM;
  calData.yaw.mid_raw = yaw_PWM;

  Serial.printf("pitch[min,mid,max]=[%d,%d,%d]\r\n",calData.pitch.min_raw,calData.pitch.mid_raw,calData.pitch.max_raw);
  Serial.printf("yaw[min,mid,max]=[%d,%d,%d]\r\n",calData.yaw.min_raw,calData.yaw.mid_raw,calData.yaw.max_raw);


  saveCal();
  Serial.println("Saved calibration data!");
  while (Serial.available()) Serial.read();
}




