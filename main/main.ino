/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
*/

#include <Servo.h>  //Need for Servo pulse output

//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

// Controller gains
#define KP_X 1
#define KI_X 0.05
#define KD_X 0.0
#define KP_Y 3
#define KI_Y 0.05
#define KD_Y 0.05
#define KP_R 300
#define KI_R 1
#define KD_R 0
//////////////

//Distance constants
#define R 0.3636 // Inverse of radius of wheels
#define L 16.25 // L2 + L3
#define D_side_sensor 10 // L1
#define D_y 7.5 // L2
#define D_x 1   // L3
//////////////

// Global variables for controller calculation
double runningEx = 0.00;
double prevEx = 0.00;
double runningEy = 0.00;
double prevEy = 0.00;
double runningEtheta = 0.00;
double prevEtheta = 0.00;
//////////////

double w[4]={0,0,0,0}; // Motor power values
double ex, ey, etheta = 0; // Errors for each direction

// FSM global variables
bool aligned = false;
bool wallCompleted = false;
int wallCtr = 0;
//////////////

//IR sensor global variables
double distanceIR1 = 0;   // Front left IR sensor
double distanceIR2 = 0;   // Back left IR sensor
double distanceIR3 = 0;   // Front IR sensor
#define sensorNoise 1
#define processNoise 1
int irSensor[3] = {A2,A3,A4};  // Pin for each sensor
double sensorConstant[3]={36291, 36731, 36317}; 
double sensorPow[3]={-1.11, -1.106, -0.89};
int signalADC = 0;  // The read ADC signal in 0-1023 corresponding to 0-5v 

double prevEstimate[3]={0.0, 0.0, 0.0};
double prevVariance[3]={0.0, 0.0, 0.0};
//////////////

// Coordinate system global variables
double coordinate[3]={0.0,0.0,0.0};  //0: X, 1: Y, 2: Theta
double startingDistance; // used for turning state
//////////////

//FSM states
enum STATE {
  INITIALISING,
  RUNNING_FORWARD,
  RUNNING_TURNING,
  RUNNING_ALIGN,
  STOPPED,
  COMPLETED
};
STATE previousState = RUNNING_FORWARD;

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed
}

void loop(void) //main loop
{
  
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {  
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING_FORWARD:
      machine_state = runningForward();
      break;
    case RUNNING_TURNING:
      machine_state = runningTurning();
      break;
    case RUNNING_ALIGN:
      machine_state = runningAlign();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
    case COMPLETED:
      machine_state = completed();
      break;
  };
}

STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING_ALIGN;
}

STATE runningAlign() {

  static unsigned long previous_millis; 

  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 300) {  //Arduino style 300ms timed execution statement
    previous_millis = millis();

    readIR(); //READ THE IR SENSOR
    convertToCoordinate(distanceIR1, distanceIR2, distanceIR3);

    //ERROR CALCULATION
    double errorX = 0.0; // disable x direction controller by forcing x error to always 0 
    double errorY = 150 - D_y - coordinate[1]; // 15cm - length from sensor to middle of robot is reference
    double errorAngle = 0 - coordinate[2]; // 0 reference to align with wall

    motorController(errorX, errorY, errorAngle);
    driveMotors();

    // Criteria for robot to be aligned
    if(abs(errorAngle) <= 4*DEG_TO_RAD && abs(errorY) <= 5.0) { 
      aligned = true;
    }

  }

  previousState = RUNNING_ALIGN;
  if (!is_battery_voltage_OK()) return STOPPED;

  if(aligned) {
    wallCtr++; // increment the wall counter, since wall is about to be traversed
    return RUNNING_FORWARD;
  }

  return RUNNING_ALIGN;

}

STATE runningForward() {

  static unsigned long previous_millis;

  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 300) {  //Arduino style 300 ms timed execution statement
    previous_millis = millis();
    
    readIR(); //READ THE IR SENSOR
    convertToCoordinate(distanceIR1, distanceIR2, distanceIR3);

    //ERROR CALCULATION
    double errorX = 150 - D_x - coordinate[0]; // 15cm - length from sensor to middle of robot is reference
    double errorY = 150 - D_y - coordinate[1]; // 15cm - length from sensor to middle of robot is reference
    double errorAngle = 0 - coordinate[2]; // theta 0 degree reference to keep alignment with wall

    motorController(errorX, errorY, errorAngle);
    driveMotors();

    //Exit case once wall following is completed within a 5 mm margin of error
    if(abs(errorX) <= 5.0 && abs(errorY) <= 5.0 && abs(errorAngle) <= 4*DEG_TO_RAD) { 
      wallCompleted = true;
    }
  }

  previousState = RUNNING_FORWARD;
  if (!is_battery_voltage_OK()) return STOPPED;


  // Once wall follow is completed, if four walls have already been traversed go to the completed state,
  // otherwise go to the turning state.
  if(wallCompleted && wallCtr < 4) {
    startingDistance=coordinate[0];
    return RUNNING_TURNING;
  } else if (wallCompleted && wallCtr > 3) {
    return COMPLETED; 
  }

  return RUNNING_FORWARD;
}


STATE runningTurning() {

  static unsigned long previous_millis; //this is the dumbest shit ever

  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 300) {  //Arduino style 300 ms timed execution statement
    previous_millis = millis();

    readIR(); //READ THE IR SENSOR

    //ERROR CALCULATION
    // use the convertToAngle function to find phi, and use this along with reference of 75deg in radians for error calculation
    double errorAngle = 75*DEG_TO_RAD - convertToAngle(distanceIR3); 
    double errorX = 0.0; // disable controller in x direction
    double errorY = 0.0; // disable controller in y direction

    motorController(errorX, errorY, errorAngle);
    driveMotors();

    // criteria for moving to align state
    if(abs(errorAngle) <= 4*DEG_TO_RAD) { 
      wallCompleted = false;
      aligned = 0;
      return RUNNING_ALIGN;
    }
  }

  previousState = RUNNING_TURNING;
  if (!is_battery_voltage_OK()) return STOPPED; // check battery level

  return RUNNING_TURNING;
}

STATE completed() {
  return COMPLETED;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


    #ifndef NO_BATTERY_V_OK
      //500ms timed if statement to check lipo and output speed settings
      if (is_battery_voltage_OK()) {
        SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
        SerialCom->println(counter_lipo_voltage_ok);
        counter_lipo_voltage_ok++;
        if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
          counter_lipo_voltage_ok = 0;
          enable_motors();
          SerialCom->println("Lipo OK returning to RUN STATE");
          return previousState;  // Return to the previous state to continue running
        }
      } else {
        counter_lipo_voltage_ok = 0;
      }
    #endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

void Analog_Range_A4() {
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}



//Serial command pasing
void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      case 'p':
      case 'P':
        w[0] = 0;
        w[3] = 0;
        w[2] = 0;
        w[1] = 0;
       
        SerialCom->println("PAUSED");
       break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }

  }

}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

/*
   _____ _    _  _____ _______ ____  __  __            ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
  / ____| |  | |/ ____|__   __/ __ \|  \/  |          |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
 | |    | |  | | (___    | | | |  | | \  / |  ______  | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___  
 | |    | |  | |\___ \   | | | |  | | |\/| | |______| |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
 | |____| |__| |____) |  | | | |__| | |  | |          | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
  \_____|\____/|_____/   |_|  \____/|_|  |_|          |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/ 
                                                                                                                   
*/                                                                                                                   

// This function powers the motors with the required motor speeds based on the global angular velocity: w
// No input or output
void driveMotors() {
    left_font_motor.writeMicroseconds(1500 + w[0]);
    right_font_motor.writeMicroseconds(1500 - w[1]);
    left_rear_motor.writeMicroseconds(1500 + w[2]);
    right_rear_motor.writeMicroseconds(1500 - w[3]);
}

// This function is used to get the distance measured by a given IR Sensor
// Input: sensorNumber : number between 0 - 2 to represent the sensor you want to read (where 0: sensor 1, 1: sensor 2, 2: sensor 3)
// Output: the distance read by the given sensor
double sensorReading(int sensorNumber){
  int signalADC = analogRead(irSensor[sensorNumber]);   // the read out is a value from 0-1023 corresponding to 0-5v
  
  if(signalADC == 0) { // lift 0 adc to 1
    signalADC = 1;
  }

  double distance = sensorConstant[sensorNumber]*pow(signalADC, sensorPow[sensorNumber]);  // calculate the distance using the calibrated graph
  double newEstimate = kalman_Filter(distance, sensorNumber); // Use kalman filter to calculate better estimate

  prevEstimate[sensorNumber]=newEstimate;
  return newEstimate;
} 


// This function acts as the controller, and calculated the required linear velocity (x & y) and angular velocity to reduce the passed error values to zero
// Input: ex: error in x direction
//        ey: error in y direction
//        eTheta: error in theta direction
void motorController(double ex, double ey, double etheta) {
  double V[3] = {0.0, 0.0, 0.0}; //Velocity in x, y, theta
  
  // X Controller
  runningEx = runningEx + ex;
  V[0] = ex*KP_X + runningEx*KI_X + (ex - prevEx)*KD_X;
  prevEx = ex;

  // Y Controller
  runningEy = runningEy + ey;
  V[1] = ey*KP_Y + runningEy*KI_Y + (ey - prevEy)*KD_Y;
  prevEy = ey;

  // Theta Controller
  runningEtheta = runningEtheta + etheta;
  V[2] = etheta*KP_R + runningEtheta*KI_R + (etheta - prevEtheta)*KD_R;
  prevEtheta = etheta;

  // Saturate the x velocity to max 400
  if(V[0] > 400) {
    V[0] = 400;
  } else if (V[0] < -400) {
    V[0] = -400;
  }

  populateWheelVelocities(V); // convert MWR velocities to wheel velocities
}

// This function converts a passed array of velocities of the MWR to individual wheel angular velocities and stores them in
// the global array 'w', with no unit
// Input: array V: velocities of the MWR (0: Vx, 1: Vy, 2: Angular velocity in theta direction)
void populateWheelVelocities(double V[3]) {

  // Equations derived from the inverse kinematics matrix
  w[0] = (V[0] + V[1] - (L*V[2])) * R;
  w[1] = (V[0] - V[1] + (L*V[2])) * R;
  w[2] = (V[0] - V[1] - (L*V[2])) * R;
  w[3] = (V[0] + V[1] + (L*V[2])) * R;
  
  // Saturate angular velocities to 200
  for(int k = 0; k < 4; k++) {
    if(w[k] > 200 ) {
      w[k] = 200;
    } else if (w[k] < -200) {
      w[k] = -200;
    }
  }
}


// This function converts the IR sensor readings into the coordinate system and saves the coordinate system in the global coordinate array (where
// 0: x , 1: y, 2: theta)
// Inputs: reading1: distance reading from IR sensor 1
//         reading2: distance reading from IR sensor 2
//         reading3: distance reading from IR sensor 3
void convertToCoordinate(double reading1,double reading2,double reading3){ //for align (error x =0) and follow (calc all errors)
  coordinate[2]=atan((reading1-reading2)/D_side_sensor);  //pos cw
  coordinate[1]=(((reading1+reading2)/2)+D_y)*cos(coordinate[2]);
  coordinate[0]=(reading3+D_x)*cos(coordinate[2]);
}

// This function converts the reading from the front IR sensor (sensor 3) into phi
// Inputs: frontReading: distance reading from IR sensor 3
// Output: the special angle phi, when in runningTurning state
double convertToAngle(double frontReading){ //for runningTurning, returns the angle (dont use coordinate[2]) and set errorx and y to 0
  double hypotenuse=D_x+frontReading;
  double phi=acos(startingDistance/hypotenuse);
  return phi;
}

// This function applies a kalman filter to a given sensor
// Inputs: newData: the reading in mm from the IR sensor 
//         sensorNumber: integer representing the sensor being used (0: IR sensor 1, 1: IR sensor 2, 2: IR sensor 3)
// Outputs: the result of the kalman filter
double kalman_Filter(double newData, int sensorNumber){
  double newEstimate, kalmanGain,meanInitial, varianceInitial;
  // Prediction
  meanInitial=prevEstimate[sensorNumber];
  varianceInitial=prevVariance[sensorNumber]+processNoise;  
  // Correction
  kalmanGain=varianceInitial/(varianceInitial+sensorNoise);
  newEstimate=meanInitial+kalmanGain*(newData-meanInitial);
  prevVariance[sensorNumber]=(1-kalmanGain)*varianceInitial;
  return newEstimate;
}

// This function reads all the IR sensors and saves the values in the global variables distanceIR1, distanceIR2 and distanceIR3
void readIR(){
  distanceIR1 = sensorReading(0); //Check sensor order
  
  distanceIR2 = sensorReading(1);
  
  distanceIR3 = sensorReading(2);

}
