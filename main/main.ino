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

#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.


// CONTROLLER VARIABLES

#define KP_X 1
#define KI_X 0
#define KD_X 0.0

#define KP_Y 3
#define KI_Y 0
#define KD_Y 0

#define KP_R 0.3
#define KI_R 0
#define KD_R 0

#define R 2.75
#define L2 7.5
#define L1 8.75


double runningEx = 0.00;
double prevEx = 0.00;

double runningEy = 0.00;
double prevEy = 0.00;

double runningEr = 0.00;
double prevEr = 0.00;

double kinematicArray[4][3] = {{1, 1, -(L2 + L1)}, {1, -1, (L2+L1)}, {1, -1, -(L1+L2)}, {1, 1, (L1+L2)}};
double w[4]={0,0,0,0};
double ex, ey, er = 0;

double distanceIR1 = 0;
double distanceIR2 = 0;
double distanceIR3 = 0;

////


//IR SENSOR VARIABLE
#define KALMAN_CONSTANT 1
int irSensor[3] = {A2,A3,A4} ;     //sensor is attached
double prevEstimate[3]={0.0, 0.0, 0.0};
double sensorConstant[3]={36291, 36731, 36317};
double sensorPow[3]={-1.11, -1.106, -0.89};

//

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;
int irsensor = A2;     //sensor is attached on pinA0
byte serialRead = 0;  //for control serial communication 
int signalADC = 0;  // the read out signal in 0-1023 corresponding to 0-5v 



//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;


int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
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
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
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

  
  return RUNNING;
}

STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement

    readIR(); //READ THE IR SENSOR
    
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    
   SerialCom->println("SENSOR 1: ");
    SerialCom->println(sensorReading(0));

    SerialCom->println("SENSOR 2: ");
    SerialCom->println(sensorReading(1));

    SerialCom->println("SENSOR 3: ");
    SerialCom->println(sensorReading(2));


    SerialCom->println("Errors - x, y, r");

    if(distanceIR1 == -1 || distanceIR2 == -1) {
      ey = 0;
      er = 0;
    } else {
       ey = ((distanceIR1 + distanceIR2)/2) - 77.5;
       er = distanceIR2 - distanceIR1;
    }

    if(distanceIR3 == -1) {
      ex = 0;
    } else {
      ex = 150 - distanceIR3;
    }
   
    

    SerialCom->println("Errors - x, y, r");
    SerialCom->println(ex);
    SerialCom->println(ey);
    SerialCom->println(er);

    motorController(ex, ey, er,w);
    
    SerialCom->println("motor powers");
    SerialCom->println(w[0]);
    SerialCom->println(w[1]);
    SerialCom->println(w[2]);
    SerialCom->println(w[3]);


    driveMotors();
    /*
    left_font_motor.writeMicroseconds(1500 - w[0]);
    right_font_motor.writeMicroseconds(1500 - w[1]);
    left_rear_motor.writeMicroseconds(1500 - w[2]);
    right_rear_motor.writeMicroseconds(1500 - w[3]);
    delay(500);
    //speed_change_smooth();*/
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC-SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


    turret_motor.write(pos);

    if (pos == 0)
    {
      pos = 45;
    }
    else
    {
      pos = 0;
    }
  }

  return RUNNING;
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
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
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



void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
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

#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command()
{
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

//This function powers the motors with the required motor speeds
void driveMotors() {
    left_font_motor.writeMicroseconds(1500 + w[0]);
    right_font_motor.writeMicroseconds(1500 - w[1]);
    left_rear_motor.writeMicroseconds(1500 + w[2]);
    right_rear_motor.writeMicroseconds(1500 - w[3]);
}

//This function is used to get the distance measured by a particular IR Sensor
//Input: int sensor_number - number between 0 - 2 to represent the sensor you want to read
//Output: the distance read by the sensor
double sensorReading(int sensorNumber){
  


  int signalADC = analogRead(irSensor[sensorNumber]);   // the read out is a signal from 0-1023 corresponding to 0-5v

  if(signalADC == 0) {
    signalADC = 1;
  }

  double distance = sensorConstant[sensorNumber]*pow(signalADC, sensorPow[sensorNumber]);  // calculate the distance using the calibrated graph
  double newEstimate = distance*KALMAN_CONSTANT+(1-KALMAN_CONSTANT)*prevEstimate[sensorNumber] + 8;
  


  prevEstimate[sensorNumber]=newEstimate;
  return newEstimate;
} 






//TODO Combine x, y and r controller
void motorController(double ex, double ey, double er, double motorPower[4]) {
   double V[3] = {0.0, 0.0, 0.0}; //x, y, r
  
  //X Controller

  runningEx = runningEx + ex;
  
  V[0] = ex*KP_X + runningEx*KI_X + (ex - prevEx)*KD_X;
  prevEx = ex;


  //Y Controller
  runningEy = runningEy + ey;
  
  V[1] = ey*KP_Y + runningEy*KI_Y + (ey - prevEy)*KD_Y;
  prevEy = ey;

  //R Controller
  runningEr = runningEr + er;
  
  V[2] = er*KP_R + runningEr*KI_R + (er - prevEr)*KD_R;
  prevEr = er;



  if(V[0] > 400) {
    V[0] = 400;
  } else if (V[0] < -400) {
    V[0] = -400;
  }
  
  SerialCom->println("Velocities:");
  SerialCom->println(V[0]);
  SerialCom->println(V[1]);
  SerialCom->println(V[2]);
  
  getMotorPower(V,motorPower);
}



void getMotorPower(double V[3] ,double motorPower[4]) {

  
 /* for(int j = 0; j < 4; j++) {// rows
    for (int i = 0; i < 3; i++) { // columns
          
        motorPower[j] = motorPower[j] + ((kinematicArray[j][i]*V[i])/R);
     
    }
  }*/

  motorPower[0] = (V[0] + V[1] - ((L1+L2)*V[2]))/R;
  motorPower[1] = (V[0] - V[1] + ((L1+L2)*V[2]))/R;
  motorPower[2] = (V[0] - V[1] - ((L1+L2)*V[2]))/R;
  motorPower[3] = (V[0] + V[1] + ((L1+L2)*V[2]))/R;
  
  for(int k = 0; k < 4; k++) {
    
    if(motorPower[k] > 200 ) {
      motorPower[k] = 200;
      SerialCom->println("SATURATE!!!");
    } else if (motorPower[k] < -200) {
      motorPower[k] = -200;
      SerialCom->println("SATURATE!!!");
    }
    
  }
}



void readIR(){
  distanceIR1 = sensorReading(0);
  
  distanceIR2 = sensorReading(1);
  
  distanceIR3 = sensorReading(2);

 /* if(distanceIR1 > 700) {
    distanceIR1 = -1;
  }

  if(distanceIR2 > 700) {
    distanceIR2 = -1;
  }

  if(distanceIR3 > 2000) {
    distanceIR3 = -1;
  }*/

}
