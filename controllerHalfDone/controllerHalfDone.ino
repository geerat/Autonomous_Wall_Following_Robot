#define KP_X 1
#define KI_X 0.1
#define KD_X 0.01

#define KP_Y 1
#define KI_Y 0.1
#define KD_Y 0.01

#define KP_R 1
#define KI_R 0.1
#define KD_R 0.01

#define R 2.75
#define L2 7.5
#define L1 8.75


double runningEx = 0.00;
double prevEx = 0.00;

double runningEy = 0.00;
double prevEy = 0.00;

double runningEr = 0.00;
double prevEr = 0.00;

void setup() {
  // put your setup code here, to run once:

}

void loop() {


  double * W = motorController(ERRORS);

}


//TODO Combine x, y and r controller
double * motorController(double ex, double ey, double er) {

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

  
  return getMotorPower(V);
}

double * getMotorPower(double V[3]) {

  double kinematicArray[4][3] = {{1, 1, -(L2 + L1)}, {1, -1, (L1+L2)}, {1, -1, -(L1+L2)}, {1, 1, (L1+L2)}};
  double motorPower[4] = {0.0, 0.0, 0.0, 0.0};
  
  for(int j = 0; j < 4; j++) {// rows
    for (int i = 0; i < 3; i++) { // columns
          
        motorPower[j] = motorPower[j] + kinematicArray[j][i]*V[i];
     
    }
  }

  return motorPower;
}
