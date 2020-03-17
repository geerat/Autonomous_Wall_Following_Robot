int irsensor[3] = {A3,A2,A4} ;     //sensor is attached
int prev_estimate[3]={0,0,0};
int signalADC = 0;  // the read out signal in 0-1023 corresponding to 0-5v 
int Sensor_Constant[3]={36291,36731,3631.7};
int Sensor_pow[3]={-1.11,-1.106,-0.89};
int Kalman_constant=0.9;

int sensorreading(int sensor_number){
  signalADC = analogRead(irsensor);   // the read out is a signal from 0-1023 corresponding to 0-5v
  int distance = Sensor_Constant[sensor_number]*pow(signalADC,Sensor_pow[sensor_number]);  // calculate the distance using the calibrated graph
  int new_estimate=distance*Kalman_constant+(1-Kalman_constant)*prev_estimate[sensor_number];
  prev_estimate[sensor_number]=new_estimate;
  return new_estimate;
} 
