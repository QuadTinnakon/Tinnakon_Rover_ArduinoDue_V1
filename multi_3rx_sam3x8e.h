/*
project_Tinnakon_Rover 32 bit Arduino Due
Tinnakon_Rover
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
//clock timer7   (1000 - 2000 us) = (2625 - 5250) = 2625 
*/
float CH_THR = 1000.0;
float CH_AIL = 1500.0;
float CH_ELE = 1500.0;
float CH_RUD = 1500.0;
float AUX_1 = 1000.0;
float AUX_2 = 1000.0;
float AUX_3 = 1000.0;
float AUX_4 = 1000.0;
float CH_THRf = 1000.0;
float CH_AILf = 1500.0;
//float CH_ELEf = 1500.0;
//float CH_RUDf = 1500.0;
int CH_AIL_Cal = 1500;
//int CH_ELE_Cal = 1500;
//int CH_RUD_Cal = 1500;

//RX PIN assignment inside the port
//SET YOUR PINS! TO MATCH RECIEVER CHANNELS
#define CHAN1PIN 62 // RECIEVER 1 PPM ,pin A8
#define CHAN2PIN 63 // RECIEVER 2     ,pin A9
#define CHAN3PIN 64 // RECIEVER 3     ,pin A10
#define CHAN4PIN 65 // RECIEVER 4     ,pin A11
//#define CHAN5PIN 61 // RECIEVER 5
//#define CHAN6PIN 67 //not used at the moment
//#define CHAN7PIN 50 //not used at the moment
//#define CHAN8PIN 51 //not used at the moment

#define ROLL       0
#define PITCH      4
#define YAW        5
#define THROTTLE   1
#define AUX1       2
#define AUX2       3
#define CAMPITCH   6
#define CAMROLL    7

volatile unsigned long pwmLast[8];
//volatile unsigned long last = 0;
uint8_t chan1 = 0;
volatile uint32_t rcValue[8] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]
void pwmHandler(int ch, int pin) {
  unsigned long cv = micros();
  if (digitalRead(pin)) {
    pwmLast[ch] = cv;
  } else {
    cv = (cv - pwmLast[ch]); // / 42;
    if (cv>900 && cv<2200) {
      rcValue[ch] = cv;
    }
  }
}
void ch1Handler() { pwmHandler(0, CHAN1PIN); }
void ch2Handler() { pwmHandler(1, CHAN2PIN); }
void ch3Handler() { pwmHandler(2, CHAN3PIN); }
void ch4Handler() { pwmHandler(3, CHAN4PIN); }
//void ch5Handler() { pwmHandler(4, CHAN5PIN); }
void configureReceiver() {
  Serial.print("Configure Receiver CPPM");Serial.print("\n");
  for (uint8_t chan = 0; chan < 8; chan++) {
    for (uint8_t a = 0; a < 4; a++) {
      rcValue[a] = 3937;//1500 = 3937.5
    }
  }
  rcValue[THROTTLE] = 1000;//2625 = 1000 us
  rcValue[AUX1] = 1000;
  rcValue[AUX2] = 1000;
  rcValue[CAMPITCH] = 1000;
  rcValue[CAMROLL] = 1000;
  attachInterrupt(CHAN1PIN,ch1Handler,CHANGE); //RISING  FALLING CHANGE
  attachInterrupt(CHAN2PIN,ch2Handler,CHANGE);
  attachInterrupt(CHAN3PIN,ch3Handler,CHANGE);
  attachInterrupt(CHAN4PIN,ch4Handler,CHANGE);
  //attachInterrupt(CHAN5PIN,ch5Handler,CHANGE);
}

void computeRC() {
  CH_THR = rcValue[THROTTLE];
  CH_AIL = rcValue[ROLL];
  AUX_1 = rcValue[AUX1];
  AUX_2 = rcValue[AUX2];
  CH_THRf = CH_THRf + (CH_THR - CH_THRf) * 0.02 / tarremote; //Low pass filter
  CH_AILf = CH_AILf + (CH_AIL - CH_AILf) * 0.02 / tarremote; //Low pass filter
}
void RC_Calibrate() {
  Serial.print("RC_Calibrate"); Serial.println("\t");  //By tinnakon
  for (int i = 0; i < 10; i++) {
    computeRC();
    delay(20);
  }
  CH_AIL_Cal = CH_AIL;
  //CH_ELE_Cal = CH_ELE;
  //CH_RUD_Cal = CH_RUD;
  Serial.print(CH_AIL_Cal); Serial.println("\t"); //1505
  //Serial.print(CH_ELE_Cal); Serial.print("\t"); //1498
  //Serial.print(CH_RUD_Cal); Serial.println("\t"); //1502
}
