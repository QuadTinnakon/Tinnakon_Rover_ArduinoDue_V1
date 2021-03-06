/*
project_Tinnakon_Rover 32 bit Arduino Due
Tinnakon_Rover
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza
*/
float u_roll = 0;
float u_pitch = 0;
float u_yaw = 0;

float roll_I_rate = 0.0;
float roll_D_rate = 0.0;
float setpoint_rollold = 0.0;
float setpoint_rate_roll = 0.0;
float error_rollold = 0.0;
float error_rate_rollold = 0.0;
float pitch_I_rate = 0.0;
float pitch_D_rate = 0.0;
float setpoint_pitchold = 0.0;
float setpoint_rate_pitch = 0.0;
float error_pitchold = 0.0;
float error_rate_pitchold = 0.0;
float yaw_I_rate = 0.0;
float yaw_D_rate = 0.0;
float error_rate_yawold = 0.0;

//Automatic take-off and landing
float err_hz = 0.0;
int time_auto = 0;
float h_counter = 0.1;//0.08
float h_counter_old = 0.1;
float Vz_Hold = 0.0;
float hz_I = 0.0;
float hz_D_rate = 0.0;
float error_Vz_old = 0.0;
uint8_t takeoff = 0;
uint8_t endAuto = 0;

void Control_PPIDRate(){
// ROLL CONTROL P-PID  control  By tinnakon///////////
  float setpoint_roll = ((CH_AILf-CH_AIL_Cal)*0.085) + Control_YBf;//0.12 max +-45 deg  ////+-18  + Control_YBf
  applyDeadband(setpoint_roll, 2.5);//1.2
  setpoint_rate_roll = (0.065*setpoint_rate_roll/(0.065+G_Dt)) + ((setpoint_roll-setpoint_rollold)/(0.065+G_Dt));//Diff remote
  setpoint_rollold = setpoint_roll;
  setpoint_rate_roll = constrain(setpoint_rate_roll, -80, 80);//+-80 deg/s
  float error_roll = setpoint_roll - ahrs_r;//ahrs_r*ToDeg
  float error_rate_roll = setpoint_rate_roll + error_roll*Kp_levelRoll  - GyroXf*RAD_TO_DEG;
  roll_I_rate += error_rate_roll*Ki_rateRoll*G_Dt;
  roll_I_rate = constrain(roll_I_rate, -50, 50);//+-150
  roll_D_rate = (tar*roll_D_rate/(tar+G_Dt)) + ((error_rate_roll-error_rate_rollold)/(tar+G_Dt));
  error_rate_rollold = error_rate_roll;
  u_roll = Kp_rateRoll*error_rate_roll + roll_I_rate + Kd_rateRoll*roll_D_rate;
  u_roll = constrain(u_roll, -280, 280);//+-250 +-300 120
}
void Chack_Command(){
   if(AUX_1 <= (AltHold-10))//Stabilize 
  {
    Mode = 0;
  }
   if(AUX_1 > (AltHold-10) && AUX_1 <= (AltHold+10))//Altitude Hold, 
  {
    Mode = 1;
  }
   if(AUX_1 > (PositionHold-10) && AUX_1 <= (PositionHold+10))//Position Hold
  {
    Mode = 2;
  }  
  if(AUX_1 > (Auto-10))//Automatic  Takeoff
  {
    Mode = 3;
  }
  if(AUX_1 > (RTH-10) && AUX_1 <= (RTH+10))//RTH
  {
   Mode = 2;
   target_LAT = GPS_LAT_HOME;//GPS_LAT_Hold
   target_LON = GPS_LON_HOME;//GPS_LON_Hold
  }
  //////////////////
   if(AUX_2 <= 1300)//Set Home 
  {
    GPS_LAT_HOME = GPS_LAT1f;
    GPS_LON_HOME = GPS_LON1f;
    //digitalWrite(Pin_LED_G, HIGH);
    //digitalWrite(Pin_LED_R, LOW);
  }
   if(AUX_2 >= 1700)//Set waypoint1  
  {
    waypoint1_LAT = GPS_LAT1f;
    waypoint1_LON = GPS_LON1f;
    //digitalWrite(Pin_LED_R, HIGH);
    //digitalWrite(Pin_LED_G, LOW);
  }
}//end Chack_Command()
///////////////////////////////////////////////////////////////////////////////////
void Automatictakeland(){
 //Altitude control and 1 waypoint navigation
  if(Mode == 3 && CH_THRf > MINCHECK && armed == 1)
  {
    if(time_auto < 5){//Check time < 5
      takeoff = 1;
    }
     if(z1_hat >= h_control && endAuto == 1)//waypoint1
    {
      target_LAT = waypoint1_LAT;
      target_LON = waypoint1_LON;
      Status_waypoint = 1;
    }
     if(time_auto > 8 && abs(error_LAT) <= 70 && abs(error_LON) <= 70 && endAuto == 1 && Status_waypoint == 1)//50 10 Landing and position hold mode
    {
      timeLanding++;
      if(timeLanding >= 20)//relay 2 s Landing
      {
        takeoff = 0;
      }
    }
  }
  else//(Mode == 3 && CH_THR > MINCHECK && armed == 1)
  {
    takeoff = 0;
    timeLanding = 0;
    timeOff = 0;
    time_auto = 0;
    h_counter = 0.1;//0.0
    h_counter_old = 0.1;
    Vz_Hold = 0.0;
    Status_waypoint = 0;
  } 
////////////////////////////////////////////////////////////////// 
       if(h_counter < h_control && takeoff == 1)//take-off
      {
        endAuto = 1;
        h_counter = h_counter + 0.019;//0.023 0.01- 0.03  0.1 = 10 s  //ramp input hz take-off
        Vz_Hold = (h_counter - h_counter_old)/0.1;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
      }
       if(takeoff == 0 && endAuto == 1)//landing
      {
        h_counter = h_counter - 0.019;//0.023 ramp input hz  landing
        Vz_Hold = (h_counter - h_counter_old)/0.1;//(m/s) roop 10 Hz
        h_counter_old = h_counter;
        if(z1_hat <= 0.13)
        {
         endAuto = 0;
        }
      }
////////////////////////////////////
      if(time_auto > 15 && endAuto == 0) //15 s End waypoint quadrotor
        {
          timeOff++;
          if(timeOff > 10)//relay 1 s time-Off
          {
           armed = 0;
          } 
        }  
///////////////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////
void Control_PositionHold(){
  if(Mode == 2 || Mode == 3 && GPS_FIX  == 1){
          error_LAT = (target_LAT - GPS_LAT1f)*6371000.0; // X Error cm
          error_LON = (target_LON - GPS_LON1f)*6371000.0;// Y Error   cm
          error_LAT = constrain(error_LAT,-500,500);//200 = +-2 m
          error_LON = constrain(error_LON,-500,500);
          float target_speedLAT = error_LAT*Kp_speed;//P Control Velocity GPS
          float target_speedLON = error_LON*Kp_speed;//P Control Velocity GPS
          target_speedLAT = constrain(target_speedLAT,-200,200);//+-100 cm/s = 1m/s
          target_speedLON = constrain(target_speedLON,-200,200);

          float error_rate_LAT = target_speedLAT - vx_hat;
          float error_rate_LON = target_speedLON - vy_hat;
          
          //float error_rate_LAT = 0.0 - posistion_Y;
          //float error_rate_LON = 0.0 - posistion_X;

          error_rate_LAT = constrain(error_rate_LAT,-300,300);//+-200 cm/s
          error_rate_LON = constrain(error_rate_LON,-300,300);
          GPSI_LAT = GPSI_LAT + (error_rate_LAT*Ki_gps*0.05);//20 Hz = 0.05
          GPSI_LON = GPSI_LON + (error_rate_LON*Ki_gps*0.05);  
          GPSI_LAT = constrain(GPSI_LAT,-200,200);//win speed +-200 cm/s
          GPSI_LON = constrain(GPSI_LON,-200,200);//250
          //Control_XEf = error_rate_LAT*Kd_gps;//P Control speed 
          //Control_YEf = error_rate_LON*Kd_gps;
          Control_XEf = error_LAT*Kp_gps + error_rate_LAT*Kd_gps + GPSI_LAT;//PID Control speed 
          Control_YEf = error_LON*Kp_gps + error_rate_LON*Kd_gps + GPSI_LON;
          Control_XEf = constrain(Control_XEf,-800,800);//PWM 1000 - 1900
          Control_YEf = constrain(Control_YEf,-800,800);
          //The desired roll and pitch angles by tinnakon 
          float urolldesir = (Control_YEf*m_quad)/uAltitude;//uAltitude/2 = 1000 - 1900
          float upitchdesir = (Control_XEf*m_quad*-1.0)/uAltitude;//*-1
          urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          upitchdesir = constrain(upitchdesir,-0.7,0.7);
          float temp_YBf = asin(urolldesir)*RAD_TO_DEG;//Control Earth Frame
          float temp_XBf = asin(upitchdesir)*RAD_TO_DEG;//Control Earth Frame
          Control_XBf = (DCM00*temp_XBf) + (DCM01*temp_YBf);//Control Body Frame use Rotation matrix
          Control_YBf = (DCM10*temp_XBf) + (DCM11*temp_YBf);//Control Body Frame use Rotation matrix
          Control_XBf = constrain(Control_XBf, -20, 20);//+-20 deg
          Control_YBf = constrain(Control_YBf, -20, 20);//+-20 deg
          
          //The desired roll and pitch angles by paper Modeling and Backstepping-based Nonlinear Control 
          //Ashfaq Ahman Mian 2008 (eq.25 and eq.26)
          //float urolldesir = ((Control_XEf*m_quad*sin(ahrs_y))/uAltitude) - ((Control_YEf*m_quad*cos_yaw)/uAltitude);
          //float upitchdesir = ((Control_XEf*m_quad)/(uAltitude*cos_roll*cos_yaw)) - ((sin(ahrs_r)*sin(ahrs_y))/(cos_roll*cos_yaw));
          //urolldesir = constrain(urolldesir,-0.7,0.7);//+-0.7 = +-44.427
          //upitchdesir = constrain(upitchdesir,-0.7,0.7);
          //Control_YBf = asin(urolldesir)*RAD_TO_DEG;//Control roll eq.25 
          //Control_XBf = asin(upitchdesir)*RAD_TO_DEG*-1.0;//Control roll eq.26
          //Control_XBf = constrain(Control_XBf, -20, 20);//+-20 +- 44
          //Control_YBf = constrain(Control_YBf, -20, 20);
  }
  else{
      Control_XBf = 0.0;
      Control_YBf = 0.0;
      GPSI_LAT = 0.0;
      GPSI_LON = 0.0;
      target_LAT = GPS_LAT1f;//GPS_LAT_Hold
      target_LON = GPS_LON1f;//GPS_LON_Hold
  }
}//end  Control_PositionHold()
/////////////////////////////////////////////////////////////////////////
void Cal_GPS(){
  if(GPS_FIX  == 1){
 //Apply moving average filter to GPS data
      GPS_filter_index = (GPS_filter_index+1) % 4;// 4 moving average
      GPS_SUM_LAT[GPS_filter_index] = GPS_LAT1;
      GPS_SUM_LON[GPS_filter_index] = GPS_LON1;
   float sum1=0.0;
   float sum2=0.0;
  for(int i=0;i<4;i++)
  {
    sum1 += GPS_SUM_LAT[i];
    sum2 += GPS_SUM_LON[i];
  }
   GPS_LAT1f = sum1/4.0;
   GPS_LON1f = sum2/4.0;
  }
/* ///////////////////////////////////////
     //Diff speed
     actual_speedX = (GPS_LAT1f - GPS_LAT1_old)*6371000.0/0.3;//5 Hz = 0.2 ,cm/s  10000000.0 ,R = 6371000.0
     actual_speedY = (GPS_LON1f - GPS_LON1_old)*637100.0/0.3;//cm/s
     //actual_speedX = constrain(actual_speedX, -400, 400);//+-400 cm/s
     //actual_speedY = constrain(actual_speedY, -400, 400);//+-400 cm/s  
     GPS_LAT1_old = GPS_LAT1f;
     GPS_LON1_old = GPS_LON1f;
     actual_speedXf = (actual_speedX + actual_speedXold)/2.0;//Moving Average Filters/
     actual_speedYf = (actual_speedY + actual_speedYold)/2.0;//Moving Average Filters/
     actual_speedXold = actual_speedX;
     actual_speedYold = actual_speedY;
  
/////////////LeadFilter GPS/////////////////////////////////
    float lag_in_seconds = 0.85;//1.0 0.5
    float accel_contribution = (actual_speedXf - _last_velocityX) * lag_in_seconds * lag_in_seconds;
    float vel_contribution = actual_speedXf * lag_in_seconds;
    _last_velocityX = actual_speedXf;    // store velocity for next iteration
    GPS_LAT1lead = GPS_LAT1f  + vel_contribution + accel_contribution;
    float accel_contributio = (actual_speedYf - _last_velocityY) * lag_in_seconds * lag_in_seconds;
    float vel_contributio = actual_speedYf * lag_in_seconds;
    _last_velocityY = actual_speedYf;    // store velocity for next iteration
    GPS_LON1lead = GPS_LON1f  + vel_contributio + accel_contributio;
    */
}
