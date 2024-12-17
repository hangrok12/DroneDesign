#include <Wire.h>
#include <VL53L0X.h>

int throttle = 0;
double m1,m2,m3,m4,T=0.0;
double K = 625.0,K_Yaw=250;

VL53L0X tofSensor;
float rangeMeasure = 0.0f;
float pre_rangeMeasure = 0.0f;

void setup(){
  Serial1.begin(115200);
  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x0);
  Wire.endTransmission(true);

  tofSensor.setTimeout(500);
  if(!tofSensor.init()){
    Serial.println("Failed to detect and initialize sonsor!");
    while(1){}
  }
  tofSensor.setSignalRateLimit(0.1);
  tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

//ToF 센서를 이용한 고도유지
void ToF_sensor(){
  
  double dt=0.0;
  unsigned long timePrevious = 0;
  unsigned long timeNow = 0;
  
  timeNow = micros();
  dt = ((double)(timeNow - timePrevious))/1000000.0d;
  timePrevious = timeNow;

  rangeMeasure = (float)tofSensor.readRangeSingleMillimeters();

  if((rangeMeasure-pre_rangeMeasure)<(-10)){
    T = T + 5;
  }
  else if((rangeMeasure-pre_rangeMeasure)>(10)){
    T = T - 4;
  }
  else{
    T=T;
  }
  pre_rangeMeasure = rangeMeasure;
  
  Serial.print("Altitude\t");Serial.print(rangeMeasure);Serial.print("\t");
  Serial.print("dt\t");Serial.println(T);
}


// 컨트롤
void control(){
  static uint8_t cnt_msg;
  while(Serial1.available()>0){
    uint8_t msp_data = Serial1.read();

    if(msp_data=='$') cnt_msg=0;
    else cnt_msg++;
    
//    설명: 
//    cnt_msg       msp_data
//    5: Roll       0 ~ 250
//    6: Pitch      0 ~ 250
//    7: Yaw        0 ~ 250
//    8: Throttle   0 ~ 145
//    
//    모터 value <= 255
//    T = 145*N <= 255
//    
//    if 8 -> m1,m2,m3,m4의 T = msp_data
//    if 7 -> msp_data < 125 m1,m3 -= 
//            msp_data > 125 m2,m4 -= 
//    if 6 -> msp_data < 125 m1,m2 -=
//            msp_data > 125 m3,m4 -=
//    if 5 -> msp_data < 125 m2,m3 -=
//            msp_data > 125 m1,m4 -=
//
//    처음에 m을 T로 설정, 조건에 따라 m값을 감소 


// setup: float m1,m2,m3,m4,roll,pitch,yaw,T=0.0;
    if (cnt_msg==5) {
      m1 = T;
      m2 = T;
      m3 = T;
      m4 = T;
      if(msp_data<125){
        m2 += T*(double(msp_data)-125.0)/K;
        m3 += T*(double(msp_data)-125.0)/K;
      }
      else{
        m1 -= T*(double(msp_data)-125.0)/K;
        m4 -= T*(double(msp_data)-125.0)/K;
      }
    }
    else if (cnt_msg==6) {
      if(msp_data<125){
        m1 += T*(double(msp_data)-125.0)/K;
        m2 += T*(double(msp_data)-125.0)/K;
      }
      else{
        m3 -= T*(double(msp_data)-125.0)/K;
        m4 -= T*(double(msp_data)-125.0)/K;
      }
    }
    else if (cnt_msg==7) {
      if(msp_data<125){
        m1 += T*(double(msp_data)-125.0)/K_Yaw;
        m3 += T*(double(msp_data)-125.0)/K_Yaw;
      }
      else{
        m2 -= T*(double(msp_data)-125.0)/K_Yaw;
        m4 -= T*(double(msp_data)-125.0)/K_Yaw;
      }
    }
    else if (cnt_msg==8) {
      T = double(msp_data);
      if(msp_data>=120 && msp_data<=130){
        ToF_sensor();
        continue;
      }
    }
  }
}

void loop(){
  control();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);
  int16_t GyXH = Wire.read();
  int16_t GyXL = Wire.read();
  int16_t GyYH = Wire.read();
  int16_t GyYL = Wire.read();
  int16_t GyZH = Wire.read();
  int16_t GyZL = Wire.read();
  
  int16_t GyX = GyXH << 8|GyXL;
  int16_t GyY = GyYH << 8|GyYL;
  int16_t GyZ = GyZH << 8|GyZL;

  static int32_t GyXSum =0, GyYSum =0, GyZSum =0;
  static double GyXOff = 0.0, GyYOff = 0.0, GyZOff = 0.0;
  static int cnt_sample = 1000;

  if(cnt_sample >0){
    GyXSum += GyX;
    GyYSum += GyY;
    GyZSum += GyZ;
    cnt_sample --;
    if(cnt_sample ==0){
      GyXOff = GyXSum/1000.0;
      GyYOff = GyYSum/1000.0;
      GyZOff = GyZSum/1000.0;
    }
    delay(1);
    return;
  }
  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;
  double GyXR = GyXD/131;
  double GyYR = GyYD/131;
  double GyZR = GyZD/131;

  static unsigned long t_prev =0;
  unsigned long t_now = micros();
  double dt = (t_now - t_prev)/1000000.0;
  t_prev=t_now;

  static double AngleX=0.0,AngleY=0.0,AngleZ=0.0;
  AngleX += GyXR* dt;
  AngleY += GyYR* dt;
  AngleZ += GyZR* dt;
  if(T ==0) AngleX = AngleY = AngleZ = 0.0;

  static double tAngleX=0.0, tAngleY=0.0, tAngleZ=0.0;
  double eAngleX = tAngleX - AngleX;
  double eAngleY = tAngleY - AngleY;
  double eAngleZ = tAngleZ - AngleZ;
  
  // Kp
  double Kp=1.0;
  double BalX = Kp*eAngleX;
  double BalY = Kp*eAngleY;
  double BalZ = Kp*eAngleZ;

  // Kd 지나친 회전 속도 거르기
  double Kd=0.4, Kd_Y=1.0;
  BalX+=Kd*-GyXR;
  BalY+=Kd*-GyYR;
  BalZ+=Kd_Y*-GyZR;
  if(T==0) BalX = BalY = BalZ = 0.0;

  // Ki
  double Ki = 0.04;
  static double ResX=0.0, ResY=0.0, ResZ=0.0;
  ResX += Ki *eAngleX *dt;
  ResY += Ki *eAngleY *dt;
  ResZ += Ki *eAngleZ *dt;
  if(T==0) ResX = ResY = ResZ = 0.0;
  BalX += ResX;
  BalY += ResY;
  BalZ += ResZ;

  double speedA = m2 + BalX + BalY + BalZ;
  double speedB = m1 + BalX - BalY - BalZ;
  double speedC = m4 - BalX - BalY + BalZ;
  double speedD = m3 - BalX + BalY - BalZ;

  int iSpeedA = constrain((int)speedA, 0, 250);
  int iSpeedB = constrain((int)speedB, 0, 250);
  int iSpeedC = constrain((int)speedC, 0, 250);
  int iSpeedD = constrain((int)speedD, 0, 250);

  analogWrite(6,  iSpeedA);
  analogWrite(10, iSpeedB);
  analogWrite(9,  iSpeedC);
  analogWrite(5,  iSpeedD);
  //test
  Serial.print(iSpeedB);Serial.print("\t");
  Serial.print(iSpeedA);Serial.print("\t");
  Serial.print(iSpeedD);Serial.print("\t");
  Serial.print(iSpeedC);Serial.print("\t");
  Serial.print(BalX);Serial.print("\t");
  Serial.print(BalY);Serial.print("\t");
  Serial.print(BalZ);Serial.print("\n");
  
}
//*/


/*
void loop(){
  comp();           //상보필터로 자세 확인
  control();        //조종기로부터 들어온 값을 통해 모터 회전속도 계산
  pid();            //자세 확인 후 pid로 모터 회전속도 조절
  altitude();       //flow sensor로 측정 후 고도유지를 위한 모터속도 조절
  motor_control();  //control pid altitude 합산해서 모터속도 조절
}
*/
