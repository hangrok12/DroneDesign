/*
***************************************************************************************************************
안전에 주의하세요.
시험할 때는 드론의 날개(프롭)을 제거하세요.
비행할 때는 반드시 보호용 안경을 착용하고, 드론과 안전거리를 확보하세요.
***************************************************************************************************************
//*/

/*  
  @file   adDroneSt.ino
  @author 한국항공대학교, 항공전자정보공학부, 노영섭
  @version V 1.0.3  @date   2021.11.01
  @version V 1.0.4  @date   2022.11.19
  @version V 1.0.5  @date   2022.11.22
  @version V 1.0.6  @date   2022.12.01 <<== DISARM인 상태에서만 Cal.이 가능함, DISARM인 상태에서 Motor 출력을 죽임.
  @version V 1.0.7  @date   2023.01.07 <<== Bluetooth Telemetry 기능 추가. 사용에 주의를 요함
  @version V 1.0.8  @date   2023.01.16 <<== 전체 코드를 간결하게 정리함.
  @version V 1.0.9  @date   2023.01.28 <<== MPU6050의 측정값을 드론의 동체기준으로 변경한 변수를 추가함.
  @version V 1.0.10 @date   2023.02.15 <<== 시간에 관련된 변수들을 unsigned long으로 통일, 일부를 전역변수로 바꾸고 setup()에서 초기화함.  
  @version V 1.0.11 @date   2023.12.19 <<== gDelta와 변수들에 대한 설명을 추가함. 
  @version V 1.0.12 @date   2024.11.07 <<== HyperTerminal과 Telemetry에 출력하는 함수안에 있는 코드의 출력 주기를 조절함.
  @brief
  @brief  adDrone Student version
  @brief   
  @brief  < 센서의 종류 >
  @brief  ------------------------------------------------------------------------------
  @brief   1. MPU6050: Gyroscope sensors,     각속도계
  @brief   2. MPU6050: Accelerometer sensors, 가속도계
  @brief  ------------------------------------------------------------------------------
  @brief
  @brief  < 각속도/가속도계: Roll/X, Pitch/Y, Yaw/Z >
  @brief  ------------------------------------------------------------------------------
  @brief   Drone  : Roll/X,  Pitch/Y, Yaw/Z
  @brief   MPU6050: Pitch/Y, Roll/X,  Yaw/Z * -1
  @brief   
  @brief < 조종기의 제어값들을 받는데 사용되는 packet의 구성 >
  @brief  ------------------------------------------------------------------------------
  @brief   조종기의 기본 제어용 명령
  @brief   Packet: $ M < T C C
  @brief   Index:  0 1 2 3 4 5
  @brief
  @brief   MultiWii protocol
  @brief   Packet: $  M  <  n   T   R   P   Y   T   A   C
  @brief   Index:  0  1  2  3   4   5   6   7   8   9  10
  @brief  ------------------------------------------------------------------------------
  @brief
  @brief  < 드론의 제어 모드 >
  @brief  ------------------------------------------------------------------------------
  @brief  1. DISARM (default)
  @brief  2. ARM
  @brief  3. CALIBRATION
  @brief  *. 조종기에서 오는 제어용 packet에 따라 동작함
  @brief  ------------------------------------------------------------------------------
  @brief
  @brief  < 제어값, 측정값, 보정값의 극성 >
  @brief  ------------------------------------------------------------------------------  
  @brief  조종기의 Aileron(Roll)/Elevator(Pitch)/Rudder(Yaw)의 극성
  @brief    기체   조종기   극성
  @brief    Roll  Aileron (왼쪽: -, 오른쪽: +)
  @brief    Pitch Elevator(앞쪽: -, 뒤쪽:   +) <== 조종기에서 보내는 값의 극성을 바꾸어 보냄.
  @brief    Yaw   Rudder  (왼쪽: -, 오른쪽: +)
  @brief  ------------------------------------------------------------------------------
  @brief      
  @brief  < 날개(Prob)의 배치 >
  @brief  ------------------------------------------------------------------------------  
  @brief                     드론 앞면
  @brief  MOTOR_A/Motor2/Pin6/CW   MOTOR_B/Motor1/Pin10/CCW
  @brief  MOTOR_D/Motor3/Pin5/CCW  MOTOR_C/Motor4/Pin9 /CW
  @brief  ------------------------------------------------------------------------------
  @brief  
  @brief  < Motor 속도의 보정 극성 >
  @brief  ------------------------------------------------------------------------------  
  @brief    기체기준
  @brief    Error 계산 방법 = 목표값 + 실제각도 <= 더하여 계산한다.
  @brief    MotorA = gThrottle - Roll - PITCH + YAW
  @brief    MotorB = gThrottle + Roll - PITCH - YAW
  @brief    MotorC = gThrottle + Roll + PITCH + YAW
  @brief    MotorD = gThrottle - Roll + PITCH - YAW
  @brief  ------------------------------------------------------------------------------
  @brief  
  @brief  < Task의 목록과 우선 순위 >
  @brief  ------------------------------------------------------------------------------  
  @brief   1 순위: 약 3.5msec 이내로 제한됨,   Gyro/Acc의 측정값을 읽고 각도계산/PID계산 등을 진행하여 Motor를 제어
  @brief   2 순위: 약 0.18msec,               수신된 제어 명령의 처리
  @brief   3 순위: 약 0.3msec,                수신된 조종 명령을 추출
  @brief   4 순위: 약 0.065msec,              대기 중인 제어 명령의 수신
  @brief   5 순위: 기타 작업
  @brief  ------------------------------------------------------------------------------
  @brief
  @brief  < Bluetooth Telemetry >
  @brief  ------------------------------------------------------------------------------
  @brief  전송되는 데이터의 경로
  @brief   드론의 Arduino Pro Micro => 드론의 HM10 Bluetooth => 조종기의 HM10 Bluetooth => 조종기의 Arduino Pro Micro => 조종기의 USB Cable => PC의 HyperTerminal
  @brief 
  @brief  직렬 포트의 전송속도 115200 baud이므로, 
  @brief  이론상 최대 전송 속도는 초당 115200/8=14400 bytes이나 실제는 이의 절반 정도로 보면 됨
  @brief  따라서 4msec마다 data를 전송하려면 14400 / 2 / (1/0.004=250) = 28bytes/4msec로 예상되나 실험적으로 10bytes 정도가 최대임.
  @brief  또한 HM10의 동작에 문제가 있는지 서로 엉키면 전송이 되지 않아서,
  @brief  전원 인가 순서를 조종기, 드론, Pairing으로 해야 조종기의 Hyper Terminal에 드론에서 전송한 정보가 표시됨.
  @brief  BLUETOOTH_TELEMETRY 에 있는 코드를 잘 보고 사용할 것
  @brief  
  @brief  사용에 제한적임. 최대 (115200 / 8 = 14400bytes) 이며, 이의 절반 정도로 사용 가능함. 4msec 주기로 data를 전송하려면 14400/2/250 = 28bytes 보다 훨씬 적게 사용해야 함.
  @brief  DT가 10msec 인 경우 MotorABCD output은 가능함. Gyro와 Acc 데이터를 같이 보내는 것은 Overflow 발생함. 따라서 Gyro 값 또는 Acc 값만 전송하는 것은 가능함.
  @brief  Overflow는 HM10 <-> HM10 간에서 발생하는 것으로 추측됨.  
  @brief  
  @brief  주의사항
  @brief  사용에 제한적이며, 반드시 전송이 잘 되는지 확인하고 사용해야 하며,
  @brief  가끔 조종기가 엉키는 경우가 있으니 조종이 잘 안되는 경우는 조종기의 전원을 재인가하여 사용할 것.
  @brief  ------------------------------------------------------------------------------  
  //*/
  
#include <Wire.h>

const char strStar[]               PROGMEM = "*****************************************************";
const char strProgName[]           PROGMEM = "* Adventure Design                                  *";
const char strNameAndVer[]         PROGMEM = "* adDrone Student Ver 1.0.12 2024.11.07 Roh         *";
const char strNameOfSchool[]       PROGMEM = "* School of Electronics and Information Engineering *";  
const char strNameOfUniv[]         PROGMEM = "* Korea Aerospace University                        *"; 

const char *const inforList[] PROGMEM = {
                              NULL,
                              strStar,          //1
                              strProgName,      //2
                              strNameAndVer,    //3
                              strNameOfSchool,  //4
                              strNameOfUniv,    //5
                              strStar,          //6
                              NULL    
};
int lenInforList = 6;

#define YES true
#define NO  false

#define DISPLAY_DELTA_TIME      NO   // 실행 시간을 측정할 때 사용한다.
#define TASK_SWITCHING          NO   // Task 실행 번호를 출력한다.

#define TIME_INTERVAL           4000   // 4000usec, 10msec까지는 안정적임, 12.5msec는 프롭에 진동이 있음, 15msec까지 조종하여 비행이 가능하지만 진동이 있고 불안정함
#define TIME_INTERVAL_EXCLUDE    300   // 300usec
#define TIME_INTERVAL_GUARD    (TIME_INTERVAL - TIME_INTERVAL_EXCLUDE)   // 3700usec

#define TIMER_START       false
#define TIMER_END         true
#define STRING_LENGTH     255       // 문자열의 길이
#define VAR_LENGTH        20        

// motor의 pin 번호를 지정함
// Motor의 배치
// MOTOR_A/Motor2/Pin6   MOTOR_B/Motor1/Pin10
// MOTOR_D/Motor3/Pin5   MOTOR_C/Motor4/Pin9
#define MOTOR_A            6
#define MOTOR_B           10
#define MOTOR_C            9 
#define MOTOR_D            5

#define NUMBER_OF_MODE_BYTES     6
#define NUMBER_OF_COMMAND_BYTES 11

// 송신기의 제어 값
#define MSP_SET_RAW_RC_SERIAL 150   //MSP RC 메세지 CodeType, RC 신호,        드론 제어용 명령
#define MSP_ARM               151   //MSP RC 메세지 CodeType, 시동,           ARM 꼭지를 누르는 경우
#define MSP_DISARM            152   //MSP RC 메세지 CodeType, 시동 끄기,       DISARM 꼭지를 누르는 경우
#define MSP_AIR               199   //MSP RC 메세지 CodeType, 데이터값을 읽음,  현재는 사용안함
#define MSP_ACC_CALIBRATION   205   //MSP RC 메세지 CodeType, 가속도 센서 보정, 보정용
#define RX_BUF_SIZE            20
#define LIMIT_RX_BYTES 4   // 송신기의 데이터를 한번에 받을 수 있는 바이트 수

#define MIM_THROTTLE_POSITION 10    // Drone에 시동을 걸 때 Throttle stick의 위치를 확인하는데 사용함.

#define GYRO_CAL_COUNT 2000   // Gyro의 Cal.에 사용할 data의 수

enum{DISARM, PRE_ARM, ARM};

// MSP packet 구성
enum {
  //        0             1             2                 3            4            5
  CONTROL_DOLLAR, CONTROL_MARK, COTROL_DIRECTION, CONTROL_LENGTH, CONTROL_TYPE, CONTROL_CRC};
enum {
  //       0         1         2              3           4         5         6          7        8            9       10  
  MSP_DOLLAR, MSP_MARK, MSP_DIRECTION, MSP_LENGTH, MSP_TYPE, MSP_ROLL, MSP_PITCH, MSP_YAW, MSP_THROTTLE, MSP_AUX, MSP_CRC};


// 드론의 제어 상태
int gArmState;

// 조종기에서 보내온 데이터
uint8_t gRxCommand[RX_BUF_SIZE] = {0, };
bool    gFlagRxCommandValid   = false;   // 제어용 명령(MSP protocol) 을 수신한 경우 true로 설정됨.
bool    gFlagRxModeValid      = false;   // 설정용 명령을 수신한 경우 true로 설정됨.
bool    gFlagRxBufferOverflow = false;   // gRxCommand[]의 내용이 overwrite 된 경우 true로 설정됨.

// 각속도계와 가속도계의 측정값, MPU6050의 측정값
int16_t gGyR          = 0,    gGyP          = 0,    gGyY          = 0;   // 자이로에서 읽은 값
int16_t gAcX          = 0,    gAcY          = 0,    gAcZ          = 0;   // 가속도계에서 읽은 값
int16_t gGyRCal       = 0,    gGyPCal       = 0,    gGyYCal       = 0;

// 드론의 동체 기준, MPU6050의 측정값 => 드론의 동체
int16_t gGyroRoll     = 0,    gGyroPitch    = 0,    gGyroYaw      = 0;
int16_t gAccX         = 0,    gAccY         = 0,    gAccZ         = 0;

uint8_t gMSPRoll, gMSPPitch, gMSPYaw, gMSPThrottle;   // 조종기로부터 수신한 조종 명령

int16_t gMotorSpeedA = 0, gMotorSpeedB = 0, gMotorSpeedC = 0, gMotorSpeedD = 0;

unsigned long currentTimeTimerPrev, currentTimeTimer;
unsigned long gDelta   = 0;                     // delta t 측정용

// 시험용 버퍼들
static char gGenBuf[STRING_LENGTH], gBuf0[VAR_LENGTH], gBuf1[VAR_LENGTH], gBuf2[VAR_LENGTH], gBuf3[VAR_LENGTH], gBuf4[VAR_LENGTH];

//**************************************************************************************
//Setup routine
//**************************************************************************************
void setup(){
  //-------------------------------
  Serial.begin(115200);   // Hyper Terminal
  //while(!Serial);       // wait for init
  delay(5000);
  Serial1.begin(115200);  // HM-10 BLE Module
  while(!Serial1);       // wait for init

  // Version 정보 출력
  ftnVersion();

  // I2C 초기화
  Wire.begin();  
  Wire.setClock(400000);

  // Motor 제어용 PIN 설정
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(MOTOR_C, OUTPUT);
  pinMode(MOTOR_D, OUTPUT);

  // Gyro 초기화
  ftnInitGyro();
  
  // Gyro 보정
  ftnGyroCalibration();

  // 초기 드론 상태
  gArmState = DISARM; 
  currentTimeTimerPrev = micros();
  
  //while(1);
}


//**************************************************************************************
//Main program loop
//**************************************************************************************
void loop(){
  static bool flagFirstTime = true;

  currentTimeTimer = micros();

  if (flagFirstTime) {    // if first loop
    currentTimeTimerPrev = currentTimeTimer;
    flagFirstTime = false;
  }

  #if DISPLAY_DELTA_TIME 
  ftnRunTimeCheck(TIMER_START, currentTimeTimer); // 시간 측정 시작
  #endif  
  
  // #1 priority, 드론 제어, 각도계산, Error계산, PID계산, Motor제어, 실행시간: 약 3500usec 이내로 제한됨
  if (  (currentTimeTimerPrev + TIME_INTERVAL_GUARD) < currentTimeTimer)  { // guard time: 300usec
    if ((currentTimeTimerPrev + TIME_INTERVAL)       < currentTimeTimer)  {    
      #if TASK_SWITCHING 
      Serial.print(":1|"); 
      #endif // 4test
      
      gDelta = currentTimeTimer - currentTimeTimerPrev;
      currentTimeTimerPrev = currentTimeTimer;

      // Gyroscope와 Accelerometer로 부터 측정값을 읽음.
      ftnMPU6050GetData();

      //**************************************************************************************
      // 여러분들의 풀그림를 여기에 넣으세요.
      // 여러분들이 사용할 수 있는 변수들, 이 변수들은 전역변수로 선언되어 있음.
      // Delta Time:        gDelta     <<== delta time, 단위는 usec로 10^6으로 나누어 계산에 사용해야 함.
      // ARM state:         gArmState  <<== ARMing 상태 표시, enum{DISARM, PRE_ARM, ARM};
      // Gyroscope:         gGyR,         gGyP,         gGyY        <<== MPU6050의 측정값
      // Accelerometer:     gAcX,         gAcY,         gAcZ        <<== MPU6050의 측정값        
      // Gyroscope:         gGyroRoll,    gGyroPitch,   gGyroYaw    <<== Drone의 동체를 기준으로 한 측정값
      // Accelerometer:     gAccX,        gAccY,        gAccZ       <<== Drone의 동체를 기준으로 한 측정값
      // MSP Packet:        gMSPRoll,     gMSPPitch,    gMSPYaw,      gMSPThrottle <<== 조종기로부터의 수신값
      // PWM Output(motor): gMotorSpeedA, gMotorSpeedB, gMotorSpeedC, gMotorSpeedD <<== Motor에 공급되는 PWM 값
      //**************************************************************************************
      
      // gMotorSpeedA/B/C/D를 Motor_A/B/C/D에 출력함
      ftnMotorOutput();  

      // HyperTerminal에서 데이터를 확인하고자 할 때 사용하세요.
      // 아래의 함수에 원하는 데이터가 출력되도록 풀그림을 작성하세요.
      ftnData2HyperTerminal();

      // Bluetooth Telemetry가 필요할 떄 사용하세요.
      // 아래의 함수에 원하는 데이터가 출력되도록 풀그림을 작성하세요.
      ftnSendTelemetryData();

      #if DISPLAY_DELTA_TIME 
      ftnRunTimeCheck(TIMER_END, ZERO);    // 시간 측정 완료, 필요할 때만 사용
      #endif
    }
  }

  // #2 priority, 수신된 제어 명령의 처리, 실행시간: 약 176usec
  else if (gFlagRxModeValid){
    #if TASK_SWITCHING 
    Serial.print(":2|"); 
    #endif // 4test
    ftnOperationMode();
    gFlagRxModeValid = NO;
    memset(gRxCommand, 0, RX_BUF_SIZE);  

    #if DISPLAY_DELTA_TIME 
    ftnRunTimeCheck(TIMER_END, ZERO);    // 시간 측정 완료      
    #endif
  }  
  // #3 priority, 수신된 조종 명령을 추출, 실행시간:약290usec
  else if (gFlagRxCommandValid) {
    #if TASK_SWITCHING 
    Serial.print(":3|"); 
    #endif // 4test
    ftnOperationCommand();
    gFlagRxCommandValid = NO;
    memset(gRxCommand, 0, RX_BUF_SIZE);
    
    #if DISPLAY_DELTA_TIME 
    ftnRunTimeCheck(TIMER_END, ZERO);    // 시간 측정 완료       
    #endif
  }
  // #4 priority, 대기 중인 제어/조종 명령의 수신, 실행시간: 약 65usec
  else if (Serial1.available()){
    #if TASK_SWITCHING 
    Serial.print(":4|"); 
    #endif  // 4test
    ftnCommandRead();
    
    #if DISPLAY_DELTA_TIME 
    ftnRunTimeCheck(TIMER_END, ZERO);    // 시간 측정 완료       
    #endif
  }  
  // #5 priority, 기타 작업
  else{
    ;
  }
}

//**************************************************************************************
// 정보를 HM10(Bluetooth Module, Serial1)으로 전송함.
// 전송되는 데이터의 경로
// 드론의 Arduino Pro Micro => 드론의 HM10 Bluetooth => 조종기의 HM10 Bluetooth => 조종기의 Arduino Pro Micro => 조종기의 USB Cable => PC의 HyperTerminal
// 안정성이 낮으니 사용에 주의할 것.
// 전송하는 데이터량이 많으면 아주 쉽게 overflow가 발생함.
// adController가 가끔 꼬이는 경우가 있으니 조종이 않될 경우는 조종기를 초기화 할 것.
// 사용 순서: 1-드론 upload, 2-Bluetooth pairing, 3-조종기 재시작
//**************************************************************************************
void ftnSendTelemetryData(){
  static uint8_t loopCounter = 0;

  if(!(loopCounter++ % 240)) {
    Serial1.print(loopCounter); Serial1.print(" "); Serial1.print(gGyroRoll); Serial1.print(" "); Serial1.print(gGyroPitch); Serial1.print(" "); Serial1.println(gGyroYaw); 
    loopCounter = 0;
  }
} 

//**************************************************************************************
// HyperTerminal(Serial)에서 데이터를 확인하고자 할 때 사용하세요.
//**************************************************************************************
void ftnData2HyperTerminal(){
  static uint8_t lpcnt = 0;

  if (!(lpcnt++ % 250)) {
    // Hyper Terminal에 측정값을 출력할 때 사용합니다.
    // 아래의 풀그림은 확인을 위하여 각속도계와 가속도계의 측정값 (드론의 동체 기준)과 수신기에서 수신한 조종기의 조종신호를 출력하는 것입니다.
    Serial.print(lpcnt);
    Serial.print(gGyroRoll); Serial.print(" "); Serial.print(gGyroPitch); Serial.print(" "); Serial.print(gGyroYaw); Serial.print("   ");
    Serial.print(gAccX);     Serial.print(" "); Serial.print(gAccY);      Serial.print(" "); Serial.print(gAccZ);    Serial.print("   ");
    Serial.print(gMSPRoll);  Serial.print(" "); Serial.print(gMSPPitch);  Serial.print(" "); Serial.print(gMSPYaw);  Serial.print(" ");   Serial.print(gMSPThrottle); Serial.print("   ");
    Serial.println(gDelta);
    lpcnt = 0;
  }
}

//**************************************************************************************
// PID에서 계산된 값들을 모터에 전달함.
//**************************************************************************************
void ftnMotorOutput(){
  if (gArmState == DISARM){   // DISARM인 상태에서 Motor 출력을 0으로 설정함.
    gMotorSpeedA = 0;
    gMotorSpeedB = 0;
    gMotorSpeedC = 0;
    gMotorSpeedD = 0;
  }

  analogWrite(MOTOR_A, gMotorSpeedA);
  analogWrite(MOTOR_B, gMotorSpeedB);
  analogWrite(MOTOR_C, gMotorSpeedC);
  analogWrite(MOTOR_D, gMotorSpeedD);
}

//**************************************************************************************
// Rreading the Gyro and Acc
//**************************************************************************************
void ftnMPU6050GetData(){
  int16_t temp;
  
  Wire.beginTransmission(0x68);     
  Wire.write(0x3B);                         
  Wire.endTransmission();                   
  Wire.requestFrom(0x68,14);        
  
  while(Wire.available() < 14);  
             
  gAcX = Wire.read() << 8 | Wire.read();  
  gAcY = Wire.read() << 8 | Wire.read();  
  gAcZ = Wire.read() << 8 | Wire.read();  
  temp = Wire.read() << 8 | Wire.read();  
  gGyR = Wire.read() << 8 | Wire.read();  
  gGyP = Wire.read() << 8 | Wire.read();  
  gGyY = Wire.read() << 8 | Wire.read(); 

  gGyR -= gGyRCal;
  gGyP -= gGyPCal;
  gGyY -= gGyYCal;

  // MPU6050에서 측정한 값을 드론의 동체 기준으로 변경
  gGyroRoll  = gGyP;
  gGyroPitch = gGyR;
  gGyroYaw   = gGyY * -1;
  gAccX      = gAcY;
  gAccY      = gAcX;
  gAccZ      = gAcZ * -1;
}

//**************************************************************************************
// Gyro의 Calibration
//**************************************************************************************
void ftnGyroCalibration(){
  float gyRCal = 0.0f, gyPCal = 0.0f, gyYCal = 0.0f;

  Serial.println(F("Gyro Calibration ....."));
  
  gGyRCal = 0;
  gGyPCal = 0;
  gGyYCal = 0;
  
  for (int i = 0; i < GYRO_CAL_COUNT ; i++){
    if(i % 100 == 0) Serial.print(".");
    
    ftnMPU6050GetData();  
    gyRCal += gGyR;
    gyPCal += gGyP;
    gyYCal += gGyY;
    delay(3);
  }
  
  gGyRCal = (int16_t)(gyRCal / (float)GYRO_CAL_COUNT); 
  gGyPCal = (int16_t)(gyPCal / (float)GYRO_CAL_COUNT); 
  gGyYCal = (int16_t)(gyYCal / (float)GYRO_CAL_COUNT); 

  Serial.println("");
  Serial.print(F("Calibrated Values: ")); Serial.print(gGyRCal); Serial.print(":"); Serial.print(gGyPCal); Serial.print(":"); Serial.println(gGyYCal);
  Serial.println("");
}

//**************************************************************************************
// Gyro의 초기화
//**************************************************************************************
void ftnInitGyro(){
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);                     
  Wire.write(0x00);              // Sleep 모드에서 Normal 모드로 변환       
  Wire.endTransmission();               

  Wire.beginTransmission(0x68); 
  Wire.write(0x1B);                     
  Wire.write(0x00);               // Gyro의 scale 설정, +/-250d/s
  //Wire.write(0x08);               // Gyro의 scale 설정, +/-500d/s
  Wire.endTransmission();               

  Wire.beginTransmission(0x68); 
  Wire.write(0x1C);                     
  Wire.write(0x00);               // Acc의 scale 설정, +/-2g
  //Wire.write(0x10);               // Acc의 scale 설정, +/-8g
  Wire.endTransmission();               

  Wire.beginTransmission(0x68); 
  Wire.write(0x1A);                     
  //Wire.write(0x00);               //MPU6050의 DLPF의 차단 주파수 선택, Fc = 256Hz, 0.98msec 지연
  Wire.write(0x03);               //MPU6050의 DLPF의 차단 주파수 선택, Fc = 42Hz, 4.8msec 지연
  Wire.endTransmission();               
}

//**************************************************************************************************************
// 이전에 호출한 시점으로부터의 시간을 측정하여 UART에 출력한다.
// flag가 false이면 시간을 저장하고, true이면 시간 차를 출력한다.
//**************************************************************************************************************
void ftnRunTimeCheck(bool flag, unsigned long t) {
  static unsigned long curTime, prevTime, diffTime;
  
  if (flag){
    curTime = micros();
    diffTime = curTime - prevTime;
    prevTime = curTime;
    dtostrf(diffTime, 8, 0, gBuf1);
    Serial.print("rt:"); Serial.println(diffTime);  // run time
  }
  else {
    if (!t)
      prevTime = micros();
    else
      prevTime = t;
  }    
}

//**************************************************************************************
// 제어 명령을 처리
//**************************************************************************************
void ftnOperationMode(void) {   // ToDo Here
  switch ((uint8_t)gRxCommand[4]) {
    case MSP_ARM:
      Serial.println("MSP_ARM");
      if(gMSPThrottle > MIM_THROTTLE_POSITION) {
        Serial.print(F("Throttle Input = "));
        Serial.println(gMSPThrottle);  // Throttle이 최소 위치로 되어 있는지 확인한다.
        Serial.println(F("Make sure that the throttle stick is in the minimum position ....."));
      }
      else {
        gArmState = ARM;
        gFlagRxBufferOverflow = false;
      }
      break;

    case MSP_DISARM:
      Serial.println("MSP_DISARM"); 
      gArmState = DISARM;
      break;

    case MSP_ACC_CALIBRATION:
      Serial.println("MSP_ACC_CALIBRATION");
      //gArmState = DISARM;
      if(gArmState == DISARM)      // DISARM인 상태에서만 Cal.이 가능.
        ftnGyroCalibration();
      break;

    default:
      Serial.println("Command: Mismatch");
      break;
  }

  Serial.print("Arm State: "); Serial.println(gArmState);
}

//**************************************************************************************
// MSP packet을 수신하는 곳
//**************************************************************************************
void ftnCommandRead(void) {
  static uint8_t flagRxStart = NO, cntRxBytes = 0;
  static uint8_t flagClearBuffer = NO;
  static uint8_t inputCh, checksum = 0;
  static uint8_t rxCommandBuffer[RX_BUF_SIZE]    = {0, };
  uint8_t i, limitRx;
  
  limitRx = 0;
  while (Serial1.available() && flagClearBuffer == NO && limitRx < LIMIT_RX_BYTES) {
    inputCh = Serial1.read();
    
    if (inputCh == '$' && flagRxStart == NO) { // init
      cntRxBytes  = 0;
      flagRxStart = YES;
      checksum = 0;
    }

    if (flagRxStart == YES)  {
      rxCommandBuffer[cntRxBytes] = inputCh;

      if ((uint8_t)rxCommandBuffer[CONTROL_LENGTH] == 0 && cntRxBytes == (NUMBER_OF_MODE_BYTES - 1)) { // if command
        if (checksum == rxCommandBuffer[NUMBER_OF_MODE_BYTES - 1]) { // verify received mode 
          for (i = 0; i < NUMBER_OF_MODE_BYTES; i++) { // Command를 복사함
            gRxCommand[i] = rxCommandBuffer[i];
          }
          if (gFlagRxModeValid) gFlagRxBufferOverflow = true;
          gFlagRxModeValid = YES;
        }
        flagClearBuffer = YES;
        //Serial.print(checksum, HEX); Serial.print("<");
      }
      else if ((uint8_t)rxCommandBuffer[MSP_TYPE] == MSP_SET_RAW_RC_SERIAL && cntRxBytes == (NUMBER_OF_COMMAND_BYTES - 1)) { // if control
        if (checksum == rxCommandBuffer[NUMBER_OF_COMMAND_BYTES - 1]) { // verify received command 
          for (i = 0; i < NUMBER_OF_COMMAND_BYTES; i++) { // Command를 복사함
            gRxCommand[i] = rxCommandBuffer[i];
          }
          // Command를 복사후 flag를 설정함. 이 flag는 반드시 command를 실행하는 곳에서 지워야 한다.
          if (gFlagRxCommandValid) gFlagRxBufferOverflow = true;
          gFlagRxCommandValid = YES;
        }
        flagClearBuffer = YES;
      }

      if (cntRxBytes > 2) // checksum 계산
        checksum ^= rxCommandBuffer[cntRxBytes];
      
      cntRxBytes++;
    }
  limitRx++;
  }

  if (flagClearBuffer == YES || cntRxBytes >= RX_BUF_SIZE - 2) {
    cntRxBytes  = 0;
    flagRxStart = NO;
    flagClearBuffer = NO;
    memset(rxCommandBuffer, 0, RX_BUF_SIZE);
  }
}

//**************************************************************************************
// MSP packter으로부터 조종 명령 추출
//**************************************************************************************
void ftnOperationCommand(void) {
  gMSPRoll     = gRxCommand[MSP_ROLL];
  gMSPPitch    = gRxCommand[MSP_PITCH];
  gMSPYaw      = gRxCommand[MSP_YAW];
  gMSPThrottle = gRxCommand[MSP_THROTTLE];
}

//**************************************************************************************
// Version 정보 출력
//**************************************************************************************
void ftnVersion(){
  char *ptr = gGenBuf;

  Serial.println("");
  for (int i = 1; i <= lenInforList; i++){
    strcpy_P(ptr, (char *)pgm_read_word(&(inforList[i])));
    Serial.println(ptr);
  }  
  Serial.println("");
}
