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

#define TIME_INTERVAL           8000  // 4000usec, 10msec까지는 안정적임, 12.5msec는 프롭에 진동이 있음, 15msec까지 조종하여 비행이 가능하지만 진동이 있고 불안정함
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

#define RAD2DEG 180/PI
#define GYRO2DEG 131.0f
#define ALPHA 0.0004f
#define BETA (1.0 - ALPHA)
#define MAXROTATION 17

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

float gyXAngVel = 0, gyYAngVel = 0, gyZAngVel = 0; // 각속도
float gyXAngle = 0.0f, gyYAngle = 0.0f, gyZAngle = 0.0f; // 적분값 
float acXAngle = 0.0f, acYAngle = 0.0f; //계산된 각도
float compXAngle = 0.0f, compYAngle = 0.0f; // 상보필터 적용된 각도
float zAngle = 0.0f;
float tempVar = 0.0f;

float speedA = 0.0f, speedB = 0.0f, speedC = 0.0f, speedD = 0.0f;
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

    gyXAngVel = (float)gGyR / GYRO2DEG;
      gyYAngVel = (float)gGyP / GYRO2DEG;
      gyZAngVel = (float)gGyY / GYRO2DEG;
      gyXAngle = gyXAngle + gyXAngVel * gDelta * 0.000001;

      gyYAngle = gyYAngle + gyYAngVel * gDelta * 0.000001;
      gyZAngle = gyZAngle + gyZAngVel * gDelta * 0.000001;
      acXAngle = atan((float)gAcY / (float)gAcZ) * RAD2DEG;
      acYAngle = atan(-1*(float)gAcX / sqrt(pow((float)gAcY, 2) + pow((float)gAcZ, 2))) * RAD2DEG;
      //AngularVelocity2Angle();
      //Acc2Angle();
      ComplementaryFilter();

      
      // targetZAngleRot = map(gMSPYaw, 1, 250, -MAXROTATION, MAXROTATION);
      


      static double tAngleX = 0.0, tAngleY = 0.0, tAngleZ = 0.0;
      tAngleY = map(gMSPRoll, 1, 250, -MAXROTATION, MAXROTATION);
      tAngleX = map(gMSPPitch, 1, 250, -MAXROTATION, MAXROTATION);
      tAngleZ = (map(gMSPYaw, 1, 250, -MAXROTATION, MAXROTATION)+1);
      tempVar = tAngleZ;

      //double compensateValue = sin(gyZAngle / GYRO2DEG * gDelta* 0.000001 /RAD2DEG);
      double Kp = 1.0;
      double Kd = 0.3;
      double Ki = 0.2;
      
      double eAngleX = tAngleX - compXAngle; 
      // 앞으로 x-- -> eAngle++ || 뒤로 x++ -> eAngle--
      double eAngleY = tAngleY - compYAngle;
      // 오른쪽으로 y-- -> eAngle++ || 왼쪽으로 y++ -> eAngle--
      //double eAngleZ = tAngleZ - gyZAngle;
      double eAngleZ =  (map(gMSPYaw, 1, 250, -MAXROTATION, MAXROTATION)+1) * -1;
      // 오른쪽으로 z-- -> eAngle++ || 왼쪽으로 z++ -> eAngle--

      
      double BalX = Kp * eAngleX;
      double BalY = Kp * eAngleY;
      double BalZ = Kp * eAngleZ;
      static double ResX =0.0, ResY = 0.0, ResZ = 0.0;
      BalX += Kd * -gyXAngVel;
      BalY += Kd * -gyYAngVel;
      BalZ += Kd * -gyZAngVel;
      ResX += Ki * eAngleX * gDelta * 0.000001;
      ResY += Ki * eAngleY * gDelta * 0.000001;
      ResZ += Ki * eAngleZ * gDelta * 0.000001;
      if(gMSPThrottle == 0) ResX = ResY = ResZ = 0;
      BalX += ResX;
      BalY += ResY;
      BalZ += ResZ;

      //BalX = BalY = BalZ = 0;

     
      speedA = gMSPThrottle - BalY + BalX + BalZ; //1
      speedB = gMSPThrottle + BalY + BalX - BalZ; //2
      speedC = gMSPThrottle + BalY - BalX + BalZ; //3 
      speedD = gMSPThrottle - BalY - BalX - BalZ; //4

      //A(1)  B(2)
      //    @
      //D(3)  C(4)
      //모터 A,B,C,D에 대한 속도를 0~250사이로 보정한다.
      
      
      gMotorSpeedA =constrain((int)speedA,0,250);
      gMotorSpeedB =constrain((int)speedB,0,250);
      gMotorSpeedC =constrain((int)speedC,0,250);
      gMotorSpeedD =constrain((int)speedD,0,250);

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

/////
void ComplementaryFilter(void){
  //fill this
  compXAngle = BETA * (acXAngle) + ALPHA*(gDelta* 0.000001 * gyXAngle + compXAngle);
  compYAngle = BETA * (acYAngle) + ALPHA*(gDelta* 0.000001 * gyYAngle + compXAngle);

  compYAngle *= -1;
}
