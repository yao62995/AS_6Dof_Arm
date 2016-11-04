/*
 * 功能：舵机基本控制
 * 读取舵机当前角度：
 *      格式一：**         ---读取所有舵机角度，返回列表,以空格间隔，如：30 40 50 60 70 80 90
 *      格式二：*X1X2      ---读取第X1X2号舵机的角度，返回单个十进制数值，如：10
 * 设置舵机角度：
 *      格式：#X1X2X3X4X5X6 ---X1X2表示舵机编号，X3X4表示要设置的舵机的目标角度，X5X6表示延时，
 *                             每隔X5X6毫秒，舵机转动一度，直到目标位置。命令以十六进制表示。
 *                             如：#010A14表示1号舵机，转动到10度，每隔30ms转动一度。
 *                             返回#X1表示舵机X1转动结束。
 * 舵机复位命令：                            
 *      格式：！           ---该命令使舵机转动到初始位置。
 *      
 * 舵机初始自测命令：
 *      格式：@1           ---使六个舵机转动到最小位置 
 *            @2           ---使六个舵机转动到最小位置 
 */

#include <Servo.h>      //调用一些库文件
#include <TimedAction.h>
#include <SimpleTimer.h> 
#include <Wire.h>


const int SERVOMAX = 2500;     //舵机性能参数，能识别的最大和最小脉冲宽度
const int SERVOMIN = 500;
                                //定义舵机位置名称，并编号。
const int base =      0;         
const int shoulder =   1;          
const int elbow =     2;        
const int wristflex =   3;        
const int wristrot =    4;          
const int gripper =    5;         

const int stdDelay =   30;         //舵机运动延时（单位ms）
const int shortDelay = 20;
const int longDelay = 50;

const int maxServos =  6;         //舵机的数量
const int centerPos =  90;        //舵机中位位置

unsigned long key_millis = 0;      
unsigned long button_millis = 0;
int keyDelay = 100;              //定义延时时间
int buttonDelay = 50;           //定义按键延时
int thisServo = base;           //定义起始电机

typedef struct{              //数组框架结构
  byte startpos;             //初始角度
  byte minPos;             //最小角度
  byte maxPos;            //最大角度
  byte delaySpeed;         //延时时间
  byte curPos;            //舵机当前角度
} ServoPos;              //结构体名称

ServoPos servosPos[] = {    //对舵机限位
  { 90, 180, 0, stdDelay, 0 }, //base  初始90，范围0~180度。    
  { 95, 180, 10, stdDelay, 0 },//shoulder 初始95，范围10~180度。
  { 90, 180, 30, stdDelay, 0 }, //elbow 初始90，范围30~180度。
  { 90, 90, 0, stdDelay, 0 },    //wristflex 初始90，范围0~90度。
  { 90, 90, 0, stdDelay, 0 },   //wristrot 初始90，范围0~90度。
  { 90, 90, 30, stdDelay, 0 }    //gripper 初始90，范围30~90度。
};

byte flag = 0;

//存储串口收到的指令
char buffer[8];
byte pointer = 0;
byte inByte = 0;

byte serv = 90;
int counter = 0;
int curServo = 0;
int sMove[] = {0, 90, 0};
int sAttach[] = {0, 0};

Servo servos[maxServos];


int destServoPos[maxServos];
int currentServoPos[maxServos];
void doServoFunc();
int hex2dec(byte c);
void Move(int servoNum, int servoPosition, int delayTime);
void resetServo();
void selfTest();

TimedAction servoMove[maxServos] = TimedAction(100, doServoFunc);   // 延时，延时时间为声明时间。
//SimpleTimer timer;            // For movement tests

void setup() {                            //设置
  
  Serial.begin(9600);                    //串口用于调试
  
  delay(200);
  Wire.begin();
  delay(500);

  for(int i=0; i<maxServos; i++) {          
    servosPos[i].curPos = servosPos[i].startpos;
    servos[i].initPulse(map(servosPos[i].curPos, 0, 180, SERVOMIN, SERVOMAX));     
    servos[i].attach(i+4,SERVOMIN,SERVOMAX);       

    destServoPos[i] = servosPos[i].curPos;
    currentServoPos[i] = servosPos[i].curPos;
    servoMove[i].disable();
  }
  //timer.setInterval(5000, servoTestFunc);      
}

void loop() {
  for(int x=0; x<maxServos; x++) {
    curServo = x;
    servoMove[x].check();
  }
  int dstangle;
  int timeDelay;
  
  if (Serial.available() > 0){
  //读取串口数据
  inByte = Serial.read();
  delay(2);
  //如果读到#符号，取之后的两个字节控制命令

  if (inByte == '#'){                        //#命令用于控制舵机转动，格式：#015A 舵机1转动到90度。
    while (pointer < 6){  //连续读取四个数字
      delay(2);
      buffer[pointer] = Serial.read();  //存储在缓存区
      pointer++; //将指针向下移动一位
      }
      thisServo = hex2dec(buffer[1]);
      dstangle = hex2dec(buffer[3]) + hex2dec(buffer[2]) * 16;
      timeDelay = hex2dec(buffer[5]) + hex2dec(buffer[4]) * 16;
      pointer = 0; //重新置位指针
      flag = 1;
    }
  else if (inByte == '*'){                    //格式：**返回所有舵机数据，以' '间隔，*01返回1号舵机的角度
    delay(2);
    inByte = Serial.read();
    if ('*' == inByte){
        //返回舵机当前角度，六个字节
        for (int i=0; i<6; i++){
            Serial.print(servos[i].read());
            //Serial.print(servos[i].readMicroseconds());
            Serial.print(' ');
          }
          Serial.println();
     }else {
        buffer[0] = inByte;
        delay(2);
        buffer[1] = Serial.read();
        Serial.print(servos[hex2dec(buffer[1])].read());
        Serial.print(' ');
     }
    }
    else if (inByte == '!'){
      resetServo();
    }
    else if (inByte == '@'){
      delay(2);
      inByte = Serial.read();
      selfTest(inByte);
      } 
    
    if (1 == flag){
      //Move(thisServo, dstangle, servosPos[thisServo].delaySpeed);
      Move(thisServo, dstangle, timeDelay);
//      Serial.print(thisServo);
//      Serial.print(dstangle);
//      Serial.print(timeDelay);
      flag = 0;
      }
    
  }

  //timer.run();            
}

void writeServo() {
  int servoNum = sMove[0];
  if(servoNum >=0 && servoNum <= maxServos) {
    destServoPos[servoNum] = sMove[1];
    servoMove[servoNum].enable();
    servoMove[servoNum].setInterval(sMove[2]);
  }
}

void setServoAttach() {
  int servo = 1;    // sAttach[0]
  int mode = 2;     // sAttach[1]
  if(servo >= 0 && servo <= maxServos) {
    if (mode == 1)
      servos[servo].attach(servo+4);
    else
      servos[servo].detach();
  }
}

void doServoFunc() {
  int x = curServo;      
  if(destServoPos[x] == currentServoPos[x]){
      servos[x].write(currentServoPos[x]);
      servoMove[x].disable();
      return;
    }
           
  if(destServoPos[x] > currentServoPos[x])
    currentServoPos[x]++;
  else
    currentServoPos[x]--;
  
  servosPos[x].curPos = constrain(currentServoPos[x], servosPos[x].maxPos, servosPos[x].minPos);
  currentServoPos[x] = servosPos[x].curPos;
  servos[x].write(currentServoPos[x]);
  jointPos(x, currentServoPos[x]);
  //Serial.println(currentServoPos[x]);
}

void Move(int servoNum, int servoPosition, int delayTime) {  //舵机驱动指令
  sMove[0] = servoNum;             //所驱动舵机号
  sMove[1] = servoPosition;          //舵机的目标位置
  sMove[2] = delayTime;            //每个舵机运动的延迟时长 
  writeServo();
}

void Attach(int servoNum, int servoMode) {
  sAttach[0] = servoNum;
  sAttach[1] = servoMode;
}
void jointPos(byte t, byte pos) {    //定义两个byte类型的变量，t,pose.
  servos[t].write(pos);
}

void resetServo(){
  //Serial.println("reset the servo");
  for (int j = 0 ;j < 6; j++){
    //Serial.println(servosPos[j].startpos);
    Move(j, servosPos[j].startpos, servosPos[j].delaySpeed);
    //Serial.print(servosPos[j].startpos);
   // Serial.print(' ');
    }
}

int hex2dec(byte c){
    if (c >= '0' && c <= '9'){
      return c - '0'; 
      }
    else if(c >= 'A' && c <= 'F'){
      return c - 'A' + 10;
      }
}

void selfTest(byte testAngle){
  Serial.println("self testing");
  if (testAngle == '1'){
      for (int i=0; i<6; i++){
          Move(i, servosPos[i].maxPos, servosPos[i].delaySpeed);
      }
    }
  else if (testAngle == '2'){
      for (int i=0; i<6; i++){
          Move(i, servosPos[i].minPos, servosPos[i].delaySpeed);
      }
   }
}
  
