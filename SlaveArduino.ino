#include "RoboClaw.h"
#include <Wire.h>
//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>

//See limitations of Arduino SoftwareSerial
//SoftwareSerial Serial(9,10);
RoboClaw front(&Serial3, 10000);
RoboClaw back(&Serial2, 10000);
SoftwareSerial LidarB(10, 11);
SoftwareSerial LidarF(12, 13);


#define address 0x80
#define address2 0x81

int motionSpeed = 2000;
int rotationSpeed = 2500;
char prevCommand;
char command;
char something;
bool runningM;
bool stopM;
bool turningM ;
int specialDist = 0;
int dist;//actual distance measurements of LiDAR
int strength;//signal strength of LiDAR
int check;//save check value
int i;
int uart[9];//save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package
void setup() {
  //Open roboclaw serial ports
  front.begin(38400);
  back.begin(38400);
  Serial.begin(9600);
  Serial1.begin(115200);
  LidarF.begin(115200);
  LidarB.begin(115200);
  LidarB.listen();
  //Wire.begin(9);
 // Wire.onReceive(receiveEvent);
}

void loop() {
  //int16_t pwm1, pwm2;
  //preCommand = command;
  //something = command;
  //  if (runningM) {
  if(Serial1.available()){
    command = Serial1.read();
    //Serial.println(command);
    Serial1.flush();
    //Serial1.end();
  }
  check = command;
  //Serial.print(command);
  if(command!=prevCommand){
  switch (command) {
    case 'f': runMotors(); runningM = 1; stopM = 1; turningM =0; specialDist = 80; LidarF.listen(); break;
    case 'p': runMotors(); runningM = 1; stopM = 1; turningM =0; specialDist = 35; LidarF.listen(); break;
    case 'i': runMotors(); runningM = 1; stopM = 1; turningM =0; specialDist = 18; LidarF.listen(); break;
    case 'b': backMotors(); runningM = 1; stopM = 1; turningM =0; specialDist = 25; LidarB.listen(); break;
    case 'r': Right(); turningM = 1; break;
    case 'l': Left();  turningM = 1; break;
    case 'q': ForwardLeft(); runningM = 1; stopM = 1; turningM =0; specialDist = 60; LidarF.listen();break;
    case 'w': ForwardRight(); runningM = 1; stopM = 1; turningM =0; specialDist = 60; LidarF.listen();break;
    case 'd': Drift(); runningM = 1; stopM = 1; break;
    case 's': stopMotors(); runningM = 0; stopM = 0;turningM =0;   break;
  }
    prevCommand = command;
  }
  if(turningM == 0){
  lidars(specialDist);
  }
  delay(1);
  
}

/*void receiveEvent(byte bytes){
  command = Wire.read();
  
}*/

void lidars(int distStop) {
  //if (check == 'f') {
    
    //Serial.println(LidarF.isListening());
    if (LidarF.isListening()) {
        
      if (LidarF.available()) { //check if serial port has data input
        if (LidarF.read() == HEADER) { //assess data package frame header 0x59
          uart[0] = HEADER;
          if (LidarF.read() == HEADER) { //assess data package frame header 0x59
            uart[1] = HEADER;
            for (i = 2; i < 9; i++) { //save data in array
              uart[i] = LidarF.read();
            }
            check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
            if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
              dist = uart[2] + uart[3] * 256; //calculate distance value
              if (dist >= distStop && runningM == 1 && !stopM) {
                runMotors();
                stopM =1;
              }
              else if (dist < distStop && ((runningM == 1 && stopM == 1) || (runningM == 0 && stopM ==1))) {
                stopMotors();
                stopM = 0;
              }
            }
          }
        }
      }
    }
 // } else if (check == 'b') {
    //Serial.println("hello");
    if (LidarB.isListening()) {
      //Serial.println("hello");
      if (LidarB.available()) { //check if serial port has data input
        
        if (LidarB.read() == HEADER) { //assess data package frame header 0x59
          uart[0] = HEADER;
          if (LidarB.read() == HEADER) { //assess data package frame header 0x59
            uart[1] = HEADER;
            for (i = 2; i < 9; i++) { //save data in array
              uart[i] = LidarB.read();
            }
            check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
            if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
              dist = uart[2] + uart[3] * 256; //calculate distance value
             //Serial.println(dist);
             if(!(dist>250)){
              if (dist >= distStop && runningM == 1 && !stopM) {
                backMotors();
                stopM =1;
              }
              else if (dist < distStop && ((runningM == 1 && stopM == 1)|| (runningM == 0 && stopM ==1))) {
                stopMotors();
                stopM = 0;
              }
             }
            }
          }
        }
      }else if(!LidarB.isListening()){
        Serial.println("Not Listening");
      }
    }
  //}
}

void runMotors(void) {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  front.TurnRightMixed(address, 0);
  front.SpeedM1(address, motionSpeed);
  front.SpeedM2(address, motionSpeed);
  back.SpeedM1(address2, motionSpeed);
  back.SpeedM2(address2, motionSpeed);
}

void backMotors(void) {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  front.TurnRightMixed(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  back.SpeedM1(address2, -motionSpeed);
  back.SpeedM2(address2, -motionSpeed);
  front.SpeedM1(address, -motionSpeed);
  front.SpeedM2(address, -motionSpeed);
}

void stopMotors(void) {
  front.ForwardMixed(address, 0);
  back.ForwardMixed(address2, 0);
  front.TurnLeftMixed(address, 0);
  back.TurnRightMixed(address2, 0);
}

void Left() {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  front.SpeedM1(address, rotationSpeed);
  front.SpeedM2(address, -rotationSpeed);
  back.SpeedM1(address2, rotationSpeed);
  back.SpeedM2(address2, -rotationSpeed);
}

void Right() {
  front.ForwardMixed(address, 0);
  back.ForwardMixed(address2, 0);
  front.SpeedM1(address, -rotationSpeed);
  front.SpeedM2(address, rotationSpeed);
  back.SpeedM1(address2, -rotationSpeed);
  back.SpeedM2(address2, rotationSpeed);
}

void ForwardLeft() {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  front.TurnRightMixed(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  front.ForwardM2(address, 105);
  front.ForwardM1(address, 20);
  back.ForwardM2(address2, 105);
  back.ForwardM1(address2, 20);
}

void ForwardRight() {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  front.TurnRightMixed(address, 0);
  front.SpeedM1(address, motionSpeed-500);
  front.SpeedM2(address, motionSpeed);
  back.SpeedM1(address2, motionSpeed-500);
  back.SpeedM2(address2, motionSpeed);
}

void BackwardLeft() {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  front.TurnRightMixed(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  front.BackwardM2(address, 105);
  front.BackwardM1(address, 20);
  back.BackwardM2(address2, 105);
  back.BackwardM1(address2, 20);
}

void BackwardRight() {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  front.TurnRightMixed(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  front.BackwardM2(address, 20);
  front.BackwardM1(address, 105);
  back.BackwardM2(address2, 20);
  back.BackwardM1(address2, 105);
}

void Drift() {
  front.ForwardM1(address, 0); //start Motor1 forward at half speed
  front.BackwardM2(address, 0);
  front.TurnRightMixed(address, 0);
  back.ForwardM1(address2, 0); //start Motor1 forward at half speed
  back.BackwardM2(address2, 0);
  back.TurnRightMixed(address2, 0);
  front.ForwardM2(address, 127);
  front.ForwardM1(address, 0);
  back.ForwardM2(address2, 127);
  back.ForwardM1(address2, 127);
}
