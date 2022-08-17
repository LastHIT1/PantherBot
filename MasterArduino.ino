#include <Wire.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

#define BMP_SCK 10
#define BMP_MISO 11
#define BMP_MOSI 12
#define BMP_CS 13
#define BNO055_SAMPLERATE_DELAY_MS (10)
#define SS_PIN 53
#define RST_PIN 5

#define SEALEVELPRESSURE_HPA (1013.25)

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;
Servo myservo;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;


const byte RELAY_PIN  = 22; //relay for lock
const byte ROW_NUM    = 4; //keypad row sizing
const byte COLUMN_NUM = 4; //keypad col sizing


char keys[ROW_NUM][COLUMN_NUM] = { //keypad mapping
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};


const byte password = 5;
const char masterCode[] = {'A', 'B', 'C', 'D'}; // Future master code. ***Needs fixing***
byte pin_rows[ROW_NUM] = {35, 33, 31, 29};   //connect to the row pinouts of the keypad
byte pin_column[COLUMN_NUM] = {43, 41, 39, 37}; //connect to the column pinouts of the keypad
byte a = 0; //variable to check the password on the lock
char var = '0'; // Variable for switch statement
char keyPad; //variable to read the value from the keyboard
char code[password] = {}; //Default Code. Do we need this??
char mainCode[4][sizeof(code)]; //initialization of the password
bool dropOff = true;
bool letMeOut = false;
String command;
unsigned long time1;
int rpiVal = 0;
int dist;
int pass = 0;
int k = 0;
int m = 0;
int check;
int servoCounter = 0;
int sendOnce = 0;
char x;
char dir;
int y;
int check2 = 0;
int i, c, initial, comp;
byte nuidPICC[4];
byte tag1[4];
unsigned long time2 = 0;
int tagXXXX = 0;
int waitflag = 0;
int tag2 = 27940, tag3 = -1629, tag4 = -16596;//double check1
int tag5 = 11861, tag6 = -18826, tag7 = -10796;//RFID #1
int tag8 = -5181, tag9 = 4031, tag10 = -7938;//RFID #2
int tag11 = -259, tag12 = -23966, tag13 = 31520;//RFID #3 Elevator button outside
int tag14 = 21500, tag15 = -14649, tag16 = -32218, tag17 = 11141, tag18 = 31761, tag19 = -1661;//RFID #4 Elevator button outside
int tag20 = -5906, tag21 = 30358, tag22 = 13641;//RFID #5 Elevator button inside
int tag23 = 13178, tag24 = 29618, tag25 = 349;//RFID #5 Elevator button inside
int tag26 = 25348, tag27 = -14338, tag28 = 12868;//RFID #5 Elevator button inside
int tagCounter = 0;
int elevatorCounter = 0;
int uart[9];//save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package
int checkLidar;//save check value
float distL = 0;
float distR = 0;
int errorDist = 0;
int velocity;
const int dirPin = 46;
const int stepPin = 47;
int stepsOutSideButton = 550;
int armCounter = 0;

int barInitial = 0;
int difference = 0;
int currentFloor = 1;
int tagTest = 15669;
int test = 0;
LiquidCrystal_I2C lcd(0x27, 20, 4); //lcd layout subject to change depending on lcd
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

void setup() {
  Serial.begin(9600);//set bit rate of serial port connecting Arduino with computer
  Serial1.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  if (!bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {
    Serial.print("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(1000);
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  Serial.setTimeout(15);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  myservo.attach(8);
  lcd.clear();
  lcd.init();
  lcd.backlight();
  //lcd.begin (20, 4);
  lcd.setCursor(0, 0); //print on first row
  lcd.print("*ENTER THE CODE*");
  lcd.setCursor(1 , 1); //print second row
  lcd.print("TO OPEN DOOR!!");
  delay(1500);
  myservo.write(0);
}

void loop() {
  switch (var) {

    case '0':
      dropOff = false;
      while (keyPad != NO_KEY) { //loop to have the arduino waiting for a key
        //Display animation while waiting
        keyPad = keypad.getKey();
        delay(50); //Check what amount of deleay is optimal for batery consuption
      }
      keyPad = NO_KEY; //resets pressed key
      a = 0;
      tagCounter = 0;
      var = '1'; //goes to case '1'
      break;

    case '1':
      SetCode();
      if (a == password) { //checks if all the digits on the password are valid
        lcd.clear();
        lcd.print("Opening door");
        digitalWrite(RELAY_PIN, HIGH); //opens door
        delay(1000);
        digitalWrite(RELAY_PIN, LOW); //closes door
        delay(1000);
        while (!dropOff) {
          delay(500);
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("door");
          TryAgain(1);
          if (test == 1) {
            dropOff = true;
            lcd.clear();
            var = '2';
            x = 'b';
            Serial1.write(x);
          }
          break;
        }
      } else if (a != password) {
        var = '1';
      }
      break;

    case '2':
      instruction(rfidScan());
      if (y == 1) {
        turning(dir, 90);
      }
      if (k == 1) {
        barometer(2);
      }
      if (m == 1) {
        turning(90);
      } else if (m == 2) {
        Serial1.write('f');
        m = 0;
        k = 0;
      }
      break;

    case '3':
      //RFID tag processing
      break;

    case '4':
      //Arrived to destination?
      ReadCode();
      if (a == password) { //check if all the values on the password match
        digitalWrite(RELAY_PIN, HIGH); //open door
        delay(2000);
        digitalWrite(RELAY_PIN, LOW); //closes door
        dropOff = true;
        var = '1';
        sendOnce --;
      }
      //sends Drop off signak=l to rasberry PI
      break;
  } //end of switch
} //end of void

void ReadCode()
{
  a = 0;
  byte i = 0;
  delay(250);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Please Enter");
  lcd.setCursor(0, 1);
  lcd.print("Password");

  while (i < password) { //gets password
    keyPad = keypad.getKey();
    if (keyPad) {
      lcd.setCursor(i, 3);
      lcd.print("*");
      mainCode[2][i] = keyPad;
      i++;
    }
  } //end of while(i<4)

  for (i = 0; i < sizeof(code) && a == i; i++) { //matches the two passwords to open lock
    if (code[i] != mainCode[2][i]) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Passwords did");
      lcd.setCursor(0, 1);
      lcd.print("not match");
      lcd.setCursor(0, 2);
      lcd.print("try again");
      delay(2000);
      lcd.clear();
    } else {
      a++;
      lcd.clear();
      lcd.print("Door opening");
    }
  } //end of for loop

} // end of readCode()

void SetCode()
{
  a = 0;
  byte j = 0;
  byte i = 0;
  byte count = 0;
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter given code");

  while (count < password * 2) { //gets 2 passwords to comfirm they are the same
    keyPad = keypad.getKey();
    if (keyPad) {
      lcd.setCursor(i, 1);
      lcd.print(keyPad);
      mainCode[j][i] = keyPad;
      i++;
      if (i == password) {
        i = 0;
        j++;
        delay(250);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Re-enter password");
      }
      count++;
      keyPad = NO_KEY;
    }
  } //end of while(count<8)

  for (i = 0; i < sizeof(code) && a == i; i++) {
    if (mainCode[0][i] == mainCode[1][i]) {
      code[i] = mainCode[1][i];
      a++;
    } else if (mainCode[0][i] != mainCode[1][i]) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Passwords did");
      lcd.setCursor(0, 1);
      lcd.print("not match");
      lcd.setCursor(0, 2);
      lcd.print("try again");
      delay(2000);
      lcd.clear();
    }
  } //end of for loop

} //end of setCode
void TryAgain(unsigned long duration) {
  lcd.setCursor(0, 1);
  lcd.print("To reopen door");
  lcd.setCursor(0, 2);
  lcd.print("please press 'A'");
  time1 = millis();
  do { //give delivery a 2 min timer to reopen storage
    keyPad = keypad.getKey();
    if (keyPad == 'A') {
      var = '1';
    } else if ((millis() - time1) >= duration * 1000) {
      letMeOut = true;
      test = 1;
    }
  } while (!letMeOut && keyPad != 'A');
  keyPad = NO_KEY;
}

//end of MasterCode()
void instruction(int tagNum) {
  //Serial.println("inside");
  if ((tagXXXX == tag2 || tagXXXX == tag3 || tagXXXX == tag4 || tagXXXX == tag8 || tagXXXX == tag9 || tagXXXX == tag10) && tagCounter == 0) {
    x = 'l';
    dir = 'b';
    turning(dir, 90);
    y = 1;
    tagCounter++;
    tagXXXX = 0;
  } else if ((tagXXXX == tag2 || tagXXXX == tag3 || tagXXXX == tag4 || tagXXXX == tag8 || tagXXXX == tag9 || tagXXXX == tag10) && tagCounter == 1) {
    x = 'r';
    dir = 'f';
    turning(dir, 90);
    y = 1;
    tagXXXX = 0;
    tagCounter = 2;
  }

  if (tagXXXX == tag5 || tagXXXX == tag6 || tagXXXX == tag7) {
    lcd.clear();
    lcd.print("RFID #1");
    x = 'r';
    dir = 'p';
    turning(dir, 90);
    y = 1;
    tagXXXX = 0;
  }
  /*if (tagXXXX == tag8 || tagXXXX == tag9 || tagXXXX == tag10) {
    lcd.clear();
    lcd.print("RFID #2");
    x = 'l';
    dir = 'b';
    turning(dir, 90);
    tagXXXX = 0;
    y = 1;
  }*/
  if (tagXXXX == tag11 || tagXXXX == tag12 || tagXXXX == tag13) {
    lcd.clear();
    lcd.print("Elevator button outside");
    x = 's';
    Serial1.write(x);
    delay(250);
    servoCounter = 0;
    armCounter = 0;
    servo();
    x = 'f';
    Serial1.write(x);
    tagXXXX = 0;
  }

  if ((tagXXXX == tag14 || tagXXXX == tag15 || tagXXXX == tag16
       || tagXXXX == tag17 || tagXXXX == tag18 || tagXXXX == tag19) && elevatorCounter == 0) {
    lcd.clear();
    lcd.print("Elevator Center #1");
    /*barometer(1);
    x = 'r';
    dir = 'i';
    turning(dir, 90);
    tagXXXX = 0;
    elevatorCounter = 1;
    y = 1;*/
    barometer(1);
        x = 'r';
    dir = 'i';
    turning(dir, 90);
    y = 1;
    tagXXXX = 0;
    elevatorCounter++;

  } else if ((tagXXXX == tag14 || tagXXXX == tag15 || tagXXXX == tag16
              || tagXXXX == tag17 || tagXXXX == tag18 || tagXXXX == tag19) && elevatorCounter == 1) {
    lcd.clear();
    lcd.print("Elevator Center #2");
    barometer(2);
    k = 1;
    x = 'l';
    m = 1;
    turning(90);
    tagXXXX = 0;
    elevatorCounter = 0;
  }
  if (tagXXXX == tag20 || tagXXXX == tag21 || tagXXXX == tag22) {
    lcd.clear();
    lcd.print("Inside elevator button");
    x = 's';
    Serial1.write(x);
    delay(250);
    armCounter = 0;
    servoCounter = 1;
    servo();
    //add servo logic for inside button according to button
    x = 'b';
    Serial1.write(x);
    tagXXXX = 0;
  }
  if (tagXXXX == tag23 || tagXXXX == tag24 || tagXXXX == tag25) {
    lcd.clear();
    lcd.print("stop");
    x = 's';
    Serial1.write(x);
    tagXXXX = 0;
    var = '4';
  }
  if (tagXXXX == tag26 || tagXXXX == tag27 || tagXXXX == tag28) {
    x = 'r';
    dir = 'f';
    turning(dir, 90);
    y = 1;
    tagXXXX = 0;
  }
  if (tagXXXX == tagTest) {
    x = 'q';
    Serial1.write(x);
    tagXXXX = 0;
  }
}


void wait() {
  if (waitflag == 0) {
    time2 = millis();
    waitflag++;
  }

  if ((millis() - time2) >= 250) {
    Serial1.write(35);
    waitflag = 0;
  }

}

void wait(int duration) {
  if (waitflag == 0) {
    time2 = millis();
    waitflag++;
  }
  if ((millis() - time2) >= 1000) {

    waitflag = 0;
  }
}

int rfidScan() {
  //delay(50);
  if ( ! rfid.PICC_IsNewCardPresent())
    return;

  if ( ! rfid.PICC_ReadCardSerial())
    return;

  if (rfid.uid.uidByte[0] != nuidPICC[0] ||
      rfid.uid.uidByte[1] != nuidPICC[1] ||
      rfid.uid.uidByte[2] != nuidPICC[2] ||
      rfid.uid.uidByte[3] != nuidPICC[3] ) {
    Serial.println(F("A new card has been detected."));

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }

    tagXXXX = getTagAsInt(rfid.uid.uidByte, rfid.uid.size);
    Serial.print(tagXXXX);
    return tagXXXX;
  }
  else {
    Serial.println(F("Card read previously."));
    Serial.print(tagXXXX);
  }

  rfid.PICC_HaltA();
  //Proximity inductive coupling card. by uding this function we are stopping it from reading
  rfid.PCD_StopCrypto1();
}

int getTagAsInt(byte *buffer, byte bufferSize) {
  int i;
  int k = 0;
  for (i = 0; i < bufferSize; i++) {
    k = 10 * k + buffer[i];
  }
  return k * -1;
}

void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void printDec(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(' ');
    Serial.print(buffer[i], DEC);
  }
}
void turning(int degree) {
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  //delay(1);
  if (check2 == 0) {
    initial = (int)event.orientation.x;
    check2 = 1;
  }
  if (check2 == 1) {
    Serial1.write(x);
    comp = (int)event.orientation.x;
    if ((!((comp >= (initial - 2)) && (comp <= (initial + 2)))) && check2 == 1) {
      i = initial % degree;
      c = comp % degree;
      if ((i >= (c - 2)) && (i <= (c + 2))) {
        Serial1.write('b');
        y = 0;
        m = 0;
        initial = 0;
        check2 = 0;
      }
    }
  }
}
void turning(char d, int degree) {
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  //delay(1);
  if (check2 == 0) {
    initial = (int)event.orientation.x;
    check2 = 1;
  }
  if (check2 == 1) {
    Serial1.write(x);
    comp = (int)event.orientation.x;
    if ((!((comp >= (initial - 2)) && (comp <= (initial + 2)))) && check2 == 1) {
      i = initial % degree;
      c = comp % degree;
      if ((i >= (c - 2)) && (i <= (c + 2))) {
        Serial1.write(d);
        y = 0;
        initial = 0;
        check2 = 0;
      }
    }
  }
}
void servo() {
  char servoState = code[0];

  if (servoCounter == 0) {
    if (armCounter < 1) {
      // Set motor direction clockwise
      stepsOutSideButton = 550;
      digitalWrite(dirPin, HIGH);

      myservo.write(117);
      delay(150);
      // Spin motor slowly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);

      }
      delay(1500); // Wait a second

      // Set motor direction counterclockwise
      digitalWrite(dirPin, LOW);

      // Spin motor quickly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);

        delayMicroseconds(1000);
      }
      myservo.write(0);
      delay(1000); // Wait a second

      armCounter++;
      servoCounter++;
    }
  }
  if(servoState == '2' && servoCounter == 1) {
    stepsOutSideButton = 680;
    if (armCounter < 1) {
      // Set motor direction clockwise
      digitalWrite(dirPin, HIGH);

      myservo.write(105);
      delay(150);
      // Spin motor slowly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);

      }
      delay(1500); // Wait a second

      // Set motor direction counterclockwise
      digitalWrite(dirPin, LOW);

      // Spin motor quickly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);

        delayMicroseconds(1000);
      }
      myservo.write(0);
      delay(1000); // Wait a second

      armCounter++;
  }
  else if (servoState == '1' && servoCounter == 1) {
    stepsOutSideButton = 450;
    if (armCounter < 1) {
      // Set motor direction clockwise
      digitalWrite(dirPin, HIGH);

      myservo.write(117);
      delay(150);
      // Spin motor slowly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);

      }
      delay(1500); // Wait a second

      // Set motor direction counterclockwise
      digitalWrite(dirPin, LOW);

      // Spin motor quickly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);

        delayMicroseconds(1000);
      }
      myservo.write(0);
      delay(1000); // Wait a second

      armCounter++;
    }

  } else if (servoState == '3' && servoCounter == 1) {
        stepsOutSideButton = 550;
    if (armCounter < 1) {
      // Set motor direction clockwise
      digitalWrite(dirPin, HIGH);

      myservo.write(117);
      delay(150);
      // Spin motor slowly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);

      }
      delay(1500); // Wait a second

      // Set motor direction counterclockwise
      digitalWrite(dirPin, LOW);

      // Spin motor quickly
      for (int x = 0; x < stepsOutSideButton; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);

        delayMicroseconds(1000);
      }
      myservo.write(0);
      delay(1000); // Wait a second

      armCounter++;
    }
  }
}
}
void barometer(int set) {
  if (set == 1) {
    barInitial = (bmp.readAltitude(SEALEVELPRESSURE_HPA));
  } else if (set == 2) {
    difference = (bmp.readAltitude(SEALEVELPRESSURE_HPA)) - barInitial;
    if (difference >= 5 && code[0] == '3') {
      m = 2;
    } else if ((difference > 2.0 && difference < 5) && code[0] == '2') {
      m = 2;
    } else if (difference < 0.5 && code[0] == '1') {
      m = 2;
    }
  }
}
