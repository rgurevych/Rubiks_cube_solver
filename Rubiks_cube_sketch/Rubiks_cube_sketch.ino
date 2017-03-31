
//Defining pins
#define rotateMotorPin 4
#define carriageMotorPin 5
#define pusherMotorPin 6
#define scannerMotorPin 7
#define leftButtonPin 2
#define rightButtonPin 3
#define S0 9
#define S1 10
#define S2 11
#define S3 12
#define scanSensor 13

//Including libraries
#include <Servo.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//Defining variables

//Initialize the variables for color frequences
int frequency = 0;
int f_RED = 0;
int f_GREEN = 0;
int f_BLUE = 0;

//Calibration matrix
byte f[][6] = {{13, 19, 14, 25, 11, 16},         //white
               {12, 21, 18, 23, 22, 27},         //yellow
               {13, 19, 27, 37, 24, 33},         //orange
               {20, 25, 36, 42, 28, 35},         //red
               {30, 35, 25, 33, 27, 36},         //green
               {36, 44, 34, 43, 22, 28}          //blue
               };

//Default motors positions
byte motor_1_Position = 90;
byte motor_2_Position = 104;
byte motor_3_Position = 89;
byte motor_4_Position = 34;

//Default motors movements limits
byte minAngleMotor1 = 0;   // minimum limit fur turning
byte maxAngleMotor1 = 179; // maximum limit for turning
byte minAngleMotor2 = 95;  // minimum limit fur turning
byte maxAngleMotor2 = 135; // maximum limit for turning
byte minAngleMotor3 = 45;  // minimum limit fur turning
byte maxAngleMotor3 = 95;  // maximum limit for turning
byte minAngleMotor4 = 35;  // minimum limit fur turning
byte maxAngleMotor4 = 98;  // maximum limit for turning

//Flags
//volatile boolean leftButtonIsPressed = false;
//volatile boolean rightButtonIsPressed = false;
volatile char pressedButton;
volatile boolean operateFlag = true;


//Data arrays and matrixes
//Arrays for color scanning
byte Motor_1_ScanAngles [] = {44, 85, 120, 4, 4, 168, 134, 96, 58};
byte Motor_4_ScanAngles [] = {71, 74, 70, 74, 82, 73, 94, 90, 94};
String SideScanResult [9];
String ScannedCube = "";
String SolvedCube = "";


//Class that defines the button as object
class Button {
  public:
    Button(byte pin, byte timeButton); 
    boolean volatile flagPress;    // признак кнопка сейчас нажата
    boolean volatile flagClick;    // признак кнопка была нажата (клик)
    void filterAvarage();     // метод проверки состояние сигнала
    void setPinTime(byte pin, byte timeButton); // метод установки номера вывода и времени (числа) подтверждения

  private:
    byte  _buttonCount;    // счетчик подтверждений стабильного состояния   
    byte  _timeButton;      // время подтверждения состояния кнопки
    byte  _pin;             // номер вывода
};

//Initializing motors, LCD and buttons
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;
LiquidCrystal_I2C lcd(0x3f,16,2);
Button leftButton(leftButtonPin, 15);
Button rightButton(rightButtonPin, 15); 

//Reset function:
void(* resetFunc) (void) = 0;


//Setup method
void setup() {
  // Initializing pins modes
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(scanSensor, INPUT);

  //Initializing buttons
  //leftButton.setPinTime(leftButtonPin, 15);
  //rightButton.setPinTime(rightButtonPin, 15);
  //pinMode(leftButtonPin, INPUT_PULLUP);
  //pinMode(rightButtonPin, INPUT_PULLUP);
  
  //Attaching the motors
  Motor1.attach(rotateMotorPin);
  Motor2.attach(carriageMotorPin);
  Motor3.attach(pusherMotorPin);
  Motor4.attach(scannerMotorPin);
  
  //Starting serial connection
  Serial.begin(9600);
  
  turnMotor1(89, 3);
  delay(200);
  turnMotor3(90, 2);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
  turnMotor4(35, 7);
  delay(200);

  // Initialise interruption pins
  //attachInterrupt (0, leftButtonPressed, RISING);
  //attachInterrupt (1, rightButtonPressed, RISING);

  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  //Initialize LCD
  lcd.init();
  lcd.backlight();
  
  MsTimer2::set(2, timerInterupt);
  //MsTimer2::start();    
}

//Main operation method
void loop() {
  
  char startOption = selectOption("Select mode", "Solve  Calibrate", "Solve", "       Calibrate");
  if (startOption == 1) calibrationScan();
  else {
    char solvingMode = selectOption("Solving mode?", "Python   Arduino", "Python", "         Arduino");
    if (solvingMode == 0) {
      pythonSolveOperation();
    }
    else {
      delay(5000);
      //arduinoSolveOperation();
    }
  }
  //solveOrCaliblate();
  
  //operateOrNot();

  //scanCube();
  //ScannedCube = "oobwwrbbboboybobwwyyybrrggwrrryyggogwrrggwooywggwoyrby";

  //writePortData();
  //delay(500);
  
  //readPortData();
  
  //String movementString = "rLUdBBUUllFFllUUllFF";
  //assembleCube(SolvedCube);

  //scanTopSide();
  //printSide(SideScanResult);
  //addSideToScanCubeString(SideScanResult);

  
  
  //calibrationScan();
  
  delay(1000);

 

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR DESCRIBING OPERATION FLOWS///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Method for general process of solving the cube via Python
void pythonSolveOperation() {
  attachInterrupt (0, leftButtonPressed, FALLING);
  operateFlag = true;
  scanCube();

  detachInterrupt(0);
}





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR MANAGING INTERRUPTIONS AND OPERATION/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//Method for defining which button was pressed
char defineButtonPressed() {
  MsTimer2::start();
  leftButton.flagClick = false;
  rightButton.flagClick = false;
  char result;
  while (leftButton.flagClick == false && rightButton.flagClick == false);
  MsTimer2::stop();
  if (leftButton.flagClick == true ) {
    leftButton.flagClick= false;
    result = 0;
    }
  if (rightButton.flagClick == true ) {
    rightButton.flagClick= false;
    result = 1;
    }
   return result;
}


//Method from class Button to perform digital debouncing
void Button::filterAvarage() {
  if (flagPress != !digitalRead(_pin) ) {
    if ( _buttonCount != 0 ) _buttonCount--;
    }
  else {
    _buttonCount++;   
    if ( _buttonCount >= _timeButton ) {
      flagPress= !flagPress; 
      _buttonCount= 0;
      if ( flagPress == true ) flagClick= true;     
     }    
  }
}


//Button class constructor:
Button::Button(byte pin, byte timeButton) {
  _pin= pin;
  _timeButton= timeButton;
  pinMode(_pin, INPUT);  
}


//Method from class Button for setting the buttons
void Button::setPinTime(byte pin, byte timeButton)  {
  _pin= pin;
  _timeButton= timeButton;
  pinMode(_pin, INPUT);  
}


//Method for dealing with timer interruption
void  timerInterupt() {
  leftButton.filterAvarage();
  rightButton.filterAvarage(); 
}


//Method for dealing with button interruptions
void leftButtonPressed() {
  operateFlag = false;
}


//Method for showing possible options and pressing the button
int selectOption(String title, String options, String opt_1, String opt_2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print(options);
  pressedButton = defineButtonPressed();
  if (pressedButton == 0) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(opt_1);
    delay(500);
  }
  else if (pressedButton == 1) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(opt_2);
    delay(500);
  }
  return pressedButton;
}

//Method for starting or stopping operation
void resumeOrRestart() {
  //detachInterrupt(0);
  if (operateFlag == false) {
    char continueOption = selectOption("Operation paused", "Resume   Restart", "Resume", "         Restart");
  if (continueOption == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Resuming...");
    operateFlag = true;
    //attachInterrupt (0, leftButtonPressed, RISING);
    
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Resetting...");
    delay(1000);
    resetFunc();
  }
  }
  
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR COMMUNICATING VIA COM-PORT/////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Method for reading data from COM-port (the string with solution from Python script)
void readPortData() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for data");
  lcd.setCursor(0, 1);
  lcd.print("from Python...");
  String DataLength = "";
  long PortTimeout = millis();
  String PortReadyFlag = "";
  do {
    PortReadyFlag = "";
    while (Serial.available()) {
      PortReadyFlag += char(Serial.read());
    }
    delay(100);
  } while ((PortReadyFlag != "ready") && (millis() - PortTimeout < 10000));
  if ((millis() - PortTimeout) >= 10000) {goto Timeout;}
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Receiving data..");
  delay(100);
  
  PortTimeout = millis();
  do {
      SolvedCube = "";
      while (Serial.available()) {
      SolvedCube += char(Serial.read());
    }
    delay(100);
  } while ((SolvedCube.length() == 0) && (millis() - PortTimeout < 10000));
  if ((millis() - PortTimeout) >= 10000) {goto Timeout;}
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Verifying data..");
  delay(100);

  PortTimeout = millis();
  do {
      DataLength = "";
      while (Serial.available()) {
      DataLength += char(Serial.read());
    } 
    delay(100);
  } while ((DataLength.length() == 0) && (millis() - PortTimeout < 10000));
  if ((millis() - PortTimeout) >= 10000) {goto Timeout;}
    
  if (DataLength.toInt() == SolvedCube.length()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data received");
    lcd.setCursor(0, 1);
    lcd.print("successfully");
    }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error, try again");
    }
    
  PortTimeout = millis();
  Timeout:
    if ((millis() - PortTimeout) >= 10000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Timeout error");
    lcd.setCursor(0, 1);
    lcd.print("try again");
    } 
}

//Method for sending data to COM-port (the string with cube colors to Python script)
void writePortData() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for");
  lcd.setCursor(0, 1);
  lcd.print("connection...");
  Serial.print("ready");
  
  String PortReadyFlag = "";
  long PortTimeout = millis();
  do {
    PortReadyFlag = "";
    while (Serial.available()) {
      PortReadyFlag += char(Serial.read());
    }
    delay(100);
  } while ((PortReadyFlag != "ok") && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {goto Timeout;}

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sending data...");
  delay(100);
  Serial.print(ScannedCube);
    
  PortReadyFlag = "";
  PortTimeout = millis();
  do {
      PortReadyFlag = "";
      while (Serial.available()) {
      PortReadyFlag += char(Serial.read());
    } 
    delay(100);
  } while ((PortReadyFlag.length() == 0) && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {goto Timeout;}  
  if (PortReadyFlag == "ok") {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data sent");
    lcd.setCursor(0, 1);
    lcd.print("successfully");
    }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error, try again");
    } 
  PortTimeout = millis();
  Timeout:
    if ((millis() - PortTimeout) >= 10000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Timeout error");
    lcd.setCursor(0, 1);
    lcd.print("try again");
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR SCANNING AND SCANNER CALIBRATION///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Method for scanning the cube
void scanCube() {  
  operateFlag = true;
  ScannedCube = "";
  //Print process on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Scanning cube");
  lcd.setCursor(0, 1);
  lcd.print("Side      of 6");
  lcd.setCursor(5, 1);
  lcd.print("1(U)");
  
  scanTopSide();
  addSideToScanCubeString(SideScanResult);
  
  push();
  turnCubeCW();
  push();
  turnCubeStraight();
  lcd.setCursor(5, 1);
  lcd.print("2(R)");
  scanTopSide();
  addSideToScanCubeString(SideScanResult);
  
  turnCubeCCW();
  push();
  turnCubeStraight();
  lcd.setCursor(5, 1);
  lcd.print("3(F)");
  scanTopSide();
  addSideToScanCubeString(SideScanResult);
  
  push();
  lcd.setCursor(5, 1);
  lcd.print("4(D)");
  scanTopSide();
  addSideToScanCubeString(SideScanResult);

  push();
  push();
  push();
  turnCubeCCW();
  push();
  turnCubeStraight();
  lcd.setCursor(5, 1);
  lcd.print("5(L)");
  scanTopSide();
  addSideToScanCubeString(SideScanResult);

  turnCubeCCW();
  push();
  turnCubeStraight();
  lcd.setCursor(5, 1);
  lcd.print("6(B)");
  scanTopSide();
  addSideToScanCubeString(SideScanResult);

  turnCubeCW();
  push();
  push();
  turnCubeStraight();
  push();
  push();
  push();

  verifyScannedCube(ScannedCube);
  
}

//Method for verifying whether the cube was scanned correctly
void verifyScannedCube(String scannedCube) {
  boolean scanErrorFlag = false;
  byte w_count = 0;
  byte y_count = 0;
  byte o_count = 0;
  byte r_count = 0;
  byte g_count = 0;
  byte b_count = 0;
  if (scannedCube.length() != 54) {scanErrorFlag = true;}
  else {
    for (byte i=0; i<54; i++) {
      if (String(scannedCube[i]) == "w") {w_count++;}
      else if (String(scannedCube[i]) == "y") {y_count++;}
      else if (String(scannedCube[i]) == "o") {o_count++;}
      else if (String(scannedCube[i]) == "r") {r_count++;}
      else if (String(scannedCube[i]) == "g") {g_count++;}
      else if (String(scannedCube[i]) == "b") {b_count++;}
      else {scanErrorFlag = true;}
    }
    if ((w_count != 9) || (y_count != 9) || (o_count != 9) || (r_count != 9) || (g_count != 9) || (b_count != 9)) {scanErrorFlag = true;}
  }

  if (scanErrorFlag == true) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Scan error");
    lcd.setCursor(0, 1);
    lcd.print("Repeat scan");
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Scan completed");
    lcd.setCursor(0, 1);
    lcd.print("successfully");
    Serial.println(ScannedCube);
  }
  
}

//Method for caibration scan of one side - scan each item once and print the RGB values
void calibrationScan() {
  attachInterrupt (0, leftButtonPressed, FALLING);
  operateFlag = true;
  turnMotor2(95, 5);
  delay(200);
  turnMotor4(35, 7);
  delay(200);
  for (int i = 0; i<9; i++) {
    turnMotor1(Motor_1_ScanAngles[i], 4);
    turnMotor4(Motor_4_ScanAngles[i], 7);
    delay(200);
    calibrate();
    delay(200);
  }
  turnMotor4(35, 7);
  turnCubeStraight();
}


//Method for scanning cube's top side
void scanTopSide() {
  turnMotor2(95, 5);
  delay(200);
  turnMotor4(35, 7);
  delay(200);
  for (int i = 0; i<9; i++) {
    turnMotor1(Motor_1_ScanAngles[i], 4);
    delay(50);
    turnMotor4(Motor_4_ScanAngles[i], 5);
    delay(200);
    SideScanResult[i] = scanResult();
    delay(200);
  }
  turnMotor4(35, 7);
  turnCubeStraight();
}


//Method to add the scanned side to the common scanCube variable
void addSideToScanCubeString(String cubeSide[9]) {
  for (byte i=0; i<9; i++) {
    ScannedCube += SideScanResult[i];
  }
  //Serial.println(ScannedCube);
}


//Method for scanning one item
void scan_item() {
  delay(50);
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  f_RED = pulseIn(scanSensor, LOW);
  delay(50);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  f_GREEN = pulseIn(scanSensor, LOW);
  delay(50);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  f_BLUE = pulseIn(scanSensor, LOW);
  delay(50);
}


//Function for detecting color (All devices and Arduino connected to power supply)
String defineColor(int r, int g, int b) {
  if(r>=f[0][0] && r<=f[0][1] && g>=f[0][2] && g<=f[0][3] && b>=f[0][4] && b<=f[0][5]){
    return "w";
  }
  else if(r>=f[1][0] && r<=f[1][1] && g>=f[1][2] && g<=f[1][3] && b>=f[1][4] && b<=f[1][5]){
    return "y";
  }
  else if(r>=f[2][0] && r<=f[2][1] && g>=f[2][2] && g<=f[2][3] && b>=f[2][4] && b<=f[2][5]){
    return "o";
  }
  else if(r>=f[3][0] && r<=f[3][1] && g>=f[3][2] && g<=f[3][3] && b>=f[3][4] && b<=f[3][5]){
    return "r";
  }
  else if(r>=f[4][0] && r<=f[4][1] && g>=f[4][2] && g<=f[4][3] && b>=f[4][4] && b<=f[4][5]){
    return "g";
  }
  else if(r>=f[5][0] && r<=f[5][1] && g>=f[5][2] && g<=f[5][3] && b>=f[5][4] && b<=f[5][5]){
    return "b";
  }
  else {
    return "undefined";
  }
}


//Function that repeats scanning up to 3 times if the color was not defined
String scanResult(){
  String color = " ";
  int wrongScan = 0;
  do{
    scan_item();
    color = defineColor(f_RED, f_GREEN, f_BLUE);
    wrongScan++ ;
    if (color != "undefined") { 
      break;
      }
  } while (wrongScan < 10);
  return color;
}

//Supportive function for device calibration
void calibrate() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(scanSensor, LOW);
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequency);//printing RED color frequency
  Serial.print("  ");
  delay(100);
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(scanSensor, LOW);
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequency);//printing GREEN color frequency
  Serial.print("  ");
  delay(100);
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(scanSensor, LOW);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(frequency);//printing BLUE color frequency
  Serial.println("  ");
  delay(100);
}

void printSide(String side[]) {
  byte e = 0; 
  for (byte i=0; i<3; i++) {
    for (byte j=0; j<3; j++) {
      Serial.print("[");
      Serial.print(side[e]);
      Serial.print("]");
      e++;
    }
    Serial.println("");
  }
}


/////////////////////////////////////////////////METHODS FOR CUBE ASSEMBLING/////////////////////////////////////////

//Method for assembling cube based on the steps string
void assembleCube(String movementString) {
for (byte i=0; i<=movementString.length(); i++) {
    String stepValue = String(movementString[i]);
    Serial.println(stepValue);

    //Print process on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Assembling cube");
    lcd.setCursor(0, 1);
    lcd.print("Step   ( ) of ");
    lcd.setCursor(5, 1);
    lcd.print(i+1);
    lcd.setCursor(14, 1);
    lcd.print(movementString.length());
    lcd.setCursor(8, 1);
    lcd.print(stepValue);
        
    makeStep(stepValue);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cube assembled");
  lcd.setCursor(0, 1);
  lcd.print("successfully!");
}

//Method for implementing one defined step
void makeStep (String stepCode){
    
  if(stepCode == "D") {
    downCW();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
    push();
    push();
    push();
    
  }
  
  else if(stepCode == "d") {
    downCCW();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
    push();
    push();
    push();
  }
  
  else if(stepCode == "U") {
    push();
    push();
    downCW();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
    push();
  }

  else if(stepCode == "u") {
    push();
    push();
    downCCW();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
    push();
  }

  else if(stepCode == "R") {
    turnCubeCCW();
    push();
    downCW();
    turnCubeCW();
    push();
    turnCubeStraight();
    push();
  }


  else if(stepCode == "r") {
    turnCubeCCW();
    push();
    downCCW();
    turnCubeCCW();
    push();
    push();
    push();
    turnCubeStraight();
    push();
    push();
    push();
  }

  else if(stepCode == "L") {
    turnCubeCW();
    push();
    downCW();
    turnCubeCW();
    push();
    push();
    push();
    turnCubeStraight();
    push();
    push();
    push();
  }

  else if(stepCode == "l") {
    turnCubeCW();
    push();
    downCCW();
    turnCubeCCW();
    push();
    turnCubeStraight();
    push();
  }

  else if(stepCode == "F") {
    push();
    push();
    push();
    downCW();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
  }

  else if(stepCode == "f") {
    push();
    push();
    push();
    downCCW();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
  }

  else if(stepCode == "B") {
    push();
    downCW();
    push();
    push();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
  }

  else if(stepCode == "b") {
    push();
    downCCW();
    push();
    push();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
  }
  
  else {
    //operateFlag = false;
  }

}

/////////////////////////////////////////////////ELEMENTARY MOTORS MOVEMENTS/////////////////////////////////////////

//Method for turning the Motor1 (the motor which rotates the cube)
void turnMotor1(byte degree, byte motorSpeed) {
  resumeOrRestart();
  operateFlag = true;
  byte limitedDegree = constrain(degree, minAngleMotor1, maxAngleMotor1);
  if(motor_1_Position < limitedDegree) {
    for(int i = motor_1_Position; i <=limitedDegree ; i++) {
    Motor1.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_1_Position > limitedDegree) {
    for(int i = motor_1_Position; i >= limitedDegree; i--) {
    Motor1.write(i);
    delay(motorSpeed);
    }
  }
  motor_1_Position = limitedDegree;
}

//Method for turning the Motor2 (the motor which moves the carriage)
void turnMotor2(byte degree, byte motorSpeed) {
  resumeOrRestart();
  operateFlag = true;
  byte limitedDegree = constrain(degree, minAngleMotor2, maxAngleMotor2);
  if(motor_2_Position < limitedDegree) {
    for(int i = motor_2_Position; i <=limitedDegree ; i++) {
    Motor2.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_2_Position > limitedDegree) {
    for(int i = motor_2_Position; i >= limitedDegree; i--) {
    Motor2.write(i);
    delay(motorSpeed);
    }
  }
  motor_2_Position = limitedDegree;
}

//Method for turning the Motor3 (the motor that moves the pusher)
void turnMotor3(byte degree, byte motorSpeed) {
  resumeOrRestart();
  operateFlag = true;
  byte limitedDegree = constrain(degree, minAngleMotor3, maxAngleMotor3);
  if(motor_3_Position < limitedDegree) {
    for(int i = motor_3_Position; i <=limitedDegree ; i++) {
    Motor3.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_3_Position > limitedDegree) {
    for(int i = motor_3_Position; i >= limitedDegree; i--) {
    Motor3.write(i);
    delay(motorSpeed);
    }
  }
  motor_3_Position = limitedDegree;
}

//Method for turning the Motor4 (the motor that moves the color scanner)
void turnMotor4(byte degree, byte motorSpeed) {
  resumeOrRestart();
  operateFlag = true;
  byte limitedDegree = constrain(degree, minAngleMotor4, maxAngleMotor4);
  if(motor_4_Position < limitedDegree) {
    for(int i = motor_4_Position; i <=limitedDegree ; i++) {
    Motor4.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_4_Position > limitedDegree) {
    for(int i = motor_4_Position; i >= limitedDegree; i--) {
    Motor4.write(i);
    delay(motorSpeed);
    }
  }
  motor_4_Position = limitedDegree;
}


//Method for pushing the cube
void push() {
  turnMotor2(105, 5);
  delay(200);
  turnMotor3(45, 2);
  delay(200);
  turnMotor2(118, 5);
  delay(200);
  turnMotor3(95, 2);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
  turnMotor3(90, 2);
  delay(200);
}

//Method for turning the cube ClockWise
void turnCubeCW() {
  turnMotor2(95, 5);
  delay(200);
  turnMotor1(5, 3);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
}

//Method for turning the cube CounterClockWise
void turnCubeCCW() {
  turnMotor2(95, 5);
  delay(200);
  turnMotor1(171, 3);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
}

//Method for turning the cube straight
void turnCubeStraight() {
  turnMotor2(95, 5);
  delay(200);
  turnMotor1(88, 3);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
}

//Method for turning the bottom layer ClockWise
void downCW() {
  turnCubeCW();
  turnMotor3(45, 2);
  delay(200);
  turnMotor2(135, 5);
  delay(200);
  turnMotor1(95, 3);
  delay(200);
  turnMotor1(88, 3);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
  turnMotor3(90, 2);
  delay(200);
}

//Method for turning the bottom layer CounterClockWise
void downCCW() {
  turnCubeCCW();
  turnMotor3(45, 2);
  delay(200);
  turnMotor2(135, 5);
  delay(200);
  turnMotor1(80, 3);
  delay(200);
  turnMotor1(88, 3);
  delay(200);
  turnMotor2(105, 5);
  delay(200);
  turnMotor3(90, 2);
  delay(200);
}
