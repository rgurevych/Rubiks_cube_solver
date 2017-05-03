
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

//Default motors movements limits
#define minAngleMotor1 0    // minimum limit fur turning
#define maxAngleMotor1 179  // maximum limit for turning
#define minAngleMotor2 95   // minimum limit fur turning
#define maxAngleMotor2 135  // maximum limit for turning
#define minAngleMotor3 45   // minimum limit fur turning
#define maxAngleMotor3 95   // maximum limit for turning
#define minAngleMotor4 35   // minimum limit fur turning
#define maxAngleMotor4 98   // maximum limit for turning
#define standardDelay 200   // standard delay for motor movement

//Including libraries
#include <Servo.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//Defining variables

//Initialize the variables for color frequences
byte frequency = 0;
byte f_RED = 0;
byte f_GREEN = 0;
byte f_BLUE = 0;

//Calibration matrix
byte f[][6] = {{13, 19, 14, 25, 11, 16},         //white
               {12, 21, 18, 23, 22, 27},         //yellow
               {12, 17, 26, 32, 24, 30},         //orange
               {20, 25, 34, 42, 26, 35},         //red
               {30, 35, 25, 32, 27, 36},         //green
               {34, 44, 33, 43, 20, 26}          //blue
               };

//Default motors positions
byte motor_1_Position = 90;
byte motor_2_Position = 104;
byte motor_3_Position = 89;
byte motor_4_Position = 36;


//Flags
volatile char pressedButton;
volatile byte selectedButton;
volatile boolean operateFlag = true;
boolean serialOutput = false;


//Data arrays and matrixes
//Arrays for color scanning
byte Motor_1_ScanAngles [9] = {44, 85, 120, 4, 4, 168, 134, 96, 58};
byte Motor_4_ScanAngles [9] = {71, 74, 70, 74, 82, 73, 94, 90, 94};
//Arrays for side and cube scanning
char SideScanResult [10];
char ScannedCube [55];               // = "woybogwrybwbbwggygoyoogoryryowbrgyrwbybbyggwgrwrrbrowo";
String SolvedCube = "";
//Other global variables
byte cubeCounter;
char string1LCD [17];
char string2LCD [17];

//Constants


//Class that defines the button as object
class Button {
  public:
    Button(byte pin, byte timeButton); 
    boolean volatile flagPress;    
    boolean volatile flagClick;    
    void filterAvarage();     
    void setPinTime(byte pin, byte timeButton); 

  private:
    byte  _buttonCount;    
    byte  _timeButton;      
    byte  _pin;             
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
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(scanSensor, INPUT);

  //Attaching the motors
  Motor1.attach(rotateMotorPin);
  Motor2.attach(carriageMotorPin);
  Motor3.attach(pusherMotorPin);
  Motor4.attach(scannerMotorPin);
  
  //Starting serial connection
  Serial.begin(9600);
  
  turnMotor1(89, 3);
  delay(standardDelay);
  turnMotor3(90, 2);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
  turnMotor4(35, 7);
  delay(standardDelay);

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
  
  normalFlow();
  
  //solveOrCaliblate();
  
  //operateOrNot();

  //scanCube();
  
  //verifyScannedCube();
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

//Method for normal flow operation
void normalFlow() {
  selectedButton = selectOption(F("Select mode"), F("Solve  Calibrate"), F("Solve"), F("       Calibrate"));
  if (selectedButton == 1) calibrateOptions();
  else solvingOptions();
}


//Method for selecting the scanning option
void calibrateOptions() {
  selectedButton = selectOption(F("Calibrate"), F("Side        Cube"), F("Side"), F("            Cube"));
  if (selectedButton == 0) {
    scanNewSide:
    serialOutput = true;
    calibrationScan();
    serialOutput = false;
    selectedButton = selectOption(F("Scan other side?"), F(" Yes        Back"), F(" Yes"), F("           Back"));
    if (selectedButton == 0) goto scanNewSide;
    else normalFlow();      
  }
  else calibrationScan();
}


//Method for selecting the solving mode
void solvingOptions() {
  selectedButton = selectOption(F("Solving mode?"), F("Python   Arduino"), F("Python"), F("         Arduino"));
  if (selectedButton == 0) pythonSolveOperation();
  else delay(5000);      //arduinoSolveOperation();
}


//Method for general process of solving the cube via Python
void pythonSolveOperation() {
  operateFlag = true;
  scanCube();  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  detachInterrupt(0);
  writeAndReadPortData();
  assembleCube(SolvedCube);
  //assembleCube("UR");
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
byte selectOption(String title, String options, String opt_1, String opt_2) {
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
    selectedButton = selectOption(F("Operation paused"), F("Resume     Reset"), F("Resume"), F("           Reset"));
  if (selectedButton == 0) {
    printLCD();
    operateFlag = true;
    //attachInterrupt (0, leftButtonPressed, RISING);
    
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Resetting..."));
    delay(1000);
    resetFunc();
  }
  }
  
}

//Method for saving the current string displayed on LCD
void storeLCD(String text, byte row) {
  if (row == 0) {
    for (byte i=0; i<16; i++) {
      string1LCD[i] = ' ';
      string1LCD[i] = text[i];
      }
  }
  else if (row == 1) {
    for (byte i=0; i<16; i++) {
      string2LCD[i] = ' ';
      string2LCD[i] = text[i];
      }
  }
  else if (row == 2) {
    for (byte i=0; i<=sizeof(text); i++) {
      if (text[i] != ' ') {
        string2LCD[i] = text[i];
        }
      }
  }
  printLCD();
}

//Method for printing saved strings to LCD
void printLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(string1LCD);
  lcd.setCursor(0, 1);
  lcd.print(string2LCD);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR COMMUNICATING VIA COM-PORT/////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*/Method for reading data from COM-port (the string with solution from Python script)
void readPortData() {
  char PortReadyFlag;
  boolean connectionError;
  volatile long PortTimeout;
  
  resendData:
  
  storeLCD(F("Waiting for data"), 0);
  storeLCD(F("from Python..."), 1);

  PortReadyFlag = '0';
  connectionError = false;
  
  PortTimeout = millis();
  do {
    PortReadyFlag = '0';
    PortReadyFlag = char(Serial.read());
    while(Serial.available()) Serial.read();
    delay(100);
    } while ((PortReadyFlag != '!') && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }
    
  delay(300);
  Serial.print('!');

  storeLCD(F("Receiving data.."), 0);
  storeLCD(F(""), 1);
  
  PortTimeout = millis();
  do {
      SolvedCube = "";
      while (Serial.available()) {
      SolvedCube += char(Serial.read());
    }
    delay(100);
  } while ((SolvedCube.length() == 0) && (millis() - PortTimeout < 10000));
  
  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }

  storeLCD(F("Verifying data.."), 0);
  
  if (SolvedCube.length() < 10) Serial.print('0');
  Serial.print(SolvedCube.length());

  PortTimeout = millis();
  do {
      PortReadyFlag = '0';
      PortReadyFlag = char(Serial.read());
      while(Serial.available()) Serial.read();
      delay(100);
  } while(!((PortReadyFlag == '+')||(PortReadyFlag == 'x')) && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }
    
  if (PortReadyFlag == '+') {
    storeLCD(F("Data received"), 0);
    storeLCD(F("successfully"), 1);
    }
  else {
    connectionError = true;
    goto sendingError;   
    } 

  sendingError:
    if (connectionError == true) {
      connectionError = false;
      selectedButton = selectOption(F("Receiving error"), F("Repeat   Restart"), F("Repeat"), F("         Restart"));
        if (selectedButton == 0) goto resendData;
        else normalFlow();
      }  
}

//Method for sending data to COM-port (the char array with cube colors to Python script)
void writePortData() {
  char PortReadyFlag;
  boolean connectionError;
  volatile long PortTimeout;
  
  resendData:
  
  storeLCD(F("Waiting for"), 0);
  storeLCD(F("connection..."), 1);

  Serial.print('!');

  PortReadyFlag = '0';
  connectionError = false;
  
  PortTimeout = millis();
  do {
    PortReadyFlag = '0';
    PortReadyFlag = char(Serial.read());
    while(Serial.available()) Serial.read();
    delay(100);
  } while ((PortReadyFlag != '!') && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }

  storeLCD(F("Sending data"), 0);
  storeLCD(F(""), 1);
  Serial.print(ScannedCube);
    
  PortTimeout = millis();
  do {
      PortReadyFlag = '0';
      PortReadyFlag = char(Serial.read());
      while(Serial.available()) Serial.read();
      delay(100);
  } while(!((PortReadyFlag == '+')||(PortReadyFlag == 'x')) && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }
    
  if (PortReadyFlag == '+') {
    storeLCD(F("Data sent"), 0);
    storeLCD(F("successfully"), 1);
    }
  else {
    connectionError = true;
    goto sendingError;   
    } 

  sendingError:
    if (connectionError == true) {
      connectionError = false;
      selectedButton = selectOption(F("Sending error"), F("Resend   Restart"), F("Resend"), F("         Restart"));
        if (selectedButton == 0) goto resendData;
        else normalFlow();
      }     
}
*/


//Method for sending and receiving data from Python script
void writeAndReadPortData() {
  char PortReadyFlag;
  boolean connectionError;
  volatile long PortTimeout;
  
  resendData:
  
  storeLCD(F("Waiting for"), 0);
  storeLCD(F("connection..."), 1);

  Serial.print('!');

  PortReadyFlag = '0';
  connectionError = false;
  
  PortTimeout = millis();
  do {
    PortReadyFlag = '0';
    PortReadyFlag = char(Serial.read());
    while(Serial.available()) Serial.read();
    delay(100);
  } while ((PortReadyFlag != '!') && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }

  storeLCD(F("Sending data"), 0);
  storeLCD(F(""), 1);
  Serial.print(ScannedCube);
    
  PortTimeout = millis();
  do {
      PortReadyFlag = '0';
      PortReadyFlag = char(Serial.read());
      while(Serial.available()) Serial.read();
      delay(100);
  } while(!((PortReadyFlag == '+')||(PortReadyFlag == 'x')) && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }
    
  if (PortReadyFlag == '+') {
    storeLCD(F("Data sent"), 0);
    storeLCD(F("successfully"), 1);
    }
  else {
    connectionError = true;
    goto sendingError;   
    } 

  PortTimeout = millis();
  do {
    PortReadyFlag = '0';
    PortReadyFlag = char(Serial.read());
    while(Serial.available()) Serial.read();
    delay(100);
    } while ((PortReadyFlag != '!') && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }
    
  delay(300);
  Serial.print('!');

  storeLCD(F("Receiving data.."), 0);
  storeLCD(F(""), 1);
  
  PortTimeout = millis();
  do {
      SolvedCube = "";
      while (Serial.available()) {
      SolvedCube += char(Serial.read());
    }
    delay(100);
  } while ((SolvedCube.length() == 0) && (millis() - PortTimeout < 10000));
  
  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }

  storeLCD(F("Verifying data.."), 0);
  
  if (SolvedCube.length() < 10) Serial.print('0');
  Serial.print(SolvedCube.length());

  PortTimeout = millis();
  do {
      PortReadyFlag = '0';
      PortReadyFlag = char(Serial.read());
      while(Serial.available()) Serial.read();
      delay(100);
  } while(!((PortReadyFlag == '+')||(PortReadyFlag == 'x')) && (millis() - PortTimeout < 10000));

  if ((millis() - PortTimeout) >= 10000) {
    connectionError = true;
    goto sendingError;
    }
    
  if (PortReadyFlag == '+') {
    storeLCD(F("Data received"), 0);
    storeLCD(F("successfully"), 1);
    }
  else {
    connectionError = true;
    goto sendingError;   
    } 

  sendingError:
    if (connectionError == true) {
      connectionError = false;
      selectedButton = selectOption(F("Connection error"), F("Resend   Restart"), F("Resend"), F("         Restart"));
        if (selectedButton == 0) goto resendData;
        else normalFlow();
      }     
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR SCANNING AND SCANNER CALIBRATION///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Method for scanning the cube
void scanCube() {  
  operateFlag = true;
  cubeCounter = 0;
  //Print process on LCD
  storeLCD(F("Scanning cube"), 0);
  storeLCD(F("Side 1(U) of 6"), 1);
    
  scanTopSide();
  
  push();
  turnCubeCW();
  push();
  turnCubeStraight();
  storeLCD(F("     2(R)"), 2);
  scanTopSide();
  
  turnCubeCCW();
  push();
  turnCubeStraight();
  storeLCD(F("     3(F)"), 2);
  scanTopSide();
  
  push();
  storeLCD(F("     4(D)"), 2);
  scanTopSide();

  push();
  push();
  push();
  turnCubeCCW();
  push();
  turnCubeStraight();
  storeLCD(F("     5(L)"), 2);
  scanTopSide();

  turnCubeCCW();
  push();
  turnCubeStraight();
  storeLCD(F("     6(B)"), 2);
  scanTopSide();

  turnCubeCW();
  push();
  push();
  turnCubeStraight();
  push();
  push();
  push();

  verifyScannedCube();
  //Serial.println(ScannedCube); /////////////////////////////////////////////////////////////////////////////////
  
}


//Method for vefirying the scanned cube side
void verifyScannedSide() {
  boolean sideOK = true;
  for (byte i=0; i<9; i++) {
    if (SideScanResult[i] == 'X') sideOK = false;
  }
  if (sideOK == false) {
    selectedButton = selectOption(F("Side scan error"), F("Repeat   Restart"), F("Repeat"), F("         Restart"));
  if (selectedButton == 0) scanTopSide();
  else normalFlow();
  }
  else addSideToScanCubeString();
}



//Method for verifying whether the cube was scanned correctly
void verifyScannedCube() {
  boolean scanErrorFlag = false;
  byte w_count = 0;
  byte y_count = 0;
  byte o_count = 0;
  byte r_count = 0;
  byte g_count = 0;
  byte b_count = 0;
  for (byte i=0; i<54; i++) {
    if (ScannedCube[i] == 'w') {w_count++;}
    else if (ScannedCube[i] == 'y') {y_count++;}
    else if (ScannedCube[i] == 'o') {o_count++;}
    else if (ScannedCube[i] == 'r') {r_count++;}
    else if (ScannedCube[i] == 'g') {g_count++;}
    else if (ScannedCube[i] == 'b') {b_count++;}
    else {scanErrorFlag = true;}
  }
  
  if ((w_count != 9) || (y_count != 9) || (o_count != 9) || (r_count != 9) || (g_count != 9) || (b_count != 9)) {scanErrorFlag = true;}
  
  Serial.println(ScannedCube);
  
  if (scanErrorFlag == true) {
    selectedButton = selectOption(F("Cube scan error"), F("Rescan   Restart"), F("Rescan"), F("         Restart"));
    if (selectedButton == 0) scanCube();
    else normalFlow();
  }
  
}

//Method for caibration scan of one side - scan each item once and print the RGB values
void calibrationScan() {
  attachInterrupt (0, leftButtonPressed, RISING);
  operateFlag = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Calibrating..."));
  turnMotor2(95, 5);
  delay(200);
  turnMotor4(35, 7);
  delay(200);
  for (byte i = 0; i<9; i++) {
    turnMotor1(Motor_1_ScanAngles[i], 4);
    turnMotor4(Motor_4_ScanAngles[i], 7);
    delay(100);
    scan_item();
    delay(100);
    if (serialOutput == true) {
      Serial.print(F("R= "));//printing name
      Serial.print(f_RED);//printing RED color frequency
      Serial.print(F("  G= "));
      Serial.print(f_GREEN);//printing RED color frequency
      Serial.print(F("  B= "));
      Serial.println(f_BLUE);
    }
  }
  turnMotor4(35, 7);
  turnCubeStraight();
  detachInterrupt (0);
}


//Method for scanning cube's top side
void scanTopSide() {
  turnMotor2(95, 5);
  delay(200);
  turnMotor4(35, 7);
  delay(200);
  attachInterrupt (0, leftButtonPressed, RISING);
  operateFlag = true;
  for (byte i = 0; i<9; i++) {
    turnMotor1(Motor_1_ScanAngles[i], 4);
    delay(50);
    turnMotor4(Motor_4_ScanAngles[i], 5);
    delay(200);
    SideScanResult[i] = scanResult();
    delay(200);
  }
  turnMotor4(35, 7);
  turnCubeStraight();
  detachInterrupt (0);
  verifyScannedSide();
}


//Method to add the scanned side to the common scanCube variable
void addSideToScanCubeString() {
  for (byte i=0; i<9; i++) {
    ScannedCube[cubeCounter] = SideScanResult[i];
    cubeCounter ++;
  }
  //Serial.println(ScannedCube); ////////////////////////////////////////////////////////////////////////
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
char defineColor(byte r, byte g, byte b) {
  if(r>=f[0][0] && r<=f[0][1] && g>=f[0][2] && g<=f[0][3] && b>=f[0][4] && b<=f[0][5]){
    return 'w';
  }
  else if(r>=f[1][0] && r<=f[1][1] && g>=f[1][2] && g<=f[1][3] && b>=f[1][4] && b<=f[1][5]){
    return 'y';
  }
  else if(r>=f[2][0] && r<=f[2][1] && g>=f[2][2] && g<=f[2][3] && b>=f[2][4] && b<=f[2][5]){
    return 'o';
  }
  else if(r>=f[3][0] && r<=f[3][1] && g>=f[3][2] && g<=f[3][3] && b>=f[3][4] && b<=f[3][5]){
    return 'r';
  }
  else if(r>=f[4][0] && r<=f[4][1] && g>=f[4][2] && g<=f[4][3] && b>=f[4][4] && b<=f[4][5]){
    return 'g';
  }
  else if(r>=f[5][0] && r<=f[5][1] && g>=f[5][2] && g<=f[5][3] && b>=f[5][4] && b<=f[5][5]){
    return 'b';
  }
  else {
    return 'X';
  }
}


//Function that repeats scanning up to 3 times if the color was not defined
char scanResult(){
  char color;
  byte wrongScan = 0;
  do{
    scan_item();
    color = defineColor(f_RED, f_GREEN, f_BLUE);
    wrongScan++ ;
    if (color != 'X') { 
      break;
      }
  } while (wrongScan < 10);
  return color;
}

//Supportive function for device calibration
/*void calibrate() {
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
}*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR CUBE ASSEMBLING////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Method for assembling cube based on the steps string
void assembleCube(String movementString) {
  attachInterrupt (0, leftButtonPressed, RISING);
  operateFlag = true;
  byte solutionLength = movementString.length();
  storeLCD(F("Assembling cube"), 0);
  storeLCD(("Step    of " + String(solutionLength)), 1);
  for (byte i=0; i<=solutionLength; i++) {
    char stepValue = movementString[i];    
    storeLCD("     " + String(i+1) + ' ', 2);
    makeStep(stepValue);
  }

  detachInterrupt(0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Cube assembled"));
  lcd.setCursor(0, 1);
  lcd.print(F("successfully!"));
  delay(3000);
}


//Method for implementing one defined step
void makeStep (char stepCode){
  operateFlag = true;  
  
  if(stepCode == 'D') {
    downCW();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
    push();
    push();
    push();
    
  }
  
  else if(stepCode == 'd') {
    downCCW();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
    push();
    push();
    push();
  }
  
  else if(stepCode == 'U') {
    push();
    push();
    downCW();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
    push();
  }

  else if(stepCode == 'u') {
    push();
    push();
    downCCW();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
    push();
  }

  else if(stepCode == 'R') {
    turnCubeCCW();
    push();
    downCW();
    turnCubeCW();
    push();
    turnCubeStraight();
    push();
  }


  else if(stepCode == 'r') {
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

  else if(stepCode == 'L') {
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

  else if(stepCode == 'l') {
    turnCubeCW();
    push();
    downCCW();
    turnCubeCCW();
    push();
    turnCubeStraight();
    push();
  }

  else if(stepCode == 'F') {
    push();
    push();
    push();
    downCW();
    push();
    turnCubeCCW();
    push();
    turnCubeStraight();
  }

  else if(stepCode == 'f') {
    push();
    push();
    push();
    downCCW();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
  }

  else if(stepCode == 'B') {
    push();
    downCW();
    push();
    push();
    push();
    turnCubeCW();
    push();
    turnCubeStraight();
  }

  else if(stepCode == 'b') {
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
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Wrong step code"));
    operateFlag = false;
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////ELEMENTARY MOTORS MOVEMENTS//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//Method for turning the Motor1 (the motor which rotates the cube)
void turnMotor1(byte degree, byte motorSpeed) {
  resumeOrRestart();
  operateFlag = true;
  byte limitedDegree = constrain(degree, minAngleMotor1, maxAngleMotor1);
  if(motor_1_Position < limitedDegree) {
    for(byte i = motor_1_Position; i <=limitedDegree ; i++) {
    Motor1.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_1_Position > limitedDegree) {
    for(byte i = motor_1_Position; i >= limitedDegree; i--) {
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
    for(byte i = motor_2_Position; i <=limitedDegree ; i++) {
    Motor2.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_2_Position > limitedDegree) {
    for(byte i = motor_2_Position; i >= limitedDegree; i--) {
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
    for(byte i = motor_3_Position; i <=limitedDegree ; i++) {
    Motor3.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_3_Position > limitedDegree) {
    for(byte i = motor_3_Position; i >= limitedDegree; i--) {
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
    for(byte i = motor_4_Position; i <=limitedDegree ; i++) {
    Motor4.write(i);
    delay(motorSpeed);
    }
  }
  else if (motor_4_Position > limitedDegree) {
    for(byte i = motor_4_Position; i >= limitedDegree; i--) {
    Motor4.write(i);
    delay(motorSpeed);
    }
  }
  motor_4_Position = limitedDegree;
}


//Method for pushing the cube
void push() {
  turnMotor2(105, 5);
  delay(standardDelay);
  turnMotor3(45, 2);
  delay(standardDelay);
  turnMotor2(118, 5);
  delay(standardDelay);
  turnMotor3(95, 2);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
  turnMotor3(90, 2);
  delay(standardDelay);
}


//Method for turning the cube ClockWise
void turnCubeCW() {
  turnMotor2(95, 5);
  delay(standardDelay);
  turnMotor1(5, 3);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
}


//Method for turning the cube CounterClockWise
void turnCubeCCW() {
  turnMotor2(95, 5);
  delay(standardDelay);
  turnMotor1(171, 3);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
}


//Method for turning the cube straight
void turnCubeStraight() {
  turnMotor2(95, 5);
  delay(standardDelay);
  turnMotor1(88, 3);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
}


//Method for turning the bottom layer ClockWise
void downCW() {
  turnCubeCW();
  turnMotor3(45, 2);
  delay(standardDelay);
  turnMotor2(135, 5);
  delay(standardDelay);
  turnMotor1(95, 3);
  delay(standardDelay);
  turnMotor1(88, 3);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
  turnMotor3(90, 2);
  delay(standardDelay);
}


//Method for turning the bottom layer CounterClockWise
void downCCW() {
  turnCubeCCW();
  turnMotor3(45, 2);
  delay(standardDelay);
  turnMotor2(135, 5);
  delay(standardDelay);
  turnMotor1(80, 3);
  delay(standardDelay);
  turnMotor1(88, 3);
  delay(standardDelay);
  turnMotor2(105, 5);
  delay(standardDelay);
  turnMotor3(90, 2);
  delay(standardDelay);
}

