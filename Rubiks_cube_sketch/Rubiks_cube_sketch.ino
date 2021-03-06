
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
#define scanSensor 8

//Default motors movements limits
#define minAngleMotor1 0    // minimum limit fur turning
#define maxAngleMotor1 179  // maximum limit for turning
#define minAngleMotor2 95   // minimum limit fur turning
#define maxAngleMotor2 135  // maximum limit for turning
#define minAngleMotor3 45   // minimum limit fur turning
#define maxAngleMotor3 95   // maximum limit for turning
#define minAngleMotor4 35   // minimum limit fur turning
#define maxAngleMotor4 98   // maximum limit for turning
#define standardDelay 175   // standard delay for motor movement

//Including libraries
#include <Servo.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/pgmspace.h>


//Defining variables

//Initialize the variables for color frequences
//byte frequency = 0;
byte f_RED = 0;
byte f_GREEN = 0;
byte f_BLUE = 0;

//Calibration matrix
byte f[][6] = {{16, 21, 16, 22, 13, 17},         //white
               {13, 18, 21, 27, 25, 29},         //yellow
               {15, 21, 31, 35, 26, 31},         //orange
               {23, 29, 34, 43, 32, 37},         //red
               {33, 39, 28, 33, 27, 36},         //green
               {38, 43, 38, 46, 25, 32}          //blue
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
const byte Motor_1_ScanAngles [9] PROGMEM = {44, 85, 120, 4, 4, 168, 134, 96, 58};
const byte Motor_4_ScanAngles [9] PROGMEM = {71, 74, 70, 74, 82, 73, 94, 90, 94};

//Arrays for side and cube scanning
char SideScanResult [10];
char ScannedCube [55] = "wwwwwwwwwbbbbbbbbbrrrrrrrrryyyyyyyyygggggggggooooooooo"; //"woybogwrybwbbwggygoyoogoryryowbrgyrwbybbyggwgrwrrbrowo";  ///////////////////////////////////////////////////////
char CubeCopy [55];
String SolvedCube = "";

//Variables for cube solving algorythms

//Map for detecting the rotation sequence number
const char v_map[12] PROGMEM = {'U', 'u', 'F', 'f', 'R', 'r', 'L', 'l', 'D', 'd', 'B', 'b'};

//Table with mapping sequences for cube virtual rotation
const byte v_table [][40] PROGMEM = {
  {0, 1, 2, 3, 5, 6, 7, 8, 18, 19, 20, 9, 10, 11, 45, 46, 47, 36, 37, 38, 6, 3, 0, 7, 1, 8, 5, 2, 9, 10, 11, 45, 46, 47, 36, 37, 38, 18, 19, 20},                   //U
  {0, 1, 2, 3, 5, 6, 7, 8, 18, 19, 20, 9, 10, 11, 45, 46, 47, 36, 37, 38, 2, 5, 8, 1, 7, 0, 3, 6, 36, 37, 38, 18, 19, 20, 9, 10, 11, 45, 46, 47},                   //u (U')
  {18, 19, 20, 21, 23, 24, 25, 26, 6, 7, 8, 9, 12, 15, 27, 28, 29, 38, 41, 44, 24, 21, 18, 25, 19, 26, 23, 20, 44, 41, 38, 6, 7, 8, 15, 12, 9, 27, 28, 29},         //F
  {18, 19, 20, 21, 23, 24, 25, 26, 6, 7, 8, 9, 12, 15, 27, 28, 29, 38, 41, 44, 20, 23, 26, 19, 25, 18, 21, 24, 9, 12, 15, 29, 28, 27, 38, 41, 44, 8, 7, 6},         //f (F')
  {9, 10, 11, 12, 14, 15, 16, 17, 2, 5, 8, 20, 23, 26, 29, 32, 35, 45, 48, 51, 15, 12, 9, 16, 10, 17, 14, 11, 20, 23, 26, 29, 32, 35, 51, 48, 45, 8, 5, 2},         //R
  {9, 10, 11, 12, 14, 15, 16, 17, 2, 5, 8, 20, 23, 26, 29, 32, 35, 45, 48, 51, 11, 14, 17, 10, 16, 9, 12, 15, 51, 48, 45, 2, 5, 8, 20, 23, 26, 35, 32, 29},         //r (R')
  {36, 37, 38, 39, 41, 42, 43, 44, 0, 3, 6, 18, 21, 24, 27, 30, 33, 47, 50, 53, 42, 39, 36, 43, 37, 44, 41, 38, 53, 50, 47, 0, 3, 6, 18, 21, 24, 33, 30, 27},       //L
  {36, 37, 38, 39, 41, 42, 43, 44, 0, 3, 6, 18, 21, 24, 27, 30, 33, 47, 50, 53, 38, 41, 44, 37, 43, 36, 39, 42, 18, 21, 24, 27, 30, 33, 53, 50, 47, 6, 3, 0},       //l (L')
  {27, 28, 29, 30, 32, 33, 34, 35, 24, 25, 26, 15, 16, 17, 51, 52, 53, 42, 43, 44, 33, 30, 27, 34, 28, 35, 32, 29, 42, 43, 44, 24, 25, 26, 15, 16, 17, 51, 52, 53}, //D
  {27, 28, 29, 30, 32, 33, 34, 35, 24, 25, 26, 15, 16, 17, 51, 52, 53, 42, 43, 44, 29, 32, 35, 28, 34, 27, 30, 33, 15, 16, 17, 51, 52, 53, 42, 43, 44, 24, 25, 26}, //d (D')
  {45, 46, 47, 48, 50, 51, 52, 53, 0, 1, 2, 11, 14, 17, 33, 34, 35, 36, 39, 42, 51, 48, 45, 52, 46, 53, 50, 47, 11, 14, 17, 35, 34, 33, 36, 39, 42, 2, 1, 0},       //B
  {45, 46, 47, 48, 50, 51, 52, 53, 0, 1, 2, 11, 14, 17, 33, 34, 35, 36, 39, 42, 47, 50, 53, 46, 52, 45, 48, 51, 42, 39, 36, 0, 1, 2, 17, 14, 11, 33, 34, 35}        //b (B')
  };

//Map for making the virtual Y move
const byte y_table[24] PROGMEM = {21, 22, 23, 12, 13, 14, 48, 49, 50, 39, 40, 41, 12, 13, 14, 48, 49, 50, 39, 40, 41, 21, 22, 23};

//Counter of Y turns
byte yTurnCounter = 0;

//Map for transferring the virtual move to physical move depending on Y-turn counter
const byte y_map[32] PROGMEM = {'F', 'f', 'R', 'r', 'B', 'b', 'L', 'l', 
                                'R', 'r', 'B', 'b', 'L', 'l', 'F', 'f', 
                                'B', 'b', 'L', 'l', 'F', 'f', 'R', 'r',
                                'L', 'l', 'F', 'f', 'R', 'r', 'B', 'b'};

//Table for F2L step possible patterns
const byte f2l_t [][5] PROGMEM = {
  {19, 20, 7, 8, 9}, {5, 8, 9, 10, 20}, {8, 37, 9, 3, 20}, {20, 1, 8, 46, 9}, {20, 37, 8, 3, 9}, {8, 1, 9, 46, 20}, {20, 46, 8, 1, 9}, {8, 3, 9, 37, 20},
  {8, 46, 9, 1, 20}, {20, 3, 8, 37, 9}, {8, 10, 9, 5, 20}, {20, 7, 19, 8, 9}, {19, 8, 9, 7, 20}, {20, 5, 8, 10, 9}, {20, 10, 8, 5, 9}, {7, 8, 9, 19, 20},
  {9, 19, 7, 20, 8}, {9, 5, 20, 10, 8}, {19, 45, 7, 11, 2}, {18, 5, 10, 38, 6}, {19, 36, 7, 47, 0}, {5, 36, 10, 47, 0}, {9, 10, 20, 5, 8}, {7, 9, 19, 20, 8},
  {19, 26, 7, 15, 29}, {5, 26, 10, 15, 29}, {19, 29, 7, 26, 15}, {5, 15, 10, 29, 26}, {19, 15, 7, 29, 26}, {5, 29, 10, 26, 15}, {23, 8, 9, 12, 20}, {20, 23, 8, 12, 9},
  {8, 12, 9, 23, 20}, {20, 12, 23, 8, 9}, {23, 9, 20, 12, 8}, {9, 12, 20, 23, 8}, {23, 15, 12, 29, 26}, {23, 29, 26, 12, 15}, {12, 15, 23, 29, 26}, {12, 29, 23, 26, 15},
  {12, 26, 23, 15, 29}
  };

//F2L solutions
const char f2l_solutions [] PROGMEM =
  "FrfR.URur.fuF.RUr.UfuFUfUUF.uRUrUURur.UfUUFUfUUF.uRUUrUURur.uRurUfuF.uRUrURUr.FUUFFuFFuf.rUURRURRUR.UfUFufuF.uRurURUr.RurUUfuF.RUrUURurURur"
  ".fUUFUfuF.RUUruRUr.fUUFufUF.RUUrURur.RUrfuF.RUrURur.FURurfRur.RurURUUrURUr.URurFrfR.rfRURurF.fUFufUF.RurURur.fuFUfuF.RUruRUr.uRurUURur.URUrUURUr"
  ".UURUrufUF.UfuFuRUr.RUruRUruRUr.RurUfUF.RuruRUrUURur.RurURUUrURur.RFURurfur.RBUUbrfuF.RurUfUUFUUfUF.";

//Table for OLL step possible patterns
const byte oll_t [][8] PROGMEM = {
  {1, 3, 5, 6, 7, 8, 47, 45}, {1, 3, 5, 6, 7, 8, 36, 11}, {0, 1, 3, 5, 7, 8, 38, 45}, {1, 3, 5, 7, 36, 11, 38, 9}, {1, 3, 5, 7, 36, 38, 45, 20},
  {1, 3, 5, 6, 7, 47, 11, 20}, {1, 3, 5, 7, 8, 36, 45, 18}, {36, 37, 38, 46, 19, 9, 10, 11}, {36, 37, 38, 45, 46, 19, 20, 10}, {6, 37, 47, 46, 10, 11, 19, 20},
  {8, 36, 37, 46, 45, 18, 19, 10}, {0, 2, 46, 37, 38, 9, 10, 19}, {0, 2, 46, 37, 10, 18, 19, 20}, {0, 8, 46, 45, 37, 38, 10, 19}, {0, 2, 6, 8, 46, 37, 10, 19},
  {0, 2, 3, 5, 6, 8, 46, 19}, {0, 1, 2, 3, 6, 8, 10, 19}, {3, 5, 47, 46, 9, 11, 18, 19}, {3, 5, 47, 46, 45, 38, 9, 19}, {3, 5, 36, 38, 46, 19, 9, 11},
  {3, 5, 47, 46, 45, 18, 19, 20}, {2, 3, 5, 36, 46, 18, 19, 9}, {0, 3, 5, 46, 38, 11, 19, 20}, {2, 3, 5, 47, 46, 38, 19, 20}, {0, 3, 5, 46, 45, 18, 19, 9},
  {2, 3, 5, 8, 36, 38, 46, 19}, {2, 3, 5, 8, 47, 46, 18, 19}, {6, 3, 5, 8, 36, 46, 11, 19}, {6, 3, 5, 8, 47, 46, 45, 19}, {0, 3, 5, 8, 46, 45, 38, 19},
  {2, 3, 5, 6, 47, 46, 19, 9}, {1, 5, 47, 45, 37, 38, 19, 9}, {1, 5, 47, 37, 18, 19, 9, 11}, {1, 5, 36, 37, 38, 9, 11, 19}, {1, 5, 37, 47, 45, 18, 19, 20},
  {1, 5, 36, 37, 38, 45, 19, 20}, {1, 5, 36, 37, 11, 18, 19, 20}, {1, 2, 5, 36, 37, 18, 19, 9}, {0, 1, 3, 38, 19, 20, 10, 11}, {1, 5, 6, 37, 47, 11, 19, 20},
  {1, 3, 8, 36, 45, 18, 19, 10}, {1, 3, 6, 47, 19, 20, 10, 11}, {1, 5, 8, 36, 37, 45, 18, 19}, {1, 3, 6, 36, 45, 19, 9, 10}, {1, 5, 8, 37, 38, 47, 19, 11},
  {0, 3, 6, 7, 46, 9, 10, 11}, {2, 5, 7, 8, 36, 37, 37, 46}, {0, 3, 6, 7, 46, 45, 10, 20}, {2, 5, 7, 8, 37, 47, 46, 18}, {1, 2, 3, 6, 47, 19, 9, 10},
  {0, 1, 5, 8, 37, 38, 45, 19}, {0, 1, 3, 8, 38, 45, 19, 10}, {0, 1, 3, 8, 10, 11, 18, 19}, {0, 2, 5, 7, 37, 46, 18, 20}, {0, 2, 3, 7, 46, 18, 20, 10},
  {0, 2, 5, 7, 37, 38, 46, 9}, {0, 2, 3, 7, 38, 46, 9, 10}
  };

//OLL solutions
const char oll_solutions [] PROGMEM =
  "RRdRUUrDRUUR.RURDruRdRR.RRdRurDRUR.RUrURurURUUr.RUURRuRRuRRUUR.RUrURUUr.luLulUUL.RUbRBRRurFRf.BlbLUllflFul.RUBubrBLUlub.RUBubrbruRUB.rUUFRUruFFUUFR"
  ".BlbLUUlubUBllul.RUrUrFRfUUrFRf.fBulULULUluFb.RUruLrFRfl.LFrflRURur.FURurURurf.BUbUBuLulb.rFRfUURurUfUUF.FUUFFuFufUULFl.lBLUlubLulUUL.rfRluLUrFR.BULUUluLUlb"
  ".lBLUlbLBub.FRUruf.RUrurFRf.RURRurFRURuf.bubRBrUB.rFRUrufUR.LfluLUFul.LFUfuFUful.fluLUluLUF.LFrFRfrFRFFl.LFUfuFufUFUfl.LfllBllFllbL"
  ".fRUUrUUrFFRf.LFFrfRfl.rFFLFlFR.LUlUlBLbLUUl.RUrurFRRUruf.LFrFRFFL.rfLflFFR.BLUlubUBLUlub.LrFRFrFRFFrFlR.burURB.BULulb.luBULulbL"
  ".RUburURBr.RUrURururFRf.LUlUUbuBuLul.LUUllBLbLUUl.FrfRURur.RurUURUBubur.lULUUlubUBUL.RRUrbRuRRURBr.lBLbLUUlubuB."
  ;

//Table for PLL step possible patterns
const byte pll_t [][12] PROGMEM = {
  {36, 46, 20, 18, 19, 11, 37, 38, 9, 47, 45, 10}, {37, 38, 45, 36, 10, 20, 47, 18, 19, 46, 9, 11}, {45, 10, 20, 37, 47, 18, 38, 46, 9, 36, 19, 11},
  {37, 47, 45, 18, 20, 10, 36, 38, 46, 19, 11, 9}, {47, 45, 19, 18, 20, 46, 36, 38, 10, 37, 9, 11}, {37, 18, 20, 47, 46, 45, 19, 11, 9, 36, 38, 10},
  {18, 20, 10, 47, 46, 45, 37, 11, 9, 36, 38, 19}, {47, 46, 9, 18, 11, 10, 36, 37, 38, 45, 19, 20}, {18, 19, 11, 47, 9, 10, 36, 37, 38, 46, 45, 20},
  {36, 46, 11, 18, 20, 10, 37, 38, 45, 47, 19, 9}, {37, 18, 20, 36, 46, 11, 47, 9, 10, 45, 38, 19}, {47, 46, 9, 18, 19, 11, 36, 38, 10, 37, 45, 20},
  {37, 45, 20, 47, 18, 19, 38, 46, 9, 36, 10, 11}, {45, 10, 20, 36, 37, 38, 47, 19, 9, 18, 46, 11}, {45, 10, 20, 47, 18, 19, 37, 38, 9, 36, 46, 11},
  {47, 46, 18, 45, 19, 20, 36, 11, 10, 37, 38, 9}, {46, 45, 20, 47, 18, 19, 38, 10, 9, 36, 37, 11}, {38, 45, 10, 36, 46, 20, 47, 18, 19, 37, 11, 9},
  {38, 46, 9, 37, 47, 45, 18, 19, 11, 36, 10, 20}, {37, 47, 9, 46, 11, 18, 36, 38, 10, 45, 19, 20}, {38, 46, 9, 47, 45, 10, 37, 18, 11, 36, 19, 20}
  };

//PLL solutions
const char pll_solutions [] PROGMEM =
  "rFrBBRfrBBRR.RRBBRFrBBRfR.rurdRurDRUrdRUrDRR.fuFuFUFufUFUFFufUU.RRllDRRllDDRRLLDRRll.RuRURURuruRR.RRURUrururUr.RUUruRUUlUruL.urUlUURurUURL.URRFRURurfRUUrUUR"
  ".LUUlUULfluLULFllU.RUrurFRRuruRUrf.FRuruRUrfRUrurFRf.urURuRRfuFURFrfRR.rUURUULurUlULuRUl.RuLUUrUlRuLUUrUlu.rURurfuFRUrFrfRuR.RRFFRUURUUrFRUrurFRR"
  ".RRfRURurfRUUrUUrFFRR.LLFFlUUlUULfluLULfll.fuFRRDbUBuBdRR."
  ;

//Variables and counters for solving steps
byte solving_level;
byte stage_solving_step;

//Variables showing the current side colors
char u_color;
char r_color;
char f_color;
char l_color;
char d_color;
char b_color;

//Counter used for different cube operations
byte cubeCounter;

//Global char strings to be printed on LCD
char string1LCD [17];
char string2LCD [17];


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
  DDRB = B00011110;

  //Attaching the motors
  Motor1.attach(rotateMotorPin);
  Motor2.attach(carriageMotorPin);
  Motor3.attach(pusherMotorPin);
  Motor4.attach(scannerMotorPin);
  
  //Starting serial connection
  Serial.begin(9600);
  
  turnMotor1(89, 3);
  //delay(standardDelay);
  turnMotor3(90, 2);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
  turnMotor4(35, 7);
  //delay(standardDelay);

  // Setting frequency-scaling to 20%
  //digitalWrite(S0, HIGH);
  //digitalWrite(S1, LOW);
  PORTB = B00000010;

  //Initialize LCD
  lcd.init();
  lcd.backlight();
  
  MsTimer2::set(2, timerInterupt);
  //MsTimer2::start();    
}

//Main operation method
void loop() {
  
  normalFlow();
  
  //String movementString = "rLUdBBUUllFFllUUllFF";
  //assembleCube(SolvedCube);

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
  else arduinoSolveOperation();
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


//Method for general process of solving the cube on Arduino
void arduinoSolveOperation() {
  operateFlag = true;
  //scanCube();  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  detachInterrupt(0);
  copyCube();
  SolvedCube = "";
  virtualSequence(F("RDURLudbrFuDlUUbRR"));  //"RDURL"
  solveCube();
  assembleCube(SolvedCube);
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
  //if (flagPress != !digitalRead(_pin) ) {
  if (flagPress != !bitRead(PIND, _pin)) {  
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
  //pinMode(_pin, INPUT);  
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
  if (selectedButton == 0) {
    printLCD();
    scanTopSide();
    }
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
  
  //Serial.println(ScannedCube);
  
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
  //delay(200);
  turnMotor4(35, 7);
  //delay(200);
  for (byte i = 0; i<9; i++) {
    turnMotor1(pgm_read_byte(&(Motor_1_ScanAngles[i])), 4);
    turnMotor4(pgm_read_byte(&(Motor_4_ScanAngles[i])), 7);
    //delay(200);
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
  //delay(200);
  turnMotor4(35, 7);
  //delay(200);
  attachInterrupt (0, leftButtonPressed, RISING);
  operateFlag = true;
  for (byte i = 0; i<9; i++) {
    turnMotor1(pgm_read_byte(&(Motor_1_ScanAngles[i])), 4);
    turnMotor4(pgm_read_byte(&(Motor_4_ScanAngles[i])), 7);
    //delay(200);
    SideScanResult[i] = scanResult();
    delay(100);
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
  //digitalWrite(S2,LOW);
  //digitalWrite(S3,LOW);
  PORTB = B00000010;
  // Reading the output frequency
  f_RED = pulseIn(scanSensor, LOW);
  delay(50);
  // Setting Green filtered photodiodes to be read
  //digitalWrite(S2,HIGH);
  //digitalWrite(S3,HIGH);
  PORTB = B00011010;
  // Reading the output frequency
  f_GREEN = pulseIn(scanSensor, LOW);
  delay(50);
  // Setting Blue filtered photodiodes to be read
  //digitalWrite(S2,LOW);
  //digitalWrite(S3,HIGH);
  PORTB = B00010010;
  // Reading the output frequency
  f_BLUE = pulseIn(scanSensor, LOW);
  delay(50);
}


//Function for detecting color (All devices and Arduino connected to power supply)
char defineColor() {
  if(f_RED>=f[0][0] && f_RED<=f[0][1] && f_GREEN>=f[0][2] && f_GREEN<=f[0][3] && f_BLUE>=f[0][4] && f_BLUE<=f[0][5]){
    return 'w';
  }
  else if(f_RED>=f[1][0] && f_RED<=f[1][1] && f_GREEN>=f[1][2] && f_GREEN<=f[1][3] && f_BLUE>=f[1][4] && f_BLUE<=f[1][5]){
    return 'y';
  }
  else if(f_RED>=f[2][0] && f_RED<=f[2][1] && f_GREEN>=f[2][2] && f_GREEN<=f[2][3] && f_BLUE>=f[2][4] && f_BLUE<=f[2][5]){
    return 'o';
  }
  else if(f_RED>=f[3][0] && f_RED<=f[3][1] && f_GREEN>=f[3][2] && f_GREEN<=f[3][3] && f_BLUE>=f[3][4] && f_BLUE<=f[3][5]){
    return 'r';
  }
  else if(f_RED>=f[4][0] && f_RED<=f[4][1] && f_GREEN>=f[4][2] && f_GREEN<=f[4][3] && f_BLUE>=f[4][4] && f_BLUE<=f[4][5]){
    return 'g';
  }
  else if(f_RED>=f[5][0] && f_RED<=f[5][1] && f_GREEN>=f[5][2] && f_GREEN<=f[5][3] && f_BLUE>=f[5][4] && f_BLUE<=f[5][5]){
    return 'b';
  }
  else {
    return 'X';
  }
}


//Function that repeats scanning up to 10 times if the color was not defined
char scanResult(){
  char color;
  byte wrongScan = 0;
  do{
    scan_item();
    //color = defineColor(f_RED, f_GREEN, f_BLUE);
    color = defineColor();
    wrongScan++ ;
    if (color != 'X') { 
      break;
      }
  } while (wrongScan < 10);
  return color;
}

/*
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
  delay(5000);
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
  delay(standardDelay);
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
  delay(standardDelay);
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
  delay(standardDelay);
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
  delay(standardDelay);
}


//Method for pushing the cube
void push() {
  turnMotor2(105, 5);
  //delay(standardDelay);
  turnMotor3(45, 2);
  //delay(standardDelay);
  turnMotor2(118, 5);
  //delay(standardDelay);
  turnMotor3(95, 2);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
  turnMotor3(90, 2);
  //delay(standardDelay);
}


//Method for turning the cube ClockWise
void turnCubeCW() {
  turnMotor2(95, 5);
  //delay(standardDelay);
  turnMotor1(5, 3);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
}


//Method for turning the cube CounterClockWise
void turnCubeCCW() {
  turnMotor2(95, 5);
  //delay(standardDelay);
  turnMotor1(171, 3);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
}


//Method for turning the cube straight
void turnCubeStraight() {
  turnMotor2(95, 5);
  //delay(standardDelay);
  turnMotor1(88, 3);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
}


//Method for turning the bottom layer ClockWise
void downCW() {
  turnCubeCW();
  turnMotor3(45, 2);
  //delay(standardDelay);
  turnMotor2(135, 5);
  //delay(standardDelay);
  turnMotor1(95, 3);
  //delay(standardDelay);
  turnMotor1(88, 3);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
  turnMotor3(90, 2);
  //delay(standardDelay);
}


//Method for turning the bottom layer CounterClockWise
void downCCW() {
  turnCubeCCW();
  turnMotor3(45, 2);
  //delay(standardDelay);
  turnMotor2(135, 5);
  //delay(standardDelay);
  turnMotor1(80, 3);
  //delay(standardDelay);
  turnMotor1(88, 3);
  //delay(standardDelay);
  turnMotor2(105, 5);
  //delay(standardDelay);
  turnMotor3(90, 2);
  //delay(standardDelay);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////METHODS FOR SOLVING THE CUBE ON ARDUINO//////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Method for overwriting the cube copy
void copyCube() {
  for (byte i=0; i<55; i++) {
    CubeCopy[i] = ScannedCube[i];
  }
  Serial.println(CubeCopy);
}


//Methods for virtual rotation of the cube
void virtualSequence(String v_sequence) {
  byte sequenceLength = v_sequence.length();
  for (byte i=0; i<sequenceLength; i++) {
    virtualMove(v_sequence[i]);    
  }
}


void virtualMove(char v_move) {
  byte index;
  for (index = 0; index <13; index++) {
    if (v_move == pgm_read_byte(&(v_map[index]))) {
      break;
    }
  }
  for (cubeCounter = 0; cubeCounter<20; cubeCounter++) {
    ScannedCube[pgm_read_byte(&(v_table[index][cubeCounter]))] = CubeCopy[pgm_read_byte(&(v_table[index][cubeCounter+20]))];
  }
  
  Serial.print(F("Virtual move: "));
  Serial.println(v_move);
  
  addMove(v_move);
  
  copyCube();
  
}


void rotateY() {
  for (byte j = 0; j<20; j++) {
    ScannedCube[pgm_read_byte(&(v_table[0][j]))] = CubeCopy[pgm_read_byte(&(v_table[0][j+20]))];
    ScannedCube[pgm_read_byte(&(v_table[9][j]))] = CubeCopy[pgm_read_byte(&(v_table[9][j+20]))];
    if (j < 12) {
      ScannedCube[pgm_read_byte(&(y_table[j]))] = CubeCopy[pgm_read_byte(&(y_table[j+12]))];
    }
  }
  Serial.println(F("Rotate Y"));
  copyCube();
  defineSideColors();
  if (yTurnCounter < 3) {
    yTurnCounter ++;
    }
  else {
    yTurnCounter = 0;
  }
}


//Method for adding the physical move to the solution string
void addMove(char p_move) {
  if (yTurnCounter == 0) {
    SolvedCube += p_move;
    }
  else {
    if (p_move == 'u' || p_move == 'U' || p_move == 'd' || p_move == 'D') {
      SolvedCube += p_move;
    }
    else {
    byte index;
    for (index = 0; index <8; index++) {
      if (p_move == pgm_read_byte(&(y_map[index]))) {
      break;
      }
    }
    SolvedCube += char(pgm_read_byte(&(y_map[index + 8 * yTurnCounter])));
    Serial.print(F("Physical move: "));
    Serial.println(char(pgm_read_byte(&(y_map[index + 8 * yTurnCounter]))));
    }
  }
}


//General method for solivng the cube
void solveCube() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Solving cube..."));
  defineSideColors();
  solving_level = 1;
  stage_solving_step = 0;
  while(solving_level != 8) {
    solveStage();
  }
  
}

//Method for defining the cube side colors
void defineSideColors() {
  u_color = ScannedCube[4];
  r_color = ScannedCube[13];
  f_color = ScannedCube[22];
  l_color = ScannedCube[40];
  d_color = ScannedCube[31];
  b_color = ScannedCube[49];
}


//Method for solving the cube depending on current solving stage
void solveStage() {
  Serial.print(F("Solving level: "));
  Serial.println(solving_level);
  switch(solving_level)
  {
    case 1:
      solveBottomCross();
      break;

    case 2:
      solveF2L();
      break;

    case 3:
      solveOLL();
      break;

    case 4:
      solvePLL();
      break;

    case 5:
      solveTLL();
      break;

    case 6:
      Serial.println(SolvedCube);
      storeLCD(F("Cube solved"), 0);
      storeLCD(F("successfully!"), 1);
      delay(3000);
      solving_level = 8;
      break;

    case 7:
      storeLCD(F("Solving error"), 0);
      storeLCD(F("Check code"), 1);
      delay(3000);
      solving_level = 8;
      break;
  }
}

//Method for solving the bottom cross (consequently checkes the F, R, B and L sides of the cube)
void solveBottomCross() {
  //Check if the bottom cross is already solved. If no - starts checking the sides.
  if (ScannedCube[28] == d_color && ScannedCube[30] == d_color && ScannedCube[32] == d_color && ScannedCube[34] == d_color &&
      ScannedCube[16] == r_color && ScannedCube[25] == f_color && ScannedCube[43] == l_color && ScannedCube[52] == b_color) {
    Serial.println(F("Bottom cross solved"));
    solving_level = 2;
    stage_solving_step = 0;
  }
  else {
    //Check the front side elements
    if (stage_solving_step < 1) {
          //Check that the front-bottom edge is correct.
          if (ScannedCube[28] == d_color) {
              if (ScannedCube[25] == f_color) {
                  stage_solving_step = 1;
              }
              else {
                  //Check if the front bottom element belongs to the cross but should be located in different place
                  if (ScannedCube[25] == r_color) {
                    virtualMove('D');
                  }
                  else if (ScannedCube[25] == l_color) {
                    virtualMove('d');
                  }
                  else {
                    virtualMove('D');
                    virtualMove('D');
                  }
              }
          }
          
          
          //Check the front bottom edge and flip it if necessary
          else if (ScannedCube[25] == d_color) {
              virtualSequence(F("fDrd"));
          }
      }
      //Check the front-right edge and move it if necessary
      if (ScannedCube[23] == d_color) {
              if (ScannedCube[12] == r_color) {
                  virtualMove('r');
                }
              else {
                  virtualMove('R');
              }
          }
      //Check the front-left edge and move it if necessary    
      if (ScannedCube[21] == d_color) {
              if (ScannedCube[41] == l_color) {
                  virtualMove('L');
                }
              else {
                  virtualMove('l');
              }
          }
      //Check the front-up edge and flips it if necessary  
      if (ScannedCube[19] == d_color) {
              if (ScannedCube[7] == f_color) {
                  virtualSequence(F("RurF"));
                }
              else {
                  virtualMove('u');
              }
          }
      //Check the front-up edge and moves it if necessary
      if (ScannedCube[7] == d_color) {
              if (ScannedCube[19] == f_color) {
                  virtualSequence(F("FF"));
                }
              else if (ScannedCube[19] == r_color) {
                  virtualSequence(F("uRR"));
                }

              else if (ScannedCube[19] == l_color) {
                  virtualSequence(F("ULL"));
                }
                
              else {
                  virtualSequence(F("UURR"));
                }  
          }


    if (stage_solving_step < 2) {
        //Check the right side elements
          if (ScannedCube[32] == d_color) {
              if (ScannedCube[16] == r_color) {
                  stage_solving_step = 2;
              }
              else {
                  if (ScannedCube[16] == b_color) {
                    virtualSequence(F("RRuBB"));
                  }
                  else if (ScannedCube[16] == f_color) {
                    virtualSequence(F("RRUFF"));
                  }
                  else {
                    virtualSequence(F("RRUUll"));
                  }
              }
          }
          
          
          else if (ScannedCube[16] == d_color) {
              virtualSequence(F("rDbd"));
          }
      }
      
      if (ScannedCube[14] == d_color) {
              if (ScannedCube[48] == b_color) {
                  virtualMove('b');
                }
              else {
                  virtualMove('B');
              }
          }
          
      if (ScannedCube[12] == d_color) {
              if (ScannedCube[23] == f_color) {
                  virtualMove('F');
                }
              else {
                  virtualMove('f');
              }
          }
        
      if (ScannedCube[10] == d_color) {
              if (ScannedCube[5] == r_color) {
                  virtualSequence(F("BubR"));
                }
              else {
                  virtualMove('u');
              }
          }

      if (ScannedCube[5] == d_color) {
              if (ScannedCube[10] == r_color) {
                  virtualSequence(F("RR"));
                }
              else if (ScannedCube[10] == b_color) {
                  virtualSequence(F("uBB"));
                }

              else if (ScannedCube[10] == f_color) {
                  virtualSequence(F("UFF"));
                }
                
              else {
                  virtualSequence(F("UUll"));
                }  
          }


    if (stage_solving_step < 3) {
        //Check the back side elements
          if (ScannedCube[34] == d_color) {
              if (ScannedCube[52] == b_color) {
                  stage_solving_step = 3;
              }
              else {
                  if (ScannedCube[52] == l_color) {
                    virtualSequence(F("BBull"));
                  }
                  else if (ScannedCube[52] == r_color) {
                    virtualSequence(F("BBURR"));
                  }
                  else {
                    virtualSequence(F("BBUUFF"));
                  }
              }
          }
          
          
          else if (ScannedCube[52] == d_color) {
              virtualSequence(F("bDld"));
          }
      }
      
      if (ScannedCube[50] == d_color) {
              if (ScannedCube[39] == l_color) {
                  virtualMove('l');
                }
              else {
                  virtualMove('L');
              }
          }
          
      if (ScannedCube[48] == d_color) {
              if (ScannedCube[14] == r_color) {
                  virtualMove('R');
                }
              else {
                  virtualMove('r');
              }
          }
        
      if (ScannedCube[46] == d_color) {
              if (ScannedCube[1] == b_color) {
                  virtualSequence(F("LulB"));
                }
              else {
                  virtualMove('u');
              }
          }

      if (ScannedCube[1] == d_color) {
              if (ScannedCube[46] == b_color) {
                  virtualSequence(F("BB"));
                }
              else if (ScannedCube[46] == l_color) {
                  virtualSequence(F("ull"));
                }

              else if (ScannedCube[46] == r_color) {
                  virtualSequence(F("URR"));
                }
                
              else {
                  virtualSequence(F("UUFF"));
                }  
          }


    if (stage_solving_step < 4) {
        //Check the left side elements
          if (ScannedCube[30] == d_color) {
              if (ScannedCube[43] == l_color) {
                  stage_solving_step = 4;
              }
              else {
                  if (ScannedCube[43] == f_color) {
                    virtualSequence(F("lluFF"));
                  }
                  else if (ScannedCube[43] == b_color) {
                    virtualSequence(F("llUBB"));
                  }
                  else {
                    virtualSequence(F("llUURR"));
                  }
              }
          }
          
          
          else if (ScannedCube[43] == d_color) {
              virtualSequence(F("lDfd"));
          }
      }
      
      if (ScannedCube[41] == d_color) {
              if (ScannedCube[21] == f_color) {
                  virtualMove('f');
                }
              else {
                  virtualMove('F');
              }
          }
          
      if (ScannedCube[39] == d_color) {
              if (ScannedCube[50] == b_color) {
                  virtualMove('B');
                }
              else {
                  virtualMove('b');
              }
          }
        
      if (ScannedCube[37] == d_color) {
              if (ScannedCube[3] == l_color) {
                  virtualSequence(F("FufL"));
                }
              else {
                  virtualMove('u');
              }
          }

      if (ScannedCube[3] == d_color) {
              if (ScannedCube[37] == l_color) {
                  virtualSequence(F("ll"));
                }
              else if (ScannedCube[37] == f_color) {
                  virtualSequence(F("uFF"));
                }

              else if (ScannedCube[37] == b_color) {
                  virtualSequence(F("UBB"));
                }
                
              else {
                  virtualSequence(F("UURR"));
                }  
          }

  }
}

//Method for solving the F2L
void solveF2L() {
   
  boolean F2Lsolved = true;
      //Check the current front-right corner. If it is solved, virtually rotate the cube to Y. If not solved - find a solution.
      if (ScannedCube[29] == d_color && ScannedCube[23] == f_color && ScannedCube[26] == f_color && ScannedCube[12] == r_color && ScannedCube[15] == r_color) {
        stage_solving_step ++;
        rotateY();
        Serial.print(F("F2L step solved: "));
        Serial.println(stage_solving_step);
      }
      
      else {
         F2Lsolved = false;
         prepareToF2LSolving();
         defineF2Lsequence();
        
      }

    if (F2Lsolved == true && stage_solving_step >= 4) {
        solving_level = 3;
        stage_solving_step = 0;
        Serial.println(F("F2L solved"));
    }
  
}

void defineF2Lsequence() {
  byte index;
  //Counter for controlling the number of u-moves
  byte uTurnCount = 0;
  //Flag showing that solution was found
  boolean f2lSequenceFound = false;
  //Counter for finding the exact solution in solutions char string
  byte f2lPointCounter = 0;
  //Search for a solution. If solution was not found - make the u-turn
  while (f2lSequenceFound == false && uTurnCount<4) {
      for (index = 0; index <41; index++) {
       if (ScannedCube[pgm_read_byte(&(f2l_t[index][0]))] == f_color && ScannedCube[pgm_read_byte(&(f2l_t[index][1]))] == f_color && 
            ScannedCube[pgm_read_byte(&(f2l_t[index][2]))] == r_color && ScannedCube[pgm_read_byte(&(f2l_t[index][3]))] == r_color && 
            ScannedCube[pgm_read_byte(&(f2l_t[index][4]))] == d_color) {
              f2lSequenceFound = true;
              break;
            }
        }
      
      if (f2lSequenceFound == false) {
        virtualMove('u');
        uTurnCount ++;
      }
  }

  Serial.print(F("F2L sequence #: "));
  Serial.println(index);
  
  //Find a solution string in solutions char array
  if (index < 40 && uTurnCount < 4) {
      for (int k = 0; k<500; k++) { 
          //Find the current dot separator number
          if (pgm_read_byte(&(f2l_solutions[k])) == '.') {
            f2lPointCounter ++;
            }
          //Perform the move if the needed solution was obtained
          if (f2lPointCounter == index && pgm_read_byte(&(f2l_solutions[k])) != '.') {
            virtualMove(pgm_read_byte(&(f2l_solutions[k])));
            }
          //Stop as soon as the ending dot was found
          else if (f2lPointCounter > index) {
            break;
            }
          }
       }   
  else {
     Serial.println(F("Error in F2L"));
     solving_level = 7;
  }

}

//Method for checking the front-left, back-left and back-right corners. If the needed items are found there, move them to the upper level.
void prepareToF2LSolving() {
  boolean preparationF2LReady;
  do {
    preparationF2LReady = true;
    if ((ScannedCube[21] == f_color && ScannedCube[41] == r_color) || (ScannedCube[21] == r_color && ScannedCube[41] == f_color) ||
        (ScannedCube[44] == f_color && ScannedCube[24] == r_color && ScannedCube[27] == d_color) ||
        (ScannedCube[44] == f_color && ScannedCube[24] == d_color && ScannedCube[27] == r_color) ||
        (ScannedCube[44] == r_color && ScannedCube[24] == d_color && ScannedCube[27] == f_color) ||
        (ScannedCube[44] == r_color && ScannedCube[24] == f_color && ScannedCube[27] == d_color) ||
        (ScannedCube[44] == d_color && ScannedCube[24] == f_color && ScannedCube[27] == r_color) ||
        (ScannedCube[44] == d_color && ScannedCube[24] == r_color && ScannedCube[27] == f_color)) {
            preparationF2LReady = false;
            virtualSequence(F("Fuf"));
     }
        
    if ((ScannedCube[39] == f_color && ScannedCube[50] == r_color) || (ScannedCube[39] == r_color && ScannedCube[50] == f_color) ||
        (ScannedCube[42] == f_color && ScannedCube[53] == r_color && ScannedCube[33] == d_color) ||
        (ScannedCube[42] == f_color && ScannedCube[53] == d_color && ScannedCube[33] == r_color) ||
        (ScannedCube[42] == r_color && ScannedCube[53] == d_color && ScannedCube[33] == f_color) ||
        (ScannedCube[42] == r_color && ScannedCube[53] == f_color && ScannedCube[33] == d_color) ||
        (ScannedCube[42] == d_color && ScannedCube[53] == f_color && ScannedCube[33] == r_color) ||
        (ScannedCube[42] == d_color && ScannedCube[53] == r_color && ScannedCube[33] == f_color)) {
            preparationF2LReady = false;
            virtualSequence(F("buB"));
     }
  
    if ((ScannedCube[14] == f_color && ScannedCube[48] == r_color) || (ScannedCube[14] == r_color && ScannedCube[48] == f_color) ||
        (ScannedCube[17] == f_color && ScannedCube[51] == r_color && ScannedCube[35] == d_color) ||
        (ScannedCube[17] == f_color && ScannedCube[51] == d_color && ScannedCube[35] == r_color) ||
        (ScannedCube[17] == r_color && ScannedCube[51] == d_color && ScannedCube[35] == f_color) ||
        (ScannedCube[17] == r_color && ScannedCube[51] == f_color && ScannedCube[35] == d_color) ||
        (ScannedCube[17] == d_color && ScannedCube[51] == f_color && ScannedCube[35] == r_color) ||
        (ScannedCube[17] == d_color && ScannedCube[51] == r_color && ScannedCube[35] == f_color)) {
            preparationF2LReady = false;
            virtualSequence(F("Bub"));
     }
  } while (preparationF2LReady == false);
}


//Method for solving the OLL
void solveOLL() {
   //Check that last layer is already solved. If it is solved, complete the step. If not solved - find a solution.
  if (ScannedCube[0] == u_color && ScannedCube[1] == u_color && ScannedCube[2] == u_color && ScannedCube[3] == u_color && ScannedCube[5] == u_color
       && ScannedCube[6] == u_color && ScannedCube[7] == u_color && ScannedCube[8] == u_color) {
        solving_level = 4;
        Serial.println(F("OLL solved"));
   }
      
   else {
       defineOLLsequence(); 
   }
}

//Method for defining the OLL solution sequence
void defineOLLsequence() {
  byte index;
  //Counter for controlling the number of u-moves
  byte uTurnCount = 0;
  //Flag showing that solution was found
  boolean ollSequenceFound = false;
  //Counter for finding the exact solution in solutions char string
  byte ollPointCounter = 0;
  //Search for a solution. If solution was not found - make the u-turn
  while (ollSequenceFound == false && uTurnCount<4) {
     for (index = 0; index <57; index++) {
       if (ScannedCube[pgm_read_byte(&(oll_t[index][0]))] == u_color && ScannedCube[pgm_read_byte(&(oll_t[index][1]))] == u_color && 
           ScannedCube[pgm_read_byte(&(oll_t[index][2]))] == u_color && ScannedCube[pgm_read_byte(&(oll_t[index][3]))] == u_color && 
           ScannedCube[pgm_read_byte(&(oll_t[index][4]))] == u_color && ScannedCube[pgm_read_byte(&(oll_t[index][5]))] == u_color &&
           ScannedCube[pgm_read_byte(&(oll_t[index][6]))] == u_color && ScannedCube[pgm_read_byte(&(oll_t[index][7]))] == u_color) {
              ollSequenceFound = true;
              break;
            }
        }
      
      if (ollSequenceFound == false) {
        virtualMove('u');
        uTurnCount ++;
      }
  }

  Serial.print(F("OLL sequence #: "));
  Serial.println(index);
  
  //Find a solution string in solutions char array
  if (index < 57 && uTurnCount < 4) {
      for (int k = 0; k<1000; k++) { 
          //Find the current dot separator number
          if (pgm_read_byte(&(oll_solutions[k])) == '.') {
            ollPointCounter ++;
            }
          //Perform the move if the needed solution was obtained
          if (ollPointCounter == index && pgm_read_byte(&(oll_solutions[k])) != '.') {
            virtualMove(pgm_read_byte(&(oll_solutions[k])));
            }
          //Stop as soon as the ending dot was found
          else if (ollPointCounter > index) {
            break;
            }
          }
       }   
  else {
     Serial.println(F("Error in OLL"));
     solving_level = 7;
  }

}


//Method for solving the PLL
void solvePLL() {
   //Check that last layer permutations are already solved. If it is solved, complete the step. If not solved - find a solution.
  if (ScannedCube[36] == ScannedCube[37] && ScannedCube[36] == ScannedCube[38] && ScannedCube[37] == ScannedCube[38] &&
      ScannedCube[47] == ScannedCube[46] && ScannedCube[47] == ScannedCube[45] && ScannedCube[46] == ScannedCube[45] &&
      ScannedCube[11] == ScannedCube[10] && ScannedCube[11] == ScannedCube[9] && ScannedCube[10] == ScannedCube[9] &&
      ScannedCube[18] == ScannedCube[19] && ScannedCube[18] == ScannedCube[20] && ScannedCube[19] == ScannedCube[20]) {
        solving_level = 5;
        Serial.println(F("PLL solved"));
   }
      
   else {
       definePLLsequence(); 
   }
}

//Method for defining the PLL solution sequence
void definePLLsequence() {
  byte index;
  //Counter for controlling the number of u-moves
  byte uTurnCount = 0;
  //Flag showing that solution was found
  boolean pllSequenceFound = false;
  //Counter for finding the exact solution in solutions char string
  byte pllPointCounter = 0;
  //Search for a solution. If solution was not found - make the u-turn
  while (pllSequenceFound == false && uTurnCount<4) {
     for (index = 0; index <21; index++) {
       if (ScannedCube[pgm_read_byte(&(pll_t[index][0]))] == ScannedCube[pgm_read_byte(&(pll_t[index][1]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][0]))] == ScannedCube[pgm_read_byte(&(pll_t[index][2]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][1]))] == ScannedCube[pgm_read_byte(&(pll_t[index][2]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][3]))] == ScannedCube[pgm_read_byte(&(pll_t[index][4]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][3]))] == ScannedCube[pgm_read_byte(&(pll_t[index][5]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][4]))] == ScannedCube[pgm_read_byte(&(pll_t[index][5]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][6]))] == ScannedCube[pgm_read_byte(&(pll_t[index][7]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][6]))] == ScannedCube[pgm_read_byte(&(pll_t[index][8]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][7]))] == ScannedCube[pgm_read_byte(&(pll_t[index][8]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][9]))] == ScannedCube[pgm_read_byte(&(pll_t[index][10]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][9]))] == ScannedCube[pgm_read_byte(&(pll_t[index][11]))] &&
           ScannedCube[pgm_read_byte(&(pll_t[index][10]))] == ScannedCube[pgm_read_byte(&(pll_t[index][11]))]) {
              pllSequenceFound = true;
              break;
            }
        }
      
      if (pllSequenceFound == false) {
        virtualMove('u');
        uTurnCount ++;
      }
  }

  Serial.print(F("PLL sequence #: "));
  Serial.println(index);
  
  //Find a solution string in solutions char array
  if (index < 21 && uTurnCount < 4) {
      for (int k = 0; k<500; k++) { 
          //Find the current dot separator number
          if (pgm_read_byte(&(pll_solutions[k])) == '.') {
            pllPointCounter ++;
            }
          //Perform the move if the needed solution was obtained
          if (pllPointCounter == index && pgm_read_byte(&(pll_solutions[k])) != '.') {
            virtualMove(pgm_read_byte(&(pll_solutions[k])));
            }
          //Stop as soon as the ending dot was found
          else if (pllPointCounter > index) {
            break;
            }
          }
       }   
  else {
     Serial.println(F("Error in PLL"));
     solving_level = 7;
  }

}


void solveTLL() {
  //Check if the upper level is rotated correctly. If not - turn it.
  if (ScannedCube[10] == r_color && ScannedCube[19] == f_color && ScannedCube[37] == l_color && ScannedCube[46] == b_color) {
    Serial.println(F("Last level rotated"));
    solving_level = 6;
    }
  else {
    if (ScannedCube[19] == r_color) {
      virtualMove('u');
    }
    else if (ScannedCube[19] == l_color) {
      virtualMove('U');
    }
    else if (ScannedCube[19] == b_color) {
      virtualMove('U');
      virtualMove('U');
    }
    else {
      Serial.println(F("Error in TLL"));
      solving_level = 7;
    }
  }

}

