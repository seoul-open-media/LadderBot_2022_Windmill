/*   12
      added thetaFilter

     11

     Autonomous mode
     turn by push and pull changing servo angle

     10

     Changed brakeActuatorServo based on basic test ( why are the pins of brake all messed up?)
     But it moves well.

    09

    previous_function, function not to send frequent unnecessary messages to servo
    Do not use D0 since it's making all the problem


    08
   Brake 3, 4 coulping
   D10 - > servo pwm
   servo_mode : 0 -> straight, 1 -> left, 2 -> right
   brake, turn code edit
   move_backward delete



   06
     Sensor Averaging
     Remote Control from PD
     destination / 10

  https://docs.google.com/document/d/19Vv85BePFuLO48Qn77FReCDTMKj7yLB8XD9vWzlmFKs/edit?usp=sharing

  D0 -> Brake 1
  D1 -> Brake 2
  D6 -> Actuator control1
  D7 -> Actuator control2 ( ATmega, PD7(AIN1))
  D9 -> Brake 3
  D10 -> Brake 4

  I2C : Oled screen, Gyro Sensor

  SPI : DWM1000
  D2 -> IRQ
  D3 -> RST
  D4 -> EXTON
  D5 ->  WAKEUP


  BRAKE1    BRAKE2
  LPS1


  BRAKE3 =  BRAKE4
  LPS2


  theta = arccos((pow(P12) + pow(P13) - pow(P23)) / (2 * P12 * P13))
  deg = rad * 57296 / 1000


*/

#include <Chrono.h>
#include <Ewma.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <Metro.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoKey_1x4.h>
#include <seesaw_neopixel.h>
#include <Bounce2.h>

int pos = 0; // variable to store the servo position

#define VERSION 0.3
// #define SOFT_VERSION 12
#define I2C_ADDRESS 0x3C
#define RST_PIN -1

#define NUM_SCENE 10

// serial message flag
#define SET_DEFAULT_VALUE 0
#define SEND_SONG_INFO 1
#define FINAL_RESULT 2
#define EXCHANGE_FINISH 3
#define SEND_POSITION 4

#define HWSERIAL Serial1
#define r_lenData 30

//======= CALIBRATION VALUES ==============
// rc receiver
#define CH2_MAX 150
#define CH3_MAX 100

#define SENSOR_MAX_LOW 30
#define SENSOR_MAX_HIGH 65
#define MIDDLE_VALUE 50
#define OFFSET 10
#define num_of_channel 6

// values for ch5
#define MODE_HIGH_MIN 60
#define MODE_HIGH_MAX 62
#define MODE_MID_MIN 28
#define MODE_MID_MAX 45
#define MODE_LOW_MIN 28
#define MODE_LOW_MAX 30

// values for ch6
#define DOWN_MIN 45
#define DOWN_MAX 55
#define UP_MIN 30
#define UP_MAX 55

// servo_value
#define FRONT_MIN -50
#define FRONT_MAX 50
#define REAR_MIN -40
#define REAR_MAX 50

// servo direction
#define FRONT_STRAIGHT -12
#define REAR_STRAIGHT 15
#define FRONT_LEFT -50
#define REAR_LEFT 50
#define FRONT_RIGHT 30
#define REAR_RIGHT -23
//==========================================

//<<<<<<< Updated upstream
#define SERVOMIN 255  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 440  // This is the 'maximum' pulse length count (out of 4096)
//=======
//#define SERVOMIN 255d  // 180// This is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX 440  // 440 This is the 'maximum' pulse length count (out of 4096)
//>>>>>>> Stashed changes
#define USMIN 600     // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400    // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

#define M1_PWM_PIN 2
// #define M1_A_PIN 3
// #define M1_B_PIN 4

#define M2_PWM_PIN 5
#define M2_A_PIN 6
#define M2_B_PIN 7

#define M3_PWM_PIN 8
#define M3_A_PIN 9
#define M3_B_PIN 22

#define M4_PWM_PIN 23
#define M4_A_PIN 26
#define M4_B_PIN 27

#define ACTUATOR_CONTROL_PIN1 3
#define ACTUATOR_CONTROL_PIN2 4

#define SERVO_1_PIN_ 0
#define SERVO_2_PIN_ 1

#define BRAKE_1_PIN 28
#define BRAKE_2_PIN 29
#define BRAKE_3_PIN 32
#define BRAKE_4_PIN 33

#define MOVE_THRESHOLD 0.5 // meter
#define THETA_THRESHOLD 15 // degree

// 45도기준 범위각 5도 : α=45˚, θ=2.5˚
//#define X_THRESHOLD 0.0302 // cosα - cos(α - θ)
//#define Y_THRESHOLD 0.0315 // sinα - sin(α - θ)

#define X_THRESHOLD 0.2
#define Y_THRESHOLD 0.2

#define MY_ADDRESS 255
#define LPS1_ADDRESS 1
#define LPS2_ADDRESS 2
#define ADAPTOR_ADDRESS 200

int16_t actuatorPwm_ = 255;

byte sensor_min = 30;  ///////////////////////////////Need to be calibrated
byte sensor_max = 55; /////////////////////////////////

uint8_t STRIGHT_MIN = 28;
uint8_t STRIGHT_MAX = 45;

////////////////// mode
#define MODE_AUTONOMOUS 0
#define MODE_BASIC_TEST 1
#define MODE_BASIC_MOVEMENT 2

////////////////// state
#define STATE_INIT 0
#define STATE_TURN_DECISION 1
#define STATE_STANDUP_BEFORE_TURN 2
#define STATE_CHECK_ANGLE 3
#define STATE_PUSH_TURN 4
#define STATE_PULL_TURN 5
#define STATE_CHECK_DESTINATION 6
#define STATE_CHECK_THETA 7
#define STATE_PUSH_FORWARD 8
#define STATE_PULL_FORWARD 9
#define STATE_FINISH 10

////////////////// rc_state
#define RCSTATE_INIT 0
#define RCSTATE_STANDUP 1
#define RCSTATE_PUSHFORWARD 2
#define RCSTATE_PULLFORWARD 3
#define RCSTATE_PUSH_TURN 4
#define RCSTATE_PULL_TURN 5

////////////////// a_state
#define STATE_GOING_DOWN 0
#define STATE_GOING_UP 1

////////////////// turn_type
#define TURN_RIGHT 0
#define TURN_LEFT 1

////////////////// servo_mode
#define SERVO_STRAIGHT 0
#define SERVO_LEFT 1
#define SERVO_RIGHT 2
#define SERVO_LEFT_VALUE 500
#define SERVO_STRAIGHT_VALUE 830
#define SERVO_RIGHT_VALUE 1200

////////////////// complete_message
#define INCOMPLETE 0
#define COMPLETE 1

////////////////// toRobot_message
#define BOW 1
#define RockIsland 2
#define RobotMaster 3

////////////////// movement_type
//#define MOVE_FORWARD 1
//#define MOVE_BACKWARD 2

//////////////////

// Neokey
#define PLAY 0
#define STOP 1
#define ROBOT 2
#define CHARGE 3

///////////////// LPS
#define d 4.6    // distance bewteen Anchor1 and Anchor2
#define p3_i 2.6 // x cordinate of Anchor3
#define p3_j 6.8 // y cordinate of Anchor3

byte r_data[r_lenData];

////////////////////////////////////////////////////////////////IMU
#define IMU_V5

int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; // Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256 // this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252) // *pi/180
#define ToDeg(x) ((x)*57.2957795131) // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07                          // X axis Gyro gain
#define Gyro_Gain_Y 0.07                          // Y axis Gyro gain
#define Gyro_Gain_Z 0.07                          // Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) // Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) // Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) // Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
// OUTPUTMODE=1 will print the corrected data,
// OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     // Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 // Will print the analog raw data
#define PRINT_EULER 1   // Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

#define DEBUG false
#define SBUF_SIZE 64
// info
// #define MY_ADDRESS 1
#define RCB_VERSION 0.1
#define SOFT_VERSION 0.1

#define pi 3.1415926536
#define radTo14 2607.594587617613379
#define oneTo14 8191
#define qCalAddr 0    // EEPROM qCal address
#define qCalAddr1 100 // EEPROM qCal address1

float G_Dt = 0.02; // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; // general purpuse timer
long timer_old;
long timer24 = 0;                      // Second timer used to print values
int AN[6];                             // array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; // Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
// int magnetom_x;
// int magnetom_y;
// int magnetom_z;
// float c_magnetom_x;
// float c_magnetom_y;
// float c_magnetom_z;
// float MAG_Heading;
float Accel_Vector[3] = {0, 0, 0};
float Gyro_Vector[3] = {0, 0, 0};  // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0};      // Omega Proportional correction
float Omega_I[3] = {0, 0, 0};      // Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter1 = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
  {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; // Gyros here

float Temporary_Matrix[3][3] = {
  {0, 0, 0}, {0, 0, 0}, {0, 0, 0}
};

///////////////////////////////////////////////////////// End of IMU

///////////////////////////////////////////////////////// Other Global Variables
float sensor_value = 0;
boolean reached_destination, reached_boundary, serial_flag, sentBow, reached_scene, isScenePlaying, toCharge, isSentRobot, isSceneStop = false;
boolean passedScene[NUM_SCENE], completeScene[NUM_SCENE];
byte mode, servo_mode, previous_servo_mode, function, previous_function, state, a_state, rc_state, movement_type, turn_type, complete, state_seq, state_charge = 0;

// 1
float destination_x = 0;
float destination_y = 0;

// 2
float LPS1_x = 0;
float LPS1_y = 0;

// 3
float LPS2_x = 0;
float LPS2_y = 1.0;
float theta_rad, theta_deg_filtered, xrange_filtered, yrange_filtered, P12, P13, P14, P23, my_coordinate_x, my_coordinate_y, distance, ax_norm, ay_norm, bx_norm, by_norm = 0;
uint32_t theta_deg, xrange, yrange = 0;

byte received_distance_result[2][3]; // received distance data from LPS1, 2
byte received_elapsed_time[2];
float coordinates_LPS[2][2]; // to store xy coordinates of LPS1, 2

// EBIMU
float euler_EBIMU[3];
char sbuf[SBUF_SIZE];
signed int sbuf_cnt = 0;

// Serial
volatile bool serialState = 0;
int action = 0;
int serToAct = 0;

// Display
#define SCREEN_ADDRESS  0x3C
#define OLED_RESET      -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH    128     // OLED display width, in pixels
#define SCREEN_HEIGHT   64      // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

// Metro
Metro ebimuMetro = Metro(100);
Metro displayMetro = Metro(1000);
Metro actuatorVelMetro = Metro(50);

// PWM
Adafruit_PWMServoDriver pwm_ = Adafruit_PWMServoDriver(0x7F); // address of U3
Adafruit_PWMServoDriver pwm1_ = Adafruit_PWMServoDriver(0x7E); // address of U4
///////////////////////////////////////////////////////// create instances

Chrono timer1, timer2, timer3, timer4, timer5, timer6, sendRockTimer, rockTimer, WaitrockTimer, anchor1ErrorTimer, anchor2ErrorTimer, anchor3ErrorTimer, lpsTag1Timer, lpsTag2Timer;
Chrono sceneTimer[NUM_SCENE];
Ewma thetaFilter(0.02); // 0.1Less smoothing - faster to detect changes, but more prone to noise, 0.01More smoothing - less prone to noise, but slower to detect change
// Ewma distanceFilter(0.01);

// Neokey Instance
Adafruit_NeoKey_1x4 neokey;

/////////////////////////////////////////////////////////
//-----------neoKey-----------------------------
boolean keyPressed[4] = {false, false, false, false};
boolean lastKeyState[4] = {false, false, false, false};
uint8_t neoKeyNum = 0;
bool key_A_predicate()
{
  return neokey.read() & 0x01;
}

bool key_B_predicate()
{
  return neokey.read() & 0x02;
}

bool key_C_predicate()
{
  return neokey.read() & 0x04;
}

bool key_D_predicate()
{
  return neokey.read() & 0x08;
}

// instantiate the PrediateDebouncer objects
PredicateDebouncer debouncers[4] = { PredicateDebouncer(&key_A_predicate, 5),
                                     PredicateDebouncer(&key_B_predicate, 5),
                                     PredicateDebouncer(&key_C_predicate, 5),
                                     PredicateDebouncer(&key_D_predicate, 5)
                                   };

// set a variable to store the led states
int led_states[4] = {LOW, LOW, LOW, LOW};
int neoKey_num, entry_counter = 0;
bool neoKey_enter, neoKey_exit = false;


// RC Receiver
bool is_connected = false;
bool serialDebug = true;

const unsigned short connnection_timeout = 940; // in us

char ch_pinNum[num_of_channel] = {39, 40, 41, 42, 43, 44};
unsigned short ch_high_time[num_of_channel] = {0, 0, 0, 0, 0, 0};
unsigned short ch_max[num_of_channel] = {1750, 1750, 2000, 1750, 2000, 2000};
unsigned short ch_min[num_of_channel] = {1250, 1250, 1000, 1250, 1000, 1000};
int ch_value[num_of_channel] = {0, 0, 0, 0, 0, 0};

//////////////////////////////////////////////////////////////// SETUP
void setup()
{
  HWSERIAL.begin(58824);
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1000);
  while (!HWSERIAL)
    ;

  // display
  Wire2.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      ;
  }

  delay(500);

  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font

  delay(500);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("RobotCtrlBoard v.");
  display.println(VERSION);
  display.println();
  display.print("Software v.");
  display.println(SOFT_VERSION);
  display.println();
  display.display();

  EBimuCommand("<sor0>"); // polling mode
  delay(100);
  while (!Serial2.available())
  {
    // we didn't get the response
    Serial.print("EBIMU not detected");
    displayError03();
    while (1)
      ;
  }

  // 4ch brake pinMode
  pinMode(BRAKE_1_PIN, OUTPUT);
  pinMode(BRAKE_2_PIN, OUTPUT);
  pinMode(BRAKE_3_PIN, OUTPUT);
  pinMode(BRAKE_4_PIN, OUTPUT);
  
  //rc receiver pinMode
  for (int ch = 0; ch < num_of_channel; ch++) {
    pinMode(ch_pinNum[ch], INPUT);
  }
  // Accel_Init();
  // Gyro_Init();
  // // Serial.println("Gyro Initialized");
  // delay(20);

  // for (int i = 0; i < 32; i++) // We take some readings...
  // {
  //   Read_Gyro();
  //   Read_Accel();
  //   for (int y = 0; y < 6; y++) // Cumulate values
  //     AN_OFFSET[y] += AN[y];
  //   delay(20);
  // }

  pinMode(ACTUATOR_CONTROL_PIN1, OUTPUT);
  pinMode(ACTUATOR_CONTROL_PIN2, OUTPUT);
  pinMode(M3_A_PIN, OUTPUT);
  pinMode(M4_A_PIN, OUTPUT);
  pinMode(M3_B_PIN, OUTPUT);
  pinMode(M4_B_PIN, OUTPUT);
  pwm_.begin();
  pwm1_.begin();
  // pwm_.setOscillatorFrequency(27000000);
  // pwm1_.setOscillatorFrequency(27000000);
  pwm_.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pwm1_.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  // delay(10);

  digitalWrite(ACTUATOR_CONTROL_PIN1, LOW);
  digitalWrite(ACTUATOR_CONTROL_PIN2, LOW);

#if RST_PIN >= 0
  // oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else  // RST_PIN >= 0
  // oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  display.clearDisplay();
  display.print("LADDERBOT v. ");
  display.println(VERSION);
  display.println();
  display.print("Software v. ");
  display.println(SOFT_VERSION);
  display.println();
  display.println("Teensy");
  display.println();
  display.print("MY_ADDRESS : ");
  display.println(MY_ADDRESS);
  display.display();
  delay(1000);

  display.clearDisplay();

  uint16_t front_value = ((SERVOMAX - SERVOMIN) / 2 + SERVOMIN) + (-12 * (SERVOMAX - SERVOMIN) / 100);
  uint16_t rear_value = ((SERVOMAX - SERVOMIN) / 2 + SERVOMIN) + (10 * (SERVOMAX - SERVOMIN) / 100);
  pwm_.setPWM(SERVO_1_PIN_, 0, front_value);
  pwm_.setPWM(SERVO_2_PIN_, 0, rear_value);

  pullActuator(); //////////////// Iintial State is Standing Up
  brakeAllRelease();
  //   delay(3000);

  //   /////////////////////////////// Initialize
  //   mode = MODE_BASIC_TEST;
  //  // mode = MODE_AUTONOMOUS;
  //  // mode = MODE_BASIC_MOVEMENT;
  //   function = 5;
  //   servoControl(SERVO_STRAIGHT);

  state = STATE_INIT;
  a_state = STATE_GOING_DOWN;
//   brakeAll();
//   brake12();
//   brake34();


  for (int i; i < NUM_SCENE; i++) {
    passedScene[i] = false;
    completeScene[i] = false;
  }

  timer = millis();
  delay(20);
  counter1 = 0;
//  Serial.print("1");

//   // if we find neokey, go into configuration mode
//  if (neokey.begin(0x30)) {     // begin with I2C address, default is 0x30
//    // enter configuration mode
//    Serial.println("NeoKey started!");
//  }
//    Serial.print("2");

  Serial.println("End of Setup");
  // step1();

}
/////////////////////////////////////////////////////////////////// End of SETUP

void loop()
{
  // neoKey();
  
  // if(isScenePlaying) scene();

  // if(toCharge) sceneCharge();

  // if(isSceneStop) sceneStop();

//  callLPSGetDataCalcCoord(); // Call LPS, get distance data and calculate coordinates and theta
  
//  if(is_connected) rcControl();

  if(is_connected) rcControlA();
  

  if (ebimuMetro.check() == 1)
    EBimuCommand("*");

  if (EBimuAsciiParser(euler_EBIMU, 3))
  {
    sensor_value = euler_EBIMU[1];
   // Serial.print(euler_EBIMU[1]); Serial.print(", "); Serial.println(euler_EBIMU[2]);
    sensor_value = map(sensor_value, 0, 70, 70, 0);
    Serial.println(sensor_value);
    is_connected = readPwm();
  
  
  }

  // getHWSerial();

  if (displayMetro.check() == 1) {
    displaySensorY();
//    print_debug();
    // sendPosition();
    //  serialDebugMessage01();
  }

//  if (mode == MODE_AUTONOMOUS)
//    modeAutonomous(); // 0
//  if (mode == MODE_BASIC_TEST)
//    modeBasicTest(); // 1
//  if (mode == MODE_BASIC_MOVEMENT)
//    modeBasicMovement(); // 2

}
void serialDebugMessage01() {
  Serial.print("EBIMU: "); Serial.println(sensor_value);

  Serial.print("state_seq: "); Serial.println(state_seq);
  Serial.print("state: "); Serial.println(state);

  Serial.print("x: "); Serial.print(my_coordinate_x); Serial.print(", ");
  Serial.print("y: "); Serial.println(my_coordinate_y);
  Serial.print("dest x: "); Serial.print(destination_x); Serial.print(", ");
  Serial.print("y: "); Serial.println(destination_y);

  Serial.print("distance: "); Serial.println(distance);

  Serial.print("reached: "); Serial.println(reached_destination);
  Serial.print("completeScene: ");
  for (int i = 0; i < NUM_SCENE; i++) {
    Serial.print(completeScene[i]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println();


}
