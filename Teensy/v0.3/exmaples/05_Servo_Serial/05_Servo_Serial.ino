
//#include <Wire.h>
#include <i2c_driver_wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x7F); // address of U3
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x7E); // address of U4

// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);


#define SERVOMIN  180 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  440 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

int16_t servo_value = 0;
int16_t prev_servo_value;
uint16_t value;

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm1.begin();
  /*
     In theory the internal oscillator (clock) is 25MHz but it really isn't
     that precise. You can 'calibrate' this by tweaking this number until
     you get the PWM update frequency you're expecting!
     The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
     is used for calculating things like writeMicroseconds()
     Analog servos run at ~50 Hz updates, It is importaint to use an
     oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
     1) Attach the oscilloscope to one of the PWM signal pins and ground on
        the I2C PCA9685 chip you are setting the value for.
     2) Adjust setOscillatorFrequency() until the PWM update frequency is the
        expected value (50Hz for most ESCs)
     Setting the value here is specific to each individual I2C PCA9685 chip and
     affects the calculations for the PWM update frequency.
     Failure to correctly set the int.osc value will cause unexpected PWM results
  */
  pwm.setOscillatorFrequency(27000000);
  pwm1.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  prev_servo_value = servo_value;
  value = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN)/100);
  // pwm.setPWM(servonum, 0, value);
  pwm1.setPWM(servonum, 0, value);
  Serial.println(servo_value);
  delay(10);
}

void loop() {
  // Drive each servo one at a time using setPWM()
  serialEvent();
  if(servo_value != prev_servo_value){
    prev_servo_value = servo_value;
    value = ((SERVOMAX - SERVOMIN)/2 + SERVOMIN) + (servo_value * (SERVOMAX - SERVOMIN)/100);
    // pwm.setPWM(servonum, 0, value);
    pwm1.setPWM(servonum, 0, value);
    Serial.println("servo move");
  }
  

}

void serialEvent() {
  char serial_byte = Serial.read();

  switch (serial_byte){
    case 'k':
      servo_value++;
      Serial.println("servo_value: ");
      Serial.println(servo_value);
      break;

    case 'j':
      servo_value--;
      Serial.println("servo_value: ");
      Serial.println(servo_value);
      break;
      
    default:
      break;
  }
}
