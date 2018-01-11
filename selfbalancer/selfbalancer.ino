
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

class Motor {
  public:
  void setup() {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
  }
  /**
   * \param speed: [-max speed, max speed]
   */
  void move(float speed) {
    int pwm_value = getPmwValue(speed);
        
   
    if (speed == 0) {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    } else if (speed > 0) {
      digitalWrite(pin2, HIGH);
      digitalWrite(pin1, LOW);
      analogWrite(pin_pwm, pwm_value);
    } else {
      digitalWrite(pin2, LOW);
      digitalWrite(pin1, HIGH);
      analogWrite(pin_pwm, pwm_value);
    }
  }
  private:
  int getPmwValue(float speed) {
    float abs_speed = abs(speed);
    if (abs_speed < 0.3) return 0;
    float pwm_jump = (255 - min_pwm_value_) / (max_speed_value - 1);
    float pwm = (abs_speed - 1) * pwm_jump + min_pwm_value_;
    if (pwm < min_pwm_value_)
      pwm = min_pwm_value_;
    if (pwm > 255) 
      pwm = 255;
    return pwm;
  }
  private:
  int pin1 = 7;
  int pin2 = 8;
  int pin_pwm = 9;
  int min_pwm_value_ = 135;  // this is speed 1
  float max_speed_value = 5;  // this will map with pwm 255
  
};

class PidController {
 public:
  float computeControl(float measure) {
    float error = balance_ - measure;
    float p_term = kp_ * error;
    integrated_error_ += error;
    float i_term = ki_ * constrain(integrated_error_, -10, 10);
    float d_term = kd_ * (error - last_error_);
    last_error_ = error;
    float control = p_term + i_term + d_term;
    return control;
  }
 private:
  float integrated_error_ = 0.0;
  float last_error_ = 0.0;
  float balance_ = 0.0;
  float kp_ = 2.0;
  float ki_ = 0.01;
  float kd_ = 0.5;
};

Motor motor;
PidController pid_controller;
MPU6050 mpu;
long initial_time;

void setup() {
  motor.setup();
  initial_time = millis();

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  Serial.begin(38400);
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  delay(1000);

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
/*  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
*/
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
     // turn on the DMP, now that it's ready
     Serial.println(F("Enabling DMP..."));
     mpu.setDMPEnabled(true);

     // enable Arduino interrupt detection
     Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
     mpuIntStatus = mpu.getIntStatus();

     // set our DMP Ready flag so the main loop() function knows it's okay to use it
     Serial.println(F("DMP ready! Waiting for first interrupt..."));
     dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
     // ERROR!
     // 1 = initial memory load failed
     // 2 = DMP configuration updates failed
     // (if it's going to break, usually the code will be 1)
     Serial.print(F("DMP Initialization failed (code "));
     Serial.print(devStatus);
     Serial.println(F(")"));
  }
}

int balance = 0;
long last_update = 0;
int print_control = 0;

void loop() {
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
     
    mpu.getFIFOBytes(fifoBuffer, packetSize);  // read a packet from FIFO
    
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180/M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180/M_PI);

    int update_motor_interval = 1;  // ms
    // update every x ms
    unsigned long current_time = millis();
    if (current_time - last_update > update_motor_interval) {
      float measure = ypr[2] * 180 / M_PI;
      float control_variable = pid_controller.computeControl(measure);
      if ((print_control++ % 10) == 0) {
        Serial.print("measured angle: ");Serial.print(measure); Serial.print("\t"); Serial.print("control: ");Serial.println(control_variable);
      }
      if (current_time - initial_time > 10000) {
        motor.move(control_variable);
      }
      last_update = current_time;
    }
  }

    
 /* motor.move(0);
  for (int i = -3; i <= 3; i++) {
    Serial.println(i);
    motor.move(0);
    delay(2000);
  }
  */
}


