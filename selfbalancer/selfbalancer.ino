
class Motor {
  public:
  /**
   * \param speed: -1, 0, 1
   */
  void move(int speed) {
    if (speed == 0) {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    } else if (speed > 0) {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);
    } else {
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, HIGH);
    }
  }
  public:
  int pin1 = 7;
  int pin2 = 8;
};

Motor motor;

void setup() {
  Serial.begin(9600);
  pinMode(motor.pin1, OUTPUT);
  pinMode(motor.pin2, OUTPUT);
}
 
void loop() {
  for (int i = -1; i <= 1; i++) {
    motor.move(0);
    delay(5000); 
  }
}


