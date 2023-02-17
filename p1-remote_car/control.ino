#define enR 9
#define in1 4
#define in2 5
#define enL 10
#define in3 6
#define in4 7

float b = 0.235; // m
float r = 0.13 / 2; // m
float omega = 2.5 * M_PI; // rad / s - 64 PWM
// float omega = 5.0 * M_PI; // rad / s - 128 PWM
// float omega = 6.6 * M_PI; // rad / s - 255 PWM

float R = 0.8; // m
float alpha = M_PI / 18; // rad
float x = R * sin(alpha); // m
float y = R * cos(alpha); // m

float theta = atan2(y, x);
float dtheta = theta;
float v = x / cos(theta);

float v_L = v - (b / 2) * dtheta;
float v_R = 2*v - v_L;

float pwm_R = v_R / (0.0026 * M_PI);
float pwm_L = v_L / (0.0026 * M_PI);

int motorSpeedR = int(pwm_R);
int motorSpeedL = int(pwm_L);

void setup() {
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // Serial.begin(9600);
  // while(!Serial) ;
  // Serial.println("coords\n");
  // Serial.println(x);
  // Serial.println(y);
  // Serial.println("v_R, v_L\n");
  // Serial.println(v_R);
  // Serial.println(v_L);
  // Serial.println("theta, v\n");
  // Serial.println(theta);
  // Serial.println(v);
  // Serial.println("pwm\n");
  // Serial.println(pwm_R);
  // Serial.println(pwm_L);
}

void loop() {
  // Set Motor R backward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set Motor L backward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enR, motorSpeedR);  // Send PWM signal to motor R
  analogWrite(enL, motorSpeedL);  // Send PWM signal to motor L
}