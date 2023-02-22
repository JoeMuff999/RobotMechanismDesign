#define enR 9
#define in1 4
#define in2 5
#define enL 10
#define in3 6
#define in4 7

int motorSpeedR = 0;
int motorSpeedL = 0;




void setup() {
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
}



void loop() {

  //Parameters and desired radius and speed
  float b = 0.135;           //m
  float alpha = 80;          //degrees rotating angle
  float dt = 0.5;            //seconds
  float R = 0.2;             //m desired radius of circular path
  float omega = alpha /dt*2*M_PI/360;   //desired angular speed around the circle
  float v = omega * (R-0.005); //0.05 accounts for additional mass and friction not modeled in the equation

  //Calculating required speed of right and left motor
  float v_R = v + b / 2 * omega;
  float v_L = v - b / 2 * omega;

  int pwm_R = v_R * 437.36 - 100.03;
  int pwm_L = v_L * 437.36 - 100.03;
Serial.println(pwm_R);
Serial.println(pwm_L);
  if (pwm_R < 0) {
    pwm_R=0;
  }
  else if (pwm_R > 255) {
    pwm_R=255;
  }

int motorSpeedR = pwm_R;
int motorSpeedL = pwm_L;
// Set Motor R backward
digitalWrite(in1, LOW);
digitalWrite(in2, HIGH);
// Set Motor L backward
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);

analogWrite(enR, motorSpeedR);  // Send PWM signal to motor R
analogWrite(enL, motorSpeedL);  // Send PWM signal to motor L
}