#define enR 9
#define in1 4
#define in2 5
#define enL 10
#define in3 6
#define in4 7

int motorSpeedR = 0;
int motorSpeedL = 0;

//rectangle states
bool onLength = true;
bool isRotating = false;
int numTurns = 0;

//parallel states 
bool turningIn = true;



//time control
float prevTime = -1;
float timeElapsed = 0;



void setup() {
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
  delay(5000);
}

void make_circle(float radius) {
  
  //Parameters and desired radius and speed
  float b = 0.135;                            //m
  float alpha = 80;                           //degrees rotating angle
  float dt = 0.7;                             //seconds
  float R = radius;                           //m desired radius of circular path
  float omega = alpha / dt * 2 * M_PI / 360;  //desired angular speed around the circle
  float v = omega * (R - 0.005);              //0.05 accounts for additional mass and friction not modeled in the equation

  float timePerLap = (M_PI * 2 * R)/v;
  if(prevTime == -1) {
    prevTime = millis();
  }

  //Calculating required speed of right and left motor
  float v_R = v + b / 2 * omega;
  float v_L = v - b / 2 * omega;

  int pwm_R = v_R * 437.36 - 100.03;
  int pwm_L = v_L * 437.36 - 100.03;
  Serial.println(pwm_R);
  Serial.println(pwm_L);
  if (pwm_R < 0) {
    pwm_R = 0;
  } else if (pwm_R > 255) {
    pwm_R = 255;
  }

  if(millis() - prevTime >= timePerLap*3000*2) {
    pwm_R = 0;
    pwm_L = 0;
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


void make_rectangle(float length, float width) {
  float v = .2;
  float t_length = length/v;
  float t_width = width/v;

  if(prevTime != -1){
    timeElapsed += millis() - prevTime;
  }
  prevTime = millis();

  Serial.print("time elapsed ");
  Serial.println(timeElapsed);

  if(onLength){
    if(timeElapsed >= t_length * 1000){
      onLength = false;
      isRotating = true;
      timeElapsed = 0;
    }
  }
  else{
    if(timeElapsed >= t_width * 1000){
      onLength = true;
      isRotating = true;
      timeElapsed = 0;
    }
  }
  int motorSpeedR = 145;
  int motorSpeedL = 140;
  // Set Motor R backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Set Motor L backward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  

  if(isRotating) {
    //   // Set Motor R backward
    // digitalWrite(in1, HIGH);
    // digitalWrite(in2, LOW);
    motorSpeedR = 64;
    motorSpeedL = 255;
    if(timeElapsed >= 1200*(length + width)/.8)
    {
    isRotating = false;
    timeElapsed = 0;
    numTurns += 1;
    }
  }

  if(numTurns == 12) {
    motorSpeedR = 0;
    motorSpeedL = 0;
  }




  analogWrite(enR, motorSpeedR);  // Send PWM signal to motor R
  analogWrite(enL, motorSpeedL);  // Send PWM signal to motor L
}

void make_parallel(bool isRight) {
  int slowMotor = 120;
  int fastMotor = 255;

  if(prevTime != -1){
    timeElapsed += millis() - prevTime;
  }
  prevTime = millis();
  int rightMotor;
  int leftMotor;

  if(turningIn) {
    if(isRight) {
      rightMotor = slowMotor;
      leftMotor = fastMotor;
    }
    else{
      rightMotor = fastMotor;
      leftMotor = slowMotor;
    }
    if(timeElapsed >= 1200) {
      timeElapsed = 0;
      turningIn = false;
    }
  }
  else{
    if(isRight) {
      rightMotor = fastMotor;
      leftMotor = slowMotor;
    }
    else{
      rightMotor = slowMotor;
      leftMotor = fastMotor;
    }
    if(timeElapsed >= 1000) {
      rightMotor = 0;
      leftMotor = 0;
    }
  }

  
  Serial.print("time elapsed ");
  Serial.println(timeElapsed);

  


  // Set Motor R backward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set Motor L backward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  analogWrite(enR, rightMotor);  // Send PWM signal to motor R
  analogWrite(enL, leftMotor);  // Send PWM signal to motor L
}

void loop() {
  make_circle(.3);
  // make_rectangle(.2,.4);
  // make_parallel(false);
}