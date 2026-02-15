/*
 * Autonomous Table Cleaner
 * Arduino Mega 2560 + Arduino Motor Shield
 * Motors: 2× Dagu DG02SS (DC gear motors)
 *
 * Drive wiring:
 *   Motor Shield M1 (Left / vänster):
 *     M1 (+) → DG02SS Red wire
 *     M1 (–) → DG02SS Black wire
 *
 *   Motor Shield M2 (Right / höger):
 *     M2 (+) → DG02SS Red wire
 *     M2 (–) → DG02SS Black wire
 *
 * Sensors:
 *   - 1× forward-facing ultrasonic (obstacle detection)
 *   - 1× forward-left ultrasonic (side obstacle detection)
 *   - 2× downward ultrasonics (table-edge detection, left/right)
 *
 * Control:
 *   - Wheel encoders (interrupt-based pulse counting)
 *   - Heading stabilization while driving (PI-controller)
 *   - Edge approach angle estimation and recovery turns
 *   - Obstacle scan + avoidance state machine
 */

// Left Motor (M1)
#define LEFT_MOTOR_PWM    3    // PWM pin (interrupt capable). Speed control
#define LEFT_MOTOR_IN1   12    // Digital pin. Direction control

int leftMotorDirection = 1;

// Right Motor (M2)
#define RIGHT_MOTOR_PWM   11    
#define RIGHT_MOTOR_IN1  13    
 
int rightMotorDirection = 1;
bool wheelsAreStill = false; //För att kunna definiera hastigheten 0

// Ultraljudssensorer
#include <NewPing.h>

//fram horisontal
#define TRIG_PIN_forward_horizontal 39   // HC-SR04 Trigger on A0
#define ECHO_PIN_forward_horizontal 38  // HC-SR04 Echo on A1
#define MAX_DISTANCE 200  // Maximum distance in cm
NewPing forward_horizontal(TRIG_PIN_forward_horizontal, ECHO_PIN_forward_horizontal, MAX_DISTANCE);

//botten, vänster, fram och höger
#define TRIG_PIN_left  22  // HC-SR04 Trigger on A0
#define ECHO_PIN_left  23  // HC-SR04 Echo on A1

#define TRIG_PIN_forward_left  46  // HC-SR04 Trigger on A0
#define ECHO_PIN_forward_left  47  // HC-SR04 Echo on A1
NewPing forward_horizontal_left(TRIG_PIN_forward_left, ECHO_PIN_forward_left, MAX_DISTANCE);

#define TRIG_PIN_right  52  // HC-SR04 Trigger on A0
#define ECHO_PIN_right  53  // HC-SR04 Echo on A1

//Vilken rotation den gjorde senast, true innebär höger, så den ska placeras på
//bordet så att den snurrar vänster vid nästa kant
boolean previous_rotation = true; 

//Blir true då något kriteria möts för att hela bordet har städats
volatile boolean table_is_clean = false;

//Array som innehåller alla ultraljudssensorer som pekar ner
NewPing bottom_edges[2]={
  NewPing(TRIG_PIN_left, ECHO_PIN_left, MAX_DISTANCE),
  
  NewPing(TRIG_PIN_right, ECHO_PIN_right, MAX_DISTANCE)
};

//Encoder
//Vänster
#define SENSOR_PIN_LEFT   18  // PIN B enligt datasheet. Interrupt pin. Läser av sensor
#define LEFT_DIR   26  // PIN A enligt datasheet. Digital pin för att avgöra riktning
volatile unsigned long durationLeft = 1000000000;
volatile long pulseCountLeft = 0;
static unsigned long lastTimeLeft = 0;
bool left_dir = true; //true framåt, false bakåt
//Höger
#define SENSOR_PIN_RIGHT  19  // PIN B enligt datasheet. Interrupt pin
#define RIGHT_DIR  27  // PIN A enligt datasheet. Digital pin för att avgöra riktning
volatile unsigned long durationRight =  1000000000;
volatile long pulseCountRight = 0;
static unsigned long lastTimeRight = 0;
bool right_dir = true; //true framåt, false bakåt
//Speedometer
float previous_speeds_left[5] = {0.0};
float previous_speeds_right[5] = {0.0};

//Övriga parametrar till encodern
float radius = 0.065/2.0; //däckets radie
float radians = 2*PI/960.0; //radianer mellan strecken
float partOfRotation = 1/960.0*1000000.0;
float cirkelsektor = radius*radians; // Cirkelsektor av ett svart/vitt område i encodern
float distanceBetweenWheels = 0.14;
float getAngleKonst = cirkelsektor/distanceBetweenWheels; //används i getAngle
float cirkelsektorMikro = cirkelsektor*1000000.0*60; //1000000.0 eftersom i metoden getSpeed är tiden i mikrosekunder, och 60 eftersom den ska ge rpm
float dist_between_URsensors = 0.05; //0.15
float dist_Center_ForwardSensor = 12.5;
int movementCount = 0;
bool edge_test_0 = false;
bool edge_test_1 = false;

// PID hastighetsstyrning parametrar (justera dessa)
float Kp = 1.0;   // Proportional gain
float Ki = 1/20;   // Integral gain
float Kd = 0.0;  // Derivative gain
float errorAngle = 0;
float lastError = 0;
float integralAngle = 0; 
// Önskad RPM för båda motorer 
//float desiredRPM = 1200;  
int basePWM = 60; // Initialt PWM-värde.

// Svägning
float basePWM_rot = 50;
float currentAngle = 0.0;  // Mäts upp av någon funktion
float error_rot = 0;
float lastError_rot = 0; 
float integral_rot = 0;
float derivative_rot = 0; 
float last_error_rot = 0;

//Encoder vänster
//leftMotorDirection och rightMotorDirection blir 1 eller -1 vid framåt resp. bakåtdrift. 
//Detta behövs för att kunna beräkna vinkeln den roterat.  
void encoderLeft() {
  unsigned long currentTime = micros();
  durationLeft = currentTime - lastTimeLeft;
  lastTimeLeft = currentTime;
  pulseCountLeft += leftMotorDirection;                 
}

//Encoder höger
void encoderRight() {
  unsigned long currentTime = micros();
  durationRight = currentTime - lastTimeRight;
  lastTimeRight = currentTime;
  pulseCountRight += rightMotorDirection;                
    
}

float getAngle(){
  return ((float)pulseCountLeft-(float)pulseCountRight)*getAngleKonst;
}


double getSpeed(int side){
  
  if(wheelsAreStill){
    return 0.0;
  }
  if(side==0){
    int sum = 0;
    for(int i = 0; i < 4; i++){
    sum = sum + previous_speeds_left[i];
    previous_speeds_left[i] = previous_speeds_left[i+1];
    }
    previous_speeds_left[4] = left_dir*partOfRotation/(double)durationLeft;
    return (sum + previous_speeds_left[4])/5;
  }

    int sum = 0;
    for(int i = 0; i < 4; i++){
    sum = sum + previous_speeds_right[i];
    previous_speeds_right[i] = previous_speeds_right[i+1];
    }
    previous_speeds_right[4] = right_dir*partOfRotation/(double)durationRight;

  return (sum + previous_speeds_right[4])/5;
}

//detekterar hinder framför
 bool is_forward_obstacle() {
  int distance = forward_horizontal.ping_cm();  // Get distance in cm
  if(distance<15.0 && distance > 1.0){
    return true;
  } 
  return false;
}

bool is_left_obstacle(){
  int distance = forward_horizontal_left.ping_cm();  // Get distance in cm
  if(distance<5.0 && distance > 1.0){
    Serial.println(distance);
    Serial.println("ll");
    return true;
  } 
  return false;

}

//Första elementet i vektorn ger distans till kanten raktframifrån, andra ger distansen vinkelrätt åt vänster, och tredje vinkelrätt åt höger.
//Roboten snurrar först åt vänster fram till dess att sensorn går förbi kanten, och därefter samma åt höger.
float* get_obstacle_distances(){
  static float arr[3] = {0.0}; 

  arr[0] = forward_horizontal.ping_cm() + dist_Center_ForwardSensor;
  while(arr[0]<11.0){
      arr[0] = forward_horizontal.ping_cm() + dist_Center_ForwardSensor;
  }
  
  float angleLeft = 0.0;
  float distLeft = 0.0;
  float newDistLeft = 0;

  while(newDistLeft<30.0){
    newDistLeft = forward_horizontal.ping_cm();
    delay(10);
    if(newDistLeft<30){
      distLeft = newDistLeft;
    }

    angleLeft = getAngle();
    setLeftMotor(50);
    setRightMotor(-(255-50));
  }
    stopMotors();
  delay(1000);
    arr[1] = sin(angleLeft)*dist_Center_ForwardSensor + sin(angleLeft-PI/6)*distLeft;
    rotate2(-angleLeft);
    delay(1000);

  float angleRight = 0.0;
  float distRight = 0.0;
  float newDistRight = 0.0;

  while(newDistRight<30.0){
    newDistRight = forward_horizontal.ping_cm();

    delay(10);
    if(newDistRight<30){
      distRight = newDistRight;
    }

    angleRight = getAngle();
    setLeftMotor(-(255-50));
    setRightMotor(50);
  }
  stopMotors();
  delay(100);
  rotate2(-angleRight);
  delay(100);
 
    arr[2] = abs(sin(angleRight)*dist_Center_ForwardSensor) + abs(sin(angleRight+PI/6)*distRight);

    for(int i = 0; i < 3; i++){
      Serial.println(arr[i]);
      arr[i] = arr[i]/100;
    }
  return arr;
}


bool obstacle_avoidance(int dir, float forw, float dist){
  
  if(movementCount==0){
  rotate2(dir*PI/2);
  float edge = forwardDistanceLoop(dist+0.1); // testa ifall den slår i kanten, om den gör det byta håll
  if(edge==0.0){
    delay(100);
    Serial.println(edge);
    return true;
  }
   Serial.println(edge);
   forwardDistanceLoop(-edge);
   return false;
  }

  if(movementCount==1){
    Serial.println("Count 1");
    pulseCountLeft = 0;
    pulseCountRight = 0;
  rotate2(dir*(-PI/2));
  delay(500);
  return true;
  }

   if(movementCount==2){
     Serial.println("Count 2");
     int edge = PIDSpeedControl(1, 1, 60);
    while(!is_left_obstacle() && (edge==2||edge==3)){
      edge = PIDSpeedControl(1, 1, 60);
    }

    if(edge==2){
      return true;
    }

    forwardDistanceLoop(-edge);
    return false;
  }
  
  if(movementCount==3){
     Serial.println("Count 3");
  float edge = forwardDistanceLoop(0.22); //om den stöter på ett hinder, kör då direkt tillbaks
  if(edge==0){
    return true;
  }
    forwardDistanceLoop(-edge);
  }

  if(movementCount==3){
  rotate2(dir*(-PI/2));
  forwardDistanceLoop(dist);
  rotate2(dir*PI/2);
  return true;
  }
  }


//Metoden returnerar distansen den hunnit köra ifall den kör över kanten och 0.0 om den hunnit hela vägen
float forwardDistanceLoop(float dist){
  int currentSteps = pulseCountLeft;
  Serial.println("mm");
  //float dist_to_sidesensor = 0.095
  float steps = dist/cirkelsektor;
  int edge = PIDSpeedControl(1, 1, 60);
  while(abs(pulseCountLeft-currentSteps) < abs(steps) && edge==2){
    if(dist>0){
    edge = PIDSpeedControl(1, 1, 60);
    }else{
      edge = PIDSpeedControl(1, 1, -(255-60));
    }
  }
  stopMotors();
  if(edge<2){
  return abs(pulseCountLeft-currentSteps);
  }
  return 0.0;
}

//detekterar kant
bool is_bottom_edge(int idx) {
  int distance = bottom_edges[idx].ping_cm();  // Get distance in cm
  if(distance>10 && distance > 1.0){
    return true;
  } 
  return false;
}


//Beräkna robotens vinkel utifrån tiden båda sensorerna når över kanten. Idx är elementet i bottom_edges
// (left/right) som drog över kanten först.
double getAngleAtEdge(int idx){
  if(idx==0){
    edge_test_0 = true;
  }else{
    edge_test_1 = true;
  }

  int otherIdx;
  float stepCount = pulseCountRight;
  if(idx==0){
    otherIdx = 1;
  }else{
    otherIdx =0;
  }
  
  float stepCounter = 0;
  
  int edge = PIDSpeedControl(1, 1, 60);
  while(edge == 2){
    //Här körs ju encoderRight() som är en interruptfunktion och räknar antalet steg för högra motorn.
    //Man skulle kunna använda vänstra motorn också men det blir mer kompakt att välja en av dem.
    edge = PIDSpeedControl(1, 1, 60); //kanske inte behövs
  }
    stopMotors();
    float deltaStepCount = pulseCountRight-stepCount;
    float dist = cirkelsektor*(double)deltaStepCount; //här beräknas streckan roboten kört efter ena sensorn drog över kanten
    float angle = atan(dist/dist_between_URsensors);
    currentAngle = angle;
    pulseCountRight = 0;
    pulseCountLeft = 0;
    edge_test_0=false; //när 0 slagit över kanten
    edge_test_1=false; //när 1 slagit över kanten
    Serial.print("Vinkel: ");
    Serial.println(angle);

    if(abs(angle)>PI/3){
      return 0;
    }  
    if(idx==0){
      Serial.println(angle);
      return angle; //vinkel 0 är rakt fram
    }
      Serial.println(-angle);
    return -angle;
    }
  


// --- Motor Control Functions ---
// Controls the left motor; positive speed spins one direction, negative reverses it.
void setLeftMotor(int speed) {
  if (speed > 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    analogWrite(LEFT_MOTOR_PWM, speed);
    leftMotorDirection = 1;
    wheelsAreStill = false;
  } else if (speed < 0) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_PWM, speed);
    leftMotorDirection = -1;
    wheelsAreStill = false;
  } else {
    // Stop left motor
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_PWM, 0);
    wheelsAreStill = true;
    if(leftMotorDirection==1){
      leftMotorDirection = -1;
    }else{
      leftMotorDirection = 1;
    }
  }
}

// Controls the right motor; positive speed spins one direction, negative reverses it.
void setRightMotor(int speed) {
  if (speed > 0) {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    analogWrite(RIGHT_MOTOR_PWM, speed);
    rightMotorDirection = 1;
    wheelsAreStill = false;
  } else if (speed < 0) {
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, speed);
    rightMotorDirection = -1;
    wheelsAreStill = false;
  } else {
    // Stop right motor
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    analogWrite(RIGHT_MOTOR_PWM, 0);
    wheelsAreStill = true;

    if(rightMotorDirection==1){
      rightMotorDirection = -1;
    }else{
      rightMotorDirection = 1;
    }
  }
}

// Stops both motors.
void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// Moves the robot forward by running both motors forward.
void moveForward(int speed) {
  setLeftMotor(speed);
  setRightMotor(speed);
}

// Moves the robot backward by running both motors in reverse.
void moveBackward(int speed) {
  setLeftMotor(-speed);
  setRightMotor(-speed);
}

// Turns the robot left in-place by spinning the left motor in reverse
// and the right motor forward.
void turnLeft(int speed) {
  setLeftMotor(-speed);
  setRightMotor(speed);
}

// Turns the robot right in-place by spinning the left motor forward
// and the right motor in reverse.
void turnRight(int speed) {
  setLeftMotor(speed);
  setRightMotor(-speed);
}

  float PWM_Left = 65.0;
  float PWM_Right = 65.0;

// leftMotor och rightMotor antingen -1 eller 1 (bakåt eller framåt)
int PIDSpeedControl(int leftMotorDir, int rightMotorDir, float minPWM){ 
  // Measure actual RPM (ensure you update these in your ISR)
  float angle =  ((float)pulseCountLeft-(float)pulseCountRight);

  // Calculate errors
  errorAngle = -angle;

  // Calculate integral terms (for steady-state errors)
  integralAngle -= angle;

  // Calculate derivative terms (for quick adjustments)
  float derivativeAngle = errorAngle - lastError;

  // PID calculations

  float sumPID = (Kp * errorAngle) + (Ki * integralAngle);
  
  PWM_Left = basePWM + sumPID;
  PWM_Right = basePWM - sumPID;

  // Limit PWM between 0 and 255
  PWM_Left = constrain(PWM_Left, minPWM-30, minPWM+45);
  PWM_Right = constrain(PWM_Right, minPWM-30, minPWM+45);
  
  // Ge motorerna hastigheterna
  setLeftMotor(leftMotorDir*PWM_Left);
  setRightMotor(rightMotorDir*PWM_Right);

  // Update last errors for derivative calculation
  lastError = errorAngle;
  
    if(!edge_test_0 && is_bottom_edge(0)){
      stopMotors();
      Serial.println(0);
      return 0;
    }
    if(!edge_test_1 && is_bottom_edge(1)){
      stopMotors();
      Serial.println(1);
      return 1;
    } 
    //delay(100);
    if(!edge_test_0 && !edge_test_1 && is_forward_obstacle()){
      stopMotors();
      Serial.println(3);
      return 3;
    }
      delay(10); // small delay for stable PID calculation
      return 2;
}


void rotate2(float angle){
  float initialAngle = getAngle();
  if(angle>0){
  setLeftMotor(50);
  setRightMotor(-(255-50));
  while(getAngle()-initialAngle<angle && !is_bottom_edge(0)){ 
    //Serial.println(getAngle()-initialAngle);
  }
  stopMotors();
  for(int i = 0; i<5; i++){
  if(is_bottom_edge(0)){
    table_is_clean=true;
  }
  }
  }else{
    if(angle<0){
  setLeftMotor(-(255-50));
  setRightMotor(50);
  while(getAngle()-initialAngle>angle&& !is_bottom_edge(1)){
      }
      stopMotors();
      for(int i = 0; i<5;i++){//typ exakt varannan sampling blir 0 när den är över kanten för denna sensorn, fett weird.
      if(is_bottom_edge(1)){
    table_is_clean=true;
        }
      } 
    }
  }
  pulseCountLeft = 0;
  pulseCountRight = 0;
}

void setup() {
  Serial.begin(9600);
  // Set all motor control pins as outputs
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  //pinMode(LEFT_MOTOR_IN2, OUTPUT);
  
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  //pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  //Encoder pins
  pinMode(SENSOR_PIN_LEFT, INPUT_PULLUP);
  pinMode(LEFT_DIR, INPUT_PULLUP);
  //digitalWrite(ENABLE_PIN_LEFT, HIGH); // Ensure the sensor is always enabled

  pinMode(SENSOR_PIN_RIGHT, INPUT_PULLUP);
  pinMode(RIGHT_DIR, INPUT_PULLUP);
  //digitalWrite(ENABLE_PIN_RIGHT, HIGH); // Ensure the sensor is always enabled

  //Ultraljudsensor pins
  pinMode(TRIG_PIN_left, OUTPUT);
  pinMode(ECHO_PIN_left, INPUT);

  pinMode(TRIG_PIN_forward_left, OUTPUT);
  pinMode(ECHO_PIN_forward_left, INPUT);

  pinMode(TRIG_PIN_right, OUTPUT);
  pinMode(ECHO_PIN_right, INPUT);

  pinMode(TRIG_PIN_forward_horizontal, OUTPUT);
  pinMode(ECHO_PIN_forward_horizontal, INPUT);

  // Ensure motors are stopped at startup.
  stopMotors();
  wheelsAreStill = false; //Hastighetsmätaren funkar inte annars
  //Vid varje flank upp eller ned så avbryts programmet för att göra metoderna encoderLeft() och encoderRight()
attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_LEFT), encoderLeft, RISING);
attachInterrupt(digitalPinToInterrupt(SENSOR_PIN_RIGHT), encoderRight, RISING);
}

void rotate(float angle){
  float initialAngle = getAngle();
  if(angle>0){
  setLeftMotor(80);
  while(getAngle()-initialAngle<angle && !is_bottom_edge(0)){ 
    //Serial.println(getAngle()-initialAngle);
  }
  stopMotors();
  for(int i = 0; i<5; i++){
  if(is_bottom_edge(0)){
    table_is_clean=true;
  }
  }
  }else{
    if(angle<0){
  setRightMotor(80);
  while(getAngle()-initialAngle>angle&& !is_bottom_edge(1)){
      }
      stopMotors();
      for(int i = 0; i<5;i++){//typ exakt varannan sampling blir 0 när den är över kanten för denna sensorn, fett weird.
      if(is_bottom_edge(1)){
    table_is_clean=true;
        }
      } 
    }
  }
}

//boolean previous_rotation = true; 
//Blir true då något kriteria möts för att hela bordet har städats
//boolean table_is_clean = false;

//boolean is_forward_edge() 
//boolean is_bottom_edge(int idx) 

//Roboten placeras i ett hörn så högersidan hamnar mot en kant. Den kör till den når kanten, snurrar åt vänster 90 grader,
//kör fram lite till och söker under tiden efter en kant (om en kant hittas räknas detta som att bordet har nått andra änden och är färdigstädat),
//sedan svänger den till vänster 90 grader och kör till andra änden där den gör samma sak spegelvänt. har ännu inte applicerat
//planering av rutt vid hinder men har gjort en metod till det.


void obstacle_avoidance_real(float* distances){
  
  while(movementCount>-1 && movementCount<5){
    if(obstacle_avoidance(1, distances[0], distances[1])){
      movementCount++;
      //obstacle_avoidance(1, distances[1]);
    }else{
      movementCount--;
      //obstacle_avoidance(-1, distances[1]);
    }
  }
}

void handle_edge(int dir, int edge){
  Serial.println("hej");
      currentAngle = getAngleAtEdge(edge);
  Serial.println("halloj");
      pulseCountLeft = 0;
      pulseCountRight = 0;
  
    stopMotors();
    delay(500);
    edge_test_0=true;
    edge_test_1=true;
    forwardDistanceLoop(-0.07);
    delay(500);
    edge_test_0=false;
    edge_test_1=false;
    rotate(dir*(PI)+currentAngle);
    delay(500);
    pulseCountLeft = 0;
    pulseCountRight = 0;
    Serial.println("tja");
}

boolean a = false;
void loop() {
  
  if(a){
    rotate(PI);
    a = false;
  }

  while(!table_is_clean){
    if(!table_is_clean){
    int edge = PIDSpeedControl(1, 1, 60);
  while(edge==2){
    edge = PIDSpeedControl(1, 1, 60);
    if(edge==3){
      stopMotors();
      Serial.println("hinder");
      float* distances = get_obstacle_distances();
      obstacle_avoidance_real(distances);
      pulseCountLeft = 0;
      pulseCountRight = 0;
    }
    edge = PIDSpeedControl(1, 1, 60);
  }
  
    if(edge<2){
      Serial.println("Hej1");
      handle_edge(1, edge);
    }

    }
if(!table_is_clean){
  currentAngle = 0;
  int edge = PIDSpeedControl(1, 1, 60);
  while(edge==2){
    //vänta till en sensor slår över kanten
    edge = PIDSpeedControl(1, 1, 60);
    if(edge==3){
      stopMotors();
      float* distances = get_obstacle_distances();
      obstacle_avoidance_real(distances);
      pulseCountLeft = 0;
      pulseCountRight = 0;
    }
    edge = PIDSpeedControl(1, 1, 60);
  }

    if(edge<2){
      Serial.println("Hej2");
      handle_edge(-1, edge);
    }
  
    }
    Serial.println(table_is_clean);
  }
}
