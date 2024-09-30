#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Math.h>

#define PI 3.1415926535897932384626433832795
#define SERVO_FREQ 50
#define SERVO_MIN 150
#define SERVO_MAX 600

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct ServoPositions {
  int top;
  int middle;
  int bottom;
};

const ServoPositions LF = {0, 1, 2};
const ServoPositions LB = {3, 4, 5};
const ServoPositions RF = {6, 7, 8};
const ServoPositions RB = {9, 10, 11};

float servo_offsets[4][3] = {
  {0, 10, 10}, {15, 10, 20}, //LF & LB
  {35, 8, 25}, {30, 20, 25} //RF & RB
};

//pre-specified values for robot positions
float neutral_position[4][3] = {
  {-60, -60, -60}, {-60, -60, -60}, //LF & LB
  {-60, -60, -60}, {-60, -60, -60} //RF & RB
};

float tilt_position[4][3] = {
  {-40, -40, -40}, {-80, -40, -40}, //LF & LB
  {-40,-80, -80}, {-80, -80, -80} //RF & RB
};

float standing_position[4][3] = {
  {-60, -30, -30}, {-60, -30, -30}, //LF & LB
  {-60, -90, -90}, {-60, -90, -90} //RF & RB
};

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  moveJIM(neutral_position);
  
  delay(500);
}

void loop() {

  //move each leg individually to specified position
  inverseKinematics(RF, 20, -87);
  inverseKinematics(RB, 35.634, -97.849);
  inverseKinematics(LF, -35.634, 97.849);
  inverseKinematics(LB, -35.634, 97.849);
  
  for(int i = 0; i < 20; i++) {
    //moveJIM(standing_position);
    //delay(300);
    //moveJIM(neutral_position);
    //delay(300);
  }
}

bool isFlipped(int index) {
  //List of flipped servo indices
  //const int flipped_servos[] = {3, 7, 8, 9, 10, 11};
  const int flipped_servos[] = {0,1,2,3,4,5};
  const int num_flipped_servos = sizeof(flipped_servos) / sizeof(flipped_servos[0]);

  for (int i = 0; i < num_flipped_servos; i++) {
    if (flipped_servos[i] == index) {
      return true;
    }
  }
  return false;
}

//implementing inverse kinematics to control leg
void inverseKinematics (const ServoPositions& leg, float x, float y) {
  float L1 = 90.0; //hip in mm
  float L2 = 111.93; //shin in mm

  //q1 and q2 are calculated in radians
  float q2 = acos( (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2));
  float pt1 = atan2(y,x);
  float pt2 = atan2( L2*sin(q2) , L1+L2*cos(q2) );
  float q1 = pt1 - pt2;

  //convert q1 and q2 to degrees
  float q1_deg = radToDeg(q1);
  float q2_deg = radToDeg(q2);

  Serial.print("middle: ");
  Serial.print(q1_deg);
  Serial.print(" -> pulse: ");
  Serial.println(angleToTicks(q1_deg));
  Serial.print("bottom: ");
  Serial.print(q2_deg);
  Serial.print(" -> pulse: ");
  Serial.println(angleToTicks(q2_deg));

  if(isFlipped(leg.middle)) {
    pwm.setPWM(leg.middle, 0, angleToTicks(q1_deg-80));
    pwm.setPWM(leg.bottom, 0, angleToTicks(q2_deg+20));
  } else {
    pwm.setPWM(leg.middle, 0, angleToTicks(q1_deg+80));
    pwm.setPWM(leg.bottom, 0, angleToTicks(q2_deg+30));
  }
}

//given specified robot position, move all the legs needed simultaneously
void moveJIM (float positions[4][3]) {
  //Array of structs for servo positions
  const ServoPositions servo_positions[] = {LF, LB, RF, RB};

  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 3; c++) {
      int servoIndex;
      switch (r) {
        case 0: servoIndex = servo_positions[0].top + c; break;
        case 1: servoIndex = servo_positions[1].top + c; break;
        case 2: servoIndex = servo_positions[2].top + c; break;
        case 3: servoIndex = servo_positions[3].top + c; break;
        default: continue;
      }
      
      if (servoIndex < 12) {
        //Adjust position based on offset
        float adjustedPosition = positions[r][c] + servo_offsets[r][c];
        // if (isFlipped(servoIndex)) {
        //   adjustedPosition = positions[r][c] - servo_offsets[r][c];
        // }
        pwm.setPWM(servoIndex, 0, angleToTicks(adjustedPosition));
      }
    }
  }
}

void setPos (int leg, int top_pos, int middle_pos, int bottom_pos) {
  pwm.setPWM(leg, 0, angleToTicks(top_pos));
  pwm.setPWM(leg+1, 0, angleToTicks(middle_pos));
  pwm.setPWM(leg+2, 0, angleToTicks(bottom_pos));
}

int angleToTicks(int angle) {
  //Theoretical range of IK angles
  float minAngle = -180.0;
  float maxAngle = 180.0;

  //Corresponding servo pulse widths for servo's 0 to 300 degree range
  int minPulse = 500;
  int maxPulse = 2500;

  //int pulse = map(angle, minAngle, maxAngle, minPulse, maxPulse);
  //int pulse = map(angle, 0, 300, SERVO_MIN, SERVO_MAX);
  int ticks = map(angle, -150, 150, SERVO_MIN, SERVO_MAX);

  //Clamp pulse value so it stays within valid range (maybe remove)
  //pulse = constrain(pulse, minPulse, maxPulse);

  return ticks;

  //90 should equal 100
  // Calculate pulse width in microseconds
  //int pulseMicroseconds = map(angle, 0, 300, 500, 2500);
  // Convert microseconds to 'pulse' for 12-bit (0-4096) scale
  //int pulse = map(pulseMicroseconds, 0, 20000, 0, 4096);
}

float radToDeg(float rad) {
  return rad * (180.0/PI);
}