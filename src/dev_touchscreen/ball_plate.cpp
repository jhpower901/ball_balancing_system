#include <stdint.h> // not sure if needed - imports integer types
#include "TouchScreen.h"
#include <ESP32Servo.h>

#define YP 2  // must be an analog pin, use "An" notation!
#define XM 1  // must be an analog pin, use "An" notation!
#define YM 41   // can be a digital pin
#define XP 42   // can be a digital pin

// full dimensions of screen is 165x105mm. 
float setpointX = 0; // x setpoint in mm. let center of screen = (0,0). bottom left edge = (-82.5,-52.5)
float setpointY = 0; // y setpoint in mm
const float width = 260; // x direction
const float height = 200; // y direction (mm) 
float timeCurr, timePrev;
float errorX, errorY, previousErrorX, previousErrorY;
TSPoint p; // current point of touchscreen
float Px, Ix, Dx, Py, Iy, Dy; // pid values for each axis
int numValidPoints = 0; // number of consecutive valid points. used to discard random measurements that may swing the motors
int numInvalidPoints = 0; // number of consecutive no-touch points. if crosses a threshold, motors are reset

/////////////////PID CONSTANTS/////////////////
// TODO: MAY NEED DIFFERENT CONSTANTS FOR X AND Y. 
// each axis has a different length, so different moment of inertia, etc.
const double Kpx = .55;
const double Kix = 0.05; //.1
const double Kdx = .275; //.25

const double Kpy = .35;
const double Kiy = 0.05;
const double Kdy = .16; //.15
///////////////////////////////////////////////

// SERVOS (doesn't need pwm pins)
const int xServoPin = 20;
const int yServoPin = 19;
Servo xServo;
Servo yServo;
const int flatXAngle = 95;
const int flatYAngle = 90;

// initialize touchscreen
// resistance across x is 274 ohms (measured)
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 360);

const int pointsPerCycle = 150;
float radialVelocity = 1; // rotations per second
int indexPattern = 0;
float trajectoryUpdateTime, lastTrajectoryUpdateTime;

// input smoothing
const int inputWindowSize = 10;
float filteredX = 0;
float filteredY = 0;
float sumX = 0;
float sumY = 0;
float readingsX[inputWindowSize];
float readingsY[inputWindowSize];


int mode = 0;

void updateSetpoint();
float clip2(float, float, float);
int clip(int, int, int);


void setup() {
  Serial.begin(9600); // is this needed at a diff baud?
  xServo.attach(xServoPin);
  yServo.attach(yServoPin);

  // todo: write the servos to their starting points (flat). determine what the starting points should be
  xServo.write(flatXAngle); // might not be 90
  yServo.write(flatYAngle);
  timeCurr = millis();
  lastTrajectoryUpdateTime = millis();
//  setpointX = 30;
}

void loop() {
  // put your main code here, to run repeatedly:

  // read time
  timeCurr = millis();
  float dt = (timeCurr - timePrev) / 1000; // get to seconds from milliseconds
//  Serial.print("dt: "); Serial.println(dt*1000);
//  Serial.println(time-lastTrajectoryUpdateTime);
  updateSetpoint();

//  Serial.println(setpointX);
  if (dt > 0.02) {
    timePrev = timeCurr;
    // read current position
    // get x and y position of ball on touchscreen
    p = ts.getPoint();
  
   Serial.print("X = "); Serial.print(p.x);
   Serial.print("\tY = "); Serial.print(p.y);
   Serial.print("\tPressure = "); Serial.println(p.z);
    
    // nothing is touching, discard this point
      if(p.z == 0) {
    //    Serial.println("discarded");
        numValidPoints = 0;
        numInvalidPoints++;
    //    Serial.print("invalid points count: "); Serial.println(numInvalidPoints);
      } else {
        numValidPoints++;
        numInvalidPoints = 0; // reset to zero because we have a a valid point now
      }
    
      // reset motors if not long enough
      if(numInvalidPoints >= 100) {
        xServo.write(flatXAngle);
        yServo.write(flatYAngle);
        Ix = 0; // reset integrals
        Iy = 0;
        return;
      }
    
     if(numInvalidPoints >= 300) {
        // not sure if this does anything
        xServo.detach();
        yServo.detach();
      }
    
      // wait for accumulation of readings to do something
      if(numValidPoints < 3) {
        return;
      }
  
      // valid point, continue
      if (p.z >= 10) {
      // convert readings to mm
      // the readings never get that close to the edges
      // actual range: x: 75-950, y: 100-870
      // using full range still because then that doesn't inflate the xy readings.
      // if i used a range of 75-950, then a reading of 950 is 82.5, but it cant read your finger that close, its
      // really just a reading of about ~75mm
      float x = map(p.x, 0, 1024, -82.5, 82.5); // x is 165 mm wide
      float y = map(p.y, 0, 1024, -52.5, 52.5); // y is 105mm wide
  //    Serial.println(x);
  
      sumX = sumX - readingsX[0]; // subtract oldest reading
      for(int i = 0; i< inputWindowSize - 1; i++) {
        // shift each reading to the left
        readingsX[i] = readingsX[i+1];
      }
      readingsX[inputWindowSize -1] = x; // add newest reading to history
      sumX = sumX + x;
      filteredX = sumX/inputWindowSize; // average 
  //    Serial.print(x); Serial.print(",");Serial.println(filteredX);
  
      sumY = sumY - readingsY[0]; // subtract oldest reading
      for(int i = 0; i< inputWindowSize - 1; i++) {
        // shift each reading to the left
        readingsY[i] = readingsY[i+1];
      }
      readingsY[inputWindowSize -1] = y; // add newest reading to history
      sumY = sumY + y;
      filteredY = sumY/inputWindowSize; // average 
  //    Serial.print(y); Serial.print(","); Serial.println(filteredY);
  //    Serial.println(x);
  //    Serial.print("\t");
  //    Serial.println(y);
  
      // calculate error
      errorX = setpointX - filteredX;
      errorY = setpointY - filteredY;
    //  Serial.print("x error = "); Serial.println(errorX);
    //  Serial.print("y error = "); Serial.println(errorY);
    
      // calculate x and y motor PID independently
      Px = Kpx*errorX;
      // only add integral if nearby to target
  //    Serial.println(errorX);
  //    if (abs(errorX) <= 30 ) {
        Ix += Kix*errorX*dt;
  //    } 
  //    else {
  //      Ix = 0;
  //    }
  //    if( (errorX > 0) != (previousErrorX > 0)) { // if crossed y axis
  //      Ix = 0;
  //    }
      // TODO: MAYBE CLIP TO EVEN TIGHTER BOUNDS ON SUM
      Ix = clip2(Ix, -10, 10);
      Dx = Kdx*(errorX-previousErrorX)/dt;
//      Dx = clip2(Dx, -30,30);
      float PIDx = Px+Ix+Dx;
//      Serial.print(Px); Serial.print(","); Serial.print(Ix); Serial.print(","); Serial.println(Dx);
    
      Py = Kpy*errorY;
  //    if(abs(errorY) <= 30 ) {
        Iy += Kiy*errorY*dt;
  //    }
  //    else {
  //      Iy = 0;
  //    }
      Iy = clip2(Iy, -height/2, height/2);
      Dy = Kdy*(errorY-previousErrorY)/dt;
//      Dy = clip2(Dy, -50,50);
      float PIDy = Py+Iy+Dy;
      Serial.print(Py); Serial.print(","); Serial.print(Iy); Serial.print(","); Serial.println(Dy);

      
  //    Serial.println(Dy);
  //    Serial.println(Py);
  //    Serial.print("PIDx = "); Serial.println(PIDx);
  //    Serial.print("PIDy = "); Serial.println(PIDy);
      
      // transform and output based on PID output
      
      // HOW DO I MAP AN ERROR IN DISTANCE (FROM TOUCHSCREEN) TO AN OUTPUT IN MOTOR ANGLE?
      // IS IT ALL IN THE PID CONSTANTS? IS THERE A TRANSFER FUNCTION?
      
      // e.g. map to output range of 0-180
      // if x is less than setpoint x, then move one servo to tilt x 
      // same for y
      // map output values to 0-180 or smaller range
      // TODO: WHAT ARE THE INPUT LIMITS?
      // do I take a linear mapping, or just clip the values?
      int xOutput = int(round(map(PIDx, -width/2, width/2, -50, 50))); // x needs larger range to achieve same angle
      int yOutput = int(round(map(PIDy, -height/2, height/2, -40, 40)));
      xOutput = clip(xOutput,-50,50);
      yOutput = clip(yOutput,-40,40);
//      Serial.print("X angle: "); 
//      Serial.println(xOutput);
//      Serial.print("Y angle: "); 
//      Serial.println(yOutput);
  
      // even point +/- whatever the PID was. so if even point is 90 degrees and PID output is 10, write 90+10 to servo
      // TODO: maybe use writeMicroseconds() to get more resolution. 1000-2000 microseconds range corresponds to 0-180
      xServo.write(flatXAngle + xOutput);
      yServo.write(flatYAngle + yOutput);
  //    Serial.println(flatXAngle + xOutput);
      // set this for the next loop
      previousErrorX = errorX;
      previousErrorY = errorY;
    }
  }
}

// setpoint draw a circle
// radius (mm)
// take an index and number of points, then calculate x and y setpoint based on index and trig
// so for index 1 and num points = 100, 1/100*360 = angle in degrees. calculate based on that
// how many indices should there be for a reasonable rotation rate?
void circle(float radius, int i) {
    float angle = float(i)/pointsPerCycle * M_PI * 2;
    setpointX = radius * cos(angle);
    setpointY = radius * sin(angle);
}

void ellipse(float a, float b, int i) {
    float angle = float(i)/pointsPerCycle * M_PI * 2;
    setpointX = a * cos(angle);
    setpointY = b * sin(angle);
}

void line(float length, int i) {
  if (i < pointsPerCycle/2) {
    setpointX = indexPattern/length/2;
  } else {
    setpointX = -indexPattern/length/2;
  }
  setpointY = 0;
}

int cornerIndex = 1;
void fourCorners(float l) {
  // todo
  float w = 32;
  float h = 13;
  switch(cornerIndex) {
    case 1:
      // quad 1
      setpointX = w;
      setpointY = h;
      break;
    case 2:
      setpointX = -w;
      setpointY = h;
      break;
    case 3:
      setpointX = -w;
      setpointY = -h;
      break;
    case 4:
      setpointX = w;
      setpointY = -h;
      break;
  }
  cornerIndex++;
  if(cornerIndex > 4) cornerIndex = 1; // reset back to beginning
}

void updateSetpoint() {
  float dt = (timeCurr - lastTrajectoryUpdateTime)/1000;
  float updateIncrement = 1/radialVelocity/pointsPerCycle;

  switch(mode) {
    case 0:
      // center
      setpointX = 0;
      setpointY = 0;
      break;
    case 1:
      // circle
      if (dt > updateIncrement) {
        circle(10, indexPattern); // set setpoint to circle trajectory
        lastTrajectoryUpdateTime = timeCurr;
        indexPattern+=int(round(dt/updateIncrement)); // often dt is larger than the ideal update time (skipping updates)
        // bc of this, update index to nearest integer. this avoids artificially lowering the angular velocity.
        if (indexPattern > pointsPerCycle) {
          indexPattern = 0;
        }
      }
      break;
    case 2:
      // four corners
      if(dt > 2) {
        fourCorners(30);
        lastTrajectoryUpdateTime = timeCurr;
      }
      break;
    case 3:
      // ellipse
      if (dt > 1/radialVelocity/pointsPerCycle) {
        ellipse(15,10, indexPattern); // set setpoint to circle trajectory
        lastTrajectoryUpdateTime = timeCurr;
        indexPattern+=int(round(dt/updateIncrement)); // if dt is more than the update time, then increments index by more than 1 
        if (indexPattern > pointsPerCycle) {
          indexPattern = 0;
        }
      }
      break;
    default:
      setpointX = 0;
      setpointY = 0;
      break;
  }
}


// helper
int clip(int value, int minimum, int maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}

// helper
float clip2(float value, float minimum, float maximum) {
  if (value > maximum) {
    return maximum;
  }
  if (value < minimum) {
    return minimum;
  }
  return value;
}