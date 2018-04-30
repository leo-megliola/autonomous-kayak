#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const uint32_t GPSBaud = 9600;
static const int STEPPER_TURN_STEPS = 100;

//Primary stepper motor controller
static const int STEPPER_DIRECTION = 4;
static const int STEPPER_STEP = 5;
static const int STEPPER_ENABLE_LOW = 6;

//Backup stepper motor controller -- comment out unless primary is not working
//static const int STEPPER_DIRECTION = 9;
//static const int STEPPER_STEP = 10;
//static const int STEPPER_ENABLE_LOW = 11;

static const int DRIVE_MOTOR_RELAY = 13;

// GPS coordinates of places to go
static const double LAT_2KR = 43.049300;
static const double LNG_2KR = -70.691242;

static const double LAT_SHOALS_RED_MARK = 42.993071;
static const double LNG_SHOALS_RED_MARK = -70.620418;

static const double LAT_SAGAMORE_RED_20 = 43.054331;
static const double LNG_SAGAMORE_RED_20 = -70.742754;

static const double LAT_SAGAMORE_RED_18 = 43.055111;
static const double LNG_SAGAMORE_RED_18 = -70.741714;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(8, 7);

int count;

double f_lat;
double f_lng;
double course_to_target;
double t_lat; //target latitude
double t_lng; //target lnggitude

boolean reportAllAhead = false;

double courseToTarget(double lat, double lng) {
  return gps.courseTo(lat, lng, t_lat, t_lng);
}

double rangeToTarget(double lat, double lng) {
  return gps.distanceBetween(lat, lng, t_lat, t_lng);
}

void setTarget(double lat, double lng) {
  t_lat = lat;
  t_lng = lng;
}

void readGps() {
  while (!(gps.location.isValid() && gps.location.isUpdated())) {
    while (ss.available() > 0) gps.encode(ss.read());
  }
}

void reportLocation(double lat, double lng) {
  Serial.print(lat, 9);
  Serial.print(" ");
  Serial.print(lng, 9);
  Serial.print(" ");
  Serial.print("Distance to target: ");
  Serial.print(rangeToTarget(lat, lng), 2);
  Serial.print(" Course to target: ");
  Serial.println(courseToTarget(lat, lng), 2);
}

void storeFix() {
  f_lat = gps.location.lat();
  f_lng = gps.location.lng();
  course_to_target = courseToTarget(f_lat, f_lng);
}

void allAheadFull() {
  if (!reportAllAhead) {
    Serial.println("ALL AHEAD FULL");
    reportAllAhead = true;
  }
  
  digitalWrite(DRIVE_MOTOR_RELAY, HIGH);
}

void allStop() {
  Serial.println("ALL STOP");
  digitalWrite(DRIVE_MOTOR_RELAY, LOW);
  digitalWrite(STEPPER_ENABLE_LOW, HIGH);
}

void turnLeft(int degrees) {
  turn(degrees, true);
}

void turnRight(int degrees) {
  turn(degrees, false);
}

void turn(int degrees, boolean left) {

  if (degrees < 3.0) {
    Serial.println(" CONTINUE STRAIGHT");
  } else {
  
    degrees = (degrees > 45) ? 45 : degrees;
  
    Serial.print(" Turn ");
    Serial.print(left ? "left " : "right ");
    Serial.print(degrees);
    Serial.println(" degrees.");
      
    digitalWrite(STEPPER_ENABLE_LOW, LOW);
  
    if (left) {
      digitalWrite(STEPPER_DIRECTION, HIGH);
    } else {
      digitalWrite(STEPPER_DIRECTION, LOW);
    }
    for (int i = 0; i < STEPPER_TURN_STEPS; i++) {
      digitalWrite(STEPPER_STEP, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEPPER_STEP, LOW);
      delayMicroseconds(500);           
    }
  
    // Calibrate turns by altering constant
    delay(degrees * 150);
  
    if (left) {
      digitalWrite(STEPPER_DIRECTION, LOW);
    } else {
      digitalWrite(STEPPER_DIRECTION, HIGH);
    }
    for (int i = 0; i < STEPPER_TURN_STEPS; i++) {
      digitalWrite(STEPPER_STEP, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEPPER_STEP, LOW);
      delayMicroseconds(500);           
    }
  }
}

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  Serial.println("DELAYING START...");
  delay(30 * 1000);

  count = 0;

  pinMode(DRIVE_MOTOR_RELAY, OUTPUT);
  pinMode(STEPPER_DIRECTION, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_ENABLE_LOW, OUTPUT);

  allStop();

  // lock rudder in place
  digitalWrite(STEPPER_ENABLE_LOW, LOW);
  
  // Set destination.
  setTarget(LAT_SHOALS_RED_MARK, LNG_SHOALS_RED_MARK);
  
  turnRight(30);
  delay(1000); 
  turnLeft(30);
  
  Serial.println("Starting GPS...");
  readGps();
  storeFix();
  reportLocation(f_lat, f_lng);
  allAheadFull();
}

void loop() {  
    readGps();
    double new_lat = gps.location.lat();
    double new_lng = gps.location.lng();
    if (rangeToTarget(new_lat, new_lng) < 50.0) {
      allStop();
    } else {
      allAheadFull();
      double distance = gps.distanceBetween(new_lat, new_lng, f_lat, f_lng);
      if (distance > 3.0) {
  
        count++;

        Serial.print("DISTANCE TRAVELED = ");
        Serial.print(distance, 3);
        double course_traveled = gps.courseTo(f_lat, f_lng, new_lat, new_lng);
        Serial.print(" COURSE TRAVELED = ");
        Serial.print(course_traveled, 3);
  
        Serial.print(" COUNT = ");
        Serial.print(count);

        double a = 360.0 - course_traveled + course_to_target;
        double turn_right = a >= 360.0 ? a - 360.0 : a;
        double turn_left = 360.0 - turn_right;

        if (turn_right < turn_left) {
          turnRight(turn_right);
        } else {
          turnLeft(turn_left);
        }
        
        f_lat = new_lat;
        f_lng = new_lng;
        course_to_target = courseToTarget(f_lat, f_lng);
        reportLocation(f_lat, f_lng);
      }
    }
}
