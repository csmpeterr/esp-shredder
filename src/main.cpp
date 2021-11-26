#include <functions.h>

// GPIO Definitions
#define USS1_TRIGGER_PIN 5
#define USS1_ECHO_PIN 18
#define USS2_TRIGGER_PIN 23
#define USS2_ECHO_PIN 22

#define COVERBUTTON_PIN 15
#define USERBUTTON_PIN 0

#define UP_COUNTER_PIN 33
#define DOWN_COUNTER_PIN 13

#define MOTOR_SIGNAL_PIN 17
#define MOTOR_ROTATE_RIGHT_PIN 26
#define MOTOR_ROTATE_LEFT_PIN 27

// Constant definitions
#define SOUND_SPEED 0.034
#define FIVE_SECOND 5000
#define THREE_SECOND 3000

#define MOTOR_AVERAGE_CURRENT 9800
#define MOTOR_MAXIMAL_CURRENT 12567
#define MOTOR_RUNNING_TIME 5
#define MOTOR_STOPPING_TIME 3

#define MOTOR_SAFETY_DELAY 50
#define ONE_MILLISECOND 1
#define ONE_SECOND ONE_MILLISECOND * 1000
#define ONE_MINUTE ONE_SECOND * 60
#define ONE_HOUR ONE_MINUTE * 60
#define ONE_DAY ONE_HOUR * 24

#define PROXY_DISTANCE 15  // in centimeters





// Enumerations

enum timeUnit{millisecond, second, minute, hour, day};
enum usSensorAnswer{empty,upper,both,lower};
enum rotationDirection{right, left};


// Global variables

// Structures

struct SensorParameters {
  struct Ultrasound{
    uint8_t currentState;
      struct USS1 {
          const uint8_t triggerPin = USS1_TRIGGER_PIN;
          const uint8_t echoPin = USS1_ECHO_PIN;
          uint16_t proximityDistance = 25;
          bool calibrated = false;          
      } upper;
      struct USS2 {
          const uint8_t triggerPin = USS2_TRIGGER_PIN;
          const uint8_t echoPin = USS2_ECHO_PIN;
          uint16_t proximityDistance = 25;
          bool calibrated = false;          
      } lower;
  } ultrasound;
  
  struct Rotation {
    bool currentDirection;
    bool previousDirection;
  } rotation;

  struct Cover {
    const uint8_t pin = COVERBUTTON_PIN;
    bool state;
  } cover;

  struct AmMeter {
    // const uint8_t pin =;
    bool init;
    int32_t current;
  } ammeter;

} sensor;

struct ShreddingProgress {
  bool startFlag;
  bool processFlag = false;
  bool coverFlag;
  bool failFlag = false;
  bool locked = false;
} shredding;

struct UserButton {
  uint32_t lastDebounceTime = 0;
  uint8_t debounceDelay = 50;
  bool lastSteadyState = HIGH;
  bool lastFlickerState = HIGH;
  bool currentState;
  bool state;
} userButton;


// Tasks for multitasking, using core 1

TaskHandle_t measureTask;
TaskHandle_t controlTask;

// skeleton for multithreading



void IRAM_ATTR Ext_INT1_ISR(){  
  
  motorStop(MOTOR_STOPPING_TIME); 
  
}


bool motorRotationSwitch(bool rotationDirection){
  
  bool direction;

  if (!rotationDirection){  
      digitalWrite(MOTOR_ROTATE_RIGHT_PIN, HIGH);
      digitalWrite(MOTOR_ROTATE_LEFT_PIN, LOW);
      direction = right;
  }
  else if(rotationDirection){
      digitalWrite(MOTOR_ROTATE_RIGHT_PIN, LOW);
      digitalWrite(MOTOR_ROTATE_LEFT_PIN, HIGH);
      direction = left;
  } 
  return direction;
}


int32_t emulateMotorVoltage(){  

  if(!digitalRead(UP_COUNTER_PIN)){
    sensor.ammeter.current = sensor.ammeter.current + 120;
  } 
  if (!digitalRead(DOWN_COUNTER_PIN)){
    sensor.ammeter.current = sensor.ammeter.current - 109;
  }
  return sensor.ammeter.current;
}





void configureIO() {
  // Input configuration
  pinMode(USS1_ECHO_PIN, INPUT);
  pinMode(USS2_ECHO_PIN, INPUT);

  pinMode(COVERBUTTON_PIN, INPUT_PULLUP);  // if button not pushed, it is on logical high
  attachInterrupt(COVERBUTTON_PIN, Ext_INT1_ISR, HIGH);
  pinMode(USERBUTTON_PIN, INPUT_PULLUP);  // if button not pushed, it is on logical high


  pinMode(UP_COUNTER_PIN, INPUT_PULLUP); // emulate motor voltage change
  pinMode(DOWN_COUNTER_PIN, INPUT_PULLUP); // emulate motor voltage change

  // Output configuration
  pinMode(USS1_TRIGGER_PIN, OUTPUT);
  pinMode(USS2_TRIGGER_PIN, OUTPUT);

  pinMode(MOTOR_SIGNAL_PIN, OUTPUT);
  pinMode(MOTOR_ROTATE_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_ROTATE_RIGHT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}


bool timeToWait(uint64_t value, uint8_t timeUnit){
  uint8_t tu = timeUnit;

  uint64_t time_unit, interval;

  switch (tu){
    case 0: time_unit = ONE_MILLISECOND; break;
    case 1: time_unit = ONE_SECOND; break;
    case 2: time_unit = ONE_MINUTE; break;
    case 3: time_unit = ONE_HOUR; break;
    case 4: time_unit = ONE_DAY; break;
    default: time_unit = ONE_SECOND; 
  }

  interval = value * time_unit;  

  uint64_t previous_millis = millis();
  uint64_t current_millis = millis();

  bool time_spent = false;
  while((uint64_t)(current_millis - previous_millis)< interval){
    current_millis = millis();
     sensor.ammeter.current = measureMotorCurrent();
    
  }
  time_spent = true;
  if (time_spent){
    return true;
  } else return false;  
}




bool buttonState(uint8_t buttonPin) {  // debouncing pushbutton
  bool flag;
  bool state;
  userButton.currentState = digitalRead(buttonPin);

  if (userButton.currentState != userButton.lastFlickerState) {
    userButton.lastDebounceTime = millis();
    userButton.lastFlickerState = userButton.currentState;
  }

  if ((millis() - userButton.lastDebounceTime) > userButton.debounceDelay) {
    if (userButton.lastSteadyState && !userButton.currentState) {
      flag = true;
      return flag;
    } else if (!userButton.lastSteadyState && userButton.currentState) {
      flag = false;
      return flag;
    }
    userButton.lastSteadyState = userButton.currentState;
  }
  state = flag;
  return state;
}

bool calibrateUss(){
  
  if(!sensor.ultrasound.upper.calibrated){
    sensor.ultrasound.upper.proximityDistance = measureDistance(sensor.ultrasound.upper.triggerPin, sensor.ultrasound.upper.echoPin);
    Serial.println(sensor.ultrasound.upper.proximityDistance);
    sensor.ultrasound.upper.calibrated = true;
  }
  if(!sensor.ultrasound.lower.calibrated){
    sensor.ultrasound.lower.proximityDistance = measureDistance(sensor.ultrasound.lower.triggerPin, sensor.ultrasound.lower.echoPin);
    Serial.println(sensor.ultrasound.lower.proximityDistance);
    sensor.ultrasound.lower.calibrated = true;
  }

  if(sensor.ultrasound.upper.calibrated && sensor.ultrasound.lower.calibrated){
    return true;
  } else return false;
}



void ultrasoundStartSignal(uint8_t sensorTriggerPin) {
  digitalWrite(sensorTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTriggerPin, LOW);
}

uint16_t measureDistance(uint8_t sensorTriggerPin, uint8_t sensorEchoPin) {
  uint32_t echo_duration;
  uint16_t echo_distance;
  uint8_t sens_epin = sensorEchoPin;
  uint8_t sens_trpin = sensorTriggerPin;

  ultrasoundStartSignal(sens_trpin);
  echo_duration = pulseIn(sens_epin, HIGH);
  echo_distance = echo_duration * SOUND_SPEED / 2;

  return echo_distance;
}

bool proximitySensing(uint8_t sensorTriggerPin, uint8_t sensorEchoPin,
                      uint8_t maximalDistance) {
  bool flag;
  uint8_t sens_epin = sensorEchoPin;
  uint8_t sens_trpin = sensorTriggerPin;
  uint8_t max_dist = maximalDistance;

  if (measureDistance(sens_trpin, sens_epin) < max_dist) {
    flag = true;
  } else if (measureDistance(sens_trpin, sens_epin) >= max_dist) {
    flag = false;
  }
  return flag;
}

void blinkingSignal(uint8_t ledPin, uint16_t value, uint8_t timeUnit) {
  
  digitalWrite(ledPin, HIGH);
  timeToWait(value, timeUnit);
  digitalWrite(ledPin, LOW);
  timeToWait(value, timeUnit);
}

uint8_t checkProxySensors() {
  uint8_t S1, S2, sum, info;
  S1 = proximitySensing(sensor.ultrasound.upper.triggerPin,
                        sensor.ultrasound.upper.echoPin, sensor.ultrasound.upper.proximityDistance);
  S2 = proximitySensing(sensor.ultrasound.lower.triggerPin,
                        sensor.ultrasound.lower.echoPin, sensor.ultrasound.lower.proximityDistance);

  sum = S1 + S2;

  switch (sum) {
    case 0: info = 0; break;
    case 1: if (S1) { info = 1; } else if (S2) { info = 3; } break;
    case 2: info = 2; break;
    default:
      Serial.println("shit happens mon");
  }
  return info;
}

bool errorHandler() { // only returns if button pressed for a least three second
  
  while (sensor.cover.state || shredding.failFlag) {
    digitalWrite(MOTOR_SIGNAL_PIN, LOW);
    
    bool distress_call = false;
    uint64_t previous_millis = millis();
    uint64_t distress_call_interval = 3 * ONE_SECOND;

    Serial.println("Error blinking");
    blinkingSignal(LED_BUILTIN, 500, millisecond);

    if (checkProxySensors() == upper){
      while (userButton.state) {
      uint64_t current_millis = millis();
        if ((uint64_t)(current_millis - previous_millis) > distress_call_interval) {
          shredding.processFlag = false;
          shredding.failFlag = false;
          distress_call = true;
          return distress_call;
        }
      }
    } else if (checkProxySensors() == empty ){
      shredding.failFlag = false;
      while (userButton.state) {
      uint64_t current_millis = millis();
        if ((uint64_t)(current_millis - previous_millis) > distress_call_interval) {
          shredding.processFlag = false;
          shredding.failFlag = false;
          distress_call = true;
          return distress_call;
        }
      }
      return false;
    }
  }
}




int32_t measureMotorCurrent(){
  // measuring algorithm for custom sensor etc..
  // now its emulated with pushbuttons
  return sensor.ammeter.current;
}


void motorStart(bool direction){
  motorRotationSwitch(direction);
  timeToWait(MOTOR_SAFETY_DELAY, millisecond);
  digitalWrite(MOTOR_SIGNAL_PIN, HIGH);  
}

void motorStop(uint8_t stopDelay){
  digitalWrite(MOTOR_SIGNAL_PIN, LOW);
  timeToWait(MOTOR_SAFETY_DELAY, millisecond);
  digitalWrite(MOTOR_ROTATE_LEFT_PIN, LOW);
  digitalWrite(MOTOR_ROTATE_RIGHT_PIN, LOW);
  timeToWait(stopDelay, second);

}

void motorPhaseOne(){
  motorStart(right);
  timeToWait(MOTOR_RUNNING_TIME, second);
                
  if(checkProxySensors() != empty){    
  return;
  } else Serial.println("Nothing is inside mon"); 
  
  motorStop(MOTOR_STOPPING_TIME);
  shredding.processFlag = false;

}

void motorPhaseTwo(){
  motorRotationSwitch(right);
  blinkingSignal(LED_BUILTIN,500, millisecond);
  motorStart(right);
  timeToWait(5, second);

  if (checkProxySensors()==upper) {
    shredding.failFlag = true;
    motorStop(MOTOR_STOPPING_TIME);
  }
}

void motorPhaseThree(){
  digitalWrite(LED_BUILTIN, HIGH);  

  if(sensor.ammeter.current < MOTOR_MAXIMAL_CURRENT){
    motorStart(right);
  } else {
    for(uint8_t tries = 0; tries < 5; tries++){
        for(uint8_t tries = 0; tries < 3; tries++){
                      motorStop(MOTOR_STOPPING_TIME);               
                      motorStart(left);
                      timeToWait(MOTOR_RUNNING_TIME, second);
                      motorStop(MOTOR_STOPPING_TIME);                            
                      motorStart(right);
                      timeToWait(MOTOR_RUNNING_TIME, second);
                      if(checkProxySensors() == empty){
                        return;
                      }             
        }
        motorStop(60);               
      }
    if(checkProxySensors() == lower || checkProxySensors() == both){
      shredding.failFlag = true;
    }
    sensor.ammeter.current = 10000; // only for debugging
  } 
}

void motorPhaseFour(){
  if(sensor.ammeter.current < MOTOR_MAXIMAL_CURRENT){
    motorStart(right);
    digitalWrite(LED_BUILTIN, LOW);  
    timeToWait(5, second);
  } else {     
              motorStop(MOTOR_STOPPING_TIME);               
              motorStart(left);
              timeToWait(MOTOR_RUNNING_TIME, second);
              motorStop(MOTOR_STOPPING_TIME);              
              motorStart(right);
              timeToWait(MOTOR_RUNNING_TIME, second);
          sensor.ammeter.current = 10000;
          } 
  
  //digitalWrite(MOTOR_SIGNAL_PIN, LOW);
  //shredding.processFlag = false;
  
}




void motorControllingProcess() {
  if (shredding.startFlag) {
    shredding.processFlag = true;
    

    while (shredding.processFlag && !sensor.cover.state) {
      if (sensor.cover.state || shredding.failFlag) {
        motorStop(MOTOR_STOPPING_TIME);
        shredding.processFlag = false;
        return;
      }      
      

      switch (sensor.ultrasound.currentState){
        case 0: Serial.println("Object not sensed in front of neither S1 nor S2");
                motorPhaseOne();                
                break;
        
        case 1: Serial.println("Object sensed in front of S1 but not in S2");
                motorPhaseTwo();               
                break;

        case 2: Serial.println("Object sensed in front of S1 and S2");
                motorPhaseThree();               
                break;
        case 3: Serial.println("Object sensed in front of S2 and not in S1");
                motorPhaseFour(); 

                break;
        default: Serial.println ("I shouldn`t print this mon.");

      }      
    }
  } else
    digitalWrite(MOTOR_SIGNAL_PIN, LOW);
  shredding.failFlag = false;
}





void measureTaskCode(void * parameter){
  Serial.println("Measure Task is running on: ");
  Serial.print(xPortGetCoreID());

  for(;;){

    userButton.state = buttonState(USERBUTTON_PIN);
    sensor.ammeter.current = measureMotorCurrent();
    emulateMotorVoltage();
    sensor.ultrasound.currentState = checkProxySensors();
    sensor.cover.state = digitalRead(COVERBUTTON_PIN);
    vTaskDelay(10);
    
  }
  
}

void controlTaskCode(void * parameter){
  Serial.println("Control Task is running on: ");
  Serial.print(xPortGetCoreID());

  for(;;){

  if (userButton.state) {
    shredding.startFlag = true;
  } else
    shredding.startFlag = false;

  if (errorHandler()) {
    shredding.locked = true;
    Serial.println("distress call");
    while (shredding.locked) {
      blinkingSignal(LED_BUILTIN, 1, second);
    }
  }

  if (!shredding.locked) {  
    motorControllingProcess();
  }
  vTaskDelay(10);
    
  }
}



void setup(){

  Serial.begin(115000);
  configureIO();
  digitalWrite(LED_BUILTIN, LOW);  
  

  xTaskCreatePinnedToCore(
          measureTaskCode, // fn to implement the task
          "Measure Task", // Task name
          10000, // stack size in words
          NULL, // task input param
          1, // task priority
          &measureTask, // task`s handle
          0); // core id
    delay(500);

  xTaskCreatePinnedToCore(
          controlTaskCode, // fn to implement the task
          "Control Task", // Task name
          10000, // stack size in words
          NULL, // task input param
          1, // task priority
          &controlTask, // task`s handle
          1); // core id
    delay(500);



}

void loop() { 
  vTaskDelete(NULL);

/*
  
  if (userButton.state) {
    shredding.startFlag = true;
  } else
    shredding.startFlag = false;

  if (errorHandler()) {
    shredding.locked = true;
    Serial.println("distress call");
    while (shredding.locked) {
      blinkingSignal(LED_BUILTIN, 1, second);
    }
  }

  if (!shredding.locked) {  
    motorControllingProcess();
  }    
  */
}
