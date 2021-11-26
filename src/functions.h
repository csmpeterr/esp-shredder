#include <Arduino.h>

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

void measureTaskCode(void * parameter);

void IRAM_ATTR Ext_INT1_ISR();


void configureIO();
bool timeToWait(uint64_t value, uint8_t timeUnit);
bool buttonState(uint8_t buttonPin);
void ultrasoundStartSignal(uint8_t sensorTriggerPin);
uint16_t measureDistance(uint8_t sensorTriggerPin, uint8_t sensorEchoPin);
bool proximitySensing(uint8_t sensorTriggerPin, uint8_t sensorEchoPin, uint8_t minimalDistance);
void blinkingSignal(uint8_t ledPin, uint16_t value, uint8_t timeUnit);
uint8_t checkProxySensors();
bool errorHandler();

bool calibrateUss();

bool motorRotationSwitch(bool rotationDirection);

int32_t measureMotorCurrent();
int32_t emulateMotorVoltage();

void motorStart(bool direction);
void motorStop(uint8_t stopDelay);
void motorPhaseOne();
void motorPhaseTwo();
void motorPhaseThree();
void motorPhaseFour();
void motorControllingProcess();
