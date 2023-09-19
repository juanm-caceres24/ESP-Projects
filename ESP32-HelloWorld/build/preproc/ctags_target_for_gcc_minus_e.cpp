# 1 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"
const int ledPin = 2;
hw_timer_t *timer = 
# 2 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino" 3 4
                   __null
# 2 "D:\\Documents\\GitHub\\ESP32-Examples\\ESP32-HelloWorld\\ESP32-HelloWorld.ino"
                       ;
portMUX_TYPE timerMux = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;
volatile bool timerFlag = false;
const int timerInterval = 1000;

void __attribute__((section(".iram1" "." "28"))) onTimer() {
 vPortEnterCritical(&timerMux);
 timerFlag = true;
 vPortExitCritical(&timerMux);
}

void setup() {
 pinMode(ledPin, 0x03);
 timer = timerBegin(0, 80, true);
 timerAttachInterrupt(timer, &onTimer, true);
 timerAlarmWrite(timer, timerInterval * 1000, true);
 timerAlarmEnable(timer);
}

void loop() {
 if (timerFlag) {
  digitalWrite(ledPin, !digitalRead(ledPin));
  vPortEnterCritical(&timerMux);
  timerFlag = false;
  vPortExitCritical(&timerMux);
   }
}
