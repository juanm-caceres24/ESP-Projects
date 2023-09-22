#define LED_PIN 2

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool timerFlag = false;
const int timerInterval = 1000;

void IRAM_ATTR onTimer( ) {
	portENTER_CRITICAL_ISR( &timerMux );
	timerFlag = true;
	portEXIT_CRITICAL_ISR( &timerMux );
}

void setup( ) {
	pinMode( LED_PIN, OUTPUT );
	timer = timerBegin( 0, 80, true );
	timerAttachInterrupt( timer, &onTimer, true );
	timerAlarmWrite( timer, timerInterval * 1000, true );
	timerAlarmEnable( timer );
}

void loop( ) {
	if ( timerFlag ) {
		digitalWrite( LED_PIN, !digitalRead( LED_PIN ));
		portENTER_CRITICAL( &timerMux );
		timerFlag = false;
		portEXIT_CRITICAL( &timerMux );
  	}
}
