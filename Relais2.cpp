
#include "Relais2.h"

// #define DEBUG

/*  newState:
	-1 		unchanged
	0		off
	1 		on
	5-95 	dutyCycle

	RELAIS_TIMER_DEBOUNCE is used for debounce and dutyCycle

	dutyCycle > 95-100 > _on=true, dutyCycle=0
	dutyCycle < 0-5    > _on=false, dutyCycle=0

	states indicators:
	isRunning()   	actual (!inverted)
	isStopped()		actual (!inverted)
	_on, isOn() 	functional (!inverted)

	when force		will  overrule manual
	when manualTimer_millis != 0  overwide normal logic			!!! upgrade to timer class !!!!
	when dutyCycle 5-95  RELAIS_TIMER_DEBOUNCE toggles between dutyOn_s - dutyOff_s
	when offDelay_min   off will be delayed


	ID ^ isrunning
	ID+1 ^ manual
	ID+2 ^ uploadDuty ????
*/


boolean Relais2::loop( )
{
	// bool manualStopped = false;
	int newState = -1;				// 1=switch on, 0=switch off, -1=unchanged

	if(checkFunc!=0){ 
		newState = checkFunc( _on, isRunning() );  // digitalRead(_pin)==!inverted
	}
	
	if( isTime(RELAIS_TIMER_UPLOAD_ISRUNNING)){ uploadIsRunning(); }
	else if( isTime(RELAIS_TIMER_UPLOAD_STATE)){ uploadState(); }
	else if( isTime(RELAIS_TIMER_UPLOAD_MANUAL)){ uploadManual(); }

	if(sayHello){	
		if(millis() >= 3000L){
			sayHello = false;
			force = true;
		}
	}



	if(!sayHello || force){

		// normal loop 
		// TODO test dutyCyvle 	 

		if(! dutyCycleMode && newState > 0){	// prevent confusion when not in dutyCycleMode
			newState = 1;
		}

		if(newState < 0){
			// nop

		} else if( !dutyCycleMode ){

			_on = newState > 0;

		} else {

			if( newState != dutyCycle
			 || force
			){
				setDutyPerc(newState );
			} 
		}		 
	}


	// reset manual when force 
	if(force){
		if( isTimerActive(RELAIS_TIMER_MANUAL_ON)
		 || isTimerActive(RELAIS_TIMER_MANUAL_OFF)
		){
			timerOff(RELAIS_TIMER_MANUAL_ON );	
			timerOff(RELAIS_TIMER_MANUAL_OFF );	
		}
	}

	if(isTimerActive(RELAIS_TIMER_MANUAL_ON)) {

		if(isTime(RELAIS_TIMER_MANUAL_ON)){
			timerOff(RELAIS_TIMER_MANUAL_ON ); 
		} else {
			_on = true;
		}

		// prevent both manual_on and manual_off are active
		timerOff(RELAIS_TIMER_MANUAL_OFF );			

	}

	if(isTimerActive(RELAIS_TIMER_MANUAL_OFF)) { 

		if(isTime(RELAIS_TIMER_MANUAL_OFF)){
			timerOff(RELAIS_TIMER_MANUAL_OFF);
		} else { 
			_on = false; 
		}

		// prevent both manual_on and manual_off are active
		timerOff(RELAIS_TIMER_MANUAL_ON );			

	}
 

	calcState();

	/*
		update digital port when needed.
	*/
	if( sayHello && !force ){

		if( ! isRunning() )  setPin(true);


	} else if( isTimerActive(RELAIS_TIMER_DUTY) 										
			&& ! force
		){

		if( isTime(RELAIS_TIMER_DUTY) 
		){
			// Serial.print("switchDuty to:"); Serial.println(! isRunning());
			setPin(! isRunning()); 

		}		

	} else if( isRunning() != _on) {

		if( isTimerInactive(RELAIS_TIMER_DEBOUNCE)
		 ||	isTime(RELAIS_TIMER_DEBOUNCE)
		 || force  
		){
		 	setPin(_on); 	
		}
	}
 
	#ifdef CALC_FREQUENCY
		frequency = getSwitchFrequency();			// calc the switching frequence per hour
	#endif

	force = false;	
	return _on;
}

void Relais2::calcState(void){
 
	state = _on ? 1:0;

	if( dutyCycleMode
	//  && _on
	){
		int part = 100 / numberOfStates;
		int parts = dutyCycle / part;

		// afronden 
		if( dutyCycle % part >= ( part / 2 ) )
		{
			parts++;
		}
		state = parts * part; 
	}

	if(state<0) state = 0;
	if(state>100) state = 100;

	if( state != stateUploaded) { 
		nextTimer(RELAIS_TIMER_UPLOAD_STATE, 0);
		// Serial.print("calcState"); Serial.println(state);
	 }
}


void Relais2::setPin( bool newState  )
{
	digitalWrite(_pin, newState == !inverted );

	if(testFlag)
		digitalWrite(LED_BUILTIN, newState == !inverted );

    timeStamp = millis();

	nextTimer(RELAIS_TIMER_UPLOAD_ISRUNNING, 0);

	// when autoActivate enabled always refresh autoTimer 
	if(_autoActivatePeriode_s>0)nextTimer(RELAIS_TIMER_AUTO, _autoActivatePeriode_s); 	 

	adjustTimers();

	// #ifdef CALC_FREQUENCY
	// 	// remember last 10 swith timeStamps
	// 	for ( int i = ( sizeof( relaisChanges ) / sizeof( unsigned long ) ) - 1
	// 			; i > 0
	// 			; i--
	// 	)	relaisChanges[i] = relaisChanges[i-1];

	// 	relaisChanges[0] = millis();				// add current switch to the list
	// #endif

	return ;
}


void Relais2::setDutyPerc(int perc, int debounce_s) { setDutyPerc(perc, debounce_s, debounce_s);}
void Relais2::setDutyPerc(int perc, int debounce_on_s, int debounce_off_s)  
{
	this->debounceOn_s = debounce_on_s;
	this->debounceOff_s = debounce_off_s;
	setDutyPerc(perc);
}
void Relais2::setDutyPerc(int perc )  // 0-100
{
	dutyCycle = perc;
	calcState();



	if( state >= 100 ){

		_on = true;

	} else if( state<=0 ){

		_on = false;

	} else {

		_on=true;
		dutyOn_s = debounceOn_s;
		dutyOff_s = debounceOff_s;
		// adept duty on debounce=0 
		if(dutyOn_s < 1) dutyOn_s = dutyOff_s;
		if(dutyOn_s < 1) dutyOn_s = 10;
		if(dutyOff_s < 1) dutyOff_s = dutyOn_s;	 

		int newDutyOn = dutyOn_s;
		int newDutyOff = dutyOff_s;

		if(state<50) {
			// Serial.print("calc state:"); Serial.print( state); Serial.print(" dutyCycle:"); Serial.println( dutyCycle);
			// dutyOn_s = debounceOn_s;
			newDutyOff = dutyOn_s * (100 - state) / state;

			// off to short
			if(newDutyOff < dutyOff_s)
			{
				newDutyOff = dutyOff_s;
				newDutyOn = newDutyOff * state / 100; 
				// newDutyOn = dutyOn_s * dutyOff_s / newDutyOff;
			}

		} else {

			// dutyOff_s = debounceOff_s;
			newDutyOn = dutyOff_s * state / ( 100 - state);

			// if debounceOff must < debounceOn			
			if(newDutyOn < dutyOn_s)
			{
				newDutyOn = dutyOn_s;
				newDutyOff = newDutyOn * (100 - state) / 100; 
				// newDutyOff = dutyOff_s * dutyOn_s / newDutyOn;
			}
			// Serial.print("state:"); Serial.print( state); Serial.print(" dutyCycle:"); Serial.print( dutyCycle);
			// Serial.print("Off_s:"); Serial.print( dutyOff_s); Serial.print(" On_s:"); Serial.println( dutyOn_s);
		}
		dutyOff_s = newDutyOff;
		dutyOn_s =  newDutyOn;		

	}

	// if(isTimerActive(RELAIS_TIMER_DEBOUNCE)  
	// ){
		adjustTimers();
	// }

	#ifdef DEBUG
		// Serial.print(F("setDuty:")); Serial.print(dutyOn_s);Serial.print(F("-"));Serial.println(dutyOff_s);
	#endif
}

// 
void Relais2::adjustTimers(){

	long allreadyInCurrentState_ms = (millis()>timeStamp ? millis()-timeStamp : timeStamp-millis());

	if( isTimerActive(RELAIS_TIMER_DEBOUNCE)
	 || debounceOn_s > 0
	 || debounceOff_s > 0
	){

		long remaining_ms = ( 1000L * (isRunning()?debounceOn_s:debounceOff_s) ) - allreadyInCurrentState_ms;

		if(remaining_ms>0){

			nextTimerMillis(RELAIS_TIMER_DEBOUNCE, remaining_ms);
		} else {

			nextTimer(RELAIS_TIMER_DEBOUNCE, 0);
		}
	}  

	if( dutyCycleMode
	 && state > 0
	 && state < 100
	){


		// Serial.print("adjustDuty:"); Serial.print(dutyOn_s);Serial.print("-"); Serial.println(dutyOff_s);

		long remaining_ms = ( 1000L * (isRunning()?dutyOn_s:dutyOff_s) ) - allreadyInCurrentState_ms;

		// Serial.print("adjustDuty:"); Serial.print(remaining_ms);Serial.print("-"); Serial.println(allreadyInCurrentState_ms);
		if(remaining_ms>0){

			nextTimerMillis(RELAIS_TIMER_DUTY, remaining_ms);
		} else {

			nextTimer(RELAIS_TIMER_DUTY, 0);
		}

	} else {
		timerOff(RELAIS_TIMER_DUTY);
	}

	return;
    // old
	long delay_s = dutyCycleMode ? isRunning()?dutyOn_s:dutyOff_s : isRunning()?debounceOn_s:debounceOff_s;

	long remaining_ms = ( delay_s * 1000 ) - allreadyInCurrentState_ms;

	// if( millis()>500
	// ){	
	if(remaining_ms>0){

		nextTimerMillis(RELAIS_TIMER_DEBOUNCE, remaining_ms);
	} else {

		nextTimer(RELAIS_TIMER_DEBOUNCE, 0);
	}
	// }

	// Serial.print("adjustTimers current_ms:"); Serial.print( allreadyInCurrentState_ms); Serial.print(" delay_s:"); Serial.println( delay_s);
	
}


void Relais2::setDebounce_s( int val) { setDebounce_s(val, val); }
void Relais2::setDebounce_s( int on_s, int off_s )
{
	debounceOn_s = on_s;
	debounceOff_s = off_s;

	if(dutyCycleMode){
		setDutyPerc(dutyCycle);
	} else {
		adjustTimers();
	}

}

void Relais2::uploadIsRunning()
{
	nextTimer(RELAIS_TIMER_UPLOAD_ISRUNNING);

	if(uploadFunc!=0   )
	{
		uploadFunc(id, isRunning(), millis() );		 
	}  

}

void Relais2::uploadState()
{
	nextTimer(RELAIS_TIMER_UPLOAD_STATE); 

	if(uploadFunc!=0   )
	{
		uploadFunc(id+1, state, millis() );	
	}  
	stateUploaded = state;
}


void Relais2::uploadManual()
{

	nextTimer(RELAIS_TIMER_UPLOAD_MANUAL); 

	if(uploadFunc!=0   )
	{
		long secondsLeft = manualTimeLeft() / 1000;
		bool off = secondsLeft < 0;
		long minutesLeft = 0;
		if(off){
			minutesLeft = (secondsLeft - 59) / 60;
		} else {
			minutesLeft = (secondsLeft + 59) / 60;
		}

		uploadFunc(id+2, minutesLeft , millis() );  // afronden naar boven 2>2, 1.9>2, 1.1>2,  1.0>1

		// Serial.print("uplMan:"); Serial.println(minutesLeft );
		manualMinutesUploaded = minutesLeft;
	}
}

long Relais2::manualTimeLeft(){

	if(isTimerActive(RELAIS_TIMER_MANUAL_OFF)){

		 return - (long) timeLeft(RELAIS_TIMER_MANUAL_OFF);
	}

	if(isTimerActive(RELAIS_TIMER_MANUAL_ON)) {

		return (long) timeLeft(RELAIS_TIMER_MANUAL_ON) ;
	}

	return 0;
}




void Relais2::setManual( int minutes ){

	// -50 >  60 > 50
	//  50 >  -60 < -50

	timerOff(RELAIS_TIMER_MANUAL_ON);
	timerOff(RELAIS_TIMER_MANUAL_OFF);


	if(minutes > 0){

		nextTimer(RELAIS_TIMER_MANUAL_ON, minutes * 60 );

	} else if(minutes < 0){
		nextTimer(RELAIS_TIMER_MANUAL_OFF, -minutes * 60);

	}  

	nextTimer( RELAIS_TIMER_UPLOAD_MANUAL, 0);
}

#ifdef CALC_FREQUENCY
	int	Relais2::getSwitchFrequency()
	{
		if ( relaisChanges[9] == 0 ) return 0;  // when less then 10 changes we don't report a frequency

		unsigned long timePassed_m = ( millis() - relaisChanges[9] ) / 60000 ;

		int freq  =	 60 / ( timePassed_m  / 5 );   // freq in changes per hour

		return freq;
	}
#endif

void Relais2::setAutoPeriode_s( long newPeriode )
{
	_autoActivatePeriode_s = newPeriode;
	
	if(_autoActivatePeriode_s <= 0){

		timerOff(RELAIS_TIMER_AUTO);
	} else {

		nextTimer(RELAIS_TIMER_AUTO, _autoActivatePeriode_s);
	} 
}

void  Relais2::stop()
{
	timerOff(RELAIS_TIMER_DEBOUNCE);
	loop();
}   

void  Relais2::restart()
{
	timerOff(RELAIS_TIMER_DEBOUNCE);
	// overrule = true;
	loop();
} 

void  Relais2::panic()
{
	timerOff(RELAIS_TIMER_DEBOUNCE);
	loop();
} 

boolean Relais2::isOn() { return _on; }   
boolean Relais2::isRunning() { return digitalRead(_pin)==!inverted; }   
boolean Relais2::isStopped() { return digitalRead(_pin)== inverted; }   


void  	Relais2::on() {   _on = true;  }	 // at least switch to on for one periode evaluate();
void  	Relais2::off() {  _on = false; }	 // at least switch to on for one periode evaluate();

// long Relais2::period()
// { 
// 	return  ( millis()>timeStamp?(millis() - timeStamp) / 1000 : (timeStamp - millis()) / 1000   );
// }  // period last value in sec.

void Relais2::trace(char*  id  )
{
	Serial.print(F("@"));
	Serial.print(millis()/1000);
	Serial.print(F(" "));
	Serial.print(id);
	Serial.print(F(": ")); Serial.print( _on?"On-":"Off-"); Serial.print(digitalRead(_pin)?"HIGH(":"LOW(");
	Serial.print(F("st:")); Serial.print( state); 
	Serial.print(F("), debounce:")); Serial.print(debounceOn_s);Serial.print(F("-")); Serial.print(debounceOff_s);
	Serial.print(F(" deb@"));     	Serial.print(timers[RELAIS_TIMER_DEBOUNCE]/1000);
	Serial.print(F(" duty@"));     	Serial.print(timers[RELAIS_TIMER_DUTY]/1000);
	Serial.print(F(", duty:"));   Serial.print(dutyCycle);  Serial.print(F(" "));  Serial.print(dutyOn_s);Serial.print(F("-")); Serial.print(dutyOff_s);	


	Serial.print(F(", inverted="));   	Serial.print( inverted);
	Serial.print(F(", auto_s="));   	Serial.print(_autoActivatePeriode_s);
 
	Serial.print(F(", isTime="));   	Serial.print( isTime(RELAIS_TIMER_DEBOUNCE));
	Serial.print(F(", manTmr="));   	Serial.print(  manualTimeLeft()   );
	Serial.print(F(", manOff@="));   	Serial.print(  timers[RELAIS_TIMER_MANUAL_OFF]/1000    );
	Serial.print(F(", manOn@="));   	Serial.print(  timers[RELAIS_TIMER_MANUAL_ON]/1000 );
	// Serial.print(F(", prev="));   			Serial.print( prevManualSecond);
 

	Serial.println();
}

void Relais2::pause(int min){

	// timerOff(RELAIS_TIMER_MANUAL_ON);
	// nextTimer(RELAIS_TIMER_MANUAL_OFF, min * 60);

	if( _on
	 && isTimerActive(RELAIS_TIMER_DEBOUNCE)
	 
	){
		timers[RELAIS_TIMER_DEBOUNCE] = timers[RELAIS_TIMER_DEBOUNCE] + min * 60000;
	} 
}

void Relais2::initTimers(int count)
{
	for(int i=0; i<count; i++){
		timers[i]=0L;
	}
	nextTimer(RELAIS_TIMER_UPLOAD_STATE, 1); 
	nextTimer(RELAIS_TIMER_UPLOAD_ISRUNNING, 2); 
	nextTimer(RELAIS_TIMER_UPLOAD_MANUAL, 3); 
}

unsigned long Relais2::timeLeft(int id){

	if(timers[id]==0){
		return 0;
	}

	return millis() > timers[id]? millis()-timers[id] : timers[id]-millis();
}

// bool Relais2::isTime( int id){

// 	unsigned long noww = millis();
 
// 	if(noww > timers[id]) return true;
// 	return (timers[id] - noww ) >  0x0fffffff;
// }
bool Relais2::isTime( int id){
	if(timers[id] == 0L) return false;

	unsigned long delta = millis() > timers[id] ? millis() - timers[id] : timers[id] - millis() ;
	return delta > 0x0fffffff ? false : millis() >= timers[id];
	// if( millis() > timers[id]) return true;
	// return (timers[id] -  millis() ) >  0x0fffffff;  
}


bool Relais2::isTimerActive( int id ){
	return timers[id] > 0;
}
bool Relais2::isTimerInactive( int id ){
	return timers[id] == 0;
}
void Relais2::timerOff( int id ){
	timers[id]=0;
}

void Relais2::nextTimerMillis( int id, unsigned long periode){

	if(periode<0) periode=0;
	timers[id] = millis() + periode;
	if(timers[id]==0) timers[id]=1;
}

void Relais2::test(bool aan){

	testFlag = aan;
	if(testFlag)
	{
		pinMode(LED_BUILTIN, OUTPUT);
		digitalWrite(LED_BUILTIN, digitalRead(_pin) );
	}  
}
