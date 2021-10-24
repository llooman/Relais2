/*   Relais.h

	Relais( int pin, bool inverted, bool onStartOn  )
  
	This utility can:
	x switch ON/OFF on a reference value
	- call a check function to control on/off
	  - onCheck( int (*function)(bool on, bool isRunning )

	x Debounce by defining a switch on off value: Relais.setSwitchValue(10.5, 9.5);
	- Debounce by minimal on off periode: Relais.setDebounce_s(10, 15);
	  - setDebounce_s	( int on, int off );

	- DutyCycle when on in % having debounce values for on/off 
	- Auto ON: i.e.: for one minute once every 24 hours: Relais.setAutoPeriode_s(86400=1 day);
	- invert output:  on=false/LOW, off=true/HIGH
	- put to sleep for x seconds
	- switch on manual for x seconds

	- if dutyCycle >= 5 of <= 95 then active above debounce

    Implement by:

	Relais(int pin, <bool inverted>)
	add in you main.loop():
	- Relais.loop()
	- Relais.checkValue(float sollValue)
	or
	- Relais.loop(checkValue)

    Relais.inverted => isRunning()|ist() versus high/low

    you can prevent flipper/klepper:
	- setting Switchvalues - hysteresis
	- setting debounce timers in seconds.

	Duty and Debounce => high/low is not always ist()
    - setSwitchValue	( float on, float off );
    - setDebounce_s	( int on, int off );  setDebounce_s	( int val );


	- setAutoPeriode_s( long autoActivatePeriode );
    - setDutyPerc    ( int perc);  // depending on the debounce timings !!!
    - setDutyTimer	( bool val ); ??

	!! return eeParms.klepDuty>0 ? eeParms.klepDuty: 1;

//		 val     inverted
//		true	 true       val == !inverted = false
//		true	 false	    val == !inverted = true
//		false    true       val == !inverted = true
//		false    false      val == !inverted = false

2020-09-09 if(!sayHello) 

2020-08-20 bug fixes 
           add force

2020-06-05 major refactor myTimers / dutyCycle / onCheck

2019-05-10 add dutyCycle
2019-03-24 add .cpp
           setSwitchValue(on off)
           setDebounce_s(on, off)

2017-01-10 sendDataWhenAvailable retry
2015-02-06 v1.02 LLO add _switchOn/OffValue
2015-01-18 v1.01 LLO add timeStamp
2013-10-02 v1.0  Copyright (c) 2013-2014 J&L Computing.  All right reserved.
*/

#ifndef Relais2_h
#define Relais2_h

#include <Arduino.h>
#include <inttypes.h>

#define RELAIS_TIMER_COUNT 8
#define RELAIS_TIMER_AUTO 0
#define RELAIS_TIMER_DEBOUNCE 1
#define RELAIS_TIMER_UPLOAD_STATE 2
#define RELAIS_TIMER_UPLOAD_ISRUNNING 3
#define RELAIS_TIMER_UPLOAD_MANUAL 4
#define RELAIS_TIMER_MANUAL_OFF 5
#define RELAIS_TIMER_MANUAL_ON 6
#define RELAIS_TIMER_DUTY 7

// #define CALC_FREQUENCY

class Relais2
{
  public:
 	unsigned long 	timers[RELAIS_TIMER_COUNT];
 	// bool            timersActive[RELAIS_TIMER_COUNT];

	int 			id = 0;
	bool			_on;
	// bool			toggle = false;
	bool	 		inverted = false;		// verwissel aan/uit: (false: on when value > switchValue,  true: on when value < switchValue)
	unsigned long 	timeStamp = 0;			// latest change
	bool			sayHello = false;		// active on start for 3 seconds 
	bool 			testFlag  = false;		// show led 


	bool 			dutyCycleMode = false;
	int 			dutyCycle;
	int 			dutyOn_s;
	int 			dutyOff_s;
	// int				offDelay_min;
	int 			debounceOn_s;
	int 			debounceOff_s;

	int 			numberOfStates = 5;	
	int 			state = 0;
	int 			stateUploaded = -1;

	long 			manualTimer_millis = 0;
	unsigned long 	prevManualSecond = 0;
	int 			manualMinutesUploaded = 0;
	bool 			force = false;				// skip delays act immediatly

	int 			uploadInterval_s = 60;   // default sence every 60 sec

	long   			_autoActivatePeriode_s ;    // activate every sec  default off = 0

	int				frequency;

    Relais2( int pin )
    {
        this->inverted = false;
    	init( pin, false);
    }
    Relais2( int pin, bool inverted )
    {
    	this->inverted = inverted;

    	init( pin, false);
    }

    Relais2( int pin, bool inverted, bool onStartOn  )
    {
    	this->inverted = inverted;

    	init( pin, onStartOn);
    }

	void init( int pin, bool onStartOn )
	{
		_pin 				= pin;				// store the pin
		_on					= onStartOn;
 
		timeStamp 			= 0;
		dutyCycle 			= onStartOn?100:0;
		sayHello 			= false;	
		manualTimer_millis	= 0;
		force 				= false;

		debounceOn_s		= 0;
		debounceOff_s		= 0;
		dutyOn_s			= 0;
		dutyOff_s			= 0;
		// offDelay_min        = 0;

		_autoActivatePeriode_s = 0;				// disable autoActivate by default

		#ifdef CALC_FREQUENCY
			frequency 			= 0;								// switches per hour
			for ( int i = 0
					; i < ( sizeof( relaisChanges )/sizeof( unsigned long ))
					; i++
			) relaisChanges[i] = 0;
		#endif

		pinMode(_pin, OUTPUT);
		digitalWrite(_pin, _on == !inverted );
		 
		initTimers(RELAIS_TIMER_COUNT);
	}


	/*  -1: unchanged
	    0: switch off
	    1: switch on
		>1: duty cycle   < 5 off >= 90 on
	*/
	int (*checkFunc) (bool on, bool isRunning ) = 0;			
    void onCheck( int (*function)(bool on, bool isRunning ) )
    {
    	checkFunc = function;
    }

	int (*uploadFunc) (int id, long val, unsigned long timeStampDebounce) = 0;
    void onUpload( int (*function)(int id, long val, unsigned long timeStampDebounce) )
    {
    	uploadFunc = function;
    }
    void onUpload( int (*function)(int id, long val, unsigned long timeStampDebounce), int id )
    {
    	this->id = id;
    	uploadFunc = function;
    }
 
	void 	uploadState(void); 
	void 	uploadIsRunning(void); 
	void 	uploadManual(void); 
	 

    boolean 	loop();                       // process basic checks ie. timers/status changes

    void 	setManual	( int minutes ); 	// >0: On seconds, <0: Sleep seconds
	long 	manualTimeLeft();
    void	calcState(void);
    void 	setDebounce_s	( int val ); 	// set debounce time in sec.
    void 	setDebounce_s	( int on, int off );
	void  	adjustTimers (void);

    void  	setDutyPerc		( int perc);
	void 	pause           ( int min);
	void 	setDutyPerc		( int perc, int debounce_s) ;
	void 	setDutyPerc		( int perc, int debounce_on_s, int debounce_off_s) ;

	void 	setAutoPeriode_s( long autoActivatePeriode );

	void 	resetDebounce(){ timerOff(RELAIS_TIMER_DEBOUNCE); force = true;}
	void 	resetManual(){ 
				timerOff(RELAIS_TIMER_MANUAL_ON );	
				timerOff(RELAIS_TIMER_MANUAL_OFF );	
			}

	#ifdef CALC_FREQUENCY
    	int		getSwitchFrequency();
	#endif

	boolean		isOn();	
	boolean		value();
	boolean		isRunning();			// absolute/physical value
	boolean		isStopped();
	
	void		on();
	void		off();
	void        stop();
	void        restart();
	void        panic();
	void		test(bool aan);

	// long 		period();						// period last value in sec.

	void 		trace(char desc[]);

	void initTimers(int count);
	bool isTime( int id);
	bool isTimerActive( int id );
	bool isTimerInactive( int id );
	void nextTimer( int id ){ nextTimerMillis(id, 60000L);}
	void nextTimer( int id, int periode_s){ nextTimerMillis(id, periode_s * 1000L );}	
	void nextTimerMillis( int id, unsigned long periode);
	void timerOff( int id );
	unsigned long timeLeft(int id);


  private:  
	#ifdef CALC_FREQUENCY
    	unsigned long 		relaisChanges[10];			// track timestamps of changing active
	#endif
	void	 	setPin( bool newState );
	int			_pin;						// digital pin
};

#endif