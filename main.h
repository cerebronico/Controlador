#include <30F3013.h>
#device ICD=TRUE 

#fuses WDT        	// Watch Dog Timer
#fuses WPSA512    	// Watch Dog Timer PreScalar A 1:512
#fuses WPSB16     	// Watch Dog Timer PreScalar B 1:1
#fuses NOBROWNOUT 	// No brownout reset
#fuses XT_PLL8      // XT Crystal Oscillator mode with 16X PLL

#use delay(clock=80MHz, crystal=10MHz, restart_wdt)

// Weighing Sensor Pins
#define WS_DAT_2	input(PIN_D8)	// pin 15
#define WS_DAT_1	input(PIN_D9)	// pin 14
#define WS_CLK_2	PIN_F4        	// pin 22
#define WS_CLK_1	PIN_F5			// pin 21                         

//   inputs
#define DUMP_GATE_LS	!input(PIN_B6)   // all inputs are normally pulled high
#define FILL_GATE_LS	!input(PIN_B7)
#define OVERFLOW		!input(PIN_B8)
#define PRESS_OK		!input(PIN_B9)	// pin 23_
#define FULL			!input(PIN_F6)	// pin 16 INT0

#define DUMP_GATE_LS_FLAG	0
#define FILL_GATE_LS_FLAG	1
#define OVERFLOW_FLAG	2
#define PRESS_OK_FLAG	3
#define FULL_FLAG	4

//  Outputs
#define FILL_ON		output_high(PIN_B0)	
#define FILL_OFF	output_low(PIN_B0)
                       
#define DUMP_ON		output_high(PIN_B1)
#define DUMP_OFF	output_low(PIN_B1)	

#define CAL_ON		output_high(PIN_B2)
#define CAL_OFF		output_low(PIN_B2)

#define ALARM_ON	output_high(PIN_B3)
#define ALARM_OFF	output_low(PIN_B3)

#define BEAT		output_toggle(PIN_B4)

#define TP1			PIN_B5

#use rs232(UART1A, baud=115200, restart_wdt, errors, stream=Host)

#use timer(timer=2, tick=1ms, bits=32, NOISR)

#define TICK_TYPE unsigned int32
                                             
#define ESPERA 100
                                    
//#use rtos(timer=1, minor_cycle=2ms, statistics)
#build(stack=512)

#define _ON true
#define _OFF false


