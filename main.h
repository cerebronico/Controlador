#include <30F3013.h>
#device ICD=TRUE 

#fuses WDT           // Watch Dog Timer
#fuses WPSA512       // Watch Dog Timer PreScalar A 1:512
#fuses WPSB16        // Watch Dog Timer PreScalar B 1:1
#fuses NOBROWNOUT    // No brownout reset
//#fuses XT_PLL8      // XT Crystal Oscillator mode with 16X PLL

#use delay(clock=80MHz, crystal=10MHz, restart_wdt)

// Weighing Sensor Pins
#define WS_DAT   input(PIN_F6)   // pin 15
#define WS_CLK   PIN_F5           // pin 22

// inputs, all inputs are normally pulled high

#define IN0 !input(PIN_B8)
#define IN1 !input(PIN_D8)   // pin 15
#define IN2 !input(PIN_D9)   // empacadora lista

// address pins

#define O0  PIN_B6
#define O1  PIN_B7

// status flags

#define RDY_2_DUMP  0
#define DUMP_DONE   1
#define ZERO   2
#define OVER   3
#define MOTION 4

//  Outputs
#define FAST_ON   output_high(PIN_B4)   
#define FAST_OFF  output_low(PIN_B4)
                       
#define SLOW_ON   output_high(PIN_B3)
#define SLOW_OFF  output_low(PIN_B3)   

#define DUMP_ON   output_high(PIN_B2)
#define DUMP_OFF  output_low(PIN_B2)

#define OUT3_ON   output_high(PIN_B5)
#define OUT3_OFF  output_low(PIN_B5)

#define DONE_ON   output_high(PIN_B1)
#define DONE_OFF  output_low(PIN_B1)

#define VIB_GRUESO_ON   output_high(PIN_B0)
#define VIB_GRUESO_OFF  output_low(PIN_B0)

#define BEAT      output_toggle(PIN_F4)

#use rs232(UART1A, baud=115200, restart_wdt, errors, ENABLE=PIN_B9, stream=HMI)
///#use rs232(ICD)

#use timer(timer=2, tick=1ms, bits=32, NOISR)

#define TICK_TYPE unsigned int32
                                             
#define ESPERA 100
                                    
#build(stack=512)

#define _ON true
#define _OFF false


