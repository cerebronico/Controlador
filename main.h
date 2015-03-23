#include <30F3013.h>

#fuses WDT            // Watch Dog Timer
#fuses WPSA512        // Watch Dog Timer PreScalar A 1:512
#fuses WPSB16          // Watch Dog Timer PreScalar B 1:1
#fuses NOBROWNOUT     // No brownout reset
#fuses XT_PLL8       // XT Crystal Oscillator mode with 16X PLL

#use delay(clock=80MHz, crystal=10MHz, restart_wdt)

// Weighing Sensor Pins
#define SDA         PIN_F2         // pin 18
#define SCL         PIN_F3         // pin 17
#define WS_CLK   PIN_F5         // pin 21
#define WS_DAT   input(PIN_F6)   // pin 16                         

//   inputs
#define START      !input(PIN_B9)   // all inputs are normally pulled high
#define ZERO      !input(PIN_B8)
#define CAL         !input(PIN_B6)
#define DISCH      !input(PIN_F4)   // pin 22_
#define SLOW      !input(PIN_D9)  // pin 14 INT2

//  Outputs
#define EV1         PIN_B0         // válvula de apertura                       
#define EV2         PIN_B1         // válvula de cierre
#define EV3         PIN_B2         // válvula de descarga
#define READY      PIN_B3         // listo para descargar
#define AUX1      PIN_B4
#define AUX2      PIN_B5

#use rs232(UART1A, baud=115200, restart_wdt, errors, stream=Host)

#use timer(timer=3, tick=100us, bits=32, NOISR)

#define TICK_TYPE unsigned int32
                                             
#define ESPERA 100
                                    
//#use rtos(timer=1, minor_cycle=2ms, statistics)
#build(stack=1024)

#define _ON true
#define _OFF false
