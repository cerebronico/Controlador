/////////////////////////////////////////////////////////////////////////
////                         MainBalanza.C                           ////
////                                                                 ////
////							Prueba de Balanza DesiCo.   		  		   ////
////									PIC C source code             		   ////
////                                                                 ////
/////////////////////////////////////////////////////////////////////////
////        (C) Copyright 1996,2003 Custom Computer Services         ////
//// This source code may only be used by licensed users of the CCS  ////
//// C compiler.  This source code may only be distributed to other  ////
//// licensed users of the CCS C compiler.  No other use,            ////
//// reproduction or distribution is permitted without written       ////
//// permission.  Derivative programs created using this software    ////
//// in object code form are not restricted in any way.              ////
/////////////////////////////////////////////////////////////////////////
                      
#include <main.h>
#include <math.h>
#include <ios.h> 
#include <string.h>      
#include <stdio.h>
#include <stdlib.h>
                                                                  
//============================================
//  Weighing parameter define                                                                           
//============================================  
                                                                                         
#define LO_FILTER 6
#define HI_FILTER	100

//============================================
//  global variable define
//============================================

#ZERO_RAM

signed long AMT1, AMT2, AMT3, AMT4, AMT5;
static signed int32 g_lPeso, g_lRawCount, g_lWeighCount, g_lP0;
float g_fPeso, g_fP1, g_fOld_Peso, g_fNew_Peso;
float g_fTarget = 2.0, g_fFlow, g_fOver, g_fFreeFall, g_fPesoMin=0.01;
int8 sem, zero_cnt, span_cnt;
boolean g_bUpdHost, g_bGotWeightData, g_bKeying, g_bDumped, g_bStart;
char g_sInputs[] = "zcssd", g_sOutputs[] = "fsd", g_sStatus;

// this character array will be used to take input from the prompt
char usr_input[30];

// this will hold the current position in the array
int index;

// this will signal to the kernal that input is ready to be processed
int1 input_ready;

// different commands
int8 Menu = 0;

typedef union {
  float f;
  struct {
    unsigned int16 mantisa;
    unsigned int16 exponent;
  } parts;
} float2eeprom;

//============================================
//  procedure define
//============================================
void Init_Hardware(void);
void Open (void);		// abrir compuerta de llenado
void Close (void);	// cerrar compuerta de llenado

//============================================
//  interrupt service
//  timer interrupt
//  uart rx interrupt
//============================================

unsigned int32 trapaddr = 0;

#INT_ADDRERR			//Address error trap
void ae_isr(void)
{
	#asm
	mov w15,w0   
	sub #36,w0
	mov [w0++],w1
	mov w1,trapaddr
	mov [w0],w1
	and #0x7f, w1
	mov w1,trapaddr+2
	#endasm
	printf("Addr %Lx", trapaddr);
	while(TRUE);
}

#INT_MATHERR
void math_err(void)	//Fallo matemático
{
	printf("Fallo matemático\r\n");
	while(TRUE);
}

#INT_RDA 
void User_Input ( )	// serial interupt
{
	g_bKeying = true;
	if(index<29) {
		usr_input [ index ] = getc ( );	// get the value in the serial recieve reg
		putc (usr_input [ index ]);		// display it on the screen
	  
	  if(usr_input[index]==0x0d)		// if the input was enter
	  {
			putc('\n');
			usr_input [ index ] = '\0';	// add the null character
			input_ready=TRUE;         	// set the input read variable to true
			index=0;						// and reset the index
	  }
	  
	  else if (usr_input[index]==0x08)
	  {
			if ( index > 1 )
	    {
				putc(' ');
				putc(0x08);
				index-=2;
	    }
	  }
	  
		index++;
	}
	
	else
	{
		putc ('\n');
		putc ('\r');
	  usr_input [ index ] = '\0';
	  index = 0;
	  input_ready = TRUE;
	}
}

#INT_EXT0		
void ReadCount_isr(void)	// lectura de convertidor, toma 56 us
{ 
	unsigned int32 Count=0;                                                                  
	unsigned char i; 
	
	for (i=0;i<24;i++)
	{ 
		output_high(WS_Clk); 
		Count=Count<<1;                                                                 
		output_low(WS_Clk);
		if(WS_DAT) Count++; 
	}	 
	
	output_high(WS_Clk); 

	g_lRawCount = Count;
	if(bit_test(Count,23))
		g_lRawCount|=0xFF800000;

	g_lWeighCount = g_lRawCount;
 	
	output_low(WS_Clk);	// start new conversion
	g_bGotWeightData = True;                                              
}
                                                                
#INT_EXT2
void ext2_isr(void)	// SLOW fill limit switch
{
	output_low(EV1); 
}

//============================================
//  
//  Functions
//
//============================================

TICK_TYPE GetTickDifference(TICK_TYPE currTick, TICK_TYPE prevTick)
{
	return(currTick-prevTick);
}

void Ticker(void)
{
	//TODO: User Code
}

void Init_Hardware(void)			//  initialize system hardware config
{	
	setup_adc_ports(NO_ANALOGS);
	set_tris_b(0b0000001111000000);
	set_tris_d(0xFFFF);           
	enable_interrupts(INT_EXT0);
	ext_int_edge(0,H_TO_L);
	enable_interrupts(INT_EXT1);
	enable_interrupts(INT_EXT2);
	ext_int_edge(2, H_TO_L);
	enable_interrupts(INT_RDA); 
	enable_interrupts(INTR_GLOBAL);
	
	output_low(WS_CLK);
}

//============================================
//  
//  RTOS Tasks
//
//============================================
void Task_Disabler();

void SET_ZERO(void);

void SET_SPAN(void);

void The_kernal (void);

void Beat(void);

void GetWeightData(void);

void UpdateHost(void);

void Process(void);

void Fill(void);		// llenar

void Dump (void);		// descargar
/*
#task(rate=10, max=2ms)
void Open(void);		// abrir compuerta de llenado

#task(rate=10, max=2ms)
void Close(void);		// cerrar compuerta de llenado*/

//esta es una función truculenta porque las tareas no pueden crearse deshabilitadas
//y no se pueden deshabilitar hasta que el RTOS esté funcionando. 
//void Task_Disabler()
//{
//_disable(SET_SPAN); 
//_disable(SET_ZERO);
//_disable(Dump);
//_disable(Task_Disabler);
//}

void Beat(void){
	output_toggle(PIN_B5);
	restart_wdt();
}

void GetWeight(void)	// read Weight data
{
	int32 lDelta, lNew, lOld_Peso;
	static int iCSB, UpdHost;
	float fCREDIT, fSCALAR;
	int tmp;

	if(g_bGotWeightData)
	{
//	output_high (pin_b4);
	
		g_lPeso = g_lWeighCount - g_lP0 ;	// ajuste del CERO
		
		lDelta = g_lPeso - lOld_Peso;
		if(abs(iCSB) <= AMT1)
			fCREDIT = lDelta / AMT2;
		else 
			fCREDIT = 0;
			
		if(((lDelta >= 0) && (iCSB < 0)) || ((lDelta <= 0) && (iCSB > 0)) || (abs(lDelta) < AMT3))
			iCSB = 0;	
		else
			iCSB = fCREDIT + iCSB + ((lDelta > 0) ? 1 : ((lDelta < 0) ? -1 : 0));
		
		tmp = (AMT4-ABS(iCSB));
		
		fSCALAR = (tmp>0)?tmp:0;
		
		g_fPeso = lOld_Peso + (lDelta/(pow(fSCALAR,2) + AMT5));
		lOld_Peso = g_fPeso;
			
		g_fPeso = floor(g_fP1 * g_lPeso/0.005)*0.005;	// peso calibrado
	
		// Update SetPoints
		g_bGotWeightData = false;
	 // output_low (pin_b4 );
		if(UpdHost++>10)
		{
			g_bUpdHost = true;
			UpdHost = 0;
		}
			
	}
		
}

void Fill(void)	//	ciclo de llenado
{
	unsigned int32 timeout;
	
	timeout = get_ticks();
	output_low(EV2);
	output_high(EV1);
//	rtos_await((get_ticks()-timeout) > 200000);
//	output_low(EV1);
}

void Dump (void)	//	ciclo de descarga
{
	output_high(EV3);
	output_low(EV3);
}

void SET_ZERO(void)		// Puesta a cero
{
	g_lP0 = g_lWeighCount;

	printf("\x1B[3;1HP0=%lu, %6u, P1=%e, %6u\x1B[K", g_lP0, zero_cnt++, g_fP1, span_cnt);                                                               	
}

void SET_SPAN(void)		// Rango
{	
	int16  j; 
	int32 g_lPeso;

	for(j=0;j<=100;j++){
	
		g_lPeso = g_lWeighCount - g_lP0;
		
		g_fNew_Peso = g_fOld_Peso + ((float)g_lPeso - g_fOld_Peso);
		g_fOld_Peso = g_fNew_Peso;
		printf("\x1B[2;15H %6u",j);
	}
	
	g_fP1 = 2.0/g_fNew_Peso;
   
	printf("\x1B[3;1HP0=%lu, %6u, P1=%e, %6u\x1B[K", g_lP0, zero_cnt, g_fP1, span_cnt++);                                                               
	
}            

void The_kernal (void) {
	while ( TRUE ) {
		switch(Menu){
			case 2:
				while(!input_ready)
		    g_fTarget = atof(usr_input);
		  	printf("\x1B[2;5H\x1B[2KTarget Weight: %f ", g_fTarget);
				menu = 1;
				break;
				
			case 1:
				cout<<"\x1b[4;1H\x1B[JChoose Operating Parameters:"<<endl
					"(f)AST FILL"<<endl
					"(s)LOW FILL"<<endl
					"F(r)EE FALL"<<endl
					"RE(t)URN"<<endl;

				while(!input_ready)

				switch (usr_input){
					case "f":
				  	printf ( "\x1B[2;5H\x1B[2KTarget Weight?" );
						menu = 2;
						break;
						
					case "s":
				  	printf ( "\x1B[2;5H\x1B[2KPreliminar?" );
						menu = 3;
						break;
					
					case "r":
				  	printf ( "\x1B[2;5H\x1B[2KFree Fall?" );
						menu = 4;
						break;
					
					case "t":
						menu=0;

					default:
						break;
				}
				break;
				
			default:
				cout<<"\x1b[4;1H\x1B[JChoose:"<<endl
					"(c)AL"<<endl
					"(z)ERO"<<endl
					"(t)ARE"<<endl
					"(f)ILTER"<<endl
					"(o)PEN"<<endl
					"C(l)OSE"<<endl
					"(d)ISCHARGE"<<endl
					"(s)TART"<<endl
					"STO(p)"<<endl
					"P(a)RMS"<<endl
					"Command?"<<endl;
					
				while(!input_ready)
				
				printf ( "\x1B[15;10H %s\x1B[K", usr_input );
				switch (usr_input){
					case "c":
				  	printf ( "\x1B[2;5HSPANNING\x1B[K" );
						break;
				
					case "z":     		
				    printf ( "\x1B[2;5HZEROING\x1B[K" );
						break;
					
					case "s":
						cout<<"\x1B[2;5HProcess started\x1B[K"<<endl;
						g_bStart = TRUE;
						break;
						
					case "p":
						cout<<"\x1B[2;5HStop req'd\x1B[K"<<endl;
						output_low(EV1);
						output_low(EV2);
						output_low(EV3);
						g_bStart = FALSE;
						break;
						
					case "o":
						cout<<"\x1B[2;5HOpening\x1B[K"<<endl;
						output_low(EV2);
						output_high(EV1);
						break;
						
					case "l":
						cout<<"\x1B[2;5HClosing\x1B[K"<<endl;
						output_low(EV1);
						output_high(EV2);
						break;
						
					case "d":
						cout<<"\x1B[2;5HDumping\x1B[K"<<endl;
						output_high(EV3);
						break;
											
					case "a":
						menu = 1;
						break;
										
					default:
						printf ( "\x1B[2;5H\x1B[2KError: unknown command" );
						break;
				}
		
				break;
		}
		
		input_ready=FALSE;
		index=0;
		g_bKeying = false;
	}
}
void UpdateHost(void)
{
	//if(get_ticks()>1000)
	//{
	if(g_bUpdHost)
	{
	g_sInputs[0]=(zero)?'z':' ';
	g_sInputs[1]=(cal)?'c':' ';
	g_sInputs[2]=(start)?'s':' ';
	g_sInputs[3]=(slow)?'s':' ';
	g_sInputs[4]=(disch)?'d':' ';
		
	if(!g_bKeying)	// user is typing
		
	  printf("\x1b[H%10ld, %10ld, %7.3f, %s, %s, %s\x1b[K", g_lRawCount, g_lPeso, g_fPeso, g_sInputs, g_sOutputs, g_sStatus);
	
	g_bUpdHost = false;
	}
	
	//set_ticks(0);
	//}  
}
/*
void Open (void)
{
	output_low(EV2);
	output_high(EV1);
}

void Close (void)
{
	output_low(EV1);
	output_high(EV2);
}*/

//=================================
//	MAIN
//=================================

// todo:	set point variable
//				filtrado
//				auto cero
//				envio de setpoint por puerto serie
	
void main()
{
	Init_Hardware();
	TICK_TYPE CurrentTick,PreviousTick;

	//Example program using Tick Timer
	CurrentTick = PreviousTick = get_ticks();

	char Text[] = "DesiCo. Systems\r";
	printf("%s\x1B[2J", Text);

	sem=1;
	g_lP0=1769696;
	g_fP1=2.1895E-06;
	
	// initialize input variables
	index=0;
	input_ready=FALSE;   
  
  AMT1 = 10;
  AMT2 = 2;
  AMT3 = 2000;
  AMT4 = 8;
  AMT5 = 1;
  
  output_high(EV3);
  delay_ms(2000);
  output_low(EV3);
	
	switch (restart_cause()){
		case RESTART_MCLR:
		{
		   printf("\r\nRestarted processor because of master clear!\r\n");
		}
		case RESTART_POWER_UP:
		{
			while(true)
			{
				GetWeight();
				The_kernal();
				UpdateHost();
			}
			break;
		}
		case RESTART_WATCHDOG:
		{
		   printf("\r\nRestarted processor because of watchdog timeout!\r\n");
		   break;
		}
	
		case RESTART_BROWNOUT:
		{
			printf("\r\nRestarted processor because of brownout!\r\n");
			break;
		}
		case RESTART_SOFTWARE:
		{
			printf("\r\nRestarted processor because of restart software!\r\n");
			break;
		}			
		case RESTART_TRAP_CONFLICT :
		{
			printf("\r\nRestarted processor because of trap conflict!\r\n");
			break;
		}
	}
	while (true);
}

//	//test
//	while(true)
//	{
//		output_high(EV1);
//		delay_ms(250);
//		output_low(EV1);
//		delay_ms(800);
//		output_high(EV2);
//		delay_ms(200);
//		output_low(EV2);
//		delay_ms(800);
//	}	
		