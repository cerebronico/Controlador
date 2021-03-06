/////////////////////////////////////////////////////////////////////////
////                         MainBalanza.C                           ////
////                                                                 ////
////   funcionamiento autom�tico cont�nuo 
////   
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
#define HI_FILTER   100

//============================================
//  global variable define
//============================================

#ZERO_RAM

signed long AMT1, AMT2, AMT3, AMT4, AMT5;
static signed int32 g_lPeso, g_lRawCount, g_lWeighCount, g_lP0;
float g_fPeso, g_fP1, g_fOld_Peso, g_fNew_Peso;
float g_fTarget = 2.0, g_fFlow, g_fOver, g_fFreeFall, g_fPesoMin=0.01;
int8 sem, zero_cnt, span_cnt;
boolean g_bUpdHost, g_bGotWeightData, g_bKeying, g_bFillDone, g_bDumped, g_bStart;
char g_sInputs[] = "     ", g_sOutputs[] = "   ", g_sStatus;

// this character array will be used to take input from the prompt
char usr_input[30];

// this will hold the current position in the array
int index;

// this will signal to the kernal that input is ready to be processed
int1 input_ready;

// different commands
int8 _Menu = 0;

typedef union 
{
	float f;
	struct
	{
		unsigned int16 mantisa;
		unsigned int16 exponent;
	} parts;
} float2eeprom;

//============================================
//  procedure define
//============================================
void INIT_HARDWARE(void);
void Open (void);		// abrir compuerta de llenado
void Close (void);		// cerrar compuerta de llenado

//============================================
//  interrupt service
//  timer interrupt
//  uart rx interrupt
//============================================

unsigned int32 trapaddr = 0;

#INT_ADDRERR				 //Address error trap
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
void math_err(void)   //Fallo matem�tico
{
	printf("error matem�tico\r\n");
	while(TRUE);
}

#INT_RDA 
void User_Input ( )		// serial port interrupt
{
	g_bKeying = true;
	if(index<29){
		usr_input [ index ] = getc ( );	// get the value in the serial recieve reg
		putc (usr_input [ index ]);		// display it on the screen

		if(usr_input[index]==0x0d){		// if the input was enter
			
			putc('\n');
			usr_input [ index ] = '\0';	// add the null character
			input_ready=TRUE;			// set the input read variable to true
			index=0;					// and reset the index
		}

		else if (usr_input[index]==0x08){
			if ( index > 1 ){
				putc(' ');
				putc(0x08);
				index-=2;
			}
		}
		index++;
	}
	
	else{
		putc ('\n');
		putc ('\r');
		usr_input [ index ] = '\0';
		index = 0;
		input_ready = TRUE;
	}
}

#INT_EXT0      
void ReadCount_isr(void)   // lectura de convertidor, toma 56 us
{ 
	unsigned int32 Count=0;
	unsigned char i; 
	
	for (i=0;i<24;i++)
	{ 
		output_high(WS_Clk); 
		if(i<24) Count=Count<<1;
		output_low(WS_Clk);
		if(WS_DAT && (i < 24)) Count++; 
	}    
	
	output_high(WS_Clk); 

	g_lRawCount = Count;
	if(bit_test(Count,23))
			g_lRawCount|=0xFF800000;

	g_lWeighCount = g_lRawCount;
		
	output_low(WS_Clk);   // start new conversion
	g_bGotWeightData = True;
}

#INT_EXT2
void ext2_isr(void)   // SLOW fill limit switch
{
	output_low(EV1); 
}

Int32 Ticker, second;
//#int_TIMER1
//void TIMER1_isr()     
//{
//   Ticker -= 65536;
//   if ( Ticker < 65536)
//   {
//      Ticker += 5000000;
//      second++;
//   }
//} 

//============================================
//  
//  Functions
//
//============================================

void WRITE_FLOAT_EEPROM(int16 n, float data) 
{
	write_eeprom(n, data, sizeof(float));
}

float READ_FLOAT_EEPROM(int16 n)
{
	float data;
	(int32)data = read_eeprom(n, sizeof(float));
	return(data);
}

void WRITE_INT32_EEPROM(int16 n, int32 data) 
{
	write_eeprom(n, data, sizeof(int32));
}

float READ_INT32_EEPROM(int16 n)
{
	int32 data;
	data = read_eeprom(n, sizeof(int32));
	return(data);
}

void INIT_HARDWARE(void)         //  initialize system hardware config
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
	
	g_lP0 = READ_INT32_EEPROM(0x0000);
	g_fP1 = READ_FLOAT_EEPROM(0x0020);
	
}

//============================================
//  
//  RTOS Tasks
//
//============================================
void Task_Disabler();

void SET_ZERO(void);

void SET_SPAN(void);

void Console_IO (void);

void Beat(void);

void GetWeightData(void);

void Update_Host(void);

void Process(void);

void Fill(void);      // llenar

void Dump (void);      // descargar

void Beat(void){
	output_toggle(AUX2);
	restart_wdt();
}

void Start_Process()
{
	cout<<"Process started20/03/2015 13:01:54"<<endl;
	g_bStart = TRUE;
	g_bFillDone = true;
	Dump();
}

void GetWeight(void)   // read Weight data
{
	int32 lDelta, lNew, lOld_Peso;
	static int iCSB, UpdHost;
	float fCREDIT, fSCALAR;
	int tmp;

	if(g_bGotWeightData){
		
		g_lPeso = g_lWeighCount - g_lP0 ;   // ajuste del CERO
		
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
			
		g_fPeso = floor(g_fP1 * g_lPeso/0.005)*0.005;   // peso calibrado
//      g_fPeso = floor(g_fP1 * g_lPeso/0.05)*0.05;   // peso calibrado
		
		if(g_fPeso>g_fTarget)      // Update SetPoints
		{
			output_high(EV2);
			g_bFillDone = true;
			g_sOutputs[0]=' ';
			//Dump();
		}
		else if(g_fPeso<g_fPesoMin)
		{   
			output_low(EV3);
			g_bDumped = true;
		}
		
		g_bGotWeightData = false;

		if(UpdHost++>10)
		{
			g_bUpdHost = true;
			UpdHost = 0;
		}
				
	}      
}

void Fill(void)   //   ciclo de llenado
{
	g_sOutputs[0]='F';
	output_low(EV2);
	output_high(EV1);
	set_ticks(0);

}

void Check_Time_Out(void)
{   
	static unsigned long cnt;
	int32 tmp;
	tmp = get_ticks();
	
	if(tmp > 800){
		set_ticks(0);
		output_low(EV1);
		//printf("\x1B[17;2H\x1B[Ktimeout %lu %lu ", tmp, cnt++);
	}
			
}

void Dump (void)   //   ciclo de descarga
{
	if(DISCH || g_bFillDone){
		output_high(EV3);
		g_bFillDone = false;
		fill();
	}
}

void SET_ZERO(void)      // Puesta a cero
{
	g_lP0 = g_lWeighCount;
//   printf(":zP0=%lu, %6u, P1=%e, %6u", g_lP0, zero_cnt++, g_fP1, span_cnt);
	WRITE_INT32_EEPROM(0x0000, g_lP0);
}

void SET_SPAN(void)      // Rango
{   
	int16  j; 
	int32 g_lPeso;

	for(j=0;j<=100;j++){
	
		g_lPeso = g_lWeighCount - g_lP0;
		
		g_fNew_Peso = g_fOld_Peso + ((float)g_lPeso - g_fOld_Peso);
		g_fOld_Peso = g_fNew_Peso;
//      printf(":j%6u",j);
	}
	
	g_fP1 = 0.8/g_fNew_Peso;
	//printf("\x1B[3;1HP0=%lu, %6u, P1=%e, %6u\x1B[K", g_lP0, zero_cnt, g_fP1, span_cnt++);
	//WRITE_FLOAT_EEPROM(0x0020, g_fP1);
	
}
				   
void MENU(int choice)
{
	switch(choice){
		
	case 0:
		_menu=0;
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

			break;

			case 1:
				_menu=1;
				cout<<"\x1b[4;1H\x1B[JChoose Operating Parameters:"<<endl
				   "(f)AST FILL"<<endl
				   "(s)LOW FILL"<<endl
				   "F(r)EE FALL"<<endl
				   "RE(t)URN"<<endl;
				break;
			default:
				break;

			case 2:
				_menu=2;
				break;
	}
				   
}

void Console_IO (void)
{
	if(!input_ready) return;
			
	switch(_menu)
	{
	case 2:
	g_fTarget = atof(usr_input);
//	printf("\x1B[2;5H\x1B[2KTarget Weight: %f ", g_fTarget);
				MENU(1);
				break;
				
			case 1:
				switch (usr_input)
				{
				   case "f":
//              printf ( "\x1B[2;5H\x1B[2KTarget Weight?" );
				      MENU(2);
				      break;
				      
				   case "s":
//              printf ( "\x1B[2;5H\x1B[2KPreliminar?" );
				      MENU(3);
				      break;
				   
				   case "r":
//              printf ( "\x1B[2;5H\x1B[2KFree Fall?" );
				      MENU(4);
				      break;
				   
				   case "t":
				      MENU(0);

				   default:
				      break;
				}

				break;
				
			default:            
//         printf ( "\x1B[15;10H %s\x1B[K", usr_input );
				switch (usr_input){
				   case "c":
//                printf ( "\x1B[2;5HSPANNING\x1B[K" );
				       set_span();
				      break;
				   
				   case "z":
//              printf ( "\x1B[2;5HZEROING\x1B[K" );
				     set_zero();
				      break;
				      
				   case "s":
				      Start_Process();
				      break;
				         
				   case "p":      // stop
				      cout<<"\x1B[2;5HStop req'd\x1B[K"<<endl;
				      output_low(EV1);
				      output_low(EV2);
				      output_low(EV3);
				      g_bStart = FALSE;
				      break;
				         
				   case "o":      // open
				      cout<<"\x1B[2;5HOpening\x1B[K"<<endl;
				      output_low(EV2);
				      delay_ms(50);
				      output_high(EV1);
				      set_ticks(0);
				      break;
				         
				   case "l":   // close
				      cout<<"\x1B[2;5HClosing\x1B[K"<<endl;
				      output_low(EV1);
				      output_high(EV2);
				      break;
				         
				   case "d":
				      cout<<"\x1B[2;5HDumping\x1B[K"<<endl;
				      output_high(EV3);
				      break;

				   case "a":
				      MENU(1);
				      break;

				   default:
//               printf ( "\x1B[2;5H\x1B[2KError: unknown command" );
				      break;
				}
			
				break;
	}
			
			input_ready=FALSE;
			index=0;
			g_bKeying = false;
}

void Read_Inputs(void)
{
			g_sInputs[0]=(zero)?'z':' ';
			g_sInputs[1]=(cal)?'c':' ';

			if(START)
			{
				g_sInputs[2]='s';
				Start_Process();   
			}
			else
				g_sInputs[2]=' ';
			
			g_sInputs[3]=(slow)?'s':' ';
			
			if(DISCH)
			{
				g_sInputs[4] = 'd';
				output_high(EV3);
			}
			else
				g_sInputs[4] = ' ';
				
}

void Update_Host(void)
{

	if(g_bUpdHost)
	{

				
			if(!g_bKeying)   // user is typing
				
//      printf("\x1b[H%10ld, %10ld, %7.3f, %s, %s, %s\x1B[K", g_lRawCount, g_lPeso, g_fPeso, g_sInputs, g_sOutputs, g_sStatus);
//      printf("%ld,%ld,%f\r\n", g_lRawCount, g_lPeso, g_fPeso);
			g_bUpdHost = false;
	}
	
	//set_ticks(0);
	//}  
}

//=================================
//   MAIN
//=================================

// todo:   set point variable
//            filtrado
//            auto cero
//            envio de setpoint por puerto serie
	
void main()
{
	INIT_HARDWARE();
	TICK_TYPE CurrentTick,PreviousTick;

	//Example program using Tick Timer
	CurrentTick = PreviousTick = get_ticks();

	char Text[] = "DesiCo. Systems\r";
//   printf("%s\x1B[2J", Text);
	
	// initialize input variables
	index=0;
	input_ready=FALSE;   
	
	AMT1 = 2;
	AMT2 = 3;
	AMT3 = 20;
	AMT4 = 8;
	AMT5 = 1;
	
	MENU(0);
	
	switch (restart_cause()){
			case RESTART_MCLR:
			{
//         printf("\r\nRestarted processor because of master clear!\r\n");
			}
			case RESTART_POWER_UP:
			{
				while(true)
				{
				   GetWeight();
			Read_Inputs();
				   Console_IO();
				   Update_Host();
				   Check_Time_Out();
				   Beat();      // keep alive
				}
				break;
			}
			case RESTART_WATCHDOG:
			{
//         printf("\r\nRestarted processor because of watchdog timeout!\r\n");
				break;
			}
	
			case RESTART_BROWNOUT:
			{
//         printf("\r\nRestarted processor because of brownout!\r\n");
				break;
			}
			case RESTART_SOFTWARE:
			{
//         printf("\r\nRestarted processor because of restart software!\r\n");
				break;
			}         
			case RESTART_TRAP_CONFLICT :
			{
//         printf("\r\nRestarted processor because of trap conflict!\r\n");
				break;
			}
	}
	while (true)
			output_toggle(AUX1);
}

//   //test
//   while(true)
//   {
//      output_high(EV1);
//      delay_ms(250);
//      output_low(EV1);
//      delay_ms(800);
//      output_high(EV2);
//      delay_ms(200);
//      output_low(EV2);
//      delay_ms(800);
//   }   
			
