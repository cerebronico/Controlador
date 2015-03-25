////////////////////////////////////////////////////////////////////////
////				 Main.c											////
////////////////////////////////////////////////////////////////////////

/*
Controlador para balanza de dos celdas de carga, no utiliza caja sumadora
ya que esta se hace digitalmente leyendo dos convertidores HX711
*/

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
static signed int32 g_lPeso, g_lWS_DAT_1_COUNT, g_lWS_DAT_2_COUNT, g_lP0;
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
enum _Menu { GetWeightCounts, SetZero, SetSpan, SetCalWeight } MnuConfig;

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
void math_err(void)   //Fallo matemático
{
	printf("error matemático\r\n");
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

#INT_EXT1	
void WS_DAT_2_isr(void)   // lectura de convertidor 2, toma 56 us
{ 
	unsigned int32 Count=0;
	unsigned char i;
	signed int32 RawCount;
	
	for (i=0;i<24;i++)
	{ 
		output_high(WS_CLK_2); 
		if(i<24) Count=Count<<1;
		output_low(WS_CLK_2);
		if(WS_DAT_2 && (i < 24)) Count++; // read input data
	}    
	
	output_high(WS_CLK_2); 

	RawCount = Count;
	if(bit_test(Count,23))
		RawCount|=0xFF800000;

	g_lWS_DAT_2_COUNT = RawCount;
		
	output_low(WS_CLK_2);   // start new conversion
//	g_bGotWeightData = True;
}

#INT_EXT2
void WS_DAT_1_isr(void)   // lectura de convertidor 2, toma 56 us
{ 
	unsigned int32 Count=0;
	unsigned char i; 
	signed int32 RawCount;
		
	for (i=0;i<24;i++)
	{ 
		output_high(WS_CLK_1); 
		if(i<24) Count=Count<<1;
		output_low(WS_CLK_1);
		if(WS_DAT_1 && (i < 24)) Count++; 
	}    
	
	output_high(WS_CLK_1); 

	RawCount = Count;
	if(bit_test(Count,23))
		RawCount|=0xFF800000;

	g_lWS_DAT_1_COUNT = RawCount;
		
	output_low(WS_CLK_1);   // start new conversion
}

Int32 Ticker, second;
//#int_TIMER1
//void TIMER1_isr()     
//{
//   Ticker -= 65536;
//   if ( Ticker < 65536)
//   {
//	Ticker += 5000000;
//	second++;
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

void INIT_HARDWARE(void)	//  initialize system hardware config
{   
	setup_adc_ports(NO_ANALOGS);
	set_tris_b(0b0000001111000000);
	set_tris_d(0xFFFF);
	enable_interrupts(INT_EXT0);
	ext_int_edge(0,H_TO_L);
	enable_interrupts(INT_EXT1);
	ext_int_edge(1, H_TO_L);	
	enable_interrupts(INT_EXT2);
	ext_int_edge(2, H_TO_L);
	enable_interrupts(INT_RDA); 
	enable_interrupts(INTR_GLOBAL);

//	output_low(WS_CLK_2);
	
	g_lP0 = READ_INT32_EEPROM(0x0000);
	g_fP1 = READ_FLOAT_EEPROM(0x0020);
	
}

signed int32 Filter(signed int32 NewValue)
{
	signed int32 NewReading, OldReading;
	NewReading = OldReading + (NewValue - OldReading)>>8;
	OldReading = newReading;
	return NewReading;
}
//============================================
//  
//  RTOS Tasks
//
//============================================
void Task_Disabler();

void SET_ZERO(void);

void SET_SPAN(void);

void Terminal (void);

void _Beat(void);

void GetWeightData(void);

void Update_Host(void);

void Process(void);

void _Fill(void);	// llenar

void _Dump (void);	// descargar

void _Beat(void){
	output_toggle(BEAT);
	restart_wdt();
}

void Start_Process()
{
	cout<<"Process started20/03/2015 13:01:54"<<endl;
	g_bStart = TRUE;
	g_bFillDone = true;
	_Dump();
}

void GetWeight(void)   // read Weight data
{
	int32 lDelta, lNew, lOld_Peso;
	static int iCSB, UpdHost;
	float fCREDIT, fSCALAR;
	int tmp;

//	if(g_bGotWeightData){
		
		g_lPeso = g_lWS_DAT_2_COUNT - g_lP0 ;   // ajuste del CERO
		
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
//		g_fPeso = floor(g_fP1 * g_lPeso/0.05)*0.05;   // peso calibrado
		
		if(g_fPeso > g_fTarget)	// Update SetPoints
		{
			output_high(FILL);
			g_bFillDone = true;
			g_sOutputs[0]=' ';
			//_Dump();
		}
		else if(g_fPeso < g_fPesoMin)
		{   
			output_low(FILL);
			g_bDumped = true;
		}
		
		g_bGotWeightData = false;

		if(UpdHost++>10)
		{
			g_bUpdHost = true;
			UpdHost = 0;
		}
				
//	}	
}

void _Fill(void)   //   ciclo de llenado
{
	g_sOutputs[0]='F';
	output_low(DUMP);
	output_high(FILL);
	set_ticks(0);

}

void Check_Time_Out(void)
{   
	static unsigned long cnt;
	int32 tmp;
	tmp = get_ticks();
	
	if(tmp > 800){
		set_ticks(0);
		output_low(FILL);
		//printf("\x1B[17;2H\x1B[Ktimeout %lu %lu ", tmp, cnt++);
	}
			
}

void _Dump (void)   //   ciclo de descarga
{
	if(g_bFillDone){
		output_high(DUMP);
		g_bFillDone = false;
		_Fill();
	}
}

void SET_ZERO(void)	// Puesta a cero
{
	g_lP0 = g_lWS_DAT_2_COUNT;
//   printf(":zP0=%lu, %6u, P1=%e, %6u", g_lP0, zero_cnt++, g_fP1, span_cnt);
	WRITE_INT32_EEPROM(0x0000, g_lP0);
}

void SET_SPAN(void)	// Rango
{
	int16  j; 
	int32 g_lPeso;

	for(j=0;j<=100;j++){
	
		g_lPeso = g_lWS_DAT_2_COUNT - g_lP0;
		
		g_fNew_Peso = g_fOld_Peso + ((float)g_lPeso - g_fOld_Peso);
		g_fOld_Peso = g_fNew_Peso;
//	printf(":j%6u",j);
	}
	
	g_fP1 = 0.8/g_fNew_Peso;
	//printf("\x1B[3;1HP0=%lu, %6u, P1=%e, %6u\x1B[K", g_lP0, zero_cnt, g_fP1, span_cnt++);
	//WRITE_FLOAT_EEPROM(0x0020, g_fP1);
	
}

/*void Terminal (void)
{
	if(!input_ready) return;
			
	switch(MnuConfig)
	{
	case SetCalWeight:
	g_fTarget = atof(usr_input);
//	printf("\x1B[2;5H\x1B[2KTarget Weight: %f ", g_fTarget);
		MENU(1);
		break;
		
	case SetZero:
		switch (usr_input)
		{
		case "f":
//		  printf ( "\x1B[2;5H\x1B[2KTarget Weight?" );
			MENU(2);
			break;
			
		case "s":
//		  printf ( "\x1B[2;5H\x1B[2KPreliminar?" );
			MENU(3);
			break;
		
		case "r":
//		  printf ( "\x1B[2;5H\x1B[2KFree Fall?" );
			MENU(4);
			break;
		
		case "t":
			MENU(0);

		default:
			break;
		}

		break;
		
	default:		
//	printf ( "\x1B[15;10H %s\x1B[K", usr_input );
		switch (usr_input){
		case "c":
//		 printf ( "\x1B[2;5HSPANNING\x1B[K" );
			 set_span();
			break;
		
		case "z":
//		  printf ( "\x1B[2;5HZEROING\x1B[K" );
		  set_zero();
			break;
			
		case "s":
			Start_Process();
			break;
			
		case "p":	// stop
			cout<<"\x1B[2;5HStop req'd\x1B[K"<<endl;
			output_low(EV1);
			output_low(EV2);
			output_low(EV3);
			g_bStart = FALSE;
			break;
			
		case "o":	// open
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
//		printf ( "\x1B[2;5H\x1B[2KError: unknown command" );
		break;
		}
	
		break;
}
	
	input_ready=FALSE;
	index=0;
	g_bKeying = false;
}*/

void Read_Inputs(void)
{
/*	g_sInputs[0]=(zero)?'z':' ';
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
		g_sInputs[4] = ' ';*/
}

void Update_Host(void)
{
	if(g_bUpdHost)
	{			
		if(!g_bKeying)   // user is typing	
			cout<<"P2 "<<filter(g_lWS_DAT_2_COUNT)<<endl;	
			cout<<"P1 "<<filter(g_lWS_DAT_1_COUNT)<<endl;
			
			g_bUpdHost = false;
	}
	
	//set_ticks(0);
	//}  
}

//=================================
//   MAIN
//=================================

// todo:   set point variable
//		filtrado
//		auto cero
//		envio de setpoint por puerto serie
	
void main()
{
	INIT_HARDWARE();
	TICK_TYPE CurrentTick,PreviousTick;

	//Example program using Tick Timer
	CurrentTick = PreviousTick = get_ticks();

	char Text[] = "DesiCo. Systems\r";
	cout<<Text<<endl;
	
	// initialize input variables
	index=0;
	input_ready=FALSE;
	
	// smart filter params
	AMT1 = 2;
	AMT2 = 3;
	AMT3 = 20;
	AMT4 = 8;
	AMT5 = 1;
	
	switch (restart_cause()){
		case RESTART_MCLR:
			cout<<"Restarted processor because of master clear!\r\n"<<endl;
			
		case RESTART_POWER_UP:
		{
			while(true)
			{
				GetWeight();
				Read_Inputs();
				//Terminal();
				Update_Host();
				Check_Time_Out();
				_Beat();	// keep alive
			}
			break;
		}
		case RESTART_WATCHDOG:
		{
			cout<<"Restarted processor because of watchdog timeout!"<<endl;
			break;
		}
		case RESTART_BROWNOUT:
		{
			cout<<"Restarted processor because of brownout!"<<endl;
			break;
		}
		case RESTART_SOFTWARE:
		{
			cout<<"Restarted processor because of restart software!"<<endl;
			break;
		}
		case RESTART_TRAP_CONFLICT :
		{
			cout<<"Restarted processor because of trap conflict!"<<endl;
			break;
		}
	}
	while (true)
		output_toggle(TP1);	// error trap
}	
