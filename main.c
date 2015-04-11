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

int AMT1, AMT2, AMT3, AMT4, AMT5;		// smmart filter parameters
signed int32 delta, CREDIT, SCALAR;
signed int iCSB;

//============================================
//  global variable define
//============================================

#ZERO_RAM
boolean newCount;

signed int32 g_lPeso, count1, count2, CountTotal, g_lP0;
float g_fPeso, g_fP1, g_fNew_Peso;
float g_fTarget = 2.0, g_fFlow, g_fOver, g_fFreeFall, g_fPesoMin=0.01;
int8 zero_cnt, span_cnt;
boolean g_bUpdHost, g_bKeying, g_bFillDone, g_bDumped, g_bStart, LC1_DATA_RDY, LC2_DATA_RDY;
char g_sInputs[] = "     ", g_sOutputs[] = "   ", g_sStatus;

// this character array will be used to take input from the prompt
char usr_input[30];

// this will hold the current position in the array
int index;

// this will signal to the kernel that input is ready to be processed
boolean input_ready;

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

//============================================
//  interrupt service
//  timer interrupt
//  uart rx interrupt
//============================================

unsigned int32 trapaddr = 0;

#INT_ADDRERR				 //Address error trap
void AE_ISR(void)
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
void MATH_ERR(void)   //Fallo matemático
{
	printf("error matemático\r\n");
	while(TRUE);
}

#INT_RDA 
void USER_INPUT ( )		// serial port interrupt
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
void WS_DAT_1_isr(void)   // lectura de convertidor 2, toma 56 us
{
	int32 count=0;
	char i;  
	for (i=0;i<24;i++)
	{ 
		output_high(WS_CLK_2);
		
		if(i<24)
			count = count << 1;
		
		output_low(WS_CLK_2);
		if(WS_DAT_2) count++;				
	}    
	
	output_high(WS_CLK_2); 

	count2 = count;
	if(bit_test(count,23))
		count2|=0xFF800000;			
	output_low(WS_CLK_2);   // start new conversion

	LC2_DATA_RDY = true;
}

#INT_EXT2	
void WS_DAT_2_isr(void)   // lectura de convertidor 2, toma 56 us
{ 
	int32 count=0;
	 char i; 
	
	for (i=0;i<24;i++)
	{ 
		output_high(WS_CLK_1); 
		
		if(i<24) 
			count = count << 1;
		
		output_low(WS_CLK_1);
		if(WS_DAT_1) count++; 
	}    
	
	output_high(WS_CLK_1); 

	count1 = count;
	if(bit_test(count,23))
		count1|=0xFF800000;
	output_low(WS_CLK_1);   // start new conversion	

	LC1_DATA_RDY = true;
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

	output_low(WS_CLK_1);	// HX711 normal operation
	output_low(WS_CLK_2);	//
	
//	g_lP0 = READ_INT32_EEPROM(0x0000);
//	g_fP1 = READ_FLOAT_EEPROM(0x0020);

}

//============================================
//  
//  RTOS Tasks
//
//============================================
void Task_Disabler();

void READ_HX711();

void SET_ZERO(void);

void SET_SPAN(void);

void TERMINAL (void);

void _BEAT(void);

void CONVERT_WEIGHT(int32 cnt);

void UPDATE_HOST(void);

void PROCESS(void);

void FILL(void);	// llenar

void DUMP (void);	// descargar

void _BEAT(void){
	BEAT;
	restart_wdt();
}

void READ_HX711()
{
	if(LC1_DATA_RDY & LC2_DATA_RDY){		
		CountTotal = count1 + count2;

		LC1_DATA_RDY = LC2_DATA_RDY = false;
		newCount = true;
	}
}

void START_PROCESS()
{
	cout<<"Process started20/03/2015 13:01:54"<<endl;
	g_bStart = TRUE;
	g_bFillDone = true;
	DUMP();
}

void CONVERT_WEIGHT()   // read Weight data
{
	static signed int32 old_cnt;
	static signed int UpdHost;
	int tmp;

	if(newCount){
		
		delta = CountTotal - old_cnt;
		if(abs(iCSB) >= AMT1)
			CREDIT = delta / AMT2;
		else 
			CREDIT = 0;
			
		if(((delta >= 0) && (iCSB < 0)) || ((delta <= 0) && (iCSB > 0)) || (abs(delta) < AMT3))
			iCSB = 0;   
		else
			iCSB = CREDIT + iCSB + ((delta > 0) ? 1 : ((delta < 0) ? -1 : 0));
		
		tmp = (AMT4-abs(iCSB));
		
		SCALAR = (tmp>0)?tmp:0;
		
		old_cnt = old_cnt + (delta/(pow(SCALAR,2) + AMT5));
		
		g_lPeso = old_cnt - g_lP0 ;   // ajuste del CERO;
			
		g_fPeso = floor(g_fP1 * g_lPeso/0.05)*0.05;   // peso calibrado
		
/*		if(g_fPeso > g_fTarget)	// Update SetPoints
		{
			FILL_OFF;
			g_bFillDone = true;
			g_sOutputs[0]=' ';
			DUMP_ON;
		}
		else if(g_fPeso < g_fPesoMin)
		{   
			DUMP_OFF;
			FILL_ON;
			g_bDumped = true;
		}*/
		
		newCount = false;

		if(UpdHost++>1)
		{
			g_bUpdHost = true;
			UpdHost = 0;
		}	
	}
}

void FILL(void)   //   ciclo de llenado
{
	g_sOutputs[0]='F';
	DUMP_OFF;
	FILL_ON;
	set_ticks(0);

}

void CHECK_TIME_OUT(void)
{   
	static unsigned long cnt;
	int32 tmp;
	tmp = get_ticks();
	
	if(tmp > 800){
		set_ticks(0);
		//FILL_OFF;
		//printf("\x1B[17;2H\x1B[Ktimeout %lu %lu ", tmp, cnt++);
	}
			
}

void DUMP (void)   //   ciclo de descarga
{
	if(g_bFillDone){
		DUMP_ON;
		g_bFillDone = false;
		FILL();
	}
}

void SET_ZERO(void)	// Puesta a cero
{
	g_lP0 = CountTotal;

	//WRITE_INT32_EEPROM(0x0000, g_lP0);
}

void SET_SPAN(void)	// Rango
{

	g_lPeso = CountTotal - g_lP0;
	
	g_fP1 = 36.54/g_lPeso;
	
}

void TERMINAL (void)
{
	if(!input_ready) return;	//Si no se ha recibido ningun comando retorna
			
	switch(MnuConfig)
	{
		case SetCalWeight:
		g_fTarget = atof(usr_input);
		//MENU(1);
		break;
		
		case SetZero:
		switch (usr_input)
		{
			case "f":
			//	MENU(2);
				break;
				
			case "s":
			//	MENU(3);
				break;
			
			case "r":
			//	MENU(4);
				break;
			
			case "t":
			//	MENU(0);
	
			default:
				break;
		}

		break;
		
		default:		

		switch (usr_input){
			case "spn":
				SET_SPAN();
			break;
		
			case "zro":
				SET_ZERO();
			break;
			
			case "str":
				START_PROCESS();
			break;
			
			case "stp":	// stop
				cout<<"\x1B[2;5HStop req'd\x1B[K"<<endl;
				DUMP_OFF;
				FILL_OFF;
				g_bStart = FALSE;
			break;
			
			case "opn":	// open
				cout<<"\x1B[2;5HOpening\x1B[K"<<endl;
				DUMP_OFF;
				delay_ms(50);
				FILL_ON;
				//set_ticks(0);
			break;
			
			case "clo":   // close
				cout<<"\x1B[2;5HClosing\x1B[K"<<endl;
				FILL_OFF;
				DUMP_ON;
			break;
			
			case "dmp":
				cout<<"\x1B[2;5HDumping\x1B[K"<<endl;
				cal_on;
				FILL_ON;
				DUMP_ON;
			break;

			case "a":
				//MENU(1);
			break;

			default:
			break;
		}
	
		break;
	}
	
	input_ready=FALSE;
	index=0;
	g_bKeying = false;
}

void UPDATE_HOST(void)
{
	if(g_bUpdHost)
	{			
		if(!g_bKeying)   // user is typing	

			//cout << CountTotal << ", " << count1 << ", " << count2 << ", " << g_fPeso << ", " << delta << ", " << CREDIT << ", " << iCSB << ", " << SCALAR << endl;			
			cout 	<< CountTotal << ", "	//
					<< g_lP0 << ", " 
					<< g_fP1 << ", " 
					<< g_lPeso << ", " 
					<< g_fPeso << ", " 
					<< CREDIT << ", " 
					<< iCSB << ", " 
					<< SCALAR 
					<< endl;			
					
			g_bUpdHost = false;
	}
	
}

//=================================
//   MAIN
//=================================

// todo:   set point variable
//		filtrado
//		auto cero
//		envio de setpoint por puerto serie
	
void MAIN()
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
	AMT1 = 50;
	AMT2 = 20;
	AMT3 = 2;
	AMT4 = 8;
	AMT5 = 1;
	
	switch (restart_cause()){
		case RESTART_MCLR:
			cout<<"Restarted processor because of master clear!\r\n"<<endl;
			
		case RESTART_POWER_UP:
		{
			while(true)
			{
				READ_HX711();
				CONVERT_WEIGHT();
				TERMINAL();
				UPDATE_HOST();
				CHECK_TIME_OUT();
				_BEAT();	// keep alive
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
