////////////////////////////////////////////////////////////////////////
////				 Main.c											////
////////////////////////////////////////////////////////////////////////

/*
Controlador para balanza de dos celdas de carga, no utiliza caja sumadora
ya que esta se hace digitalmente leyendo dos convertidores HX711
*/

#define debug		// comment this line in release version

#include <main.h>
#include <math.h>
#include <ios.h> 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <input.c>
#include <internal_eeprom.c>
#include "limits.h"

#ZERO_RAM

//============================================
//  Weighing parameter define
//============================================  

int16 g_iAMT[5];		// smmart filter parameters must be stored and recalled from EEPROM
char sParms[32];		// variable to hold intermediate SMART FILTER values
 
signed int32 g_lP0;	// unloaded scale count value
float g_fP1, g_fKf1, g_fKf2;		// gain & filter
int16 g_iZtDly, g_iMtDly, g_iFillDly, g_iDumpDly;	// zero tracking and motion delay in msec
float g_fAcumulado, g_fPesoMin, g_fTarget, g_fFlow, g_fOver, g_fFreeFall, g_fZtDiv, g_fMotn, g_fTemp;

//============================================
//  global variable define
//============================================

boolean newCount;

signed int32 g_lPeso, g_lCnt_LC1, g_lCnt_LC2, g_lLC1_P0, g_lLC2_P0, g_lCount1, g_lCount2, g_lCountTotal;
float g_fPeso, g_fTestWeight, g_fMotion;
boolean g_bDiags, g_bUpdHost, g_bKeying, g_bFillDone,  g_bStart, LC1_DATA_RDY, LC2_DATA_RDY, g_bSetZERO, g_bSetGAIN;
int16 ZERO_SET_COUNTER, GAIN_SET_COUNTER;

char usr_input[30];		// this character array will be used to take input from the prompt
int index;				// this will hold the current position in the array
boolean input_ready;	// this will signal to the kernel that input is ready to be processed

// enumerate different commands
enum _Menu {GetWeightCounts, SetZero, SetSpan, SetCalWeight } MnuConfig;
enum _State {llenando, vaciando, calibrando, esperando, encerando, reposando} staEstado;

typedef union 
{
	float f;
	struct{
		unsigned int16 mantisa;
		unsigned int16 exponent;
	} parts;
} float2eeprom;

typedef unsigned int32 tick_t;

tick_t current, Tx_t;

// flag definitions
char g_cInputs, g_cOutputs, g_cStatus, g_cTx;

#bit DGLS = g_cInputs.0		// dump gate limit switch
#bit FGLS = g_cInputs.1		// fill gate limit switch
#bit OVRF = g_cInputs.2		// overrun flag
#bit FULF = g_cInputs.3		// overdump flag
#bit PRSF = g_cInputs.4		// airpressure flag
#bit XTRA = g_cInputs.5		// extra flag

#bit Filling = g_cOutputs.0
#bit Dumping = g_cOutputs.1
#bit Calibrating = g_cOutputs.2
#bit Alarming = g_cOutputs.3

#bit Motion = g_cStatus.0
#bit Over = g_cStatus.1
#bit Under = g_cStatus.2


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
	if(index<29){
		usr_input [index] = getc();	// get the value in the serial recieve reg
		putc(usr_input[index]);
		
		if(usr_input[index]==0x0d){		// if the input was enter
			
			usr_input [ index ] = '\0';	// add the null character
			input_ready=TRUE;			// set the input read variable to true
			index=0;					// and reset the index
		}

		else if (usr_input[index]==0x08){
			if ( index > 1 ){
				index-=2;
			}
		}
		index++;
	}
	
	else{
		usr_input [ index ] = '\0';
		index = 0;
		input_ready = TRUE;
	}
}

#INT_EXT1
void WS_DAT_1_isr(void)   // lectura de convertidor 2
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

	g_lCount2 = count;
	if(bit_test(count,23))
		g_lCount2|=0xFF800000;			
	output_low(WS_CLK_2);   // start new conversion

	LC2_DATA_RDY = true;
}

#INT_EXT2	
void WS_DAT_2_isr(void)   // lectura de convertidor 2
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

	g_lCount1 = count;
	if(bit_test(count,23))
		g_lCount1|=0xFF800000;
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
	
	// smart filter params

	read_eeprom(0, g_iAMT[1]);	// g_iAMT[1] = 50;  
	read_eeprom(2, g_iAMT[2]);	// g_iAMT[2] = 20;  
	read_eeprom(4, g_iAMT[3]);	// g_iAMT[3] = 2;   
	read_eeprom(6, g_iAMT[4]);	// g_iAMT[4] = 8;   
	read_eeprom(8, g_iAMT[5]);	// g_iAMT[5] = 1; 
	
	// operational params
	
	read_eeprom(10, g_iZtDly);
	read_eeprom(12, g_iMtDly);
	read_eeprom(14, g_iFillDly);
	read_eeprom(16, g_iDumpDly);
	
	// scale params	                       	
	g_lP0 = read_int32_eeprom(40);	// ZERO
	
	g_fP1 = read_float_eeprom(60);	// GAIN
	g_fTarget = read_float_eeprom(64);	//TARGET WEIGHT
	g_fPesoMin = read_float_eeprom(68);	// MIN WEIGHT TO CONSIDER FOR ZERO
	g_fTestWeight = read_float_eeprom(72);	// CAL TEST WEIGHT
	g_fKf1 = read_float_eeprom(76);	// moving average filter constants
	g_fKf2 = read_float_eeprom(80);	// 
	g_fZtDiv = read_float_eeprom(84);	// zero tracking range
	g_fMotn	= read_float_eeprom(88);	// motion tracking
}

//============================================
//  
//  Function prototypes
//
//============================================

int READ_HX711();

void SET_ZERO(void);

void SET_SPAN(void);

void HOST_COMMANDS (void);

void _BEAT(void);

void CONVERT_WEIGHT(int32 cnt);

void UPDATE_HOST(int Tx);

void PROCESS(void);

void FILL();	// llenar

void DUMP ();	// descargar

VOID CALIB();	// calibrar

int READ_HX711()
{
	static signed int32 newCount1, newCount2;
	 
	if(LC1_DATA_RDY & LC2_DATA_RDY){	// si los convertidores tienen datos
		
		g_lCnt_LC1 = g_lCount1 - g_lLC1_P0;
		g_lCnt_LC2 = g_lCount2 - g_lLC2_P0;
		
		if(g_bSetZERO){

			if((ZERO_SET_COUNTER--) >=0){
				
				newCount1 = newCount1 + g_fKf1*(g_lCount1-NewCount1);
				newCount2 = newCount2 + g_fKf2*(g_lCount2-NewCount2);
				printf("ZEROING... %d\n\r", ZERO_SET_COUNTER);
			}
			else{
				g_lLC1_P0 = NewCount1;
				g_lLC2_P0 = NewCount2;
				g_bSetZERO = false;
				printf("ZEROING DONE!\n\r");
			}

		LC1_DATA_RDY = LC2_DATA_RDY = false;			
		newCount = false;

		return(0);
			
		}
		
		if (g_bSetGAIN){
			
			if(GAIN_SET_COUNTER-- >=0){
								
				GAIN_SET_COUNTER =7;
			}
		}
		
		g_lCountTotal = g_lCnt_LC1 + g_lCnt_LC2;

		LC1_DATA_RDY = LC2_DATA_RDY = false;
		newCount = true;
	}
	
	return(1);
}

void START_PROCESS()
{
	printf("Process started\n");
	g_bStart = TRUE;
	g_bFillDone = true;
	staEstado = vaciando;
 	
}

void CONVERT_WEIGHT()   // read Weight data
{
	static signed int32 old_cnt, DELTA, CREDIT, SCALAR;
	static signed int iCSB, iMotionCounter;
	static float fPesoAnterior;
	signed int tmp;
	boolean TEST;

	if(newCount){
		
		DELTA = g_lCountTotal - old_cnt;

		if(abs(iCSB) >= g_iAMT[1])
			CREDIT = DELTA / g_iAMT[2];
		else 
			CREDIT = 0;
			
		if(((DELTA >= 0) && (iCSB < 0)) || ((DELTA <= 0) && (iCSB > 0)) || (abs(DELTA) < g_iAMT[3]))
			iCSB = 0;   
		else
			iCSB = CREDIT + iCSB + ((DELTA > 0) ? 1 : ((DELTA < 0) ? -1 : 0));
		
		tmp = (g_iAMT[4]-abs(iCSB));
		SCALAR = (tmp>0)?tmp:0;
		old_cnt = old_cnt + (DELTA/(pow(SCALAR,2) + g_iAMT[5]));

		g_lPeso = old_cnt - g_lP0 ;   // ajuste del CERO;
		g_fPeso = floor(g_fP1 * g_lPeso/0.05)*0.05;   // peso calibrado

		if(g_bDiags) printf("%10ld, %10ld, %10ld, %10d\r", DELTA, CREDIT, SCALAR, iCSB);
		 
		// ZERO TRACKING
		
		g_fTemp = abs(g_fPeso-fPesoAnterior); // < g_fMotion;
	
		// MOTION
		if(g_fTemp < g_fMotn){
			//cout<<"menor \n\r"<<endl;
			if((iMotionCounter++ > g_iMtDly) && Motion)
				Motion = 0;
		}
		else{
			//printf("mayor %10f\n\r", g_fMotion);
			iMotionCounter = 0;
			Motion = 1;
			Under  = 1;
		}
	
		fPesoAnterior = g_fPeso;	
		newCount = false;

	}
}

void FILL()   //   ciclo de llenado
{
	if(staEstado == llenando){
	//	g_sOutputs[0]='F';
		if(g_fPeso < g_fTarget){
			FILL_ON;
			Filling = 1;
			//cout<<"FILL_ON"<<endl;
		}
		else{
			FILL_OFF;
			Filling = 0;
			//cout<<"FILL_OFF"<<endl;
					
			if(g_bStart == true && !Motion){
				//delay_ms(g_iDumpDly);
				
				g_fAcumulado+=g_fPeso;
				staEstado = vaciando;
				//cout<<"vaciando"<<endl;
			}
			else{
				staEstado = llenando;
				//cout<<"reposando"<<endl;
			}
		}
		//set_ticks(0);
	}
}

void DUMP ()   //   ciclo de descarga
{
	if(staEstado == vaciando){
		if(g_fPeso > g_fPesoMin){
		
			DUMP_ON;
			//cout<<"DUMP_ON"<<endl;
		}
		else{
			DUMP_OFF;
			//cout<<"DUMP_OFF"<<endl;
			if(g_bStart == true){
				//delay_ms(g_iFillDly);
				staEstado = llenando;
				//cout<<"llenando"<< endl;
			}
		}
	}
}

void CALIB()
{
	if(staEstado == calibrando)
		;//		cout << "\x1B[3;0H calib" << endl;	 	
	else
		;//		cout << "\x1B[3;0H      " << endl;
	 	
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

void SET_ZERO(void)	// Puesta a cero
{
	g_lP0 = g_lCountTotal;
	write_int32_eeprom(40, g_lP0);
	printf("P0=%lu\n\r",g_lP0);
}

void SET_SPAN(void)	// Rango
{
	g_lPeso = g_lCountTotal - g_lP0;
	g_fP1 = g_fTestWeight/g_lPeso;
	write_float_eeprom(60, g_fP1);
	printf("P1=%e\n\r",g_fP1);
}

void HOST_COMMANDS(void)
{
	char commands[3][10];
	char tok[] = ",";


	if(!input_ready) return;	//Si no se ha recibido ningun comando retorna
	cout << usr_input << endl;  
	
	char *ptr;	
	ptr = strtok(usr_input, tok);
	
	int i=0;	
	while(ptr!=0){
		strcpy(commands[i],ptr);
		i++;
		ptr = strtok(0, tok);
	}
		
	switch (commands[0]){
		case "S":
			g_bSetGAIN = true;
			GAIN_SET_COUNTER = atoi(commands[1]);
			break;
	
		case "Z":
			g_bSetZERO = true;
			ZERO_SET_COUNTER = atoi(commands[1]);
			break;
		
		case "R":
			START_PROCESS();
			break;
		
		case "P":	// stop
			//cout << "\x1B[3;1HStop req'd\x1B[K" << endl;
			DUMP_OFF;
			FILL_OFF;
			g_bStart = FALSE;
			break;
				
		case "PO":	// peso objetivo
			g_fTarget = atof(commands[1]);
			write_float_eeprom(64,g_fTarget);
			printf("PO, %f \r\n",g_fTarget);
			break;
			
		case "O":	// open
			if(!g_bStart){				
				DUMP_OFF;
				delay_ms(g_iFillDly);	//TODO: debe ser parametizable
				FILL_ON;
			}	//set_ticks(0);
			break;
		
		case "C":   // close
			if(!g_bStart){
				FILL_OFF;
				delay_ms(g_iDumpDly);
				DUMP_ON;
			}
			break;
		
		case "D":
			//cout<<"\x1B[3;0HDumping\x1B[K"<<endl;
			cal_on;
			FILL_ON;
			DUMP_ON;
			break;

		case "Q":	// request data
			UPDATE_HOST(atoi(commands[1]));
			break;

		case "PM":	// minimum weight
			g_fPesoMin = atof(commands[1]);
			write_float_eeprom(68,g_fPesoMin);
			printf("PM, %f \r\n",g_fPesoMin);
			break;
			
		case "ZT":	// zero tracking delay
			g_iZtDly = atoi(commands[1]);
			write_eeprom(10, g_iZtDly);
			printf("ZT, %u \r\n", g_iZtDly);
			break;

		case "ZD":	// zero divisions
			g_fZtDiv = atof(commands[1]);
			write_float_eeprom(84, g_fZtDiv);
			printf("ZD, %u \r\n", g_fZtDiv);
			break;
		
		case "MT":	// motion divisions
			g_fMotn = atof(commands[1]);
			write_float_eeprom(88, g_fMotn);
			printf("MT, %f \r\n", g_fMotn);
			break;
		
		case "MD":	// motion delay
			g_iMtDly = atoi(commands[1]);
			write_eeprom(12, g_iMtDly);
			printf("MD, %u \r\n", g_iMtDly);			
			break;
			
		case "A":	// smart filter input as: param, value
			i = atoi(commands[1]);
			g_iAMT[i] = atoi(commands[2]);
			write_eeprom((i-1)*2, g_iAMT[i]);
			delay_ms(4);
			int variable;
			read_eeprom(i,variable);
			cout << "Parametro " << i-1 << ", " << g_iAMT[i] << ": " << variable<< endl;
			break;
			
		case "TW":	// test weight
			g_fTestWeight = atof(commands[1]);
			write_float_eeprom(72,g_fTestWeight);
			printf("TW,%f\n\r",g_fTestWeight);
			break;
			
		case "FD":	// fil delay
			g_iFillDly = atoi(commands[1]);
			write_eeprom(18,g_iFillDly);
			printf("FD,%u\n\r",g_iFillDly);
			break;
		
		case "DD":	// dump delay
			g_iDumpDly = atoi(commands[1]);
			write_eeprom(20,g_iDumpDly);
			printf("DD,%lu\n\r",g_iDumpDly);
			break;
			
		case "K":	// moving average filter constant
			g_fKf1 = atof(commands[1]);
			g_fKf2 = atof(commands[2]);
			write_float_eeprom(76,g_fKf1);
			write_float_eeprom(80,g_fKf2);
			printf("k1 = %10f, k2 = %10f", g_fKf1, g_fKf2);
			break;
		
		case "DIAGS":
			g_bDiags = (g_bDiags) ? false:true;
			break;
			
		default:
			cout << "Comando nulo" << endl;
		break;
		}
	
	input_ready=FALSE;
	index=0;
	g_bKeying = false;
}

void UPDATE_HOST(int Tx = 0)
{
	current = get_ticks();
	switch (Tx){
		case 0:
			if ((current - Tx_t) > 100){
				printf("0,%12.2f, %12.2f, %x, %x, %x\r", g_fPeso, g_fAcumulado, g_cInputs, g_cOutputs, g_cStatus);
         		
//				printf("0,%12.2f, %12.2f, %x, %x, %x\r", g_fPeso, g_fTemp, g_cInputs, g_cOutputs, g_cStatus);
				Tx_t = current;
			}
			break;

		case 1:
			printf("1,%ld, %12ld, %12.2f\n\r", g_lCountTotal, g_lPeso, g_fPeso);							 
			break;
			
		case 2:
			printf("2,%ld, %ld, %e, %ld, %12.2f\n\r",g_lCountTotal, g_lP0, g_fP1, g_lPeso, g_fPeso);
		//		<< g_cStatus << g_cInputs << g_cOutputs
		//		<< endl;
			break;
			
		case 3:
			printf("3,LC1: %10ld, LC2: %10ld\n\r", g_lCnt_LC1, g_lCnt_LC2);
			break;
			
		case 4:
			printf("4,LC1: %10ld, %10ld, LC2: %10ld, %ld\n\r", g_lCount1, g_lLC1_P0, g_lCount2, g_lLC2_P0);
			break;
			
		case 5:
			cout << g_iAMT[1] << ", " << g_iAMT[2] << ", " << g_iAMT[3] << ", " << g_iAMT[4] << ", " << g_iAMT[5] << endl;
			printf("P0 = %ld, P1 = %e, PO = %f, Pmin = %f, Pp = %f\n\r", g_lP0, g_fP1, g_fTarget, g_fPesoMin, g_fTestWeight);
			printf("zt %u, zd %10f, md %u, mt %10f, fd %u, dd %u\n\r", g_iZtDly, g_fZtDiv, g_iMtDly, g_fMotn, g_iFillDly, g_iDumpDly);
			printf("kf1 = %10f, kf2 = %10f\n\r", g_fKf1, g_fKf2);
			break;
		
		case 99:
			break;
			
		default:
			printf("99,bad juju\r");
			break;			
				
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
	input_ready=false;
	
	
	// process variables

	staEstado = reposando;
	
	UPDATE_HOST(5);
	
	switch (restart_cause()){
		case RESTART_MCLR:
			cout<<"Restarted processor because of master clear!\r\n"<<endl;
			
		case RESTART_POWER_UP:
		{
			while(true)
			{
				restart_wdt();	
				READ_HX711();
				CONVERT_WEIGHT();
				FILL();
				DUMP();
				CALIB();
				HOST_COMMANDS();
				UPDATE_HOST(0);
				CHECK_TIME_OUT();
				BEAT;	// keep alive
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
