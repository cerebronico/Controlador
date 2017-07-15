////////////////////////////////////////////////////////////////////////
////             Main.c                                             ////
////////////////////////////////////////////////////////////////////////
/*

Promarisco - Clasificadoras
julio/2017

Controlador para balanza con HX711
   3 entradas IN0..IN2 + 2 E/S configurables A0, A1
   6 salidas (-1 por falta de IC apropiado)

   IN2 empacadora lista para recibir

   DONE se usa para indicar descarga de producto realizada

*/

#define debug      // comment this line in release version

#include <main.h>
#include <math.h>
#include <ios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <input.c>
#include <internal_eeprom.c>
#include <limits.h>

#ZERO_RAM

#define MaxTimeOut 4  //
#define tIni 0x8FFF

//============================================
//  Weighing parameter define
//============================================

char ID = '1';    	//device physical address

struct scale_parms{
	int16	displayed_resolution;
	int16	division_increment;
	int16	decimal_positions;
	float calibration_weight;
} scale;

int16 g_iAMT[6], 	// smart filter parameters must be stored and recalled from EEPROM
		g_iZtDly,	// zero tracking and motion delay in msec
		g_iMtDly,	//
		g_lFillDly,	// fill delay
		g_lDumpDly;	// dump delay

//int32 g_lP0;  		// unloaded scale count value

float g_fP0,
		g_fP1,		// gain &
		g_fKf,	 	//filter
		g_fAcumulado,
		g_fPesoMin,
		g_fTarget,
		g_fZtDiv,
		g_fMotn;


long t;	//RS-232 timeout

//============================================
//  global variable define
//============================================

boolean is_newCount,
		timeout_filling,
		TimeOut = False,
		g_bDiags,
		is_lc_data_ready,
		input_ready,   // this will signal to the kernel that input is ready to be processed
		BoRx, EoRx, BadRx = False;  //controles de recepción de datos del PC

int16 ZERO_SET_COUNTER, GAIN_SET_COUNTER, lDropCounter;
int32 g_lPeso, g_lCnt_LC, g_lLC_P0, g_lCount, g_lCountTotal;
float g_fPeso, multiplicador, g_fLastWeight;

char usr_input[30];      // this character array will be used to take input from the prompt
int index;              // this will hold the current position in the array

// enumerate different commands
enum _Mode
	{ Normal
	, SetZero
	, SetGain
	} eMode;

enum _State
	{ llenando
	, completando
	, comprobando
	, reposando
	, vaciando
	, esperandoLlenar
	, esperandoVaciar
	, vaciar
	, lleno
	, vacio
	} eEstado;

typedef union
{
   float f;
   struct{
      unsigned int16 mantisa;
      unsigned int16 exponent;
   } parts;
} float2eeprom;

typedef unsigned int32 tick_t;
tick_t fill_timer,
		fill_delay,
		fill_initiated,
		fill_timeout_timer,
		g_lFill_TOut,
		dump_timer,
		dump_delay;

// flag definitions
char g_cInputs, g_cOutputs, g_cStatus;

#bit I0 = g_cInputs.0      //
#bit I1 = g_cInputs.1      //
#bit I2 = g_cInputs.2      // empacadora lista
#bit I3 = g_cInputs.3      //
#bit I4 = g_cInputs.4      //
#bit I5 = g_cInputs.5      //
#bit descarga = g_cInputs.6   // habilita la descarga si está lista

#bit bFast_Fill = g_cOutputs.0
#bit bSlow_Fill = g_cOutputs.1
#bit bDump = g_cOutputs.2
#bit bFilling = g_cOutputs.3
#bit bDumping = g_cOutputs.4
#bit bVacio = g_cOutputs.5
#bit bRdy2Dump = g_cOutputs.6
#bit bfeeder = g_cOutputs.7

#bit bZero = g_cStatus.0
#bit bMotion = g_cStatus.1
#bit bOver = g_cStatus.2
#bit bUnder = g_cStatus.3
#bit bStart = g_cStatus.4
#bit bcheck = g_cStatus.5
#bit bEmpacadoraLista = g_cStatus.6 // captura del pulso de empacadora lista
#bit bCalibrating = g_cStatus.7

//============================================
//  interrupt service
//  timer interrupt
//  uart rx interrupt
//============================================

unsigned int32 trapaddr = 0;

#INT_ADDRERR				//Address error trap
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
   //printf("Addr %Lx", trapaddr);
   while(TRUE);
}

#INT_MATHERR
void MATH_ERR(void)		//Fallo matemático
{
   //printf("error matemático\r\n");
   while(TRUE);
}

#INT_RDA
void USER_INPUT ( )		// serial port interrupt (host communication)
{
   static int8 idx; //puntero índice;
   char c;
   c = getc();

   set_timer1(tIni);
   clear_interrupt(int_timer1);
   enable_interrupts(int_timer1);

   if (BadRx) return;

   if (!BoRx){
      if (c!=ID){
         BadRx= true;
         return;
      }
      else{
         idx=0;
         BoRx = true;
      }
   }

   if( idx > 30){
      idx = 0;
      usr_input[1]='e';
      usr_input[2] ='\0';
      BadRx = True;
      return;
   }

   usr_input[idx++] = c;
   usr_input[idx+1] ='\0';
}

#INT_EXT0
void WS_DAT_isr(void)	// lectura de convertidor
{
   int32 count=0;
   char i;
   for (i=0;i<24;i++)
   {
      output_high(WS_CLK);

      if(i<24)
         count = count << 1;

      output_low(WS_CLK);
      if(WS_DAT) count++;
   }

   output_high(WS_CLK);

   g_lCount = count;
   if(bit_test(count,23))
      g_lCount|=0xFF800000;
   output_low(WS_CLK);   // start new conversion

   is_lc_data_ready = true;
}


#INT_TIMER1
void timer1_isr(void)	// con este verificamos el fin de la trasnmision del mensaje de RS232
{
  SET_TIMER1(tIni);
  t++;

  if(t>MaxTimeOut)
    {
    disable_interrupts(int_timer1);
    t=0;
    BoRx = false;
    EoRx = BadRx?False:True; // si la trasnmision es buena señalamos con EoRx
    BadRx = False;
    TimeOut = True;
    }
}

//============================================
//
//  Functions
//
//============================================

void init_hardware(void)	//  initialize system hardware config
{
   setup_adc_ports(NO_ANALOGS);
   set_tris_b(0b0000001111000000);
   set_tris_d(0xFFFF);
   setup_timer1 ( TMR_INTERNAL | TMR_DIV_BY_8 );

   enable_interrupts(INT_EXT0);
   ext_int_edge(0,H_TO_L);
   enable_interrupts(INT_EXT1);
   ext_int_edge(1, H_TO_L);
   enable_interrupts(INT_RDA);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INTR_GLOBAL);

   output_low(WS_CLK);   // HX711 normal operation

   // smart filter params

   read_eeprom(0, g_iAMT[1]);   // g_iAMT[1] = 50;
   read_eeprom(2, g_iAMT[2]);   // g_iAMT[2] = 20;
   read_eeprom(4, g_iAMT[3]);   // g_iAMT[3] = 2;
   read_eeprom(6, g_iAMT[4]);   // g_iAMT[4] = 8;
   read_eeprom(8, g_iAMT[5]);   // g_iAMT[5] = 1;

   // operational params

   read_eeprom(10, g_iZtDly);
   read_eeprom(12, g_iMtDly);
   read_eeprom(14, g_lFillDly);
   read_eeprom(18, g_lDumpDly);
   read_eeprom(22, g_lFill_TOut);

   // scale params
   scale.calibration_weight = read_float_eeprom(26);   // CAL TEST WEIGHT
   scale.decimal_positions = read_eeprom(30);
   scale.displayed_resolution = read_eeprom(32);
   scale.division_increment = read_eeprom(34);
   
   multiplicador = pow(10.0,(float)scale.decimal_positions);

   g_lLC_P0 = read_int32_eeprom(44);  	// ZERO
   g_fP1 = read_float_eeprom(60);   	// GAIN
   g_fTarget = read_float_eeprom(64);  // TARGET WEIGHT
   g_fPesoMin = read_float_eeprom(68); // MIN WEIGHT TO CONSIDER FOR ZERO
   g_fKf = read_float_eeprom(80);   	// moving average filter constants
   g_fZtDiv = read_float_eeprom(84);   // zero tracking range
   g_fMotn  = read_float_eeprom(88);   // motion tracking
}

//============================================
//
//  Function prototypes
//
//============================================

int read_hx711(int MODE = 0);
void host_commands (void);
void convert_to_weight(int32 cnt);
void update_host(int Tx=99);
void update_io();
void fill();   // llenar

void read_hx711(_Mode m)
{
   static int32 ad_count;	// analog-to-digital converter count

   if(is_lc_data_ready){   // si el convertidor tiene datos

      g_lCnt_LC = g_lCount - g_lLC_P0;
      g_lCountTotal = g_lCnt_LC;

      switch (m)
      {
         case Normal:
            is_lc_data_ready = false;
            is_newCount = true;
            break;

         case SetZero:
            bCalibrating = 1;
            if((ZERO_SET_COUNTER--) >=0){
               ad_count = ad_count + g_fKf*(g_lCount-ad_count);
            }
            else{
               g_lLC_P0 = ad_count;
               write_int32_eeprom(44, g_lLC_P0);
               bCalibrating = 0;
               printf("ZEROING DONE!, P0 = %ld\r", g_lLC_P0);
               eMode = Normal;
            }

            is_lc_data_ready = false;
            is_newCount = false;
            break;

         case SetGain:
            if(GAIN_SET_COUNTER-- >=0){

               g_lPeso = g_lCnt_LC;

               g_fP1 = scale.calibration_weight * multiplicador / g_lPeso;
               write_float_eeprom(60, g_fP1);
               printf("P1=%e, lc= %ld, mul= %f\n\r",g_fP1, g_lCnt_LC, multiplicador);
               printf("%c,GAIN DONE!\r",ID);
               eMode = Normal;
            }
            break;
            
         default:
      }
   }
}

void start_process()
{
   bStart = true;
   bVacio = true;
   eEstado = vacio;
}

void peso_minimo(char *par)
{
   g_fPesoMin = atof(par);
   write_float_eeprom(68, g_fPesoMin);
   printf("PM, %8f\r", g_fPesoMin);
}

void zero_tracking(char *par)
{
   g_iZtDly = atoi(par);
   write_eeprom(10, g_iZtDly);
   printf("ZT, %u\r", g_iZtDly);
}

void zero_divisions(char *par)
{
   g_fZtDiv = atof(par);
   write_float_eeprom(84, g_fZtDiv);
   printf("ZD, %12f\r", g_fZtDiv);
}

void motion_tracking(char *par)
{
   g_iMtDly = atoi(par);
   write_eeprom(12, g_iMtDly);
   printf("MT, %u\r", g_iMtDly);
}

void motion_divisions(char *par)
{
   g_fMotn = atof(par);
   write_float_eeprom(88, g_fMotn);
   printf("MD, %12f\r", g_fMotn);
}

void convert_to_weight()   // read Weight data
{
   static int32 old_cnt, DELTA, CREDIT, SCALAR;
   static int iCSB, iMotionCounter, iZeroCounter;
   static float fPesoAnterior;
   signed int tmp;

   if(is_newCount){

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

      g_lPeso = old_cnt;	// - g_lP0 ;   // ajuste del CERO;
      
      // PESO CALIBRADO
      g_fPeso = (g_fP1 * g_lPeso);
      g_fPeso = (floor((0.5 + g_fPeso)/scale.division_increment))*scale.division_increment;
      g_fPeso /= multiplicador;
      g_fPeso -= g_fP0;

      if(g_bDiags) printf("%10ld, %10ld, %10ld, %10d\r", DELTA, CREDIT, SCALAR, iCSB);

      // MOTION
      if(g_fMotn > abs(g_fPeso-fPesoAnterior)){
         if((iMotionCounter++ > g_iMtDly) && bMotion)
            bMotion = 0;
      }
      else{
         iMotionCounter = 0;
         bMotion = 1;
      }
      
      // ZERO TRACKING
      if(abs(g_fPeso) < g_fZtDiv){
         if((iZeroCounter++ > g_iZtDly) && !bZero)
         bZero = 1;
      }
      else{
         iZeroCounter = 0;
         bZero = 0;
      }
      
      fPesoAnterior = g_fPeso;
      is_newCount = false;
   }
}

void fill()   //   ciclo de llenado
{
	if(bStart){
		fill_timer = get_ticks();
		fill_timeout_timer = fill_timer;
      dump_timer = fill_timer;
      switch (eEstado){
         case vacio:
            if(g_fPeso < g_fTarget) {
            	if(abs(g_fPeso) < g_fPesoMin){
            		g_fP0 = g_fPeso;
            	}
            	fill_initiated = get_ticks();
               FAST_ON;   // this is the physical output
               SLOW_ON;
               bFast_Fill = 1;
               bSlow_Fill = 1;
               bFilling = 1;
               bDumping = 0;
               bRdy2Dump = 0;
               bVacio = 0;
               eEstado = llenando;
            }
            else{
            	FAST_OFF;   // this is the physical output
            	SLOW_OFF;
            	eEstado = lleno;
            }
            
            break;

         case llenando:
         	if(g_fPeso > g_fPesoMin){
	            timeout_filling = (fill_timeout_timer - fill_initiated) > g_lFill_TOut;	
         	}
         	else{
         		fill_initiated = get_ticks();
         		timeout_filling = false;
         	}

				if ((g_fPeso >= g_fTarget) || timeout_filling){
                  FAST_OFF;
                  SLOW_OFF;
                  bFast_Fill = 0;   // this is a flag only
                  bSlow_Fill = 0;
                  bFilling = 1;
                  eEstado = lleno;
            }
            break;

         case lleno:
            if(!bMotion){     
               lDropCounter++;   // contador de descargas
               g_fAcumulado += g_fPeso;
               g_fLastWeight = g_fPeso;

               bFilling = 0;
               bRdy2Dump = 1;

               eEstado = vaciar;
               }
            
            break;
            
         case vaciar:
            bDumping = 1;
            DUMP_ON;
            bDump = 1;
            eEstado = vaciando;
            bRdy2Dump = 0;
            break;

         case vaciando:
            if(g_fPeso < g_fPesoMin){
               eEstado = esperandoVaciar;
               dump_delay = get_ticks();
            }
            break;

         case esperandoVaciar:

            if((dump_timer - dump_delay) > g_lDumpDly){
               DUMP_OFF;
               bDumping = 0;
               descarga = 0;
               bDump = 0;
               eEstado = esperandoLlenar;
               g_fP0 = 0;
               fill_delay = get_ticks();
            }
            break;			      

         case esperandoLlenar:
            if((fill_timer - fill_delay) > g_lFillDly){
               eEstado = vacio;
               bEmpacadoraLista = 0;
            }
            break;
         default:
      }
   }
}

void host_commands(void)
{
   char commands[4][10];
   char tok[] = ",";
   char n_usr_input[30];

   if(EoRx){

      if(usr_input[0] == ID) {   //Si no se ha recibido ningun comando retorna

	      // cout << usr_input[0] << endl;
	      strcpy(n_usr_input, &usr_input + 1);
	      // cout << n_usr_input << endl;

	      char *ptr;   // split commands
	      ptr = strtok(n_usr_input, tok);

	      int i=0;
	      while(ptr!=0){
	         strcpy(commands[i],ptr);
	         i++;
	         ptr = strtok(0, tok);
      }

      switch (commands[0]){
         case "G":
            eMode = SetGain;
            GAIN_SET_COUNTER = atoi(commands[1]);
            break;

         case "Z":
            eMode = SetZero;
            ZERO_SET_COUNTER = atoi(commands[1]);
            printf("ZEROING, WAIT... \r");
            break;

			case "TZ":	// temporal zero
				g_lLC_P0 = g_lCount;
         	break;

			case "TW":   // test weight
            scale.calibration_weight = atof(commands[1]);
            write_float_eeprom(26,scale.calibration_weight);
            printf("TW,%f\n\r", scale.calibration_weight);
            break;
 	
         case "RES":	// resolucion
         	scale.displayed_resolution = atoi(commands[1]);
         	write_eeprom(32, scale.displayed_resolution);
            printf("RES,%u\n\r", scale.displayed_resolution);
         	break;
         	
         case "DIV":
         	scale.division_increment = atoi(commands[1]);
         	write_eeprom(34, scale.division_increment);
            printf("DIV,%u\n\r", scale.division_increment);
         	break;
         	
         case "DEC":
         	scale.decimal_positions = atoi(commands[1]);
         	multiplicador = pow(10.0, (float)scale.decimal_positions);
         	write_eeprom(30, scale.decimal_positions);
            printf("DEC,%f\n\r", multiplicador);
         	break;
         	
         case "R":
            if(!bStart)
               start_process();
            break;

         case "P":   // stop
            DUMP_OFF;
            DONE_OFF;
            VIB_GRUESO_OFF;
            FAST_OFF;
            SLOW_OFF;
            bFast_Fill = 0;
            bSlow_Fill = 0;
            bStart = FALSE;
            bDump = bDumping = bFilling = bRdy2Dump = descarga = bVacio = 0;
            break;

         case "PO":   // peso objetivo
            g_fTarget = atof(commands[1]);
            write_float_eeprom(64,g_fTarget);
            printf("PO, %f\r",g_fTarget);
            break;

         case "FE":
            if(!bStart){
               if( bFeeder ) {
                   VIB_GRUESO_OFF;
                   bFeeder = 0;
               }
               else {
                  bFeeder = 1;
                  VIB_GRUESO_ON;
               }
            }
            break;

         case "FF":   // fast feed test
            if(!bStart){
               if( bFast_Fill ) {
                  bFast_Fill = 0;
                  FAST_OFF;
               }
               else {
                  bFast_Fill = 1;
                  FAST_ON;
               }
            }
            break;

         case "SF":   // slow feed test
            if(!bStart) {
               IF (bSlow_Fill) {
                  bSlow_Fill = 0;
                  SLOW_OFF;
               }
               else {
                  bSlow_Fill = 1;
                  SLOW_ON;
               }
            }
            break;

         case "D":   // dump test
            IF (bDump) {
               bDump = 0;
               DUMP_OFF;
               DONE_ON;
            }
            else {
               bDump = 1;
               DUMP_ON;
               DONE_OFF;
            }
            break;

         case "Q":   // request data
            int8 temp = atoi(commands[2]);
            if(bDumping && IN2){
               if( temp & 0x40){
                  descarga = TRUE;
                  DONE_ON;
               }
            }

            update_host(atoi(commands[1]));
            break;

         case "PM":   // minimum weight
            peso_minimo(commands[1]);
            break;

         case "ZD":   // zero divisions
            zero_divisions(commands[1]);
            break;

         case "ZT":   // zero tracking delay
            zero_tracking(commands[1]);
            break;

         case "MD":   // motion divisions
            motion_divisions(commands[1]);
            break;

         case "MT":   // motion delay
            motion_tracking(commands[1]);
            break;

         case "A":   // smart filter input as: param, value
            i = atoi(commands[1]);
            g_iAMT[i] = atoi(commands[2]);
            write_eeprom((i-1)*2, g_iAMT[i]);
            delay_ms(4);
            int variable;
            read_eeprom((i-1)*2,variable);
            cout << "Parametro " << i-1 << ", " << g_iAMT[i] << ": " << variable<< endl;
            break;

         case "OP":   // operating parameters fill delay, dump delay
         	int16 to_secs = 1220;
            g_lFillDly = atoi(commands[1])*to_secs;
            write_eeprom(14,g_lFillDly);
            g_lDumpDly = atoi(commands[2])*to_secs;
            write_eeprom(18,g_lDumpDly);
            g_lFill_TOut = atoi32(commands[3])*to_secs;
            write_eeprom(22,g_lFill_TOut);

            printf("OP, %lu, %lu, %lu\n\r",g_lFillDly, g_lDumpDly, g_lFill_TOut);
            break;

         case "K":   // moving average filter constant
            g_fKf = atof(commands[1]);
            write_float_eeprom(80,g_fKf);
            printf("k = %10f", g_fKf);
            break;

         case "DIAGS":
            g_bDiags = (g_bDiags) ? false:true;
            break;

         default:
            cout << "Comando nulo" << endl;
         break;
         }
      }

      EoRx = FALSE;
      index = 0;
   }
}

void update_host(int Tx = 99)
{

   switch (Tx){
      case 0:   //default transmission
         printf("%9f,0,%lu,%12f,%12f,%x,%x,%x\r", g_fP0, lDropCounter, g_fPeso, g_fLastWeight, g_cInputs, g_cOutputs, g_cStatus);
         break;

      case 1:
         printf("%c,1,%ld,%12ld,%12f,%12f,", ID, g_lCountTotal, g_lPeso, g_fPeso, g_fTarget);
         printf("%u,%u,%u\r", scale.displayed_resolution, scale.division_increment, scale.decimal_positions);
         break;

      case 4:
         printf("%c,4, %10ld, %ld\r", ID, g_lCount, g_lLC_P0);
         break;

      case 5:
         printf("%c,5,%ld,%ld,%ld,%ld,%ld,%.3f,", ID, g_iAMT[1], g_iAMT[2], g_iAMT[3], g_iAMT[4], g_iAMT[5], g_fKf);
         printf("%ld,%e,%12f,%12f,%12f,", g_lLC_P0, g_fP1, g_fTarget, g_fPesoMin, scale.calibration_weight);
         printf("%u,%6.3f,%u,%6.3f,%lu,%lu,%lu\r", g_iZtDly, g_fZtDiv, g_iMtDly, g_fMotn, g_lFillDly, g_lDumpDly, g_lFill_TOut);
         break;

      case 99:
         break;

      default:
         printf("%c,99, bad juju\r", ID);
         break;
   }
}

void update_io(void)
{
   I0 = IN0;
   I1 = IN1;
   I2 = IN2;
}

//=================================
//   MAIN
//=================================
//	TODO:
//  auto cero
//	decimales y minima división
//	auto descarga

void main()
{
   init_hardware();
   
   cout<<"Clasificadora No.1\r"<<endl;

   // initialize input variables
   index=0;
   input_ready = false;

   // process variables

   eEstado = reposando;
   eMode = Normal;
   g_fP0 = 0;

   switch (restart_cause()){
      case RESTART_MCLR:
        //cout<<"Restarted processor because of master clear!\r"<<endl;

      case RESTART_POWER_UP:
      {
         while(true)
         {
            restart_wdt();
            read_hx711(eMode);
            convert_to_weight();
            fill();
            host_commands();
            update_host();
            update_io();
            BEAT;   // keep alive
         }
         break;
      }
      case RESTART_WATCHDOG:
      {
         //cout<<"Restarted processor because of watchdog timeout!"<<endl;
         break;
      }
      case RESTART_BROWNOUT:
      {
         //cout<<"Restarted processor because of brownout!"<<endl;
         break;
      }
      case RESTART_SOFTWARE:
      {
         //cout<<"Restarted processor because of restart software!"<<endl;
         break;
      }
      case RESTART_TRAP_CONFLICT :
      {
         //cout<<"Restarted processor because of trap conflict!"<<endl;
         break;
      }
   }
    while (true){
      BEAT;   // error trap
  	}
}
