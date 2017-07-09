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
#include "limits.h"

#ZERO_RAM

#define MaxTimeOut 4  // 
#define tIni 0x8FFF

//============================================
//  Weighing parameter define
//============================================

char ID = '1';    	//device physical address

struct scale_parms{
	int displayed_resolution;
	int	division_increment;
	int	decimal_position;
	float calibration_weight;
} balanza;

int16 g_iAMT[5]  	// smart filter parameters must be stored and recalled from EEPROM
	, g_iZtDly
	, g_iMtDly
	, g_lFillDly
	, g_lDumpDly
	, g_lLockDly;   // zero tracking and motion delay in msec
 
int32 g_lP0;  		// unloaded scale count value 

float g_fP1, g_fKf	// gain & filter
	, g_fAcumulado
	, g_fPesoMin
	, g_fTarget
	, g_fPreliminar
	, g_fFree_Fall
	, g_fTol
	, g_fZtDiv
	, g_fMotn
	, g_fTemp;

long t;	//RS-232 timeout

//============================================
//  global variable define
//============================================

boolean newCount,
      	TimeOut = False,
      	g_bDiags,
      	is_lc_data_ready,
      	input_ready,   // this will signal to the kernel that input is ready to be processed
      	BoRx, EoRx, BadRx = False;  //controles de recepción de datos del PC

int16 ZERO_SET_COUNTER, GAIN_SET_COUNTER, lDropCounter, check_counter;
int32 g_lPeso, g_lCnt_LC, g_lLC_P0, g_lCount, g_lCountTotal;
float g_fPeso, g_fLastWeight, g_fTestWeight;

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
	} staEstado;
	
typedef union 
{
   float f;
   struct{
      unsigned int16 mantisa;
      unsigned int16 exponent;
   } parts;
} float2eeprom;

typedef unsigned int32 tick_t;
tick_t fill_timer, fill_delay, dump_timer, dump_delay, lock_timer, lock_delay;

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
#bit bPack = g_cOutputs.5
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

#INT_ADDRERR             //Address error trap
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
void MATH_ERR(void)   //Fallo matemático
{
   //printf("error matemático\r\n");
   while(TRUE);
}

#INT_RDA 
void USER_INPUT ( )      // serial port interrupt
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
void WS_DAT_isr(void)   // lectura de convertidor
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
void timer1_isr(void)  // con este verificamos el fin de la trasnmision del mensaje de RS232
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

void init_hardware(void)   //  initialize system hardware config
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
   read_eeprom(22, g_lLockDly);
   
   // scale params                             
   g_lLC_P0 = read_int32_eeprom(44);   	// ZERO
   g_fP1 = read_float_eeprom(60);   	// GAIN
   g_fTarget = read_float_eeprom(64);   //TARGET WEIGHT
   g_fPesoMin = read_float_eeprom(68);  // MIN WEIGHT TO CONSIDER FOR ZERO
   g_fTestWeight = read_float_eeprom(72);   // CAL TEST WEIGHT
   g_fPreliminar = read_float_eeprom(76);	// preliminar weight for fast feed
   g_fKf = read_float_eeprom(80);   	// moving average filter constants
   g_fZtDiv = read_float_eeprom(84);   	// zero tracking range
   g_fMotn   = read_float_eeprom(88);   // motion tracking
   g_fTol = read_float_eeprom(92);   	// motion tracking
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
void dump();   // descargar

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
            newCount = true;                
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
               printf("%c,ZEROING DONE!\r",ID);
               eMode = Normal;
            }            
            
            is_lc_data_ready = false;         
            newCount = false;
            break;
         
         case SetGain:
            if(GAIN_SET_COUNTER-- >=0){
               
               g_lPeso = g_lCnt_LC;
                                 
               g_fP1 = g_fTestWeight / g_lPeso;
               write_float_eeprom(60, g_fP1);
               //printf("P1=%e, lc= %ld\n\r",g_fP1, g_lCnt_LC);
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
   bStart = TRUE;
   staEstado = vacio;    
}

void peso_minimo(char *par)
{
   g_fPesoMin = atof(par);
   write_float_eeprom(68,g_fPesoMin);
   printf("PM, %f\r",g_fPesoMin);
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
   printf("ZD, %f\r", g_fZtDiv);
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
   printf("MD, %f\r", g_fMotn);
}

void convert_to_weight()   // read Weight data
{
   static int32 old_cnt, DELTA, CREDIT, SCALAR;
   static int iCSB, iMotionCounter, iZeroCounter;
   static float fPesoAnterior;
   signed int tmp;

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
      g_fPeso = floor(g_fP1 * g_lPeso / balanza.division_increment + 0.5) * balanza.division_increment;   // peso calibrado
      
      if(g_bDiags) printf("%10ld, %10ld, %10ld, %10d\r", DELTA, CREDIT, SCALAR, iCSB);
       
      g_fTemp = abs(g_fPeso-fPesoAnterior); // < g_fMotion;
   
      // MOTION
      if(g_fTemp < g_fMotn){
         //cout<<"menor \n\r"<<endl;
         if((iMotionCounter++ > g_iMtDly) && bMotion)
            bMotion = 0;
      }
      else{
         //printf("mayor %10f\n\r", g_fMotion);
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
      newCount = false;
   }
}

void fill()   //   ciclo de llenado
{
   if(bStart){
      lock_timer = get_ticks();
      
      switch (staEstado){
         case vacio:
            if(g_fPeso < g_fPreliminar) {//         }&& (fill_timer - fill_delay) > g_iFillDly){
               VIB_GRUESO_ON;
               lock_delay = get_ticks();
               FAST_ON;   // this is the physical output
               SLOW_ON;
               bFast_Fill = 1;
               bSlow_Fill = 1;
               bFilling = 1;
               bDumping = 0;
               bRdy2Dump = 0;
               bPack = 0;
            }
            staEstado = llenando;   
            break;
         
         case llenando:
            if (g_fPeso >= g_fPreliminar) {
               if((lock_timer - lock_delay) > g_lLockDly){             
                  VIB_GRUESO_OFF;
                  FAST_OFF;
                  SLOW_ON;
                  bFast_Fill = 0;   // this is a flag only
                  bSlow_Fill = 1;
                  bFilling = 1;
                  staEstado = Completando;
               }
            }
            break;
            
         case completando:
            if(check_counter > 0){
               if(g_fFree_Fall > 0){
                  g_fFree_Fall -= 1;
               }              
            }
            
            if(g_fPeso >= (g_fTarget - g_fFree_Fall)) {
               SLOW_OFF;
               bSlow_Fill = 0;
               staEstado = lleno;
            }
            break;
                      
         case lleno:
            if(!bMotion){               
               if(g_fPeso > g_fTarget){
                  lDropCounter++;   // contador de descargas
                  g_fAcumulado += g_fPeso;
                  g_fLastWeight = g_fPeso;

                  bFilling = 0;
                  bRdy2Dump = 1;
                  check_counter = 0;
                  staEstado = vaciar;
               }
               
            }
            break;
            
         default:
      }
   }
}

void dump ()   //   ciclo de descarga
{
   if(bStart){
      dump_timer = get_ticks();
      fill_timer = dump_timer;

      switch (staEstado){
         case vaciar:
            bDumping = 1;         
            DUMP_ON;
            bDump = 1;
            staEstado = vaciando;
            bDumping = 0;
            bRdy2Dump = 0;
            break;

         case vaciando:
            if(g_fPeso < g_fPesoMin){
               staEstado = esperandoVaciar;
               dump_delay = get_ticks();             
            }
            break;

         case esperandoVaciar:
            
            if((dump_timer - dump_delay) > g_lDumpDly){
               DUMP_OFF;
               DONE_OFF; // indica finalizada la descarga
               descarga = 0;
               bPack = 1;
               bDump = 0;
               staEstado = esperandoLlenar;
               fill_delay = get_ticks();
            }
            break;
            
         case esperandoLlenar:
            if((fill_timer - fill_delay) > g_lFillDly){
               staEstado = vacio;
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
            
		case "TZ":
			g_lLC_P0 = g_lCount;
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
            bDump = bDumping = bFilling = bRdy2Dump = descarga = bPack = 0;
            break;
               
         case "PO":   // peso objetivo
            g_fTarget = atof(commands[1]);
            write_float_eeprom(64,g_fTarget);
            g_fPreliminar = atof(commands[2]);
            write_float_eeprom(76,g_fPreliminar);
            g_fTol = atof(commands[3]);
            write_float_eeprom(92,g_fTol);
            printf("PO, %f, %f, %f \r",g_fTarget,g_fPreliminar,g_fTol);
            break;

         case "PK":
            if(!bStart){
               if(bPack) {
                  DONE_OFF;
                  bPack = 0;
               }
               else{
                  DONE_ON;
                  bPack = 1;
               }
            }
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
            read_eeprom(i,variable);
            cout << "Parametro " << i-1 << ", " << g_iAMT[i] << ": " << variable<< endl;
            break;
            
         case "TW":   // test weight
            g_fTestWeight = atof(commands[1]);
            write_float_eeprom(72,g_fTestWeight);
            printf("TW,%f\n\r",g_fTestWeight);
            break;
            
         case "OP":   // operating parameters fill delay, dump delay
            g_lFillDly = atoi(commands[1]);
            write_eeprom(14,g_lFillDly);
            g_lDumpDly = atoi(commands[2]);
            write_eeprom(18,g_lDumpDly);
            g_lLockDly = atoi(commands[3]);
            write_eeprom(22,g_lLockDly);
            printf("OP, %lu, %lu, %lu\n\r",g_lFillDly, g_lDumpDly, g_lLockDly);
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
         printf("%c,0,%lu,%8.0f,%8.0f,%10.0f,%x,%x,%x\r", ID, lDropCounter, g_fPeso, g_fLastWeight, g_fFree_Fall, g_cInputs, g_cOutputs, g_cStatus);
         break;

      case 1:
         printf("%c,1,%ld,%12ld,%12.0f,%12.0f,%12.0f, %12.0f, %12.0f\r", ID, g_lCountTotal, g_lPeso, g_fPeso, g_fTarget, g_fPreliminar, g_fTol, g_fFree_Fall);                      
         break;
                  
      case 4:
         printf("%c,4, %10ld, %ld\r", ID, g_lCount, g_lLC_P0);
         break;
         
      case 5:
         printf("%c,5,%ld,%ld,%ld,%ld,%ld,%.3f,", ID, g_iAMT[1], g_iAMT[2], g_iAMT[3], g_iAMT[4], g_iAMT[5], g_fKf);
         printf("%ld,%e,%f,%f,%f,", g_lLC_P0, g_fP1, g_fTarget, g_fPesoMin, g_fTestWeight);
         printf("%u,%6.2f,%u,%6.2f,%lu,%lu, %lu\r", g_iZtDly, g_fZtDiv, g_iMtDly, g_fMotn, g_lFillDly, g_lDumpDly, g_lLockDly);
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
   bEmpacadoraLista = I2;
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

   // initialize input variables
   index=0;
   input_ready = false;
   
   // process variables

   staEstado = reposando;
   eMode = Normal;
   
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
            dump();
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
