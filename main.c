////////////////////////////////////////////////////////////////////////
////             Main.c                                 ////
////////////////////////////////////////////////////////////////////////

/*
Controlador para balanza con HX711
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

//============================================
//  Weighing parameter define
//============================================

char ID = '3';    //device physical address

int16 g_iAMT[5];  // smmart filter parameters must be stored and recalled from EEPROM
 
signed int32 g_lP0;  // unloaded scale count value
float g_fP1, g_fKf;  // gain & filter
int16 g_iZtDly, g_iMtDly, g_iFillDly, g_iDumpDly, g_iTimeOut;   // zero tracking and motion delay in msec
float g_fAcumulado, g_fPesoMin, g_fTarget, g_fZtDiv, g_fMotn, g_fTemp;

//============================================
//  global variable define
//============================================

boolean newCount;

signed int32 g_lPeso, g_lCnt_LC, g_lLC1_P0, g_lLC_P0, g_lCount, g_lCountTotal;
float g_fPeso, g_fLastWeight, g_fTestWeight;
boolean g_bDiags, g_bKeying, LC_DATA_RDY;
int16 ZERO_SET_COUNTER, GAIN_SET_COUNTER, lDropCounter;

char usr_input[30];      // this character array will be used to take input from the prompt
int index;            // this will hold the current position in the array
boolean input_ready;   // this will signal to the kernel that input is ready to be processed

// enumerate different commands
enum _Mode {Normal, SetZero, SetGain} eMode;
enum _State {llenando, reposando, vaciando, esperandoLlenar, esperandoVaciar, vaciar, lleno, vacio} staEstado;

typedef union 
{
   float f;
   struct{
      unsigned int16 mantisa;
      unsigned int16 exponent;
   } parts;
} float2eeprom;

typedef unsigned int32 tick_t;

tick_t fill_timer, fill_delay, dump_timer, dump_delay, fill_timeout_timer, fill_timeout_delay;

// flag definitions
char g_cInputs, g_cOutputs, g_cStatus;

#bit I0 = g_cInputs.0      // dump gate limit switch
#bit I1 = g_cInputs.1      // fill gate limit switch
#bit I2 = g_cInputs.2      // overrun flag
#bit I3 = g_cInputs.3      // overdump flag
#bit I4 = g_cInputs.4      // airpressure flag
#bit I5 = g_cInputs.5      // extra flag

#bit bFast_Fill = g_cOutputs.0
#bit bDump = g_cOutputs.1
#bit bCalibrating = g_cOutputs.2
#bit bAlarming = g_cOutputs.3
#bit bFilling = g_cOutputs.4
#bit bDumping = g_cOutputs.5
#bit bSlow_Fill = g_cOutputs.6

#bit bMotion = g_cStatus.0
#bit bOver = g_cStatus.1
#bit bUnder = g_cStatus.2
#bit bStart = g_cStatus.3
#bit bTime_Out = g_cStatus.4
#bit bZero = g_cStatus.5


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
   output_high(O0);
   output_high(O1);
   
   if(index<29){
      output_low(O0);
      usr_input [index] = getc();   // get the value in the serial recieve reg

      if(usr_input[index]==0x0d) {
         
         output_low(O1);   // test output
         
         if(usr_input[0]==ID) {      // if the input was enter
            usr_input [ index ] = '\0';   // add the null character
            input_ready=TRUE;         // set the input read variable to true
            
            return;
         }
         else {
            input_ready=FALSE;
            usr_input[0] = '\0';
            index=0; // and reset the index
            return;
         }
      }
      else
            index++;
      }
   
   else{
      usr_input [ 0 ] = '\0';
      index = 0;
      input_ready = FALSE;
   }
}

#INT_EXT0
void WS_DAT_isr(void)   // lectura de convertidor 2
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

   LC_DATA_RDY = true;
}

#INT_EXT2   
void in1_isr(void)   // lectura de convertidor 2
{ 
   ;
}


//============================================
//  
//  Functions
//
//============================================

void INIT_HARDWARE(void)   //  initialize system hardware config
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
   read_eeprom(14, g_iFillDly);
   read_eeprom(16, g_iDumpDly);
   read_eeprom(18, g_iTimeOut);
         
   // scale params                             
   g_lLC_P0 = read_int32_eeprom(44);   // ZERO
   g_fP1 = read_float_eeprom(60);   // GAIN
   g_fTarget = read_float_eeprom(64);   //TARGET WEIGHT
   g_fPesoMin = read_float_eeprom(68);   // MIN WEIGHT TO CONSIDER FOR ZERO
   g_fTestWeight = read_float_eeprom(72);   // CAL TEST WEIGHT
   g_fKf = read_float_eeprom(80);   // moving average filter constants
   g_fZtDiv = read_float_eeprom(84);   // zero tracking range
   g_fMotn   = read_float_eeprom(88);   // motion tracking
}

//============================================
//  
//  Function prototypes
//
//============================================

int READ_HX711(int MODE = 0);
void SET_ZERO(void);
void SET_GAIN(void);
void HOST_COMMANDS (void);
void _BEAT(void);
void CONVERT_WEIGHT(int32 cnt);
void UPDATE_HOST(int Tx=99);
void UPDATE_IO();
void PROCESS(void);
void FILL();   // llenar
void DUMP ();   // descargar
void CALIB();   // calibrar

void READ_HX711(_Mode m)
{
   static signed int32 newCount2;
    
   if(LC_DATA_RDY){   // si el convertidor tiene datos
      
      g_lCnt_LC = g_lCount - g_lLC_P0;
      g_lCountTotal = g_lCnt_LC;

      switch (m)
      {
         case Normal:
            LC_DATA_RDY = false;
            newCount = true;                
            break;
         
         case SetZero:
            bCalibrating = 1;
            if((ZERO_SET_COUNTER--) >=0){
               newCount2 = newCount2 + g_fKf*(g_lCount-NewCount2);
               //printf("ZEROING... %d, %ld\n\r", ZERO_SET_COUNTER, newCount2);
            }
            else{
               g_lLC_P0 = NewCount2;
               write_int32_eeprom(44, g_lLC_P0);
               bCalibrating = 0;
               //printf("ZEROING DONE!\n\r");
               eMode = Normal;
            }            
            
            LC_DATA_RDY = false;         
            newCount = false;
            break;
         
         case SetGain:
            if(GAIN_SET_COUNTER-- >=0){
               
               g_lPeso = g_lCnt_LC;
                                 
               g_fP1 = g_fTestWeight / g_lPeso;
               write_float_eeprom(60, g_fP1);
               //printf("P1=%e, lc= %ld\n\r",g_fP1, g_lCnt_LC);
               eMode = Normal;
            }
            break;
            
         default:
      }      
   }
}

void START_PROCESS()
{
   printf("Process started\n");
   bStart = TRUE;
   staEstado = vaciar;
    
}

void PESO_MINIMO(par)
{
   g_fPesoMin = atof(par);
   write_float_eeprom(68,g_fPesoMin);
   printf("PM, %f \r\n",g_fPesoMin);
}

void ZERO_TRACKING_DELAY(par)
{
   g_iZtDly = atoi(par);
   write_eeprom(10, g_iZtDly);
   printf("ZT, %u \r\n", g_iZtDly);
}

void ZERO_DIVISIONS(par)
{
   g_fZtDiv = atof(par);
   write_float_eeprom(84, g_fZtDiv);
   printf("ZD, %f \r\n", g_fZtDiv);
}

void MOTION_TRACKING_DELAY(par)
{
   g_fMotn = atof(par);
   write_float_eeprom(88, g_fMotn);
   printf("MT, %f \r\n", g_fMotn);
}

void MOTION_DELAY(par)
{
   g_iMtDly = atoi(par);
   write_eeprom(12, g_iMtDly);
   printf("MD, %u \r\n", g_iMtDly);         
}

void CONVERT_WEIGHT()   // read Weight data
{
   static signed int32 old_cnt, DELTA, CREDIT, SCALAR;
   static signed int iCSB, iMotionCounter, iZeroCounter;
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
      g_fPeso = floor(g_fP1 * g_lPeso / 2) * 2;   // peso calibrado

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
         if((iZeroCounter++ > g_iZtDly) && !Zero)
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

void FILL()   //   ciclo de llenado
{
//!   fill_timeout_timer = get_ticks();
//!
//!   if(bStart){
//!      switch (staEstado){
//!         case vacio:
//!            
//!            if(g_fPeso < g_fTarget) {//         }&& (fill_timer - fill_delay) > g_iFillDly){
//!               FAST_ON;   // this is the physical output
//!               bFill = 1;
//!               fill_timeout_delay = get_ticks();
//!               staEstado = llenando;
//!            }
//!            else{
//!               FAST_OFF;
//!               staEstado = lleno;
//!               bFill = 0;   // this is a flag only
//!            }
//!   
//!            break;
//!         
//!         case llenando:
//!            btime_out = (fill_timeout_timer - fill_timeout_delay) > g_iTimeOut;   
//!            if ((g_fPeso >= g_fTarget) || (btime_out && (g_fPeso > g_fPesoMin))){
//!               FAST_OFF;
//!               bFill = 0;   // this is a flag only
//!               staEstado = lleno;
//!            }
//!            break;
//!         
//!         case lleno:
//!            if(!Motion){
//!               lDropCounter++;   // contador de descargas
//!               g_fAcumulado += g_fPeso;
//!               g_fLastWeight = g_fPeso;
//!               staEstado = vaciar;
//!            }
//!            break;
//!         
//!         default:
//!      }
//!   }
}

void DUMP ()   //   ciclo de descarga
{
//!   if(bStart){
//!      dump_timer = get_ticks();
//!      fill_timer = dump_timer;
//!
//!      switch (staEstado){
//!         case vaciar:
//!            DUMP_ON;
//!            bDump = 1;
//!            staEstado = vaciando;
//!            break;
//!
//!         case vaciando:
//!            if(g_fPeso < g_fPesoMin){
//!               staEstado = esperandoVaciar;
//!               dump_delay = get_ticks();         
//!            }
//!            break;
//!
//!         case esperandoVaciar:
//!            if((dump_timer - dump_delay) > g_iDumpDly){
//!               DUMP_OFF;
//!               bDump = 0;
//!               staEstado = esperandoLlenar;
//!               fill_delay = get_ticks();
//!            }
//!            break;
//!            
//!         case esperandoLlenar:
//!            if((fill_timer - fill_delay) > g_iFillDly){
//!               staEstado = vacio;
//!            }
//!            break;
//!            
//!         default:
//!      }
//!   }
}

void HOST_COMMANDS(void)
{
   char commands[3][10];
   char tok[] = ",";
   char n_usr_input[30];
   
   if(input_ready){
      
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
            break;
         
         case "R":
            START_PROCESS();
            break;
         
         case "P":   // stop
            DUMP_OFF;
            bDump = 0;
            FAST_OFF;
            bFast_Fill = 0;
            bStart = FALSE;
            break;
               
         case "PO":   // peso objetivo
            g_fTarget = atof(commands[1]);
            write_float_eeprom(64,g_fTarget);
            printf("PO, %f \r\n",g_fTarget);
            break;
            
         case "O":   // open
            if(!bStart){
               FAST_ON;
               SLOW_ON;
               DUMP_OFF;
               bDump = 0;
               FAST_ON;
               bFast_Fill = 0;
            }
            break;
         
         case "C":   // close
            if(!bStart){
               FAST_OFF;
               SLOW_OFF;
               bFast_Fill = 0;
               DUMP_ON;
               bDump = 0;
            }
            break;
         
         case "D":
            FAST_ON;
            bFast_Fill = 0;
            DUMP_ON;
            bDump = 0;
            break;
   
         case "Q":   // request data
            UPDATE_HOST(atoi(commands[1]));
            break;
   
         case "PM":   // minimum weight
            PESO_MINIMO(commands[1]);
            break;
            
         case "ZT":   // zero tracking delay
            ZERO_TRACKING_DELAY(commands[1]);
            break;
   
         case "ZD":   // zero divisions
            ZERO_DIVISIONS(commands[1]);
            break;
         
         case "MT":   // motion divisions
            MOTION_TRACKING_DELAY(commands[1]);
            break;
        
         case "MD":   // motion delay
            MOTION_DELAY(commands[1]);
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
            
         case "FD":   // fil delay
            g_iFillDly = atoi(commands[1]);
            write_eeprom(14,g_iFillDly);
            printf("FD,%u\n\r",g_iFillDly);
            break;
         
         case "DD":   // dump delay
            g_iDumpDly = atoi(commands[1]);
            write_eeprom(16,g_iDumpDly);
            printf("DD,%lu\n\r",g_iDumpDly);
            break;
         
         case "TO":   // fill time out
            g_iTimeOut = atoi(commands[1]);
            write_eeprom(18,g_iTimeOut);
            printf("TO,%lu\n\r", g_iTimeOut);
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
      
      input_ready=FALSE;
      index=0;
   }
}

void UPDATE_HOST(int Tx = 99)
{

   switch (Tx){
      case 0:   //default transmission
         printf("0,%c,%lu,%8.0f,%8.0f,%10.0f,%x,%x,%x\r", ID, lDropCounter, g_fPeso, g_fLastWeight, g_fAcumulado, g_cInputs, g_cOutputs, g_cStatus);
         break;

      case 1:
         printf("1,%c,%ld, %12ld, %12.0f\n\r", ID, g_lCountTotal, g_lPeso, g_fPeso);                      
         break;
                  
      case 4:
         printf("4,%c, %10ld, %10ld, %ld\n\r", ID, g_lLC1_P0, g_lCount, g_lLC_P0);
         break;
         
      case 5:
         printf("5,%c, %ld, %ld, %ld, %ld, %ld, %10f,", ID, g_iAMT[1], g_iAMT[2], g_iAMT[3], g_iAMT[4], g_iAMT[5], g_fKf);
         printf("%ld, %e, %f, %f, %f,", g_lLC_P0, g_fP1, g_fTarget, g_fPesoMin, g_fTestWeight);
         printf("%u, %6.2f, %u, %6.2f, %u, %u, %u\n\r", g_iZtDly, g_fZtDiv, g_iMtDly, g_fMotn, g_iFillDly, g_iDumpDly, g_iTimeOut);
         break;
      
      case 99:
         break;
         
      default:
         printf("99,%c bad juju\r", ID);
         break;                  
   }   
}

void UPDATE_IO(void)
{
//!   DGLS = DUMP_GATE_LS;
//!   FGLS = FILL_GATE_LS;
//!   OVRF = OVERFLOW   ;
//!   PRSF = PRESS_OK   ;
//   FULF = FULL   ;
}            

//=================================
//   MAIN
//=================================

// todo:   set point variable

//      auto cero

   
void MAIN()
{
   INIT_HARDWARE();

   char Text[] = "DesiCo. Systems\r";
   cout<<Text<<endl;
   // initialize input variables
   index=0;
   input_ready=false;
   
   // process variables

   staEstado = reposando;
   eMode = Normal;
   
   switch (restart_cause()){
      case RESTART_MCLR:
         cout<<"Restarted processor because of master clear!\r\n"<<endl;
         
      case RESTART_POWER_UP:
      {
         while(true)
         {
            restart_wdt();   
            READ_HX711(eMode);
            CONVERT_WEIGHT();
            FILL();
            DUMP();
            HOST_COMMANDS();
            UPDATE_HOST();
            UPDATE_IO();
            BEAT;   // keep alive
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
      BEAT;   // error trap
}   
