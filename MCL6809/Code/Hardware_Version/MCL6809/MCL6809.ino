//
//
//  File Name   :  MCL6809.ino
//  Used on     :  
//  Author      :  Ted Fried, MicroCore Labs
//  Creation    :  6/18/2024
//
//   Description:
//   ============
//   
//  Motorola 6809 emulator running on a Teensy 4.1.
//
//------------------------------------------------------------------------
//
// Modification History:
// =====================
//
// Revision 1 6/18/2024
// Initial revision
//
//------------------------------------------------------------------------
//
// Copyright (c) 2024 Ted Fried
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//------------------------------------------------------------------------
                                                  


#include <stdint.h>
#include <stdio.h>
#include "SD.h"


// Teensy 4.1 pin assignments
//
#define PIN_RESET_n          5 // In-Buffer       1/8       
#define PIN_FIRQ_n           2 // In-Buffer       2/8        
#define PIN_IRQ_n            3 // In-Buffer       3/8
#define PIN_HALT_n           4 // In-Buffer       4/8        
#define PIN_NMI_n           33 // In-Buffer       5/8        
#define PIN_CLK_E           29 // In-Buffer       6/8        
#define PIN_CLK_Q            1 // In-Buffer       7/8        
#define PIN_RDWR_n          25 // Out-direct-HiZ  
      

#define PIN_ADDR15          27 // Out-Buffer-HiZ     
#define PIN_ADDR14          26 // Out-Buffer-HiZ    
#define PIN_ADDR13          39 // Out-Buffer-HiZ    
#define PIN_ADDR12          38 // Out-Buffer-HiZ    
#define PIN_ADDR11          21 // Out-Buffer-HiZ    
#define PIN_ADDR10          20 // Out-Buffer-HiZ    
#define PIN_ADDR9           23 // Out-Buffer-HiZ    
#define PIN_ADDR8           22 // Out-Buffer-HiZ                                          
#define PIN_ADDR7           16 // Out-Buffer-HiZ       
#define PIN_ADDR6           17 // Out-Buffer-HiZ       
#define PIN_ADDR5           41 // Out-Buffer-HiZ       
#define PIN_ADDR4           40 // Out-Buffer-HiZ       
#define PIN_ADDR3           15 // Out-Buffer-HiZ       
#define PIN_ADDR2           14 // Out-Buffer-HiZ       
#define PIN_ADDR1           18 // Out-Buffer-HiZ       
#define PIN_ADDR0           19 // Out-Buffer-HiZ       
#define PIN_ADDR_OE_n       24 // Out-direct        
                                                   
#define PIN_DATA_IN7        37 // In-Buffer        
#define PIN_DATA_IN6        36 // In-Buffer        
#define PIN_DATA_IN5         7 // In-Buffer        
#define PIN_DATA_IN4         8 // In-Buffer        
#define PIN_DATA_IN3        13 // In-Buffer        
#define PIN_DATA_IN2        11 // In-Buffer        
#define PIN_DATA_IN1        12 // In-Buffer        
#define PIN_DATA_IN0        10 // In-Buffer     
                                                      
#define PIN_DATA_OUT7       34 // Out-Buffer-HiZ        
#define PIN_DATA_OUT6       35 // Out-Buffer-HiZ        
#define PIN_DATA_OUT5       32 // Out-Buffer-HiZ        
#define PIN_DATA_OUT4        9 // Out-Buffer-HiZ        
#define PIN_DATA_OUT3        6 // Out-Buffer-HiZ        
#define PIN_DATA_OUT2       30 // Out-Buffer-HiZ        
#define PIN_DATA_OUT1       31 // Out-Buffer-HiZ        
#define PIN_DATA_OUT0       28 // Out-Buffer-HiZ     
#define PIN_DATA_OE_n        0 // Out-direct         
   

// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------


#define REGISTER_CC  ( (flag_e<<7) | (flag_f<<6) | (flag_h<<5) | (flag_i<<4) | (flag_n<<3) | (flag_z<<2) | (flag_v<<1) | (flag_c) )

#define REGISTER_D   ( (register_A<<8) | register_B )

#define TRUE  1
#define FALSE 0

#define DIRECT   1    
#define EXTENDED 2    
#define INDEXED  3    


// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------

int       incomingByte;   
uint8_t   mode              = 1;
uint8_t   cart              = 0;
uint8_t   halt_occured      = 0;
uint8_t   ea_data8          = 0;
uint8_t   flag_e            = 0;
uint8_t   flag_f            = 0;
uint8_t   flag_h            = 0;
uint8_t   flag_i            = 0;
uint8_t   flag_n            = 0;
uint8_t   flag_z            = 0;
uint8_t   flag_v            = 0;
uint8_t   flag_c            = 0;
uint8_t   temp8             = 0;
uint8_t   page2set          = 0;
uint8_t   page3set          = 0;
uint8_t   register_A        = 0;
uint8_t   register_B        = 0;
uint8_t   register_DP       = 0; 
uint8_t   nmi_enabled       = 0;
uint8_t   nmi_latched       = 0;
uint8_t   opcode_byte       = 0;
  
uint16_t  register_X        = 0;   
uint16_t  register_Y        = 0;   
uint16_t  register_USP      = 0;   
uint16_t  register_SSP      = 0;   
uint16_t  register_PC       = 0;   
uint16_t  ea                = 0;   
uint16_t  ea_address        = 0;
uint16_t  ea_data16         = 0;
uint16_t  temp16            = 0;
                            
uint32_t  direct_reset_n    = 1; 
uint32_t  direct_nmi_n      = 1; 
uint32_t  direct_nmi_n_d1   = 1; 
uint32_t  direct_nmi_n_d2   = 1; 
uint32_t  direct_irq_n_d1   = 1;
uint32_t  direct_irq_n_d2   = 1;
uint32_t  direct_firq_n_d1  = 1;
uint32_t  direct_firq_n_d2  = 1;
uint32_t  direct_halt_n     = 1; 
uint32_t  direct_irq_n      = 1; 
uint32_t  direct_firq_n     = 1; 
uint32_t  GPIO6_raw_data    = 0; 
uint32_t  GPIO7_raw_data    = 0; 
uint32_t  GPIO9_raw_data    = 0; 
uint32_t  old_GPIO6         = 0; 
uint8_t interruptBegin = 0;

uint8_t cart_ROM[0x2000];
#define SHADOW_RAM 1
#ifdef SHADOW_RAM
uint8_t mmu[2][8];
uint8_t *internal_RAM[64];
#define LOW_RAM_BARRIER 32
uint8_t low_RAM[LOW_RAM_BARRIER][0x2000];
#endif

// coco3 registers
#define CC3_INIT0 0xff90
#define CC3_INIT1 0xff91
uint8_t cc3_init0 = 0;
uint8_t cc3_init1 = 0;
#define TASK_REG  (cc3_init1 & 0x01)
#define VECTOR_RAM  (cc3_init0 & 0x08)
#define CC3_ROMMODE  0xffde
#define CC3_RAMMODE  0xffdf
uint8_t cc3_romram = 0;
#define CC3_IRQENR  0xff92
#define CC3_FIRQENR 0xff93
#define CC3_TIMERMSB  0xff94
#define CC3_TIMERLSB  0xff95
#define CC3_MMU_0_START 0xffa0
#define CC3_MMU_1_START 0xffa8


#define EMUDSK_LSN  0xff80
#define EMUDSK_COMMAND  0xff83
#define EMUDSK_BUFFER 0xff84
#define EMUDSK_DRIVE  0xff86
uint32_t  emudsk_lsn = 0;
uint16_t  emudsk_buffer = 0;
uint8_t emudsk_status = 255;
uint8_t emudsk_drive = 0;


#define BECKER_STATUS 0xff41
#define BECKER_DATA 0xff42




// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
//
// Begin MC6809 Bus Interface Unit 
//
// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------


// Setup Teensy 4.1 IO's
//
void setup() {

  pinMode(PIN_RESET_n,         INPUT);  
  pinMode(PIN_FIRQ_n,          INPUT);  
  pinMode(PIN_IRQ_n,           INPUT);  
  pinMode(PIN_HALT_n,          INPUT);  
  pinMode(PIN_NMI_n,           INPUT);  
  pinMode(PIN_CLK_E,           INPUT);    
  pinMode(PIN_RDWR_n,          OUTPUT); 
  #define DRIVE_STRENGTH 3
  #define SPEED 3
  *(portControlRegister(PIN_RDWR_n)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);

  pinMode(PIN_ADDR15,          OUTPUT); 
  pinMode(PIN_ADDR14,          OUTPUT); 
  pinMode(PIN_ADDR13,          OUTPUT); 
  pinMode(PIN_ADDR12,          OUTPUT); 
  pinMode(PIN_ADDR11,          OUTPUT); 
  pinMode(PIN_ADDR10,          OUTPUT); 
  pinMode(PIN_ADDR9,           OUTPUT); 
  pinMode(PIN_ADDR8,           OUTPUT); 
  pinMode(PIN_ADDR7,           OUTPUT); 
  pinMode(PIN_ADDR6,           OUTPUT); 
  pinMode(PIN_ADDR5,           OUTPUT); 
  pinMode(PIN_ADDR4,           OUTPUT); 
  pinMode(PIN_ADDR3,           OUTPUT); 
  pinMode(PIN_ADDR2,           OUTPUT); 
  pinMode(PIN_ADDR1,           OUTPUT); 
  pinMode(PIN_ADDR0,           OUTPUT); 
  pinMode(PIN_ADDR_OE_n,       OUTPUT); 
  *(portControlRegister(PIN_ADDR15)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR14)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR13)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR12)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR11)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR10)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR9)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR8)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR7)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR6)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR5)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR4)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR3)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR2)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR1)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_ADDR0)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);

  pinMode(PIN_DATA_IN7,        INPUT); 
  pinMode(PIN_DATA_IN6,        INPUT); 
  pinMode(PIN_DATA_IN5,        INPUT); 
  pinMode(PIN_DATA_IN4,        INPUT); 
  pinMode(PIN_DATA_IN3,        INPUT); 
  pinMode(PIN_DATA_IN2,        INPUT); 
  pinMode(PIN_DATA_IN1,        INPUT); 
  pinMode(PIN_DATA_IN0,        INPUT); 

  pinMode(PIN_DATA_OUT7,       OUTPUT); 
  pinMode(PIN_DATA_OUT6,       OUTPUT); 
  pinMode(PIN_DATA_OUT5,       OUTPUT); 
  pinMode(PIN_DATA_OUT4,       OUTPUT); 
  pinMode(PIN_DATA_OUT3,       OUTPUT); 
  pinMode(PIN_DATA_OUT2,       OUTPUT); 
  pinMode(PIN_DATA_OUT1,       OUTPUT); 
  pinMode(PIN_DATA_OUT0,       OUTPUT); 
  pinMode(PIN_DATA_OE_n,       OUTPUT); 
  
  *(portControlRegister(PIN_DATA_OUT7)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT6)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT5)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT4)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT3)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT2)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT1)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);
  *(portControlRegister(PIN_DATA_OUT0)) = IOMUXC_PAD_DSE(DRIVE_STRENGTH) | IOMUXC_PAD_SPEED(SPEED);

  /*CORE_PIN25_PADCONFIG |= 0xf9;
  
  CORE_PIN27_PADCONFIG |= 0xf9;
  CORE_PIN26_PADCONFIG |= 0xf9;
  CORE_PIN39_PADCONFIG |= 0xf9;
  CORE_PIN38_PADCONFIG |= 0xf9;
  CORE_PIN21_PADCONFIG |= 0xf9;
  CORE_PIN20_PADCONFIG |= 0xf9;
  CORE_PIN23_PADCONFIG |= 0xf9;
  CORE_PIN22_PADCONFIG |= 0xf9;
  CORE_PIN16_PADCONFIG |= 0xf9;
  CORE_PIN17_PADCONFIG |= 0xf9;
  CORE_PIN41_PADCONFIG |= 0xf9;
  CORE_PIN40_PADCONFIG |= 0xf9;
  CORE_PIN15_PADCONFIG |= 0xf9;
  CORE_PIN14_PADCONFIG |= 0xf9;
  CORE_PIN18_PADCONFIG |= 0xf9;
  CORE_PIN19_PADCONFIG |= 0xf9;
  CORE_PIN24_PADCONFIG |= 0xf9;

  CORE_PIN34_PADCONFIG |= 0xf9;
  CORE_PIN35_PADCONFIG |= 0xf9;
  CORE_PIN32_PADCONFIG |= 0xf9;
  CORE_PIN9_PADCONFIG |= 0xf9;
  CORE_PIN6_PADCONFIG |= 0xf9;
  CORE_PIN30_PADCONFIG |= 0xf9;
  CORE_PIN31_PADCONFIG |= 0xf9;
  CORE_PIN28_PADCONFIG |= 0xf9;

  CORE_PIN0_PADCONFIG |= 0xf9;*/

  digitalWriteFast(PIN_RDWR_n,0x1);
  digitalWriteFast(PIN_ADDR_OE_n,0x0);
  digitalWriteFast(PIN_DATA_OE_n,0x1);

  SD.begin(BUILTIN_SDCARD);
  Serial.begin(115200);
  SerialUSB1.begin(115200);
  SerialUSB1.println("Init start");
#ifdef SHADOW_RAM
  int i;
  for (i = 0; i < LOW_RAM_BARRIER; i++)
  {
    internal_RAM[i] = low_RAM[i];
    memset(low_RAM[i], 0, sizeof(low_RAM[i]));
  }
  for (; i < 64; i++)
  {
    internal_RAM[i] = (uint8_t *)malloc(0x2000);
    memset(internal_RAM[i], 0, 0x2000);
  }
  for (i = 0; i < 8; i++)
  {
    mmu[0][i] = mmu[1][i] = i + 0x38;
  }
  #endif

  SerialUSB1.println("Init done");
  
}

File  femuDsk;
inline void openEmuDsk()
{
  static uint8_t  lastVHD = 0;
  if (femuDsk && lastVHD == emudsk_drive)
  {
    emudsk_status = 0;
    return;
  }
  if (femuDsk)
    femuDsk.close();
  char  caName[30];
  sprintf(caName, "hd%d.vhd", emudsk_drive);
  SerialUSB1.print("Opening ");
  SerialUSB1.println(caName);
  femuDsk = SD.open(caName, FILE_READ);

  if (!femuDsk)
  {
    SerialUSB1.println(" failed");
    emudsk_status = 2;
  }
  else
  {
    SerialUSB1.println(" success");
    emudsk_status = 0;
  }
  lastVHD = emudsk_drive;
}

uint8_t diskBuf[256];
inline void doEmuDisk(uint8_t command)
{
  interrupts();
  switch (command)
  {
    case 0: // read
      openEmuDsk();
      if (emudsk_status == 0)
      {
        SerialUSB1.print("Reading LSN:");
        SerialUSB1.println(emudsk_lsn);
        femuDsk.seek(emudsk_lsn << 8);
        femuDsk.read(diskBuf, 256);
        uint8_t *p = diskBuf;
        for (int i = emudsk_buffer; i < emudsk_buffer + 256; i++)
          Write_Byte(i, *p++);
      }
      break;
    case 1: // write
      openEmuDsk();
      if (emudsk_status == 0)
      {
          //SerialUSB1.print("Writing LSN:");
          //SerialUSB1.println(emudsk_lsn);
          uint8_t *p = diskBuf;
          for (int i = emudsk_buffer; i < emudsk_buffer + 256; i++)
          {
            *p = Read_Byte(i);
            p++;
          }

        femuDsk.seek(emudsk_lsn << 8);
        femuDsk.write(diskBuf, 256);
      }
      break;
    case 2: // close
      if (femuDsk)
        femuDsk.close();
      //SerialUSB1.println("close");
      break;
  }
//  noInterrupts();
}


// ----------------------------------------------------------
// Address range check
// Mode 0 = All acceleration and mirroring disabled   
// Mode 1 = Eliminates 6809 VMA cycles   
// Mode 2 = Reads and writes are cycle accurate using internal memory with writes passing through to motherboard   
// Mode 3 = Reads accelerated using internal memory and writes are cycle accurate and pass through to motherboard   
// Mode 4 = All read and write accesses use accelerated internal memory    
// ----------------------------------------------------------
FASTRUN inline uint8_t internal_address_check(uint16_t local_address) {
    
    if ( (local_address >= 0xC000) && (local_address <= 0xFEFF) ) return mode;     //  Tandy CoCo - Cartridge ROM
    if ( (local_address >= 0xA000) && (local_address <= 0xBFFF) ) return 0;        //  Tandy CoCo - BASIC ROM
    if ( (local_address >= 0x0600) && (local_address <= 0x9FFF) ) return mode;     //  Tandy CoCo - RAM
    if ( (local_address >= 0x0400) && (local_address <= 0x05FF) ) return 0;        //  Tandy CoCo - Normal Video RAM
    if ( (local_address >= 0x0000) && (local_address <= 0x03FF) ) return mode;     //  Tandy CoCo - RAM

    return 0x0;
} 
  

// -------------------------------------------------
// Wait for the CLK rising edge  
// -------------------------------------------------         
FASTRUN inline void wait_for_CLK_rising_edge() {

    while ( (GPIO9_DR&0x80000000) != 0) {}                                              // First ensure clock is at a low level
    do { GPIO9_raw_data = GPIO9_DR;   } while ( (GPIO9_raw_data&0x80000000) == 0);      // Then poll for the first instance where clock is not low
    
    return;}

/*// -------------------------------------------------
// Wait for the CLK rising edge  
// -------------------------------------------------         
FASTRUN inline void wait_for_Q_CLK_rising_edge() {

    while ( (GPIO6_DR&0x00000004) != 0) {}                                              // First ensure clock is at a low level
    do { GPIO6_raw_data = GPIO6_DR;   } while ( (GPIO6_raw_data&0x00000004) == 0);      // Then poll for the first instance where clock is not low
    
    return;}*/
      
// -------------------------------------------------
// Wait for the CLK rising edge  
// -------------------------------------------------         
FASTRUN inline uint8_t is_Q_CLK_low() {

    return ( (GPIO6_DR&0x00000004) != 0);                                              // First ensure clock is at a low level
}

// -------------------------------------------------
// Wait for the CLK falling edge  
// -------------------------------------------------         
FASTRUN inline void wait_for_CLK_falling_edge() {

  static uint32_t lastFall = 20000000;
  old_GPIO6 = GPIO6_DR;
  
  
  // Halt handler
  // ---------------
  
  while (direct_halt_n==0)  {                             // If HALT_n detected, immediately hi-Z signals and poll CLL until HALT_n deasserted
      halt_occured=1;
        digitalWriteFast(PIN_ADDR_OE_n,0x1);
        digitalWriteFast(PIN_DATA_OE_n,0x1);
        pinMode(PIN_RDWR_n,          INPUT);
      
      while ( (GPIO9_DR&0x80000000) == 0) {}                                          // Poll for next falling CLK edge
        do { GPIO9_raw_data   = GPIO9_DR; 
             direct_halt_n    = (GPIO9_raw_data&0x00000040);         
           } while ( (GPIO9_raw_data&0x80000000) != 0); 
    }
  
  if (halt_occured==1)  {                               // Re-enable OE drivers and RD_WR_n to pre-HALT_n cycle settings
    halt_occured=0;
    GPIO6_DR = old_GPIO6;
        pinMode(PIN_RDWR_n,          OUTPUT);
  }
           
    
  // Normal CLK falling edge handler
  // --------------------------------
   if (ARM_DWT_CYCCNT - lastFall > 60) 
    while ( (GPIO9_DR&0x80000000) == 0) {}                                              // First ensure clock is at a high level

    do { GPIO7_raw_data   = GPIO7_DR;                         // Then poll for the first instance where clock is not high
      GPIO9_raw_data   = GPIO9_DR;   
    } while ( (GPIO9_raw_data&0x80000000) != 0);  
    lastFall = ARM_DWT_CYCCNT;                   

    GPIO7_raw_data   = GPIO7_DR;
    GPIO9_raw_data   = GPIO9_DR;  
    
    direct_nmi_n_d2  = direct_nmi_n_d1;      
    direct_nmi_n_d1  = direct_nmi_n;
	
    direct_irq_n_d2  = direct_irq_n_d1;
    direct_irq_n_d1  = direct_irq_n;
	
    direct_firq_n_d2 = direct_firq_n_d1;
    direct_firq_n_d1 = direct_firq_n;

    direct_reset_n    = (GPIO9_raw_data&0x00000100);                    // Sample all signals at E falling edge
    direct_nmi_n      = (GPIO9_raw_data&0x00000080);   
    direct_halt_n     = (GPIO9_raw_data&0x00000040);   
    direct_irq_n      = (GPIO9_raw_data&0x00000020);   
    direct_firq_n     = (GPIO9_raw_data&0x00000010);   
  
    if (direct_nmi_n_d2!=0 && direct_nmi_n_d1==0 && nmi_enabled==1) nmi_latched=1;         
    
    return;
  }
    

uint8_t busWait = 0;
// -------------------------------------------------
// Initiate a MC6809 Read Bus Cycle
// -------------------------------------------------    
FASTRUN inline uint8_t BIU_Read_Byte(uint16_t local_address)  {
    uint8_t read_data;
 
    if ( ((local_address>=0xC000) && (local_address<0xFF00)) && (((cc3_init0 & 0x2) == 0) || ((cc3_init0 & 0x03) == 3)) && (cc3_romram == 0) )  { read_data = cart_ROM[local_address-0xc000];  return read_data; }   // Cartridge ROMs always internal for testing

    if ((local_address < 0xfe00) && (((cc3_init0 & 0x2) == 0) || ((cc3_init0 & 0x03) == 2)) && (cc3_romram == 1))
    {
#ifdef SHADOW_RAM
      uint8_t a;
      a = (mmu[TASK_REG][local_address >> 13]) & 0x3f;
      if (busWait)
      {
        wait_for_CLK_falling_edge();
//        d = 0;
        busWait = 0;
      }
      //wait_for_CLK_falling_edge();

      return internal_RAM[a][local_address & 0x1fff];
#endif
    }

    wait_for_CLK_falling_edge();
 
    //else if ( (internal_address_check(local_address) > 2) && local_address<0xA000 )                               { read_data = internal_RAM[local_address];  return read_data;}   // Accelerated internal reads below 0xA000
  if (local_address > 0xff00)
  {
    switch (local_address)
    {
      case BECKER_STATUS:
        {
          if (Serial.available())
              return 0xff;
            else
              return 0;
        }
        break;
      case BECKER_DATA:
        return Serial.read();
        break;

      case EMUDSK_LSN+0:
        return (uint8_t)((emudsk_lsn >> 16) & 0xff);
        break;
      case EMUDSK_LSN+1:
        return (uint8_t)((emudsk_lsn >> 8) & 0xff);
        break;
      case EMUDSK_LSN+2:
        return (uint8_t)((emudsk_lsn) & 0xff);
        break;
      case EMUDSK_COMMAND:
        return emudsk_status;
        break;
      case EMUDSK_BUFFER:
        return (emudsk_buffer >> 8);
        break;
      case EMUDSK_BUFFER+1:
        return (emudsk_buffer & 0xff);
        break;
      case EMUDSK_DRIVE:
        return emudsk_drive;
        break;
    }
  }

  busWait = 1;
  GPIO6_DR = (local_address << 16) | 0x2008;          // Drive address, RDWR_n(bit13), ADDR_OE_n(bit12), DATA_OE_n(bit3)

  wait_for_CLK_falling_edge();
  
  read_data = ((GPIO7_raw_data&0x000F0000)>>12) | (GPIO7_raw_data&0x0000000F) ; 
  
  return read_data;
}

FASTRUN inline uint16_t BIU_Read_Word(uint16_t local_address)  {
    uint16_t read_data;
 
    if ( ((local_address>=0xC000) && (local_address<0xFe00)) && (((cc3_init0 & 0x2) == 0) || ((cc3_init0 & 0x02) == 3)) && (cc3_romram == 0) )  { read_data = cart_ROM[local_address-0xc000] << 8 | cart_ROM[local_address-0xc000+1];  return read_data; }   // Cartridge ROMs always internal for testing

     if (local_address < 0xfe00)
     {
#ifdef SHADOW_RAM
      uint8_t a;
      a = (mmu[TASK_REG][local_address >> 13]) & 0x3f;
      return (internal_RAM[a][local_address & 0x1fff] << 8) | internal_RAM[a][(local_address+1) & 0x1fff];
#endif
     }


	 if (local_address == BECKER_STATUS)
   {
      // TBR
   }
     
  wait_for_CLK_falling_edge();
 // wait_for_Q_CLK_rising_edge();                                   // Disable Teensy interupts so the MC6809 bus cycle can complete without interruption
  GPIO6_DR = (local_address << 16) | 0x2008;          // Drive address, RDWR_n(bit13), ADDR_OE_n(bit12), DATA_OE_n(bit3)

  wait_for_CLK_falling_edge();
  //delayNanoseconds(10);
  
  read_data = ((GPIO7_raw_data&0x000F0000)>>4) | ((GPIO7_raw_data&0x0000000F) << 8); 
  GPIO6_DR = ((local_address + 1) << 16) | 0x2008;

  wait_for_CLK_falling_edge();
  read_data |= ((GPIO7_raw_data&0x000F0000)>>12) | (GPIO7_raw_data&0x0000000F); 
  
  return read_data;
}

// -------------------------------------------------
// Initiate a MC6809 Write Bus Cycle
// -------------------------------------------------    
FASTRUN inline void BIU_Write_Byte(uint16_t local_address , uint8_t local_data)  {
  //unsigned long cycles_s0 = ARM_DWT_CYCCNT;
  uint32_t address_shifted_16;
  
  //if ( (internal_address_check(local_address) > 3) && (local_address<0xA000) )  { internal_RAM[local_address]=local_data; return;  }  // Accelerated internal writes
  //if (local_address == 65497 && init)
  //{
  //  return;
  //}

  if (local_address < (0xfe00))
  {
#ifdef SHADOW_RAM
    uint8_t a;
    a = (mmu[TASK_REG][local_address >> 13]) & 0x3f;
    internal_RAM[a][local_address & 0x1fff] = local_data;
#endif
    }
  else
  {
    switch (local_address)
    {
	    case BECKER_DATA:
        Serial.write(&local_data, 1);
        return;
        break;
      case EMUDSK_LSN+0:
        emudsk_lsn = (emudsk_lsn & 0xff00ffff) | (local_data << 16);
        break;
      case EMUDSK_LSN+1:
        emudsk_lsn = (emudsk_lsn & 0xffff00ff) | (local_data << 8);
        break;
      case EMUDSK_LSN+2:
        emudsk_lsn = (emudsk_lsn & 0xffffff00) | local_data;
        break;
      case EMUDSK_COMMAND:
        doEmuDisk(local_data);
        break;
      case EMUDSK_BUFFER:
        emudsk_buffer = (emudsk_buffer & 0x00ff) | (local_data<<8);
        break;
      case EMUDSK_BUFFER+1:
        emudsk_buffer = (emudsk_buffer & 0xff00) | local_data;
        break;
      case EMUDSK_DRIVE:
        emudsk_drive = local_data;
        break;

      case CC3_INIT0:
        cc3_init0 = local_data;
        break;
      case CC3_INIT1:
        cc3_init1 = local_data;
        break;
      case CC3_ROMMODE:
        cc3_romram = 0;
        break;
      case CC3_RAMMODE:
        cc3_romram = 1;
        break;
#ifdef SHADOW_RAM
      case CC3_MMU_0_START+0:
      case CC3_MMU_0_START+1:
      case CC3_MMU_0_START+2:
      case CC3_MMU_0_START+3:
      case CC3_MMU_0_START+4:
      case CC3_MMU_0_START+5:
      case CC3_MMU_0_START+6:
      case CC3_MMU_0_START+7:
        mmu[0][local_address - CC3_MMU_0_START] = local_data;
        break;
      case CC3_MMU_1_START+0:
      case CC3_MMU_1_START+1:
      case CC3_MMU_1_START+2:
      case CC3_MMU_1_START+3:
      case CC3_MMU_1_START+4:
      case CC3_MMU_1_START+5:
      case CC3_MMU_1_START+6:
      case CC3_MMU_1_START+7:
        mmu[1][local_address - CC3_MMU_1_START] = local_data;
        break;
#endif
    }
  }

  busWait = 1;
  wait_for_CLK_falling_edge();

  address_shifted_16 = (local_address << 16);
  GPIO6_DR = address_shifted_16 | 0x0008;          // Drive address, RDWR_n(bit13), ADDR_OE_n(bit12), DATA_OE_n(bit3)
 
  GPIO7_DR = ((local_data&0xC0)<<22) | ((local_data&0x38)<<7); // Dataout[7:3]
  GPIO8_DR = ((local_data&0x06)<<21) | ((local_data&0x01)<<18); // Dataout[2:0]
  
  GPIO6_DR = address_shifted_16;         // Turn on Data bus buffer output on rising edge of E_CLK
 
  wait_for_CLK_falling_edge();
  GPIO6_DR = address_shifted_16 | 0x2008;         // Debounce RDWR_n and disable Data bus buffer outputs

  //if (local_address<0xA000) internal_RAM[local_address]=local_data;  // Always mirror to internal RAM for writes below 0xA000 
  
  return;
}

// -------------------------------------------------
// Initiate a MC6809 Write Bus Cycle
// -------------------------------------------------    
FASTRUN inline void BIU_Write_Word(uint16_t local_address , uint16_t local_data)  {
  //unsigned long cycles_s0 = ARM_DWT_CYCCNT;
  uint32_t address_shifted_16;
  
  //if ( (internal_address_check(local_address) > 3) && (local_address<0xA000) )  { internal_RAM[local_address]=local_data; return;  }  // Accelerated internal writes
  //if (local_address == 65497 && init)
  //{
  //  return;
  //}
  if (local_address < 0xfe00)
  {
#ifdef SHADOW_RAM
    uint8_t a;
    a = (mmu[TASK_REG][local_address >> 13]) & 0x3f;
    internal_RAM[a][local_address & 0x1fff] = local_data >> 8;
    internal_RAM[a][(local_address+1) & 0x1fff] = local_data & 0xff;
#endif
  }

	 if (local_address == BECKER_DATA)
   {
    // TBR
   }
/**  if (local_address == CC3_INIT0)
    cc3_init0 = local_data;
   if (local_address == CC3_ROMMODE)
    cc3_romram = 0;
   if (local_address == CC3_RAMMODE)
    cc3_romram = 1;*/

  wait_for_CLK_falling_edge();

  address_shifted_16 = (local_address << 16);
  GPIO6_DR = address_shifted_16 | 0x0008;          // Drive address, RDWR_n(bit13), ADDR_OE_n(bit12), DATA_OE_n(bit3)

  uint8_t upper = local_data >> 8;
  GPIO7_DR = ((upper&0xC0)<<22) | ((upper&0x38)<<7); // Dataout[7:3]
  GPIO8_DR = ((upper&0x06)<<21) | ((upper&0x01)<<18); // Dataout[2:0]
  
  GPIO6_DR = address_shifted_16;         // Turn on Data bus buffer output on rising edge of E_CLK

  wait_for_CLK_falling_edge();
  GPIO6_DR = address_shifted_16 | 0x2008;         // Debounce RDWR_n and disable Data bus buffer outputs
  

  address_shifted_16 = ((local_address+1) << 16);
  GPIO6_DR = address_shifted_16 | 0x0008;          // Drive address, RDWR_n(bit13), ADDR_OE_n(bit12), DATA_OE_n(bit3)

  
  GPIO7_DR = ((local_data&0xC0)<<22) | ((local_data&0x38)<<7); // Dataout[7:3]
  GPIO8_DR = ((local_data&0x06)<<21) | ((local_data&0x01)<<18); // Dataout[2:0]
  
  GPIO6_DR = address_shifted_16;         // Turn on Data bus buffer output on rising edge of E_CLK

  wait_for_CLK_falling_edge();
  GPIO6_DR = address_shifted_16 | 0x2008;         // Debounce RDWR_n and disable Data bus buffer outputs
  return;
}

// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------
//
// End MC6809 Bus Interface Unit
//
// --------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------


uint16_t Sign_Extend5(uint8_t local_data)  {  if ((0x0010&local_data)!=0) return (0xFFF0|local_data); else return (0x001F&local_data); }
uint16_t Sign_Extend8(uint8_t local_data)  {  if ((0x0080&local_data)!=0) return (0xFF00|local_data); else return (0x00FF&local_data); }


inline void UpdateFlags(uint8_t local_flags)  {
  
  flag_c = (local_flags>>0 & 0x01);
  flag_v = (local_flags>>1 & 0x01);
  flag_z = (local_flags>>2 & 0x01);
  flag_n = (local_flags>>3 & 0x01);
  flag_i = (local_flags>>4 & 0x01);
  flag_h = (local_flags>>5 & 0x01);
  flag_f = (local_flags>>6 & 0x01);
  flag_e = (local_flags>>7 & 0x01);
  
  return;
}

inline void UpdateRegisterD(uint16_t local_word)  {
  
  register_A = local_word >> 8;
  register_B = local_word & 0x00FF;
  
  return;
}


// ------------------------------------------------------
// Fetch an opcode byte
// ------------------------------------------------------
inline uint8_t Fetch_Opcode_Byte()  {    
    uint8_t local_byte;
    
    local_byte = BIU_Read_Byte(register_PC);     
    register_PC++;

    return local_byte;
}

// ------------------------------------------------------
// Fetch an opcode word
// ------------------------------------------------------
inline uint16_t Fetch_Opcode_Word()  {    
    uint16_t local_word;
    local_word = BIU_Read_Byte(register_PC);     
    register_PC++;
    local_word = (local_word<<8) | BIU_Read_Byte(register_PC);     
    register_PC++;
//    local_word = BIU_Read_Word(register_PC);
//    register_PC+=2;
    return local_word;
}


// ------------------------------------------------------
// Read and Write data from BIU
// ------------------------------------------------------
inline uint8_t Read_Byte(uint16_t local_address)  {    
    uint8_t local_byte;
    local_byte = BIU_Read_Byte(local_address);     
    return local_byte;
}

inline uint16_t Read_Word(uint16_t local_address)  {    
    uint16_t local_word;

    local_word = BIU_Read_Byte(local_address);
    local_word = (local_word<<8) | BIU_Read_Byte(local_address+1);
    //local_word = BIU_Read_Word(local_address);
    return local_word;
}

inline void Write_Byte(uint16_t local_address , uint8_t local_data)  {    
    BIU_Write_Byte(local_address , local_data );
    return;
}

inline void Write_Word(uint16_t local_address , uint16_t local_data)  {    
    BIU_Write_Byte(local_address   , local_data>>8 );  
    BIU_Write_Byte(local_address+1 , local_data );  
//    BIU_Write_Word(local_address, local_data);
    return;
}


inline void VMA_Cycle(uint8_t local_count , uint16_t local_address)  { 

    if (mode==0)   for (uint8_t i=1; i<=local_count; i++)  temp8 = BIU_Read_Byte(local_address);    
     
  return;
}
//#define VMA_Cycle(a,b)


// ------------------------------------------------------
// Stack
// ------------------------------------------------------
inline void PushU8(uint8_t local_data)  {    
    register_USP--;
    Write_Byte(register_USP,local_data); 
    return;
}

inline void PushU16(uint16_t local_data)  {    
    PushU8(local_data    );  // Low  Byte
    PushU8(local_data>>8 );  // High Byte
    return;
}

inline uint8_t PopU8()  {    
    uint8_t local_data_low;
  
    local_data_low = Read_Byte(register_USP );
    register_USP++;
    return local_data_low;
}

inline uint16_t PopU16()  {    
    uint8_t local_data_low;
    uint8_t local_data_high;
  
    local_data_high = PopU8();
    local_data_low  = PopU8();
    return( (local_data_high<<8) | local_data_low);
}


inline void PushS8(uint8_t local_data)  {    
    register_SSP--;
    Write_Byte(register_SSP,local_data); 
    return;
}

inline void PushS16(uint16_t local_data)  {    
    PushS8(local_data    );  // Low  Byte
    PushS8(local_data>>8 );  // High Byte
    return;
}

inline uint8_t PopS8()  {    
    uint8_t local_data_low;
  
    local_data_low = Read_Byte(register_SSP);
    register_SSP++;
    return local_data_low;
}

inline uint16_t PopS16()  {    
    uint8_t local_data_low;
    uint8_t local_data_high;
  
    local_data_high = PopS8();
    local_data_low  = PopS8();
    return( (local_data_high<<8) | local_data_low);
}


// ------------------------------------------------------
// Reset routine
// ------------------------------------------------------
//

void Reset_sequence()  {
  uint32_t q=0;


  // Allow for cartridge switching during chip reset
  //
    File f = SD.open("cart0.rom");
    if (f)
    {
      uint32_t sz = f.size();
      f.read(&cart_ROM, min(sz, sizeof(cart_ROM)));
      f.close();
    }
    else
    {
      Serial.println("unable to open cart0.rom");
    }
/* switch (cart)  {
    case 0:     for ( q=0; q<=0x3EFF ; q++ )   { internal_RAM[q+0xC000] = CARTRIDGE_0[q]; }   break;
    case 1:     for ( q=0; q<=0x3EFF ; q++ )   { internal_RAM[q+0xC000] = CARTRIDGE_1[q]; }   break;
    case 2:     for ( q=0; q<=0x3EFF ; q++ )   { internal_RAM[q+0xC000] = CARTRIDGE_2[q]; }   break;
    case 3:     for ( q=0; q<=0x3EFF ; q++ )   { internal_RAM[q+0xC000] = CARTRIDGE_3[q]; }   break;
  }*/

   // Copy ROM to internal memory
  //for ( q=0xA000; q<=0xA008 ; q++ )   { internal_RAM[q] = Read_Byte(q); } 
    
  register_DP = 0;
  flag_i = 1;                                                       // Set I and F flags to mask interrupts
  flag_f = 1; 
  GPIO6_DR = (0xFFFF << 16) | 0x2008;                               // Debounce RDWR_n and disable Data bus buffer outputs
  nmi_latched = 0;
  nmi_enabled = 0;
    
    while (direct_reset_n==0) wait_for_CLK_falling_edge();          // Stay here until HALT_n and RESET_n de-aserted
  
  VMA_Cycle(3,0xFFFE);
    
  register_PC = Read_Word(0xFFFE);

  //for (uint16_t i=0; i<0x2000; i++)  internal_RAM[i]=0;  // Clear all RAM so that each reset is a Cold-Boot
    
    return;
}


// -------------------------------------------------
// NMI Handler     
// -------------------------------------------------
inline void NMI_Handler() {

  flag_e = 1;             // Stacking all state registers
  
  VMA_Cycle(1,0xFFFF);
  PushS16(register_PC);
  PushS16(register_USP);
  PushS16(register_Y);
  PushS16(register_X);
  PushS8(register_DP);
  PushS8(register_B);
  PushS8(register_A);
  PushS8(REGISTER_CC);
  flag_i = 1;
  flag_f = 1;
  VMA_Cycle(1,0xFFFF);
  register_PC = Read_Word(0xFFFC);
  VMA_Cycle(1,0xFFFF);
   nmi_latched=0;
  
    return;
}

// -------------------------------------------------
// INTR Handler     
// -------------------------------------------------
inline void IRQ_Handler() {

  flag_e = 1;             // Stacking all state registers
  
  VMA_Cycle(1,0xFFFF);
  PushS16(register_PC);
  PushS16(register_USP);
  PushS16(register_Y);
  PushS16(register_X);
  PushS8(register_DP);
  PushS8(register_B);
  PushS8(register_A);
  PushS8(REGISTER_CC);
  flag_i = 1;
  VMA_Cycle(1,0xFFFF);
  register_PC = Read_Word(0xFFF8);
  VMA_Cycle(1,0xFFFF);
  nmi_latched=0;
  
    return;
}

// -------------------------------------------------
// FINTR Handler     
// -------------------------------------------------
inline void FIRQ_Handler() {

  flag_e = 0;
  
  VMA_Cycle(1,0xFFFF);
  PushS16(register_PC);
  PushS8(REGISTER_CC);
  flag_i = 1;
  flag_f = 1;
  VMA_Cycle(1,0xFFFF);
  register_PC = Read_Word(0xFFF6);
  VMA_Cycle(1,0xFFFF);
  nmi_latched=0;
  
    return;
}


// -------------------------------------------------
// SWI Handler     
// -------------------------------------------------
void opcode_0x3F() {          // SWI

  flag_e = 1;             // Stacking all state registers
  
  VMA_Cycle(1,register_PC);
  VMA_Cycle(1,0xFFFF);
  PushS16(register_PC);
  PushS16(register_USP);
  PushS16(register_Y);
  PushS16(register_X);
  PushS8(register_DP);
  PushS8(register_B);
  PushS8(register_A);
  PushS8(REGISTER_CC);
  flag_i=1;
  flag_f=1;
  VMA_Cycle(1,0xFFFF);
       if (page3set==1) register_PC = Read_Word(0xFFF2);  // SWI3
  else if (page2set==1) register_PC = Read_Word(0xFFF4);  // SWI2
  else                  register_PC = Read_Word(0xFFFA);  // SWI

  VMA_Cycle(1,0xFFFF);
  page2set=0;
  page3set=0;
  
  return;
}

void opcode_0x3B ()   { VMA_Cycle(1,register_PC);             // RTI
                        UpdateFlags(PopS8());
            if (flag_e==1)  {
              register_A   = PopS8();
              register_B   = PopS8();
              register_DP  = PopS8();
              register_X   = PopS16();
              register_Y   = PopS16();
              register_USP = PopS16();
            }
            register_PC=PopS16();
            VMA_Cycle(1,register_USP);
            return;  } 
  
void opcode_0x13 ()   { VMA_Cycle(1,register_PC);             // SYNC
                        digitalWriteFast(PIN_ADDR_OE_n,0x1);
                        digitalWriteFast(PIN_DATA_OE_n,0x1);
                        pinMode(PIN_RDWR_n,          INPUT);
            VMA_Cycle(1,0xFFFF); 
                      while ( (direct_irq_n_d2!=0) && (direct_firq_n_d2!=0) && (direct_nmi_n!=0) )  wait_for_CLK_falling_edge();
                        digitalWriteFast(PIN_ADDR_OE_n,0x0);
                        pinMode(PIN_RDWR_n,          OUTPUT);
            VMA_Cycle(1,0xFFFF); 
            return;  } 
            


void opcode_0x3C ()   {                   // CWAI
  
  UpdateFlags(REGISTER_CC & Fetch_Opcode_Byte());
  VMA_Cycle(1,register_PC);  
  VMA_Cycle(1,0xFFFF);
  flag_e = 1;
  PushS16(register_PC);
  PushS16(register_USP);
  PushS16(register_Y);
  PushS16(register_X);
  PushS8(register_DP);
  PushS8(register_B);
  PushS8(register_A);
  PushS8(REGISTER_CC);
  VMA_Cycle(1,0xFFFF);
  while ( (direct_irq_n_d2!=0) && (direct_firq_n_d2!=0) && (direct_nmi_n!=0) )  wait_for_CLK_falling_edge();
    
       if (direct_nmi_n==0)                register_PC = Read_Word(0xFFFC);
  else if (flag_f==0 && direct_firq_n_d2==0)  register_PC = Read_Word(0xFFF6);
  else if (flag_i==0 && direct_irq_n_d2==0)   register_PC = Read_Word(0xFFF8);
  
  VMA_Cycle(1,0xFFFF);
  return;
}



void Set_Flags_Byte_NZ (uint8_t local_byte)   { 

  if ( (local_byte&0x80) != 0)  flag_n=1;  else flag_n=0;
  if ( (local_byte&0xFF) == 0)  flag_z=1;  else flag_z=0;
  
  return;
}


void Set_Flags_Word_NZ (uint16_t local_word)   {  
 
  if ( (local_word&0x8000) != 0)  flag_n=1;  else flag_n=0;
  if (  local_word         == 0)  flag_z=1;  else flag_z=0;
  
  return;
}

  
void opcode_0x12 ()   {  VMA_Cycle(1,register_PC);  return; } // NOP


void opcode_0x19 ()   {  // DAA
  uint8_t lsn;
  uint8_t msn;
  uint8_t cf=0;
    
  lsn = register_A & 0x0F;
  msn = (register_A & 0xF0) >> 4;
  
  if ( flag_h==1 || lsn>0x09 )                              cf = cf + 0x06;
  if ( flag_c==1 || msn>0x09  || (msn>0x08 && lsn>0x09 ) )  cf = cf + 0x60;
  
  temp16 = register_A + cf;
  
  if (temp16>0xFF) flag_c=1;
  
  register_A = (uint8_t) temp16;
  Set_Flags_Byte_NZ(register_A);
  
  VMA_Cycle(1,register_PC);  
  return;
}
  
  
inline void Branch_Handler(uint8_t local_condition)   {
  uint16_t offset;

  if (page2set==0) {  offset = Fetch_Opcode_Byte();
                    VMA_Cycle(1,0xFFFF);
                    if (local_condition==TRUE) register_PC = register_PC + Sign_Extend8(offset);
                   }
  else             {  offset = Fetch_Opcode_Word();
                    VMA_Cycle(1,0xFFFF);
                    if (local_condition==TRUE) { VMA_Cycle(1,0xFFFF);  register_PC = register_PC + offset;  }
                   }  
  page2set=0;          
  return;
} 


void opcode_0x21 ()   {             Branch_Handler(FALSE);                  return;  }  // BRN
void opcode_0x20 ()   {             Branch_Handler(TRUE);                   return;  }  // BRA
void opcode_0x16 ()   { page2set=1; Branch_Handler(TRUE);                   return;  }  // LBRA

void opcode_0x2B ()   { Branch_Handler(flag_n==1);                          return;  }  // BMI
void opcode_0x27 ()   { Branch_Handler(flag_z==1);                          return;  }  // BEQ
void opcode_0x29 ()   { Branch_Handler(flag_v==1);                          return;  }  // BVS
void opcode_0x25 ()   { Branch_Handler(flag_c==1);                          return;  }  // BCS
void opcode_0x2A ()   { Branch_Handler(flag_n==0);                          return;  }  // BPL
void opcode_0x26 ()   { Branch_Handler(flag_z==0);                          return;  }  // BNE
void opcode_0x28 ()   { Branch_Handler(flag_v==0);                          return;  }  // BVC
void opcode_0x24 ()   { Branch_Handler(flag_c==0);                          return;  }  // BCC

void opcode_0x2C ()   { Branch_Handler(flag_n==flag_v);                     return;  }  // BGE
void opcode_0x2E ()   { Branch_Handler( (flag_n==flag_v) && (flag_z==0) );  return;  }  // BGT
void opcode_0x22 ()   { Branch_Handler( (flag_z==0) && (flag_c==0) );       return;  }  // BHI
void opcode_0x2F ()   { Branch_Handler( (flag_n!=flag_v) || (flag_z==1) );  return;  }  // BLE
void opcode_0x23 ()   { Branch_Handler( (flag_z!=0) || (flag_c!=0) );       return;  }  // BLS
void opcode_0x2D ()   { Branch_Handler(flag_n!=flag_v);                     return;  }  // BLT

void opcode_0x8D ()   { uint16_t offset;
                        offset=Fetch_Opcode_Byte();                 // BSR
                        VMA_Cycle(1,0xFFFF); 
                        VMA_Cycle(1,offset); 
                        VMA_Cycle(1,0xFFFF); 
                        PushS16(register_PC);  
                        register_PC = register_PC + Sign_Extend8(offset);                
                        return;  } 


void opcode_0x17 ()   { uint16_t offset;
                        offset=Fetch_Opcode_Word();                 // LBSR
                        VMA_Cycle(2,0xFFFF); 
                        VMA_Cycle(1,offset); 
                        VMA_Cycle(1,0xFFFF); 
                        PushS16(register_PC);  
                        register_PC = register_PC + offset;                
                        return;  } 




void opcode_0x3A ()   { register_X = register_X + register_B;       // ABX
                        VMA_Cycle(1,register_PC);
                        VMA_Cycle(1,0xFFFF);
                        return;  } 
  

void opcode_0x39 ()   { VMA_Cycle(1,register_PC);                   // RTS
                        register_PC = PopS16();
                        VMA_Cycle(1,0xFFFF);
                        return;  } 
  
  


  
void opcode_0x37 ()   {                         // PULU
    uint8_t postbyte;
    
    postbyte = Fetch_Opcode_Byte();
    VMA_Cycle(2,0xFFFF);
    if ( (postbyte & 0x01) != 0)  UpdateFlags(PopS8());
    if ( (postbyte & 0x02) != 0)  register_A   = PopU8();
    if ( (postbyte & 0x04) != 0)  register_B   = PopU8();
    if ( (postbyte & 0x08) != 0)  register_DP  = PopU8();
    if ( (postbyte & 0x10) != 0)  register_X   = PopU16();
    if ( (postbyte & 0x20) != 0)  register_Y   = PopU16();
     if ( (postbyte & 0x40) != 0)  { register_SSP = PopU16(); nmi_enabled=1; }
    if ( (postbyte & 0x80) != 0)  register_PC  = PopU16();
    VMA_Cycle(1,0xFFFF);
  
  return;
}
  
void opcode_0x35 ()   {                         // PULS
    uint8_t postbyte;
    
    postbyte = Fetch_Opcode_Byte();
    VMA_Cycle(2,0xFFFF);
    nmi_enabled=1; // (undocumented)
    if ( (postbyte & 0x01) != 0)  UpdateFlags(PopS8());
    if ( (postbyte & 0x02) != 0)  register_A   = PopS8();
    if ( (postbyte & 0x04) != 0)  register_B   = PopS8();
    if ( (postbyte & 0x08) != 0)  register_DP  = PopS8();
    if ( (postbyte & 0x10) != 0)  register_X   = PopS16();
    if ( (postbyte & 0x20) != 0)  register_Y   = PopS16();
    if ( (postbyte & 0x40) != 0)  register_USP = PopS16();
    if ( (postbyte & 0x80) != 0)  register_PC  = PopS16();
    VMA_Cycle(1,0xFFFF);
  
  return;
}

void opcode_0x36 ()   {                         // PSHU
    uint8_t postbyte;
    
    postbyte = Fetch_Opcode_Byte();
    VMA_Cycle(2,0xFFFF);
    VMA_Cycle(1,register_SSP);
    if ( (postbyte & 0x80) != 0)  PushU16(register_PC);
    if ( (postbyte & 0x40) != 0)  PushU16(register_SSP);
    if ( (postbyte & 0x20) != 0)  PushU16(register_Y);
    if ( (postbyte & 0x10) != 0)  PushU16(register_X);
    if ( (postbyte & 0x08) != 0)  PushU8(register_DP);
    if ( (postbyte & 0x04) != 0)  PushU8(register_B);
    if ( (postbyte & 0x02) != 0)  PushU8(register_A);
    if ( (postbyte & 0x01) != 0)  PushU8(REGISTER_CC);
  
  return;
}

void opcode_0x34 ()   {                         // PSHS
    uint8_t postbyte;
    
    postbyte = Fetch_Opcode_Byte();
    VMA_Cycle(2,0xFFFF);
    VMA_Cycle(1,register_SSP);
    nmi_enabled=1; // (undocumented)
    if ( (postbyte & 0x80) != 0)  PushS16(register_PC);
    if ( (postbyte & 0x40) != 0)  PushS16(register_USP);
    if ( (postbyte & 0x20) != 0)  PushS16(register_Y);
    if ( (postbyte & 0x10) != 0)  PushS16(register_X);
    if ( (postbyte & 0x08) != 0)  PushS8(register_DP);
    if ( (postbyte & 0x04) != 0)  PushS8(register_B);
    if ( (postbyte & 0x02) != 0)  PushS8(register_A);
    if ( (postbyte & 0x01) != 0)  PushS8(REGISTER_CC);
  
  return;
}
  
    
inline uint16_t EXG_Read_Source(uint8_t reg_select)   {
  uint16_t source_value=0;
  
  switch (reg_select)  {
    case 0x0: source_value = REGISTER_D;                      break;
    case 0x1: source_value = register_X;                      break;
    case 0x2: source_value = register_Y;                      break;
    case 0x3: source_value = register_USP;                    break;
    case 0x4: source_value = register_SSP;                    break;
    case 0x5: source_value = register_PC;                     break;
    case 0x6: source_value = 0xEEEE;                          break;
    case 0x7: source_value = 0xEEEE;                          break;
    case 0x8: source_value = 0xFF00 | register_A;             break;
    case 0x9: source_value = 0xFF00 | register_B;             break;
    case 0xa: source_value = (REGISTER_CC<<8 | REGISTER_CC);  break;
    case 0xb: source_value = (register_DP<<8 | register_DP);  break;
    case 0xc: source_value = 0xEEEE;                          break;
    case 0xd: source_value = 0xEEEE;                          break;
    case 0xe: source_value = 0xEEEE;                          break;
    case 0xf: source_value = 0xEEEE;                          break;
  }
  return source_value;
}       

    
inline void EXG_Write_Destination(uint8_t reg_select, uint16_t write_value)   {
  
  switch (reg_select)  {
    case 0x0:   UpdateRegisterD(write_value);                  break;
    case 0x1: register_X       = write_value;                  break;
    case 0x2: register_Y       = write_value;                  break;
    case 0x3: register_USP     = write_value;                  break;
    case 0x4: register_SSP     = write_value;  nmi_enabled=1;  break;
    case 0x5: register_PC      = write_value;                  break;
                      
    case 0x8: register_A       = write_value;                  break;
    case 0x9: register_B       = write_value;                  break;
    case 0xA:        UpdateFlags(write_value);                 break;
    case 0xB: register_DP      = write_value;                  break;
 
  }
  return;
}       

void opcode_0x1E ()   {                         // EXG
    uint8_t  postbyte;
    uint8_t  reg_upper;
    uint8_t  reg_lower;
    uint16_t upper_value;
    uint16_t lower_value;
        
    postbyte = Fetch_Opcode_Byte();
    VMA_Cycle(6,0xFFFF);
    reg_upper = postbyte >>   4;
    reg_lower = postbyte & 0x0F;
    
    upper_value = EXG_Read_Source(reg_upper);
    lower_value = EXG_Read_Source(reg_lower);
    
    EXG_Write_Destination(reg_upper , lower_value);
    EXG_Write_Destination(reg_lower , upper_value);
    
  return;
} 

void opcode_0x1F ()   {                         // TFR
    uint8_t  postbyte;
    uint8_t  reg_upper;
    uint8_t  reg_lower;
    uint16_t upper_value;
        
    postbyte = Fetch_Opcode_Byte();
    VMA_Cycle(4,0xFFFF);
    reg_upper = postbyte >>   4;
    reg_lower = postbyte & 0x0F;
    
    upper_value = EXG_Read_Source(reg_upper);
    
    EXG_Write_Destination(reg_lower , upper_value);
    
  return;
}       

 

void opcode_0x1D ()   { VMA_Cycle(1,register_PC);           // SEX
                        UpdateRegisterD(Sign_Extend8(register_B));
                        Set_Flags_Byte_NZ(register_B);
                        return;  } 
    



inline uint16_t FetchReg_EA(uint8_t local_postbyte)   {
  uint16_t local_regval;

   switch ((local_postbyte&0x60)>>5)  {
     case 0x0:  local_regval = register_X;      break;
     case 0x1:  local_regval = register_Y;      break;
     case 0x2:  local_regval = register_USP;    break;
     case 0x3:  local_regval = register_SSP;    break;
   }
   return local_regval;
}
    

inline void WBReg_EA(uint8_t local_postbyte , uint16_t local_data)   {

   switch ((local_postbyte&0x60)>>5)  {
     case 0x0:  register_X   = local_data;    break;
     case 0x1:  register_Y   = local_data;    break;
     case 0x2:  register_USP = local_data;    break;
     case 0x3:  register_SSP = local_data;  nmi_enabled=1;   break;// (undocumented)
  
   }
   return;
}



// Calculate Effective Address - EA
// -------------------------------------
inline uint16_t Calculate_EA(uint8_t ea_type)   {
  uint16_t  local_offset;
  uint8_t   local_postbyte;
  uint16_t  local_postword;
  uint16_t  local_ea = 0xEEEE;


   switch (ea_type)  {
     case DIRECT:    local_postbyte=Fetch_Opcode_Byte();  VMA_Cycle(1,0xFFFF);  local_ea=((register_DP<<8) | local_postbyte);    break;
     case EXTENDED:  local_postword=Fetch_Opcode_Word();  VMA_Cycle(1,0xFFFF);  local_ea=local_postword;                         break;
     
     case INDEXED:   local_postbyte=Fetch_Opcode_Byte();
     
                     if ( (local_postbyte&0x80) == 0)  { VMA_Cycle(2,0xFFFF);  local_ea=(FetchReg_EA(local_postbyte) + Sign_Extend5(local_postbyte&0x1F) );   }  // Non-Indirect Register 5-bit offset
                     else  {
               
        switch (local_postbyte&0x1F)  {   
               
  case 0x04: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(1,register_PC);  local_ea=local_offset;                                                                                                      break;  // Non-Indirect Register no offset
  case 0x14: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(1,register_PC);  local_ea=Read_Word(local_offset);                                                                  VMA_Cycle(1,0xFFFF);     break;  // Indirect Register no offset
                                                                                                                                                                                            
  case 0x08: local_offset=Fetch_Opcode_Byte();                VMA_Cycle(1,0xFFFF);  local_ea=          (FetchReg_EA(local_postbyte) + Sign_Extend8(local_offset) );                                                  break;  // Non-Indirect Register 8-bit offset
  case 0x18: local_offset=Fetch_Opcode_Byte();                VMA_Cycle(1,0xFFFF);  local_ea=Read_Word((FetchReg_EA(local_postbyte) + Sign_Extend8(local_offset) ));                        VMA_Cycle(1,0xFFFF);     break;  // Indirect Register 8-bit offset
                                                                                                                                                                                            
  case 0x09: local_offset=Fetch_Opcode_Word();                VMA_Cycle(3,0xFFFF);  local_ea=          (FetchReg_EA(local_postbyte) + local_offset );                                                                break;  // Non-Indirect Register 16-bit offset
  case 0x19: local_offset=Fetch_Opcode_Word();                VMA_Cycle(3,0xFFFF);  local_ea=Read_Word((FetchReg_EA(local_postbyte) + local_offset ));                                      VMA_Cycle(1,0xFFFF);     break;  // Indirect Register 16-bit offset
                                                                                                                                                                                            
                                                                                                                                                                                            
  case 0x06: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(2,0xFFFF);  local_ea= Sign_Extend8(register_A) + local_offset;                                                                               break;  // Non-Indirect A Accumulator offset
  case 0x07: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(2,0xFFFF);  local_ea= Sign_Extend8(register_A) + local_offset;                                                                               break;  // Non-Indirect A Accumulator offset (undocumented)
  case 0x05: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(2,0xFFFF);  local_ea= Sign_Extend8(register_B) + local_offset;                                                                               break;  // Non-Indirect B Accumulator offset
  case 0x0B: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(5,0xFFFF);  local_ea= REGISTER_D + local_offset;                                                                                             break;  // Non-Indirect D Accumulator offset
                                                                                                                                                                                            
                                                                                                                                                                                            
  case 0x16: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(2,0xFFFF);  local_ea= Read_Word(Sign_Extend8(register_A) + local_offset);                                           VMA_Cycle(1,0xFFFF);     break;  // Indirect A Accumulator offset
  case 0x15: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(2,0xFFFF);  local_ea= Read_Word(Sign_Extend8(register_B) + local_offset);                                           VMA_Cycle(1,0xFFFF);     break;  // Indirect B Accumulator offset
  case 0x1B: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(5,0xFFFF);  local_ea= Read_Word(REGISTER_D + local_offset);                                                         VMA_Cycle(1,0xFFFF);     break;  // Indirect D Accumulator offset
                                                                                                       
                                                                                                       
  case 0x00: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(3,0xFFFF);  local_ea= local_offset;             WBReg_EA(local_postbyte,local_offset+1);                                                     break;  // Non-Indirect Post-Increment by 1
                                                                                                       
  case 0x01: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(3,0xFFFF);  local_ea= local_offset;             WBReg_EA(local_postbyte,local_offset+2);                                                     break;  // Non-Indirect Post-Increment by 2
  case 0x11: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(4,0xFFFF);  local_ea= Read_Word(local_offset);  WBReg_EA(local_postbyte,local_offset+2);                            VMA_Cycle(1,0xFFFF);     break;  // Indirect Post-Increment by 2
                                                                                                       
                                                                                                       
  case 0x02: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(3,0xFFFF); local_offset=local_offset-1;  local_ea= local_offset;             WBReg_EA(local_postbyte,local_offset);                          break;  // Non-Indirect Pre-Decrement by 1
                                                                                                        
  case 0x03: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(3,0xFFFF); local_offset=local_offset-2;  local_ea= local_offset;             WBReg_EA(local_postbyte,local_offset);                          break;  // Non-Indirect Pre-Decrement by 2
  case 0x13: local_offset=FetchReg_EA(local_postbyte);        VMA_Cycle(4,0xFFFF); local_offset=local_offset-2;  local_ea= Read_Word(local_offset);  WBReg_EA(local_postbyte,local_offset); VMA_Cycle(1,0xFFFF);     break;  // Indirect Pre-Decrement by 2
                                                                                                    
                                                                                                    
  case 0x0C: local_offset=Sign_Extend8(Fetch_Opcode_Byte());  VMA_Cycle(1,0xFFFF);  local_ea=register_PC+local_offset;                                                                                               break;  // Non-Indirect Constant 8-bit  Offset from PC
  case 0x1C: local_offset=Sign_Extend8(Fetch_Opcode_Byte());  VMA_Cycle(1,0xFFFF);  local_ea=Read_Word(register_PC+local_offset);                                                           VMA_Cycle(1,0xFFFF);     break;  // Indirect Constant 8-bit  Offset from PC
                                                                                                                                                                                            
  case 0x0D: local_offset=Fetch_Opcode_Word();                VMA_Cycle(4,0xFFFF);  local_ea=register_PC+local_offset;                                                                                               break;  // Non-IndirectConstant 16-bit Offset from PC
  case 0x1D: local_offset=Fetch_Opcode_Word();                VMA_Cycle(4,0xFFFF);  local_ea=Read_Word(register_PC+local_offset);                                                           VMA_Cycle(1,0xFFFF);     break;  // Indirect Constant 16-bit Offset from PC
                                                                                                                                                                                            
  case 0x1F: local_offset=Fetch_Opcode_Word();                VMA_Cycle(1,register_PC);  local_ea=Read_Word(local_offset);                                                                  VMA_Cycle(1,0xFFFF);     break;  // Extended Indirect
           
            
   }
   }
   }
    
   return local_ea;
}

    
inline uint8_t ASL_Common(uint8_t local_byte)   {
  flag_c = local_byte >> 7;
  if ( ((local_byte&0xC0) == 0x80) || ((local_byte&0xC0) == 0x40) )  flag_v=1; else flag_v=0;
  local_byte = local_byte << 1;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
}   
void opcode_0x48 ()  {  VMA_Cycle(1,register_PC);                                        register_A = ASL_Common(register_A);                                                        return;  }   // ASLA
void opcode_0x58 ()  {  VMA_Cycle(1,register_PC);                                        register_B = ASL_Common(register_B);                                                        return;  }   // ASLB
void opcode_0x08 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  ea_data8=ASL_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ASL Direct         
void opcode_0x68 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  ea_data8=ASL_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ASL Indexed          
void opcode_0x78 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  ea_data8=ASL_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ASL Extended         
    
    

inline uint8_t ASR_Common(uint8_t local_byte)   {  
  temp8 = local_byte & 0x80;
  flag_c = local_byte & 0x01;
  local_byte = local_byte >> 1;
  local_byte = temp8 | local_byte;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
}
void opcode_0x47 ()  {  VMA_Cycle(1,register_PC);                                        register_A = ASR_Common(register_A);                                                        return;  }   // ASRA
void opcode_0x57 ()  {  VMA_Cycle(1,register_PC);                                        register_B = ASR_Common(register_B);                                                        return;  }   // ASRB
void opcode_0x07 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  ea_data8=ASR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ASR Direct         
void opcode_0x67 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  ea_data8=ASR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ASR Indexed          
void opcode_0x77 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  ea_data8=ASR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ASR Extended         
  

    
inline void CLR_Common()   {
  VMA_Cycle(1,register_PC);
  flag_n = 0;
  flag_z = 1;
  flag_v = 0;
  flag_c = 0;
  return;
} 
void opcode_0x4F ()   { CLR_Common();   register_A = 0;                                                                                                         return;  }      // CLRA
void opcode_0x5F ()   { CLR_Common();   register_B = 0;                                                                                                         return;  }      // CLRB
void opcode_0x0F ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,0x00);  CLR_Common();  return;  }  // CLR DIRECT
void opcode_0x6F ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,0x00);  CLR_Common();  return;  }  // CLR INDEXED
void opcode_0x7F ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,0x00);  CLR_Common();  return;  }  // CLR EXTENDED
                        


  
inline uint8_t COM_Common(uint8_t local_byte)   {
  VMA_Cycle(1,register_PC);
  local_byte = ~local_byte;
  flag_v = 0;
  flag_c = 1;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
} 
void opcode_0x43 ()   { register_A = COM_Common(register_A);                                                                                                     return;  }      // COMA
void opcode_0x53 ()   { register_B = COM_Common(register_B);                                                                                                     return;  }      // COMB
void opcode_0x03 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,COM_Common(ea_data8));  return;  }  // COM DIRECT
void opcode_0x63 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,COM_Common(ea_data8));  return;  }  // COM INDEXED
void opcode_0x73 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,COM_Common(ea_data8));  return;  }  // COM EXTENDED
                        

  
inline uint8_t DEC_Common(uint8_t local_byte)   {
  VMA_Cycle(1,register_PC);
  if (local_byte == 0x80) flag_v=1; else flag_v=0;
  local_byte--;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
} 
void opcode_0x4A ()   { register_A = DEC_Common(register_A);                                                                                                     return;  }      // DECA
void opcode_0x5A ()   { register_B = DEC_Common(register_B);                                                                                                     return;  }      // DECB
void opcode_0x0A ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,DEC_Common(ea_data8));  return;  }  // DEC DIRECT
void opcode_0x6A ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,DEC_Common(ea_data8));  return;  }  // DEC INDEXED
void opcode_0x7A ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,DEC_Common(ea_data8));  return;  }  // DEC EXTENDED
                        

  
  
inline uint8_t INC_Common(uint8_t local_byte)   {
  VMA_Cycle(1,register_PC);
  if (local_byte == 0x7F) flag_v=1; else flag_v=0;
  local_byte++;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
} 
void opcode_0x4C ()   { register_A = INC_Common(register_A);                                                                                                     return;  }      // INCA
void opcode_0x5C ()   { register_B = INC_Common(register_B);                                                                                                     return;  }      // INCB
void opcode_0x0C ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,INC_Common(ea_data8));  return;  }  // INC DIRECT
void opcode_0x6C ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,INC_Common(ea_data8));  return;  }  // INC INDEXED
void opcode_0x7C ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,INC_Common(ea_data8));  return;  }  // INC EXTENDED
                        
    

inline uint8_t LSR_Common(uint8_t local_byte)   {
  flag_c = local_byte & 0x01;
  local_byte = local_byte >> 1;
  Set_Flags_Byte_NZ(local_byte);
  flag_n = 0;
  return local_byte;
}
void opcode_0x44 ()  {  VMA_Cycle(1,register_PC);                                        register_A = LSR_Common(register_A);                                                        return;  }   // LSRA
void opcode_0x54 ()  {  VMA_Cycle(1,register_PC);                                        register_B = LSR_Common(register_B);                                                        return;  }   // LSRB
void opcode_0x04 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  ea_data8=LSR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // LSR Direct         
void opcode_0x64 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  ea_data8=LSR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // LSR Indexed          
void opcode_0x74 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  ea_data8=LSR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // LSR Extended         
  
  
inline uint8_t NEG_Common(uint8_t local_byte)   {
  VMA_Cycle(1,register_PC);
  if (local_byte==0x00) flag_c=0; else flag_c=1;
  if (local_byte==0x80) flag_v=1; else flag_v=0;
  local_byte = 0 - local_byte;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
} 
void opcode_0x40 ()   { register_A = NEG_Common(register_A);                                                                                                     return;  }      // NEGA
void opcode_0x50 ()   { register_B = NEG_Common(register_B);                                                                                                     return;  }      // NEGB
void opcode_0x00 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NEG_Common(ea_data8));  return;  }  // NEG DIRECT
void opcode_0x60 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NEG_Common(ea_data8));  return;  }  // NEG INDEXED
void opcode_0x70 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NEG_Common(ea_data8));  return;  }  // NEG EXTENDED
 
 

    

inline uint8_t ROL_Common(uint8_t local_byte)   {
  uint8_t old_c;
  
  old_c = flag_c;
  flag_c = local_byte >> 7;
  if ( ((local_byte&0xC0) == 0x80) || ((local_byte&0xC0) == 0x40)  ) flag_v=1; else flag_v=0;
  local_byte = local_byte << 1;
  local_byte = local_byte | old_c;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
}
void opcode_0x49 ()  {  VMA_Cycle(1,register_PC);                                        register_A = ROL_Common(register_A);                                                        return;  }   // ROLA
void opcode_0x59 ()  {  VMA_Cycle(1,register_PC);                                        register_B = ROL_Common(register_B);                                                        return;  }   // ROLB
void opcode_0x09 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  ea_data8=ROL_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ROL Direct         
void opcode_0x69 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  ea_data8=ROL_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ROL Indexed          
void opcode_0x79 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  ea_data8=ROL_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ROL Extended         
  
      
    
inline uint8_t ROR_Common(uint8_t local_byte)   {
  uint8_t old_c;
  
  old_c = flag_c;
  flag_c = local_byte & 0x01;
  local_byte = local_byte >> 1;
  local_byte = (old_c<<7) | local_byte;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
}
void opcode_0x46 ()  {  VMA_Cycle(1,register_PC);                                        register_A = ROR_Common(register_A);                                                        return;  }   // RORA
void opcode_0x56 ()  {  VMA_Cycle(1,register_PC);                                        register_B = ROR_Common(register_B);                                                        return;  }   // RORB
void opcode_0x06 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  ea_data8=ROR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ROR Direct         
void opcode_0x66 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  ea_data8=ROR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ROR Indexed          
void opcode_0x76 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  ea_data8=ROR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // ROR Extended         
  
  

inline void TST_Common(uint8_t local_byte)   {
  flag_v = 0;
  Set_Flags_Byte_NZ(local_byte);
  return;
}
void opcode_0x4D ()  {  VMA_Cycle(1,register_PC);                                            TST_Common(register_A);                                              return;  }   // TSTA
void opcode_0x5D ()  {  VMA_Cycle(1,register_PC);                                            TST_Common(register_B);                                              return;  }   // TSTB
void opcode_0x0D ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  TST_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  VMA_Cycle(1,0xFFFF);    return;  }   // TST Direct         
void opcode_0x6D ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  TST_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  VMA_Cycle(1,0xFFFF);    return;  }   // TST Indexed          
void opcode_0x7D ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  TST_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  VMA_Cycle(1,0xFFFF);    return;  }   // TST Extended         
  


  

inline uint8_t AND_Common(uint8_t local_byte1, uint8_t local_byte2)   {
  local_byte1 = local_byte1 & local_byte2;
  flag_v = 0;
  Set_Flags_Byte_NZ(local_byte1);
  return local_byte1;
}
void opcode_0x84 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = AND_Common(register_A,ea_data8);     return;  }   // ANDA Immediate         
void opcode_0x94 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = AND_Common(register_A,ea_data8);     return;  }   // ANDA Direct          
void opcode_0xA4 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = AND_Common(register_A,ea_data8);     return;  }   // ANDA Indexed         
void opcode_0xB4 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = AND_Common(register_A,ea_data8);     return;  }   // ANDA Extended  

void opcode_0xC4 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = AND_Common(register_B,ea_data8);     return;  }   // ANDB Immediate         
void opcode_0xD4 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = AND_Common(register_B,ea_data8);     return;  }   // ANDB Direct          
void opcode_0xE4 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = AND_Common(register_B,ea_data8);     return;  }   // ANDB Indexed         
void opcode_0xF4 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = AND_Common(register_B,ea_data8);     return;  }   // ANDB Extended  

void opcode_0x85 ()  {                                      ea_data8=Fetch_Opcode_Byte();    temp8 = AND_Common(register_A,ea_data8);          return;  }   // BITA Immediate         
void opcode_0x95 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  temp8 = AND_Common(register_A,ea_data8);          return;  }   // BITA Direct          
void opcode_0xA5 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  temp8 = AND_Common(register_A,ea_data8);          return;  }   // BITA Indexed         
void opcode_0xB5 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  temp8 = AND_Common(register_A,ea_data8);          return;  }   // BITA Extended  
                                                                         
void opcode_0xC5 ()  {                                      ea_data8=Fetch_Opcode_Byte();    temp8 = AND_Common(register_B,ea_data8);          return;  }   // BITB Immediate         
void opcode_0xD5 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  temp8 = AND_Common(register_B,ea_data8);          return;  }   // BITB Direct          
void opcode_0xE5 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  temp8 = AND_Common(register_B,ea_data8);          return;  }   // BITB Indexed         
void opcode_0xF5 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  temp8 = AND_Common(register_B,ea_data8);          return;  }   // BITB Extended  

void opcode_0x1C ()  {                                      ea_data8=Fetch_Opcode_Byte();    UpdateFlags( AND_Common(REGISTER_CC,ea_data8) );  return;  }   // ANDCC Immediate          


inline uint8_t OR_Common(uint8_t local_byte1, uint8_t local_byte2)   {
  local_byte1 = local_byte1 | local_byte2;
  flag_v = 0;
  Set_Flags_Byte_NZ(local_byte1);
  return local_byte1;
}
void opcode_0x8A ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = OR_Common(register_A,ea_data8);     return;  }   // ORA Immediate         
void opcode_0x9A ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = OR_Common(register_A,ea_data8);     return;  }   // ORA Direct          
void opcode_0xAA ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = OR_Common(register_A,ea_data8);     return;  }   // ORA Indexed         
void opcode_0xBA ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = OR_Common(register_A,ea_data8);     return;  }   // ORA Extended  
                                                                                
void opcode_0xCA ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = OR_Common(register_B,ea_data8);     return;  }   // ORB Immediate         
void opcode_0xDA ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = OR_Common(register_B,ea_data8);     return;  }   // ORB Direct          
void opcode_0xEA ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = OR_Common(register_B,ea_data8);     return;  }   // ORB Indexed         
void opcode_0xFA ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = OR_Common(register_B,ea_data8);     return;  }   // ORB Extended  
                                                                                 
void opcode_0x1A ()  {                                      ea_data8=Fetch_Opcode_Byte();    UpdateFlags( OR_Common(REGISTER_CC,ea_data8) );  return;  }   // ORCC Immediate          



inline uint8_t EOR_Common(uint8_t local_byte1, uint8_t local_byte2)   {
  local_byte1 = local_byte1 ^ local_byte2;
  flag_v = 0;
  Set_Flags_Byte_NZ(local_byte1);
  return local_byte1;
}
void opcode_0x88 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = EOR_Common(register_A,ea_data8);     return;  }   // EORA Immediate         
void opcode_0x98 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = EOR_Common(register_A,ea_data8);     return;  }   // EORA Direct          
void opcode_0xA8 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = EOR_Common(register_A,ea_data8);     return;  }   // EORA Indexed         
void opcode_0xB8 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = EOR_Common(register_A,ea_data8);     return;  }   // EORA Extended  
                                                                                  
void opcode_0xC8 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = EOR_Common(register_B,ea_data8);     return;  }   // EORB Immediate         
void opcode_0xD8 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = EOR_Common(register_B,ea_data8);     return;  }   // EORB Direct          
void opcode_0xE8 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = EOR_Common(register_B,ea_data8);     return;  }   // EORB Indexed         
void opcode_0xF8 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = EOR_Common(register_B,ea_data8);     return;  }   // EORB Extended  
                                                                                  

void opcode_0x3D ()   {                         // MUL
  uint16_t product;
  
  VMA_Cycle(1,register_PC);           
  
  product = register_A * register_B;
  UpdateRegisterD(product);
  if (product==0) flag_z=1; else flag_z=0;
  flag_c = register_B >> 7;

  VMA_Cycle(9,0xFFFF);
  return;  
}



inline uint8_t ADD8_Common(uint8_t local_byte1, uint8_t local_byte2, uint8_t local_carry)   {
  uint8_t  operand0=0;
  uint8_t  operand1=0;
  uint8_t  result=0;
  
  temp8  = (local_byte1&0x0F) + (local_byte2&0x0F) + local_carry;
  if (temp8>0xF)  flag_h = 1; else flag_h = 0;
    
  temp16 = local_byte1 + local_byte2 + local_carry;
  if (temp16>0xFF)  flag_c = 1; else flag_c = 0;
  
  operand0 = (local_byte1  & 0x80);  
  operand1 = (local_byte2  & 0x80);  
  result   = (temp16       & 0x80); 
  flag_v = 0;
  if      (operand0==0 && operand1==0 && result!=0)  flag_v = 1; 
  else if (operand0!=0 && operand1!=0 && result==0)  flag_v = 1;      

  Set_Flags_Byte_NZ(temp16);
  return (temp16&0xFF);
}
void opcode_0x8B ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = ADD8_Common(register_A,ea_data8,0);         return;  }   // ADDA Immediate          
void opcode_0x9B ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = ADD8_Common(register_A,ea_data8,0);         return;  }   // ADDA Direct         
void opcode_0xAB ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = ADD8_Common(register_A,ea_data8,0);         return;  }   // ADDA Indexed          
void opcode_0xBB ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = ADD8_Common(register_A,ea_data8,0);         return;  }   // ADDA Extended 

void opcode_0x89 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = ADD8_Common(register_A,ea_data8,flag_c);    return;  }   // ADCA Immediate          
void opcode_0x99 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = ADD8_Common(register_A,ea_data8,flag_c);    return;  }   // ADCA Direct         
void opcode_0xA9 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = ADD8_Common(register_A,ea_data8,flag_c);    return;  }   // ADCA Indexed          
void opcode_0xB9 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = ADD8_Common(register_A,ea_data8,flag_c);    return;  }   // ADCA Extended 
    
    
void opcode_0xCB ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = ADD8_Common(register_B,ea_data8,0);         return;  }   // ADDBB Immediate         
void opcode_0xDB ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = ADD8_Common(register_B,ea_data8,0);         return;  }   // ADDBB Direct          
void opcode_0xEB ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = ADD8_Common(register_B,ea_data8,0);         return;  }   // ADDBB Indexed         
void opcode_0xFB ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = ADD8_Common(register_B,ea_data8,0);         return;  }   // ADDBB Extended  
    
void opcode_0xC9 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = ADD8_Common(register_B,ea_data8,flag_c);    return;  }   // ADCBB Immediate         
void opcode_0xD9 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = ADD8_Common(register_B,ea_data8,flag_c);    return;  }   // ADCBB Direct          
void opcode_0xE9 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = ADD8_Common(register_B,ea_data8,flag_c);    return;  }   // ADCBB Indexed         
void opcode_0xF9 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = ADD8_Common(register_B,ea_data8,flag_c);    return;  }   // ADCBB Extended  
  
  
inline uint16_t ADD16_Common(uint16_t local_word1, uint16_t local_word2, uint16_t local_carry)   {
  uint8_t  operand0=0;
  uint8_t  operand1=0;
  uint8_t  result=0;
  uint32_t temp32=0;
    
  temp32 = local_word1 + local_word2 + local_carry;
  if (temp32>0xFFFF)  flag_c = 1; else flag_c = 0;
  
  operand0 = (local_word1  & 0x8000);  
  operand1 = (local_word2  & 0x8000);  
  result   = (temp32       & 0x8000); 
  flag_v = 0;
  if      (operand0==0 && operand1==0 && result!=0)  flag_v = 1; 
  else if (operand0!=0 && operand1!=0 && result==0)  flag_v = 1;     

  Set_Flags_Word_NZ(temp32);    
  return (temp32&0xFFFF);
} 
void opcode_0xC3 ()  {                                      ea_data16=Fetch_Opcode_Word();    UpdateRegisterD(ADD16_Common(REGISTER_D,ea_data16,0)); VMA_Cycle(1,0xFFFF);  return;  }   // ADDD Immediate         
void opcode_0xD3 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data16=Read_Word(ea_address);  UpdateRegisterD(ADD16_Common(REGISTER_D,ea_data16,0)); VMA_Cycle(1,0xFFFF);  return;  }   // ADDD Direct          
void opcode_0xE3 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data16=Read_Word(ea_address);  UpdateRegisterD(ADD16_Common(REGISTER_D,ea_data16,0)); VMA_Cycle(1,0xFFFF);  return;  }   // ADDD Indexed         
void opcode_0xF3 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data16=Read_Word(ea_address);  UpdateRegisterD(ADD16_Common(REGISTER_D,ea_data16,0)); VMA_Cycle(1,0xFFFF);  return;  }   // ADDD Extended  
  
  
inline uint8_t SUB8_Common(uint8_t local_byte1, uint8_t local_byte2, uint8_t local_carry)   {
  uint8_t  operand0=0;
  uint8_t  operand1=0;
  uint8_t  result=0;
  
  temp8  = (local_byte1&0x0F) - (local_byte2&0x0F) - local_carry;
  if (temp8>0xF)  flag_h = 1; else flag_h = 0;
    
  temp16 = local_byte1 - local_byte2 - local_carry;
  if (temp16>0xFF)  flag_c = 1; else flag_c = 0;
  
  operand0 = (local_byte1  & 0x80);  
  operand1 = (local_byte2  & 0x80);  
  result   = (temp16       & 0x80); 
  flag_v = 0;
  if      (operand0==0 && operand1!=0 && result!=0)   flag_v = 1; 
  else if (operand0!=0 && operand1==0 && result==0)   flag_v = 1;    

  Set_Flags_Byte_NZ(temp16);  
  return (temp16&0xFF);
}
void opcode_0x80 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = SUB8_Common(register_A,ea_data8,0);         return;  }   // SUBA Immediate          
void opcode_0x90 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = SUB8_Common(register_A,ea_data8,0);         return;  }   // SUBA Direct         
void opcode_0xA0 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = SUB8_Common(register_A,ea_data8,0);         return;  }   // SUBA Indexed          
void opcode_0xB0 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = SUB8_Common(register_A,ea_data8,0);         return;  }   // SUBA Extended 
                                                   
void opcode_0x82 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_A = SUB8_Common(register_A,ea_data8,flag_c);    return;  }   // SBCA Immediate          
void opcode_0x92 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_A = SUB8_Common(register_A,ea_data8,flag_c);    return;  }   // SBCA Direct         
void opcode_0xA2 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_A = SUB8_Common(register_A,ea_data8,flag_c);    return;  }   // SBCA Indexed          
void opcode_0xB2 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_A = SUB8_Common(register_A,ea_data8,flag_c);    return;  }   // SBCA Extended                                                        
                                                       
void opcode_0xC0 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = SUB8_Common(register_B,ea_data8,0);         return;  }   // SUBB Immediate          
void opcode_0xD0 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = SUB8_Common(register_B,ea_data8,0);         return;  }   // SUBB Direct         
void opcode_0xE0 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = SUB8_Common(register_B,ea_data8,0);         return;  }   // SUBB Indexed          
void opcode_0xF0 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = SUB8_Common(register_B,ea_data8,0);         return;  }   // SUBB Extended 

void opcode_0xC2 ()  {                                      ea_data8=Fetch_Opcode_Byte();    register_B = SUB8_Common(register_B,ea_data8,flag_c);    return;  }   // SBCB Immediate          
void opcode_0xD2 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  register_B = SUB8_Common(register_B,ea_data8,flag_c);    return;  }   // SBCB Direct         
void opcode_0xE2 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  register_B = SUB8_Common(register_B,ea_data8,flag_c);    return;  }   // SBCB Indexed          
void opcode_0xF2 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  register_B = SUB8_Common(register_B,ea_data8,flag_c);    return;  }   // SBCB Extended 


void opcode_0x81 ()  {                                      ea_data8=Fetch_Opcode_Byte();    temp8 = SUB8_Common(register_A,ea_data8,0);              return;  }   // CMPA Immediate          
void opcode_0x91 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  temp8 = SUB8_Common(register_A,ea_data8,0);              return;  }   // CMPA Direct         
void opcode_0xA1 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  temp8 = SUB8_Common(register_A,ea_data8,0);              return;  }   // CMPA Indexed          
void opcode_0xB1 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  temp8 = SUB8_Common(register_A,ea_data8,0);              return;  }   // CMPA Extended 

void opcode_0xC1 ()  {                                      ea_data8=Fetch_Opcode_Byte();    temp8 = SUB8_Common(register_B,ea_data8,0);              return;  }   // CMPB Immediate          
void opcode_0xD1 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  temp8 = SUB8_Common(register_B,ea_data8,0);              return;  }   // CMPB Direct         
void opcode_0xE1 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  temp8 = SUB8_Common(register_B,ea_data8,0);              return;  }   // CMPB Indexed          
void opcode_0xF1 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  temp8 = SUB8_Common(register_B,ea_data8,0);              return;  }   // CMPB Extended 


  
  
inline uint16_t SUB16_Common(uint16_t local_word1, uint16_t local_word2, uint16_t local_carry)   {
  uint8_t  operand0=0;
  uint8_t  operand1=0;
  uint8_t  result=0;
  uint32_t temp32=0;
    
  temp32 = local_word1 - local_word2 - local_carry;
  if (temp32>0xFFFF)  flag_c = 1; else flag_c = 0;
  
  operand0 = (local_word1  & 0x8000);  
  operand1 = (local_word2  & 0x8000);  
  result   = (temp32       & 0x8000); 
  flag_v = 0;
  if      (operand0==0 && operand1==0 && result!=0)  flag_v = 1; 
  else if (operand0!=0 && operand1!=0 && result==0)  flag_v = 1;   

  Set_Flags_Word_NZ(temp32);   
  return (temp32&0xFFFF);
} 
void opcode_0x83 ()  {  if (page3set==1) {                                    ea_data16=Fetch_Opcode_Word();             temp16=SUB16_Common(register_USP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPU Immediate 
                   else if (page2set==1) {                                    ea_data16=Fetch_Opcode_Word();             temp16=SUB16_Common(REGISTER_D,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPD Immediate 
                   else if (page2set==0) {                                    ea_data16=Fetch_Opcode_Word();    UpdateRegisterD(SUB16_Common(REGISTER_D,ea_data16,0));   VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // SUBD Immediate         
                                                                 
void opcode_0x93 ()  {  if (page3set==1) { ea_address=Calculate_EA(DIRECT);   ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_USP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPU Direct  
                   else if (page2set==1) { ea_address=Calculate_EA(DIRECT);   ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(REGISTER_D,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPD Direct  
                   else if (page2set==0) { ea_address=Calculate_EA(DIRECT);   ea_data16=Read_Word(ea_address);  UpdateRegisterD(SUB16_Common(REGISTER_D,ea_data16,0));   VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // SUBD Direct  
                                                                 
void opcode_0xA3 ()  {  if (page3set==1) { ea_address=Calculate_EA(INDEXED);  ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_USP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPU Indexed 
                   else if (page2set==1) { ea_address=Calculate_EA(INDEXED);  ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(REGISTER_D,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPD Indexed 
                   else if (page2set==0) { ea_address=Calculate_EA(INDEXED);  ea_data16=Read_Word(ea_address);  UpdateRegisterD(SUB16_Common(REGISTER_D,ea_data16,0));   VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }  } // SUBD Indexed 
                                                                 
void opcode_0xB3 ()  {  if (page3set==1) { ea_address=Calculate_EA(EXTENDED); ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_USP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPU Extended  
                   else if (page2set==1) { ea_address=Calculate_EA(EXTENDED); ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(REGISTER_D,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPD Extended  
                   else if (page2set==0) { ea_address=Calculate_EA(EXTENDED); ea_data16=Read_Word(ea_address);  UpdateRegisterD(SUB16_Common(REGISTER_D,ea_data16,0));   VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // SUBD Extended  
                                                                 
                                                                 
void opcode_0x8C ()  {  if (page3set==1) {                                    ea_data16=Fetch_Opcode_Word();             temp16=SUB16_Common(register_SSP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPS Immediate 
                   else if (page2set==1) {                                    ea_data16=Fetch_Opcode_Word();             temp16=SUB16_Common(register_Y,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPY Immediate 
                   else if (page2set==0) {                                    ea_data16=Fetch_Opcode_Word();             temp16=SUB16_Common(register_X,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // CMPX Immediate 
                                                                       
void opcode_0x9C ()  {  if (page3set==1) { ea_address=Calculate_EA(DIRECT);   ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_SSP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPS Direct  
                   else if (page2set==1) { ea_address=Calculate_EA(DIRECT);   ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_Y,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPY Direct  
                   else if (page2set==0) { ea_address=Calculate_EA(DIRECT);   ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_X,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // CMPX Direct  
                                                                       
void opcode_0xAC ()  {  if (page3set==1) { ea_address=Calculate_EA(INDEXED);  ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_SSP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPS Indexed 
                   else if (page2set==1) { ea_address=Calculate_EA(INDEXED);  ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_Y,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPY Indexed 
                   else if (page2set==0) { ea_address=Calculate_EA(INDEXED);  ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_X,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // CMPX Indexed 
                                                                       
void opcode_0xBC ()  {  if (page3set==1) { ea_address=Calculate_EA(EXTENDED); ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_SSP,ea_data16,0);  VMA_Cycle(1,0xFFFF);  page3set=0;  return;  }    // CMPS Extended  
                   else if (page2set==1) { ea_address=Calculate_EA(EXTENDED); ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_Y,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  }    // CMPY Extended  
                   else if (page2set==0) { ea_address=Calculate_EA(EXTENDED); ea_data16=Read_Word(ea_address);           temp16=SUB16_Common(register_X,ea_data16,0);    VMA_Cycle(1,0xFFFF);  page2set=0;  return;  } }  // CMPX Extended  
  
                
  

void opcode_0x86 ()   {                                     ea_data8=Fetch_Opcode_Byte();     register_A=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDA Immediate          
void opcode_0x96 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);   register_A=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDA DIRECT
void opcode_0xA6 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);   register_A=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDA INDEXED
void opcode_0xB6 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);   register_A=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDA EXTENDED
                                                                              
void opcode_0xC6 ()   {                                     ea_data8=Fetch_Opcode_Byte();     register_B=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDB Immediate          
void opcode_0xD6 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);   register_B=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDB DIRECT
void opcode_0xE6 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);   register_B=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDB INDEXED
void opcode_0xF6 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);   register_B=ea_data8;         Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // LDB EXTENDED
   
void opcode_0xCC ()   {                                     ea_data16=Fetch_Opcode_Word();    UpdateRegisterD(ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // LDD Immediate          
void opcode_0xDC ()   { ea_address=Calculate_EA(DIRECT);    ea_data16=Read_Word(ea_address);  UpdateRegisterD(ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // LDD DIRECT
void opcode_0xEC ()   { ea_address=Calculate_EA(INDEXED);   ea_data16=Read_Word(ea_address);  UpdateRegisterD(ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // LDD INDEXED
void opcode_0xFC ()   { ea_address=Calculate_EA(EXTENDED);  ea_data16=Read_Word(ea_address);  UpdateRegisterD(ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // LDD EXTENDED
              
void opcode_0xCE ()   {                                     ea_data16=Fetch_Opcode_Word();    if (page2set==1) register_SSP=ea_data16; else register_USP=ea_data16;  Set_Flags_Word_NZ(ea_data16);  flag_v=0;  nmi_enabled=1; page2set=0; return;  }  // LDS/LSU Immediate          
void opcode_0xDE ()   { ea_address=Calculate_EA(DIRECT);    ea_data16=Read_Word(ea_address);  if (page2set==1) register_SSP=ea_data16; else register_USP=ea_data16;  Set_Flags_Word_NZ(ea_data16);  flag_v=0;  nmi_enabled=1; page2set=0; return;  }  // LDS/LSU DIRECT
void opcode_0xEE ()   { ea_address=Calculate_EA(INDEXED);   ea_data16=Read_Word(ea_address);  if (page2set==1) register_SSP=ea_data16; else register_USP=ea_data16;  Set_Flags_Word_NZ(ea_data16);  flag_v=0;  nmi_enabled=1; page2set=0; return;  }  // LDS/LSU INDEXED
void opcode_0xFE ()   { ea_address=Calculate_EA(EXTENDED);  ea_data16=Read_Word(ea_address);  if (page2set==1) register_SSP=ea_data16; else register_USP=ea_data16;  Set_Flags_Word_NZ(ea_data16);  flag_v=0;  nmi_enabled=1; page2set=0; return;  }  // LDS/LSU EXTENDED
        
void opcode_0x8E ()   {                                     ea_data16=Fetch_Opcode_Word();    if (page2set==0) register_X=ea_data16; else register_Y=ea_data16;      Set_Flags_Word_NZ(ea_data16);  flag_v=0;   page2set=0; return;  }  // LDX/LSY Immediate          
void opcode_0x9E ()   { ea_address=Calculate_EA(DIRECT);    ea_data16=Read_Word(ea_address);  if (page2set==0) register_X=ea_data16; else register_Y=ea_data16;      Set_Flags_Word_NZ(ea_data16);  flag_v=0;   page2set=0; return;  }  // LDX/LSY DIRECT
void opcode_0xAE ()   { ea_address=Calculate_EA(INDEXED);   ea_data16=Read_Word(ea_address);  if (page2set==0) register_X=ea_data16; else register_Y=ea_data16;      Set_Flags_Word_NZ(ea_data16);  flag_v=0;   page2set=0; return;  }  // LDX/LSY INDEXED
void opcode_0xBE ()   { ea_address=Calculate_EA(EXTENDED);  ea_data16=Read_Word(ea_address);  if (page2set==0) register_X=ea_data16; else register_Y=ea_data16;      Set_Flags_Word_NZ(ea_data16);  flag_v=0;   page2set=0; return;  }  // LDX/LSY EXTENDED
    
  

void opcode_0x97 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=register_A;     Write_Byte(ea_address, ea_data8);   Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // STA DIRECT
void opcode_0xA7 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=register_A;     Write_Byte(ea_address, ea_data8);   Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // STA INDEXED
void opcode_0xB7 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=register_A;     Write_Byte(ea_address, ea_data8);   Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // STA EXTENDED

void opcode_0xD7 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=register_B;     Write_Byte(ea_address, ea_data8);   Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // STB DIRECT
void opcode_0xE7 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=register_B;     Write_Byte(ea_address, ea_data8);   Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // STB INDEXED
void opcode_0xF7 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=register_B;     Write_Byte(ea_address, ea_data8);   Set_Flags_Byte_NZ(ea_data8);   flag_v=0;   return;  }  // STB EXTENDED

void opcode_0xDD ()   { ea_address=Calculate_EA(DIRECT);    ea_data16=REGISTER_D;    Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // STD DIRECT
void opcode_0xED ()   { ea_address=Calculate_EA(INDEXED);   ea_data16=REGISTER_D;    Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // STD INDEXED
void opcode_0xFD ()   { ea_address=Calculate_EA(EXTENDED);  ea_data16=REGISTER_D;    Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16);  flag_v=0;   return;  }  // STD EXTENDED
    
void opcode_0xDF ()   { ea_address=Calculate_EA(DIRECT);    if (page2set==1) ea_data16=register_SSP; else ea_data16=register_USP;  Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16); page2set=0; flag_v=0;   return;  }  // STS_U DIRECT
void opcode_0xEF ()   { ea_address=Calculate_EA(INDEXED);   if (page2set==1) ea_data16=register_SSP; else ea_data16=register_USP;  Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16); page2set=0; flag_v=0;   return;  }  // STS_U INDEXED
void opcode_0xFF ()   { ea_address=Calculate_EA(EXTENDED);  if (page2set==1) ea_data16=register_SSP; else ea_data16=register_USP;  Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16); page2set=0; flag_v=0;   return;  }  // STS_U EXTENDED
  
void opcode_0x9F ()   { ea_address=Calculate_EA(DIRECT);    if (page2set==1) ea_data16=register_Y;   else ea_data16=register_X;    Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16); page2set=0; flag_v=0;   return;  }  // STX_Y DIRECT
void opcode_0xAF ()   { ea_address=Calculate_EA(INDEXED);   if (page2set==1) ea_data16=register_Y;   else ea_data16=register_X;    Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16); page2set=0; flag_v=0;   return;  }  // STX_Y INDEXED
void opcode_0xBF ()   { ea_address=Calculate_EA(EXTENDED);  if (page2set==1) ea_data16=register_Y;   else ea_data16=register_X;    Write_Word(ea_address, ea_data16);  Set_Flags_Word_NZ(ea_data16); page2set=0; flag_v=0;   return;  }  // STX_Y EXTENDED
  

  
void opcode_0x0E ()   { ea_address=Calculate_EA(DIRECT);                                                 register_PC = ea_address;   return;  }  // JMP DIRECT
void opcode_0x6E ()   { ea_address=Calculate_EA(INDEXED);                                                register_PC = ea_address;   return;  }  // JMP INDEXED
void opcode_0x7E ()   { ea_address=Calculate_EA(EXTENDED);                                               register_PC = ea_address;   return;  }  // JMP EXTENDED
  
void opcode_0x9D ()   { ea_address=Calculate_EA(DIRECT);    VMA_Cycle(1,0xFFFF);  PushS16(register_PC);  register_PC = ea_address;   return;  }  // JSR DIRECT
void opcode_0xAD ()   { ea_address=Calculate_EA(INDEXED);   VMA_Cycle(1,0xFFFF);  PushS16(register_PC);  register_PC = ea_address;   return;  }  // JSR INDEXED
void opcode_0xBD ()   { ea_address=Calculate_EA(EXTENDED);  VMA_Cycle(1,0xFFFF);  PushS16(register_PC);  register_PC = ea_address;   return;  }  // JSR EXTENDED



void opcode_0x30 ()   { register_X=Calculate_EA(INDEXED);    VMA_Cycle(1,0xFFFF);   if (register_X==0) flag_z=1; else flag_z=0;      return;  }  // LEAX INDEXED
void opcode_0x31 ()   { register_Y=Calculate_EA(INDEXED);    VMA_Cycle(1,0xFFFF);   if (register_Y==0) flag_z=1; else flag_z=0;      return;  }  // LEAY INDEXED
void opcode_0x32 ()   { register_SSP=Calculate_EA(INDEXED);  VMA_Cycle(1,0xFFFF);         nmi_enabled=1;                             return;  }  // LEAS INDEXED
void opcode_0x33 ()   { register_USP=Calculate_EA(INDEXED);  VMA_Cycle(1,0xFFFF);                                                    return;  }  // LEAU INDEXED



// Undocumented opcodes
//------------------------    
//------------------------   
 
void opcode_0x41 ()   { register_A = NEG_Common(register_A);      return;  }      // NEGA
void opcode_0x51 ()   { register_B = NEG_Common(register_B);      return;  }      // NEGB
void opcode_0x01 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NEG_Common(ea_data8));  return;  }  // NEG DIRECT
void opcode_0x61 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NEG_Common(ea_data8));  return;  }  // NEG INDEXED
void opcode_0x71 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NEG_Common(ea_data8));  return;  }  // NEG EXTENDED
                        
   
inline uint8_t NGC_Common(uint8_t local_byte)   {
  VMA_Cycle(1,register_PC);
  if (local_byte==0x00) flag_c=0; else flag_c=1;
  if (local_byte==0x80) flag_v=1; else flag_v=0;
    local_byte = local_byte + 1 - flag_c;
  Set_Flags_Byte_NZ(local_byte);
  return local_byte;
} 
void opcode_0x42 ()   { register_A = NGC_Common(register_A);                                                                                                     return;  }      // NGCA
void opcode_0x52 ()   { register_B = NGC_Common(register_B);                                                                                                     return;  }      // NGCB
void opcode_0x02 ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NGC_Common(ea_data8));  return;  }  // NGC DIRECT
void opcode_0x62 ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NGC_Common(ea_data8));  return;  }  // NGC INDEXED
void opcode_0x72 ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,NGC_Common(ea_data8));  return;  }  // NGC EXTENDED
   
   
   
void opcode_0x14()   {  Write_Byte(0xDEAD, 0x77);  while (direct_reset_n!=0) {VMA_Cycle(1,0x1100);Fetch_Opcode_Byte();}    return;  }  // HCF - Halt and Catch Fire until RESET_n is aserted
void opcode_0x15()   {  Write_Byte(0xDEAD, 0x77);  while (direct_reset_n!=0) {VMA_Cycle(1,0x2200);Fetch_Opcode_Byte();}    return;  } 
void opcode_0xCD()   {  Write_Byte(0xDEAD, 0x77);  while (direct_reset_n!=0) {VMA_Cycle(1,0x3300);Fetch_Opcode_Byte();}    return;  } 

   
void opcode_0x45 ()  {  VMA_Cycle(1,register_PC);                                        register_A = LSR_Common(register_A);                                                        return;  }   // LSRA
void opcode_0x55 ()  {  VMA_Cycle(1,register_PC);                                        register_B = LSR_Common(register_B);                                                        return;  }   // LSRB
void opcode_0x05 ()  {  ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  ea_data8=LSR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // LSR Direct         
void opcode_0x65 ()  {  ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  ea_data8=LSR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // LSR Indexed          
void opcode_0x75 ()  {  ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  ea_data8=LSR_Common(ea_data8);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,ea_data8);  return;  }   // LSR Extended         
  

void opcode_0x18 ()  {  UpdateFlags( (REGISTER_CC<<1) & Fetch_Opcode_Byte() );  VMA_Cycle(1,0xFFFF);  return;  }   // LSACC

void opcode_0x87 ()  {  Fetch_Opcode_Byte();  flag_n=1;  flag_z=0;  flag_v=0;   return;  }   // BROK
void opcode_0xC7 ()  {  Fetch_Opcode_Byte();  flag_n=1;  flag_z=0;  flag_v=0;   return;  }  

void opcode_0x38 ()  {  ea_data8=Fetch_Opcode_Byte();  UpdateFlags( AND_Common(REGISTER_CC,ea_data8) );  VMA_Cycle(1,0xFFFF);  return;  }   // ANDCC2         

void opcode_0x1B ()   {  VMA_Cycle(1,register_PC);  return; } // NOP


  
inline uint8_t DEC2_Common(uint8_t local_byte)   {
  VMA_Cycle(1,register_PC);
  if (local_byte == 0x80) flag_v=1; else flag_v=0;
  local_byte--;
  Set_Flags_Byte_NZ(local_byte);
  if (local_byte == 0xFF) flag_c=0; else flag_c=1;
  return local_byte;
} 
void opcode_0x4B ()   { register_A = DEC2_Common(register_A);                                                                                                     return;  }  // DECA2
void opcode_0x5B ()   { register_B = DEC2_Common(register_B);                                                                                                     return;  }  // DECB2
void opcode_0x0B ()   { ea_address=Calculate_EA(DIRECT);    ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,DEC2_Common(ea_data8));  return;  }  // DEC2 DIRECT
void opcode_0x6B ()   { ea_address=Calculate_EA(INDEXED);   ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,DEC2_Common(ea_data8));  return;  }  // DEC2 INDEXED
void opcode_0x7B ()   { ea_address=Calculate_EA(EXTENDED);  ea_data8=Read_Byte(ea_address);  VMA_Cycle(1,0xFFFF);  Write_Byte(ea_address,DEC2_Common(ea_data8));  return;  }  // DEC2 EXTENDED
           
       
void opcode_0x3E() {          // SWI4
  
  VMA_Cycle(1,register_PC);
  VMA_Cycle(1,0xFFFF);
  PushS16(register_PC);
  PushS16(register_USP);
  PushS16(register_Y);
  PushS16(register_X);
  PushS8(register_DP);
  PushS8(register_B);
  PushS8(register_A);
  PushS8(REGISTER_CC);
  flag_i=1;
  flag_f=1;
  VMA_Cycle(1,0xFFFF);
  register_PC = Read_Word(0xFFFE);  // Reset Vector
  VMA_Cycle(1,0xFFFF);
  page2set=0;
  page3set=0;
  
    return;
}


inline void CLR2_Common()   {
  VMA_Cycle(1,register_PC);
  flag_n = 0;
  flag_z = 1;
  flag_v = 0;
  return;
} 
void opcode_0x4E ()   { CLR2_Common();  register_A = 0;   return;  }      // CLRA2
void opcode_0x5E ()   { CLR2_Common();  register_B = 0;   return;  }      // CLRB2



void opcode_0x8F ()   { ea_data16=Fetch_Opcode_Word();   flag_n = 1; flag_z = 0; flag_v = 0;    return;  }  // STX Immediate          
void opcode_0xCF ()   { ea_data16=Fetch_Opcode_Word();   flag_n = 1; flag_z = 0; flag_v = 0;    return;  }  // STU Immediate          



// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------

void opcode_0x10 ()   {  if (page2set==0 && page3set==0) page2set=1;  return;  }     // Set Page 2/3
void opcode_0x11 ()   {  if (page2set==0 && page3set==0) page3set=1;  return;  }     //  ** Ignore all page-sets after the first instance

void (*opcodeMap[])() = {
	opcode_0x00, opcode_0x01, opcode_0x02, opcode_0x03, opcode_0x04, opcode_0x05, opcode_0x06, opcode_0x07,
	opcode_0x08, opcode_0x09, opcode_0x0A, opcode_0x0B, opcode_0x0C, opcode_0x0D, opcode_0x0E, opcode_0x0F,
	opcode_0x10, opcode_0x11, opcode_0x12, opcode_0x13, opcode_0x14, opcode_0x15, opcode_0x16, opcode_0x17,
	opcode_0x18, opcode_0x19, opcode_0x1A, opcode_0x1B, opcode_0x1C, opcode_0x1D, opcode_0x1E, opcode_0x1F,
	opcode_0x20, opcode_0x21, opcode_0x22, opcode_0x23, opcode_0x24, opcode_0x25, opcode_0x26, opcode_0x27,
	opcode_0x28, opcode_0x29, opcode_0x2A, opcode_0x2B, opcode_0x2C, opcode_0x2D, opcode_0x2E, opcode_0x2F,
	opcode_0x30, opcode_0x31, opcode_0x32, opcode_0x33, opcode_0x34, opcode_0x35, opcode_0x36, opcode_0x37,
	opcode_0x38, opcode_0x39, opcode_0x3A, opcode_0x3B, opcode_0x3C, opcode_0x3D, opcode_0x3E, opcode_0x3F,
	opcode_0x40, opcode_0x41, opcode_0x42, opcode_0x43, opcode_0x44, opcode_0x45, opcode_0x46, opcode_0x47,
	opcode_0x48, opcode_0x49, opcode_0x4A, opcode_0x4B, opcode_0x4C, opcode_0x4D, opcode_0x4E, opcode_0x4F,
	opcode_0x50, opcode_0x51, opcode_0x52, opcode_0x53, opcode_0x54, opcode_0x55, opcode_0x56, opcode_0x57,
	opcode_0x58, opcode_0x59, opcode_0x5A, opcode_0x5B, opcode_0x5C, opcode_0x5D, opcode_0x5E, opcode_0x5F,
	opcode_0x60, opcode_0x61, opcode_0x62, opcode_0x63, opcode_0x64, opcode_0x65, opcode_0x66, opcode_0x67,
	opcode_0x68, opcode_0x69, opcode_0x6A, opcode_0x6B, opcode_0x6C, opcode_0x6D, opcode_0x6E, opcode_0x6F,
	opcode_0x70, opcode_0x71, opcode_0x72, opcode_0x73, opcode_0x74, opcode_0x75, opcode_0x76, opcode_0x77,
	opcode_0x78, opcode_0x79, opcode_0x7A, opcode_0x7B, opcode_0x7C, opcode_0x7D, opcode_0x7E, opcode_0x7F,
	opcode_0x80, opcode_0x81, opcode_0x82, opcode_0x83, opcode_0x84, opcode_0x85, opcode_0x86, opcode_0x87,
	opcode_0x88, opcode_0x89, opcode_0x8A, opcode_0x8B, opcode_0x8C, opcode_0x8D, opcode_0x8E, opcode_0x8F,
	opcode_0x90, opcode_0x91, opcode_0x92, opcode_0x93, opcode_0x94, opcode_0x95, opcode_0x96, opcode_0x97,
	opcode_0x98, opcode_0x99, opcode_0x9A, opcode_0x9B, opcode_0x9C, opcode_0x9D, opcode_0x9E, opcode_0x9F,
	opcode_0xA0, opcode_0xA1, opcode_0xA2, opcode_0xA3, opcode_0xA4, opcode_0xA5, opcode_0xA6, opcode_0xA7,
	opcode_0xA8, opcode_0xA9, opcode_0xAA, opcode_0xAB, opcode_0xAC, opcode_0xAD, opcode_0xAE, opcode_0xAF,
	opcode_0xB0, opcode_0xB1, opcode_0xB2, opcode_0xB3, opcode_0xB4, opcode_0xB5, opcode_0xB6, opcode_0xB7,
	opcode_0xB8, opcode_0xB9, opcode_0xBA, opcode_0xBB, opcode_0xBC, opcode_0xBD, opcode_0xBE, opcode_0xBF,
	opcode_0xC0, opcode_0xC1, opcode_0xC2, opcode_0xC3, opcode_0xC4, opcode_0xC5, opcode_0xC6, opcode_0xC7,
	opcode_0xC8, opcode_0xC9, opcode_0xCA, opcode_0xCB, opcode_0xCC, opcode_0xCD, opcode_0xCE, opcode_0xCF,
	opcode_0xD0, opcode_0xD1, opcode_0xD2, opcode_0xD3, opcode_0xD4, opcode_0xD5, opcode_0xD6, opcode_0xD7,
	opcode_0xD8, opcode_0xD9, opcode_0xDA, opcode_0xDB, opcode_0xDC, opcode_0xDD, opcode_0xDE, opcode_0xDF,
	opcode_0xE0, opcode_0xE1, opcode_0xE2, opcode_0xE3, opcode_0xE4, opcode_0xE5, opcode_0xE6, opcode_0xE7,
	opcode_0xE8, opcode_0xE9, opcode_0xEA, opcode_0xEB, opcode_0xEC, opcode_0xED, opcode_0xEE, opcode_0xEF,
	opcode_0xF0, opcode_0xF1, opcode_0xF2, opcode_0xF3, opcode_0xF4, opcode_0xF5, opcode_0xF6, opcode_0xF7,
	opcode_0xF8, opcode_0xF9, opcode_0xFA, opcode_0xFB, opcode_0xFC, opcode_0xFD, opcode_0xFE, opcode_0xFF,
};

inline void execInstruction()
{
      if (page2set==0 && page3set==0)  {  // Dont allow interrupts between prefix and opcode
    
          if (direct_reset_n==0)                  { Reset_sequence(); }
    else if (nmi_latched==1)                     { NMI_Handler();    }           
    else if (flag_f==0 && direct_firq_n_d2==0)   { FIRQ_Handler();   }
    else if (flag_i==0 && direct_irq_n_d2==0)    { IRQ_Handler();    }

    }
    

  // Process new instruction
  //
  noInterrupts();
  opcode_byte = Fetch_Opcode_Byte();   
  opcodeMap[opcode_byte]();
  interrupts();
  /*switch (opcode_byte)  {
      
      case 0x00:  opcode_0x00();  break;
      case 0x01:  opcode_0x01();  break;
      case 0x02:  opcode_0x02();  break;
      case 0x03:  opcode_0x03();  break;
      case 0x04:  opcode_0x04();  break;
      case 0x05:  opcode_0x05();  break;
      case 0x06:  opcode_0x06();  break;
      case 0x07:  opcode_0x07();  break;
      case 0x08:  opcode_0x08();  break;
      case 0x09:  opcode_0x09();  break;
      case 0x0A:  opcode_0x0A();  break;
      case 0x0B:  opcode_0x0B();  break;
      case 0x0C:  opcode_0x0C();  break;
      case 0x0D:  opcode_0x0D();  break;
      case 0x0E:  opcode_0x0E();  break;
      case 0x0F:  opcode_0x0F();  break;
      case 0x10:  opcode_0x10();  break;
      case 0x11:  opcode_0x11();  break;
      case 0x12:  opcode_0x12();  break;
      case 0x13:  opcode_0x13();  break;
      case 0x14:  opcode_0x14();  break;
      case 0x15:  opcode_0x15();  break;
      case 0x16:  opcode_0x16();  break;
      case 0x17:  opcode_0x17();  break;
      case 0x18:  opcode_0x18();  break;
      case 0x19:  opcode_0x19();  break;
      case 0x1A:  opcode_0x1A();  break;
      case 0x1B:  opcode_0x1B();  break;
      case 0x1C:  opcode_0x1C();  break;
      case 0x1D:  opcode_0x1D();  break;
      case 0x1E:  opcode_0x1E();  break;
      case 0x1F:  opcode_0x1F();  break;
      case 0x20:  opcode_0x20();  break;
      case 0x21:  opcode_0x21();  break;
      case 0x22:  opcode_0x22();  break;
      case 0x23:  opcode_0x23();  break;
      case 0x24:  opcode_0x24();  break;
      case 0x25:  opcode_0x25();  break;
      case 0x26:  opcode_0x26();  break;
      case 0x27:  opcode_0x27();  break;
      case 0x28:  opcode_0x28();  break;
      case 0x29:  opcode_0x29();  break;
      case 0x2A:  opcode_0x2A();  break;
      case 0x2B:  opcode_0x2B();  break;
      case 0x2C:  opcode_0x2C();  break;
      case 0x2D:  opcode_0x2D();  break;
      case 0x2E:  opcode_0x2E();  break;
      case 0x2F:  opcode_0x2F();  break;
      case 0x30:  opcode_0x30();  break;
      case 0x31:  opcode_0x31();  break;
      case 0x32:  opcode_0x32();  break;
      case 0x33:  opcode_0x33();  break;
      case 0x34:  opcode_0x34();  break;
      case 0x35:  opcode_0x35();  break;
      case 0x36:  opcode_0x36();  break;
      case 0x37:  opcode_0x37();  break;
      case 0x38:  opcode_0x38();  break;
      case 0x39:  opcode_0x39();  break;
      case 0x3A:  opcode_0x3A();  break;
      case 0x3B:  opcode_0x3B();  break;
      case 0x3C:  opcode_0x3C();  break;
      case 0x3D:  opcode_0x3D();  break;
      case 0x3E:  opcode_0x3E();  break;
      case 0x3F:  opcode_0x3F();  break;
      case 0x40:  opcode_0x40();  break;
      case 0x41:  opcode_0x41();  break;
      case 0x42:  opcode_0x42();  break;
      case 0x43:  opcode_0x43();  break;
      case 0x44:  opcode_0x44();  break;
      case 0x45:  opcode_0x45();  break;
      case 0x46:  opcode_0x46();  break;
      case 0x47:  opcode_0x47();  break;
      case 0x48:  opcode_0x48();  break;
      case 0x49:  opcode_0x49();  break;
      case 0x4A:  opcode_0x4A();  break;
      case 0x4B:  opcode_0x4B();  break;
      case 0x4C:  opcode_0x4C();  break;
      case 0x4D:  opcode_0x4D();  break;
      case 0x5C:  opcode_0x5C();  break;
      case 0x5D:  opcode_0x5D();  break;
      case 0x4E:  opcode_0x4E();  break;
      case 0x4F:  opcode_0x4F();  break;
      case 0x50:  opcode_0x50();  break;
      case 0x51:  opcode_0x51();  break;
      case 0x52:  opcode_0x52();  break;
      case 0x53:  opcode_0x53();  break;
      case 0x54:  opcode_0x54();  break;
      case 0x55:  opcode_0x55();  break;
      case 0x56:  opcode_0x56();  break;
      case 0x57:  opcode_0x57();  break;
      case 0x58:  opcode_0x58();  break;
      case 0x59:  opcode_0x59();  break;
      case 0x5A:  opcode_0x5A();  break;
      case 0x5B:  opcode_0x5B();  break;
      case 0x5E:  opcode_0x5E();  break;
      case 0x5F:  opcode_0x5F();  break;
      case 0x60:  opcode_0x60();  break;
      case 0x61:  opcode_0x61();  break;
      case 0x62:  opcode_0x62();  break;
      case 0x63:  opcode_0x63();  break;
      case 0x64:  opcode_0x64();  break;
      case 0x65:  opcode_0x65();  break;
      case 0x66:  opcode_0x66();  break;
      case 0x67:  opcode_0x67();  break;
      case 0x68:  opcode_0x68();  break;
      case 0x69:  opcode_0x69();  break;
      case 0x6A:  opcode_0x6A();  break;
      case 0x6B:  opcode_0x6B();  break;
      case 0x6C:  opcode_0x6C();  break;
      case 0x6D:  opcode_0x6D();  break;
      case 0x6E:  opcode_0x6E();  break;
      case 0x6F:  opcode_0x6F();  break;
      case 0x70:  opcode_0x70();  break;
      case 0x71:  opcode_0x71();  break;
      case 0x72:  opcode_0x72();  break;
      case 0x73:  opcode_0x73();  break;
      case 0x74:  opcode_0x74();  break;
      case 0x75:  opcode_0x75();  break;
      case 0x76:  opcode_0x76();  break;
      case 0x77:  opcode_0x77();  break;
      case 0x78:  opcode_0x78();  break;
      case 0x79:  opcode_0x79();  break;
      case 0x7A:  opcode_0x7A();  break;
      case 0x7B:  opcode_0x7B();  break;
      case 0x7C:  opcode_0x7C();  break;
      case 0x7D:  opcode_0x7D();  break;
      case 0x7E:  opcode_0x7E();  break;
      case 0x7F:  opcode_0x7F();  break;
      case 0x80:  opcode_0x80();  break;
      case 0x81:  opcode_0x81();  break;
      case 0x82:  opcode_0x82();  break;
      case 0x83:  opcode_0x83();  break;
      case 0x84:  opcode_0x84();  break;
      case 0x85:  opcode_0x85();  break;
      case 0x86:  opcode_0x86();  break;
      case 0x87:  opcode_0x87();  break;
      case 0x88:  opcode_0x88();  break;
      case 0x89:  opcode_0x89();  break;
      case 0x8A:  opcode_0x8A();  break;
      case 0x8B:  opcode_0x8B();  break;
      case 0x8C:  opcode_0x8C();  break;
      case 0x8D:  opcode_0x8D();  break;
      case 0x8E:  opcode_0x8E();  break;
      case 0x8F:  opcode_0x8F();  break;
      case 0x90:  opcode_0x90();  break;
      case 0x91:  opcode_0x91();  break;
      case 0x92:  opcode_0x92();  break;
      case 0x93:  opcode_0x93();  break;
      case 0x94:  opcode_0x94();  break;
      case 0x95:  opcode_0x95();  break;
      case 0x96:  opcode_0x96();  break;
      case 0x97:  opcode_0x97();  break;
      case 0x98:  opcode_0x98();  break;
      case 0x99:  opcode_0x99();  break;
      case 0x9A:  opcode_0x9A();  break;
      case 0x9B:  opcode_0x9B();  break;
      case 0x9C:  opcode_0x9C();  break;
      case 0x9D:  opcode_0x9D();  break;
      case 0x9E:  opcode_0x9E();  break;
      case 0x9F:  opcode_0x9F();  break;
      case 0xA0:  opcode_0xA0();  break;
      case 0xA1:  opcode_0xA1();  break;
      case 0xA2:  opcode_0xA2();  break;
      case 0xA3:  opcode_0xA3();  break;
      case 0xA4:  opcode_0xA4();  break;
      case 0xA5:  opcode_0xA5();  break;
      case 0xA6:  opcode_0xA6();  break;
      case 0xA7:  opcode_0xA7();  break;
      case 0xA8:  opcode_0xA8();  break;
      case 0xA9:  opcode_0xA9();  break;
      case 0xAA:  opcode_0xAA();  break;
      case 0xAB:  opcode_0xAB();  break;
      case 0xAC:  opcode_0xAC();  break;
      case 0xAD:  opcode_0xAD();  break;
      case 0xAE:  opcode_0xAE();  break;
      case 0xAF:  opcode_0xAF();  break;
      case 0xB0:  opcode_0xB0();  break;
      case 0xB1:  opcode_0xB1();  break;
      case 0xB2:  opcode_0xB2();  break;
      case 0xB3:  opcode_0xB3();  break;
      case 0xB4:  opcode_0xB4();  break;
      case 0xB5:  opcode_0xB5();  break;
      case 0xB6:  opcode_0xB6();  break;
      case 0xB7:  opcode_0xB7();  break;
      case 0xB8:  opcode_0xB8();  break;
      case 0xB9:  opcode_0xB9();  break;
      case 0xBA:  opcode_0xBA();  break;
      case 0xBB:  opcode_0xBB();  break;
      case 0xBC:  opcode_0xBC();  break;
      case 0xBD:  opcode_0xBD();  break;
      case 0xBE:  opcode_0xBE();  break;
      case 0xBF:  opcode_0xBF();  break;
      case 0xC0:  opcode_0xC0();  break;
      case 0xC1:  opcode_0xC1();  break;
      case 0xC2:  opcode_0xC2();  break;
      case 0xC3:  opcode_0xC3();  break;
      case 0xC4:  opcode_0xC4();  break;
      case 0xC5:  opcode_0xC5();  break;
      case 0xC6:  opcode_0xC6();  break;
      case 0xC7:  opcode_0xC7();  break;
      case 0xC8:  opcode_0xC8();  break;
      case 0xC9:  opcode_0xC9();  break;
      case 0xCA:  opcode_0xCA();  break;
      case 0xCB:  opcode_0xCB();  break;
      case 0xCC:  opcode_0xCC();  break;
      case 0xCD:  opcode_0xCD();  break;
      case 0xCE:  opcode_0xCE();  break;
      case 0xCF:  opcode_0xCF();  break;
      case 0xD0:  opcode_0xD0();  break;
      case 0xD1:  opcode_0xD1();  break;
      case 0xD2:  opcode_0xD2();  break;
      case 0xD3:  opcode_0xD3();  break;
      case 0xD4:  opcode_0xD4();  break;
      case 0xD5:  opcode_0xD5();  break;
      case 0xD6:  opcode_0xD6();  break;
      case 0xD7:  opcode_0xD7();  break;
      case 0xD8:  opcode_0xD8();  break;
      case 0xD9:  opcode_0xD9();  break;
      case 0xDA:  opcode_0xDA();  break;
      case 0xDB:  opcode_0xDB();  break;
      case 0xDC:  opcode_0xDC();  break;
      case 0xDD:  opcode_0xDD();  break;
      case 0xDE:  opcode_0xDE();  break;
      case 0xDF:  opcode_0xDF();  break;
      case 0xE0:  opcode_0xE0();  break;
      case 0xE1:  opcode_0xE1();  break;
      case 0xE2:  opcode_0xE2();  break;
      case 0xE3:  opcode_0xE3();  break;
      case 0xE4:  opcode_0xE4();  break;
      case 0xE5:  opcode_0xE5();  break;
      case 0xE6:  opcode_0xE6();  break;
      case 0xE7:  opcode_0xE7();  break;
      case 0xE8:  opcode_0xE8();  break;
      case 0xE9:  opcode_0xE9();  break;
      case 0xEA:  opcode_0xEA();  break;
      case 0xEB:  opcode_0xEB();  break;
      case 0xEC:  opcode_0xEC();  break;
      case 0xED:  opcode_0xED();  break;
      case 0xEE:  opcode_0xEE();  break;
      case 0xEF:  opcode_0xEF();  break;
      case 0xF0:  opcode_0xF0();  break;
      case 0xF1:  opcode_0xF1();  break;
      case 0xF2:  opcode_0xF2();  break;
      case 0xF3:  opcode_0xF3();  break;
      case 0xF4:  opcode_0xF4();  break;
      case 0xF5:  opcode_0xF5();  break;
      case 0xF6:  opcode_0xF6();  break;
      case 0xF7:  opcode_0xF7();  break;
      case 0xF8:  opcode_0xF8();  break;
      case 0xF9:  opcode_0xF9();  break;
      case 0xFA:  opcode_0xFA();  break;
      case 0xFB:  opcode_0xFB();  break;
      case 0xFC:  opcode_0xFC();  break;
      case 0xFD:  opcode_0xFD();  break;
      case 0xFE:  opcode_0xFE();  break;
      case 0xFF:  opcode_0xFF();  break;
    }*/


}

// -------------------------------------------------
//
// Main loop
//
// -------------------------------------------------
void loop() {

  uint16_t local_counter=0;


  // Give Teensy 4.1 a moment
  //
  delay (100);
  wait_for_CLK_rising_edge();
  wait_for_CLK_rising_edge();
  wait_for_CLK_rising_edge();
  //BIU_Write_Byte(65497, 0);
  wait_for_CLK_rising_edge();
  wait_for_CLK_rising_edge();
  wait_for_CLK_rising_edge();
 

  Reset_sequence();

//  attachInterrupt(digitalPinToInterrupt(PIN_CLK_E), execInstruction, FALLING);

  
  while (1) {
   
      // Acceleration Modes
      // -----------------------
      // Press 0 = All acceleration and mirroring disabled   
      // Press 1 = Eliminates 6809 VMA cycles   
      // Press 2 = Reads and writes are cycle accurate using internal memory with writes passing through to motherboard   
      // Press 3 = Reads accelerated using internal memory and writes are cycle accurate and pass through to motherboard   
      // Press 4 = All read and write accesses use accelerated internal memory    

      // Press 6 = Load Cartridge - Tandy diagnostics 
      // Press 7 = DragonFire
      // Press 8 = Stellar Lifeline
      // Press 9 = Canyon Climber
/*
      local_counter++;
      if (local_counter==8000){
        if (Serial.available() ) { 
          incomingByte = Serial.read();   
          switch (incomingByte){
            case 48: mode=0;    break;
            case 49: mode=1;    break;
            case 50: mode=2;    break;
            case 51: mode=3;    break;
            case 52: mode=4;    break;
            
            case 54: cart=0;    break;
            case 55: cart=1;    break;
            case 56: cart=2;    break;
            case 57: cart=3;    break;
          }
        }
      }
      
      // Poll for interrupts between instructions
      //
*/
    execInstruction();
// ** End main loop
  } 
}
      
