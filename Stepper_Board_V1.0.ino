#include <SimpleModbusSlave.h>
#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "SparkFunAutoDriver.h"
#include <Adafruit_NeoPixel.h>
//#include <avr/pgmspace.h>


#define NUM_BOARDS 1

AutoDriver boardA(0,10, 6);


// Using the enum instruction allows for an easy method for adding and 
// removing registers. Doing it this way saves you #defining the size 
// of your slaves register array each time you want to add more registers
// and at a glimpse informs you of your slaves register layout.

//////////////// registers of your slave ///////////////////
enum 
{     
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  Modbus_Board_Serial_number,
  Modbus_Bootloader_enable,              // write back serial number for go to boot 
  Modbus_temperature,
  Modbus_DC_POWER_BUS_VOLT,
  Modbus_VFO_FREQ_KHZ,
  Modbus_Stepper_Status,
  Modbus_Move_mm,                 //
  Modbus_Rotate_DEG,  
  Modbus_Position_Register_Calculated,  
  Modbus_Position_Register_Read,
 
    
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};
unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

word serial_num;
word Bootloader_enable;
static uint32_t Calculated_Position_mm ;
static uint32_t Calculated_Position_mm_old ;
uint16_t Move_mm ;
int TMP36_Pin = A7;
int Power_in_ADC = A6;
boolean motor_move = false;


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, 14, NEO_GRB + NEO_KHZ800);

void *goto_boot = (void *) 0x3E00;   

void setup()
{  
 
 // serial_num =  pgm_read_word_near(0x3fff); 
  serial_num =  0x0001;
  
  EEPROM.write(0, 0xff);  
  
  pinMode(6, OUTPUT);     // L6472 RESET
  pinMode(MOSI, OUTPUT);  // MOSI
  pinMode(MISO, INPUT);   // MISO
  pinMode(13, OUTPUT);    // SCK
  pinMode(10, OUTPUT);    //~chip Select
  pinMode(7,  INPUT_PULLUP);      // Busy
  digitalWrite(10, HIGH); //~chip Select
  digitalWrite(6, LOW);   // RESET PIN  PUT CHIP IN RESET
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
 
  	
  modbus_configure(&Serial, 38400, SERIAL_8E1, 10, 16, HOLDING_REGS_SIZE, holdingRegs);
    
 delay(900);
 digitalWrite(6, HIGH); // RESET PIN  RELASE RESET PIN
 delay(100);

  boardA.configSyncPin(BUSY_PIN, 7);// BUSY pin low during operations;

// setup  Chip parameter register ,change only one that not default value 
// we write to register directly since lib was make for 6470 chip and we use 6472 (voltage VS current regulation)

  boardA.setParam(0x05, 0x40); // Acceleration           931 step /s
  boardA.setParam(0x06, 0x40); // Decelleration          931 step /s
  boardA.setParam(0x07, 0x30); // Max Speed              732 step /s
  boardA.setParam(0x09, 0x00); // Holding current        31ma
  boardA.setParam(0x0A, 0x10); // TVAL_RUN               531ma
  boardA.setParam(0x0B, 0x10); // TVAL_ACC               531ma
  boardA.setParam(0x0C, 0x10); // TVAL_DEC               531ma  
  boardA.setParam(0x16, 0x0F); // STEP MODE AND SYNC     1/16 no sync 
  
  boardA.move(FWD,160);
  delay(300);
  boardA.move(REV,160);
  delay(300);
  boardA.softHiZ();
        
  pixels.begin();        
  pixels.setPixelColor(0, pixels.Color(0,10,0)); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.        
}

void loop()
{
  // modbus_update() is the only method used in loop(). It returns the total error
  // count since the slave started. You don't have to use it but it's useful
  // for fault finding by the modbus master.
  
  modbus_update();   
  wdt_reset();  // kil the dog
  
// 40 turn = 117.5"  (2984.50mm)
// 3200 motor step (1trun) = 74,6125mm
// so 42.88825 step  per mm
// max step for whole antenna 231000 (5386.09mm / 212")
// 70742100 div  Modbus_VFO_FREQ_KHZ =  lenght_mm * 42.88825 = Calculated_Position_mm

// long lenght_mm = (long) (70742100 / holdingRegs[Modbus_VFO_FREQ_KHZ]);
 long lenght_mm =  (70742100 / holdingRegs[Modbus_VFO_FREQ_KHZ]);
 Calculated_Position_mm = (lenght_mm * 42.88825);
 holdingRegs[Modbus_Move_mm] = Calculated_Position_mm;
 //Calculated_Position_mm =   holdingRegs[Modbus_Move_mm] * 42.88825 ;   ///compute requested step per mm  from modbus register 0  
  
 int reading = analogRead(TMP36_Pin);  //getting the voltage reading from the temperature sensor
 float voltage = reading * 5.0;    // converting that reading to voltage
 voltage /= 1024.0; 
 float temperatureC = (voltage - 0.5) * 100 ; // converting Celsus
 
 int reading2 = analogRead(Power_in_ADC);  //getting the voltage reading from power suppply input
 float voltage2 = reading2 * 5.0;    // converting that reading to voltage
 voltage2 /= 1024.0; 
 float Power_in_ADC_Volt =(voltage2 * 6.5) ; // converting variable
 
 holdingRegs[Modbus_Board_Serial_number] = serial_num; // report Board Serial over modbus
 holdingRegs[Modbus_temperature] = temperatureC; // report temperature over modbus
 holdingRegs[Modbus_DC_POWER_BUS_VOLT] = Power_in_ADC_Volt; // report mot volage over modbus
 holdingRegs[Modbus_Position_Register_Calculated] = Calculated_Position_mm;  //  return calculated mm value for debog 
 holdingRegs[Modbus_Position_Register_Read] = boardA.getPos();  //  return calculated mm value for debog
 holdingRegs[Modbus_Stepper_Status] = boardA.getStatus();  //  return motor Status
 
 if (Calculated_Position_mm_old != Calculated_Position_mm ) 
   { 
     pixels.setPixelColor(0, pixels.Color(10,0,0)); 
     pixels.show(); // This sends the updated pixel color to the hardware.
     boardA.goTo(Calculated_Position_mm );
     motor_move = true; 
     Calculated_Position_mm_old = Calculated_Position_mm;     
   }  
   
 if (holdingRegs[Modbus_Bootloader_enable] == serial_num)  // if user sent board serial mumber in boot enable register
   {
     pixels.setPixelColor(0, pixels.Color(0,0,10));
     pixels.show(); // This sends the updated pixel color to the hardware.
     boardA.goTo(0);  // retrack antenne before firm upgrade
     EEPROM.write(0, 0xAD);   // mark  eeprom flag for stay in bootloader
     while(true){}       // just do noting until  watchdog reset the whole thing
 //    goto *goto_boot;   // jump to bootlaoder 
 //    asm volatile ("jmp 0x3E00");  
   }  
   
  if (holdingRegs[Modbus_Bootloader_enable] == 0xaadd)  // if user sent board 0xFFFF in boot enable register
   {
     pixels.setPixelColor(0, pixels.Color(10,10,10));
     pixels.show(); // This sends the updated pixel color to the hardware.
     boardA.setParam(0x0A, 0x05); // TVAL_RUN               187ma
     boardA.setParam(0x0B, 0x05); // TVAL_ACC               187ma
     boardA.setParam(0x0C, 0x05); // TVAL_DEC               187ma 
     boardA.move(REV,232000);           // home antenna element (retracted)
     while(digitalRead(7) == LOW)   // whait it completely retrackted  
      {wdt_reset();}   
      asm volatile ("jmp 0x0");      
    }    
   
 if ((digitalRead(7) == HIGH) && (motor_move == true))  // fix bog in chip after move stop send soft stop command 
   {
     boardA.softStop();
     boardA.softHiZ();
     motor_move = false;
     pixels.setPixelColor(0, pixels.Color(0,10,0));
     pixels.show(); // This sends the updated pixel color to the hardware.
   }  

}

