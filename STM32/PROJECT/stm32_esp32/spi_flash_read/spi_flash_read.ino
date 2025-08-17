#include <SPI.h>        // Include SPI library
#include <SPIFlash.h>   // Include SPI Flash memory library

#define ext_flash_cs_u8 5           // Chip Select (CS) pin for external flash
#define ext_flash_base_addr_u8 0    // External flash starting address
#define ext_flash_size_u16 (2*1024) // External flash size (2 KB)

// Create flash object with given CS pin
SPIFlash flash(ext_flash_cs_u8);  

uint32_t pc_u32 =0;      // Starting read address (0)
char rcvData[26];        // Buffer to store data read from flash

// Function to read data from external flash
void ReadDataExtFlash()
{  
  // Check if current address is within flash size limit
  if (pc_u32 <= ext_flash_base_addr_u8 + ext_flash_size_u16) 
  {
      // Read data from current address into rcvData
      flash.readBytes(pc_u32, rcvData, sizeof(rcvData));

      // Move address pointer 
      pc_u32 += 26 + 1;

      // Print the received data
      Serial.print(" ");
      Serial.println(rcvData);

      // Delay 1 second
      delay(1000);
  }   
  else
  {  
    // If size limit reached, reset pointer to base address
    pc_u32 = ext_flash_base_addr_u8;
    // Print the maximum allowed address
    Serial.println(ext_flash_base_addr_u8 + ext_flash_size_u16);
    // Erase the entire flash chip
    //flash.chipErase();
    Serial.println("size reached"); 
  }   
}

void setup() 
{
  Serial.begin(9600); // Start serial communication
  Serial.println("External flash");

  SPI.begin();        // Initialize SPI interface
  delay(2000);        // Wait for flash to stabilize

  // Initialize flash chip and check for success
  if (!flash.initialize()) 
  {
    Serial.println("Flash memory initialization failed!");
    while (1)  // Infinite loop to indicate failure
      Serial.print(".");
  }
}

void loop() 
{
  // Continuously read data from external flash
  ReadDataExtFlash(); 
}



