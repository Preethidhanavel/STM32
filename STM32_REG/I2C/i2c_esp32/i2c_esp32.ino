#include <Wire.h>
#include<stdio.h>

#define MY_ADDR   0x68       // I2C slave address
#define SDA_PIN 21           // I2C data pin
#define SCL_PIN 22           // I2C clock pin

int LED = 2;                // Onboard LED pin
char rx_buffer[32];         // Buffer to store received I2C data
uint32_t cnt = 0;           // Counter for buffer

void setup() {
  Serial.begin(9600);       // Initialize Serial for debugging
  delay(500);

  pinMode(LED, OUTPUT);     
  digitalWrite(LED, HIGH);  // Turn LED ON initially

  // Initialize I2C as slave with given address and pins
  Wire.begin((uint8_t)MY_ADDR, SDA_PIN, SCL_PIN);

  pinMode(SDA_PIN, INPUT_PULLUP); // Pull-up for I2C pins
  pinMode(SCL_PIN, INPUT_PULLUP); // Pull-up for I2C pins

  Wire.onReceive(receiveEvent);   // Set function to call when data is received

  Serial.println("Waiting for data from master");  
}

void loop(void)
{
  // Main loop does nothing; all action happens in receiveEvent
}

// Function called when data is received from I2C master
void receiveEvent(int bytes) 
{
  digitalWrite(LED, LOW);         // Turn LED OFF when data received
  while(Wire.available())         // Read all bytes sent by master
  {
    rx_buffer[cnt++] = Wire.read();
  }
  rx_buffer[cnt] = '\0';          // Null-terminate string
  cnt = 0;                        // Reset counter

  Serial.print("Received:");  
  Serial.println((char*)rx_buffer); // Print received data
}
