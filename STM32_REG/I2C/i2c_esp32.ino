
#include <Wire.h>
#include<stdio.h>
#define MY_ADDR   0x68
#define SDA_PIN 21
#define SCL_PIN 22

int LED = 2;
char rx_buffer[32] ;
uint32_t cnt =0;
//uint8_t message[50];


void setup() {

  Serial.begin(9600);
  delay(500);

  pinMode (LED, OUTPUT);
  digitalWrite(2,HIGH);
  Wire.begin((uint8_t)MY_ADDR, SDA_PIN, SCL_PIN);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.onReceive(receiveEvent);
  Serial.println("Waiting for data from master");  
}

void loop(void)
{
 
}

void receiveEvent(int bytes) 
{
  digitalWrite(2,LOW);
 while( Wire.available() )
 {
   rx_buffer[cnt++] = Wire.read();
 }
  rx_buffer[cnt] = '\0';
  cnt=0;
  Serial.print("Received:");  
  Serial.println((char*)rx_buffer);  
}
