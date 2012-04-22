
#include <SPI.h>


void setup()  
{
  Serial.begin(9600);
  //flash.attach(6,7,8);
	pinMode(10, OUTPUT);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2); //fastest SPI clock is 8Mhz

}

byte status = 0x0F;

void loop() 
{
	digitalWrite(10, HIGH);
	delayMicroseconds(10);
	digitalWrite(10, LOW);
	SPI.transfer(0x05);
	delayMicroseconds(2);
    status = SPI.transfer(0x00);
	digitalWrite(10, HIGH);
	Serial.print("Sent: ");
	Serial.println(0x05, BIN);
	Serial.print("Received: ");
  	Serial.println(status, BIN);
	Serial.println();

  	delay(2000);
/*

digitalWrite(2,LOW);
delay(1000);
digitalWrite(2,HIGH);
delay(1000);
*/
}
