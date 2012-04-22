
#include <SPI.h>

void setup()  
{
  Serial.begin(115200);
  //flash.attach(6,7,8);
	pinMode(10, OUTPUT);
	
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2); //fastest SPI clock is 8Mhz
	//writeStatus();
}

byte status1 = 0x00;
byte status2 = 0x00;

void writeConfiguration(byte config1, byte config2) 
{
	digitalWrite(10, HIGH);
	delayMicroseconds(10);
	digitalWrite(10, LOW);
	delayMicroseconds(1);
	SPI.transfer(B11100000); 
    SPI.transfer(B00001111);
	digitalWrite(10, HIGH);
}

void loop() 
{
	digitalWrite(10, HIGH);
	delayMicroseconds(10);
	digitalWrite(10, LOW);
	delayMicroseconds(1);
	status1 = SPI.transfer(0x40); //01000000b
    status2 = SPI.transfer(0x00);
	digitalWrite(10, HIGH);

  	Serial.print(ConvertNumberToBinaryString(status1));
	Serial.print(" ");
	Serial.println(ConvertNumberToBinaryString(status2));

  	delay(2000);
/*

digitalWrite(2,LOW);
delay(1000);
digitalWrite(2,HIGH);
delay(1000);
*/
}

String ConvertNumberToBinaryString(byte myNum)
{
	int zeros = 8 - String(myNum,BIN).length();
	String myStr;
	for (int i=0; i<zeros; i++) {
	  myStr = myStr + "0";
	}
	return myStr + String(myNum, BIN);
}
