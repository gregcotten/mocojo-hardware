void setup()
{
	Serial.begin(1000000);
}

void loop()
{
	if(Serial.available()){
			Serial.read();
			Serial.println("hello");
	}
}