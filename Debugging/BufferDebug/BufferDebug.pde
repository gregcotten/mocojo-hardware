#include <LongBuffer.h>

LongBuffer buffer(5);

void setup(){
  Serial.begin(115200);
}

void loop(){
  int i = 5;
  while(!buffer.isFull()){
    buffer.addLong(i);
    i++;
  }
  Serial.println("Amount Fresh: " + String(buffer.amountFresh(), DEC));
  
  while(buffer.amountFresh() > 0){
    Serial.println(buffer.nextLong());
  }

  while(!buffer.isFull()){
    buffer.addLong(i);
    i++;
  }
  Serial.println("Amount Fresh: " + String(buffer.amountFresh(), DEC));
  while(buffer.amountFresh() > 0){
    Serial.println(buffer.nextLong());
  }
 delay(10000);
}
