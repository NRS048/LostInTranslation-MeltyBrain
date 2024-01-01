/*
accel1->convertToG(a1.accel_x)
accel1->convertToG(a1.accel_y)
accel1->convertToG(a2.accel_x)
accel1->convertToG(a2.accel_y)
*/
float a = -3;
float b = 13.5;
float c = 14.5;
float d = -1.2;

float rpm = 0;

void setup() {
  Serial.println(micros());
  //(500/sqrt(1.118))
  //sqrt(sq(a)+sq(b))
  //sqrt( sq(sqrt(2)*(-a-b+c+d)+30) + 2 * sq(a-b-c+d) )
  //sqrt(sqrt(sq(a)+sq(b))/(sqrt( sq(sqrt(2)*(-a-b+c+d)+30) + 2 * sq(a-b-c+d) )))
  //sqrt(sq(c)+sq(d))
  //sqrt( sq(sqrt(2)*(-a-b+c+d)-30) + 2 * sq(a-b-c+d) )
  //sqrt(sqrt(sq(c)+sq(d))/(sqrt( sq(sqrt(2)*(-a-b+c+d)-30) + 2 * sq(a-b-c+d) )))
  
  //( sqrt(sqrt(sq(a)+sq(b))/(sqrt( sq(sqrt(2)*(-a-b+c+d)+30) + 2 * sq(a-b-c+d) ))) + sqrt(sqrt(sq(c)+sq(d))/(sqrt( sq(sqrt(2)*(-a-b+c+d)-30) + 2 * sq(a-b-c+d) ))) )


  rpm = (500/sqrt(1.118)) * ( sqrt(sqrt(sq(a)+sq(b))/(sqrt( sq(sqrt(2)*(-a-b+c+d)+30) + 2 * sq(a-b-c+d) ))) + sqrt(sqrt(sq(c)+sq(d))/(sqrt( sq(sqrt(2)*(-a-b+c+d)-30) + 2 * sq(a-b-c+d) ))) );


  Serial.println(rpm);
  Serial.println(micros());
}

void loop() {
}
