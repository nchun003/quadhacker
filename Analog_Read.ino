void setup() {
  // open a serial connection
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  analogReadResolution(8);
  unsigned short bit_1;
  unsigned short bit_2;
  unsigned short bit_3;
  unsigned short bit_4;

  bit_1 = analogRead(A0);
  bit_2 = analogRead(A1);
  bit_3 = analogRead(A2);
  bit_4 = analogRead(A3);

  delay(100);

}
