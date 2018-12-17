
int a = 2;  //For displaying segment "a"
int b = 3;  //For displaying segment "b"
int c = 4;  //For displaying segment "c"
int d = 5;  //For displaying segment "d"
int e = 6;  //For displaying segment "e"
int f = 7;  //For displaying segment "f"
int g = 8;  //For displaying segment "g"
int h = 9;  //Segment h
int LedMoter1 = 10;
int LedMoter2 = 11;
int LedMoter3 = 12;
int LedMoter4 = 13;

int incomingByte;      // a variable to read incoming serial data into

void setup() {
  // initialize serial communication:
  Serial.begin(115200);
  // initialize the LED pin as an output:
  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(d, OUTPUT);
  pinMode(e, OUTPUT);
  pinMode(f, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(h, OUTPUT);
  pinMode(LedMoter1, OUTPUT);
  pinMode(LedMoter2, OUTPUT);
  pinMode(LedMoter3, OUTPUT);
  pinMode(LedMoter4, OUTPUT);
  

}

void loop() {
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == '1') {
      digitalWrite(LedMoter1, HIGH); delay(1);
      digitalWrite(LedMoter2, LOW); delay(1);
      digitalWrite(LedMoter3, LOW); delay(1);
      digitalWrite(LedMoter4, LOW); delay(1);

      digitalWrite(a, LOW); delay(1);
      digitalWrite(b, HIGH); delay(1);
      digitalWrite(c, HIGH); delay(1);
      digitalWrite(d, LOW); delay(1);
      digitalWrite(e, LOW); delay(1);
      digitalWrite(f, LOW); delay(1);
      digitalWrite(g, LOW); delay(1);
      digitalWrite(h, HIGH); delay(1);
      //digitalWrite(ledPin, HIGH);
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == '2') {
      digitalWrite(LedMoter1, LOW); delay(1);
      digitalWrite(LedMoter2, HIGH); delay(1);
      digitalWrite(LedMoter3, LOW); delay(1);
      digitalWrite(LedMoter4, LOW); delay(1);

      digitalWrite(a, HIGH); delay(1);
      digitalWrite(b, HIGH); delay(1);
      digitalWrite(c, LOW); delay(1);
      digitalWrite(d, HIGH); delay(1);
      digitalWrite(e, HIGH); delay(1);
      digitalWrite(f, LOW); delay(1);
      digitalWrite(g, HIGH); delay(1);
      digitalWrite(h, HIGH); delay(1);
    }
    if (incomingByte == '3') {
      digitalWrite(LedMoter1, LOW); delay(1);
      digitalWrite(LedMoter2, LOW); delay(1);
      digitalWrite(LedMoter3, HIGH); delay(1);
      digitalWrite(LedMoter4, LOW); delay(1);

      digitalWrite(a, HIGH); delay(1);
      digitalWrite(b, HIGH); delay(1);
      digitalWrite(c, HIGH); delay(1);
      digitalWrite(d, HIGH); delay(1);
      digitalWrite(e, LOW); delay(1);
      digitalWrite(f, LOW); delay(1);
      digitalWrite(g, HIGH); delay(1);
      digitalWrite(h, HIGH); delay(1);
    }
    if (incomingByte == '4') {
      digitalWrite(LedMoter1, LOW); delay(1);
      digitalWrite(LedMoter2, LOW); delay(1);
      digitalWrite(LedMoter3, LOW); delay(1);
      digitalWrite(LedMoter4, HIGH); delay(1);

      digitalWrite(a, LOW); delay(1);
      digitalWrite(b, HIGH); delay(1);
      digitalWrite(c, HIGH); delay(1);
      digitalWrite(d, LOW); delay(1);
      digitalWrite(e, LOW); delay(1);
      digitalWrite(f, HIGH); delay(1);
      digitalWrite(g, HIGH); delay(1);
      digitalWrite(h, HIGH); delay(1);
    }
  }
}
