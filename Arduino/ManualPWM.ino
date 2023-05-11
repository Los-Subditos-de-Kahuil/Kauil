void setCompareRegs(const int channel, const int value) {
  if (channel == 1) {
    OCR1A = value;
  } else {
    OCR1B = value;
  }
}

int getValue(const String inString, const int colonIndex) {
  return inString.substring(colonIndex + 1).toInt();
}

int getChannel(const String inString, const int colonIndex) {
  return inString.substring(2, colonIndex).toInt();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(9, OUTPUT); // OC1A
  pinMode(10, OUTPUT); // OC1B
  /*
   * Using 16-bit counter, when match up RESET, when match down SET.
   * TOP comes from ICR1.
   * Prescaler N = 8.
  */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS11);
  ICR1 = 20000;
  setCompareRegs(1, 1432);
  setCompareRegs(2, 1390);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() == 0) {}
  String inString = Serial.readStringUntil('\n');
  inString.trim();
  int colonIndex = inString.indexOf(":");

  int value = getValue(inString, colonIndex);
  int channel = getChannel(inString, colonIndex);
  setCompareRegs(channel, value);
}
