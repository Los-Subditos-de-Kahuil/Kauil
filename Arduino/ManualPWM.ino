/*
Read requiered values of duty cycle and update.

Authors:
    - Alejandro Dominguez Lugo      (A01378028)
    - Diego Alberto Anaya Marquez   (A01379375)
    - Nancy Lesly Garcia Jimenez    (A01378043)

Date: 2023/04/24
*/

void setCompareRegs(const int channel, const int value) {
  /*
  Set duty cycle values in the requiered channel.
  
  Args:
    channel (int) : channel required to be updated

    value (int) : value to be updated to
  */
  if (channel == 1) {
    OCR1A = value;
  } else {
    OCR1B = value;
  }
}

int getValue(const String inString, const int colonIndex) {
  /*
  Get value from string

  Args:
    inString (String) : string that holds the value

    colonIndex (int) : index of the colon identifier

  Return:
    (int) : value
  */
  return inString.substring(colonIndex + 1).toInt();
}

int getChannel(const String inString, const int colonIndex) {
  /*
  Get channel from string

  Args:
    inString (String) : string that holds the value

    colonIndex (int) : index of the colon identifier

  Return:
    (int) : channel 
  */
  return inString.substring(2, colonIndex).toInt();
}

void setup() {
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

  // * Set to angular and linear velocities = 0.0
  setCompareRegs(1, 1432);
  setCompareRegs(2, 1390);
}

void loop() {
  // * Reed serial
  while (Serial.available() == 0) {}
  String inString = Serial.readStringUntil('\n');
  inString.trim();

  // * Get position of colon identifier
  int colonIndex = inString.indexOf(":");

  // * Get channel and value and update
  int value = getValue(inString, colonIndex);
  int channel = getChannel(inString, colonIndex);
  setCompareRegs(channel, value);
}
