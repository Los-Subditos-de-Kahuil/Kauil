#define RCPin 9
int RCValue;

void setup() {
  Serial.begin(9600);
  pinMode(RCPin, INPUT);
}

void loop() {
  RCValue = pulseIn(RCPin, HIGH);
  Serial.println(RCValue);
}
