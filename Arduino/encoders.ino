/////////////////////////// COMUNICACION SERIAL //////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 1;
double data[dataLength]; // Valor regula ciclo de trabajo (PWM)

const byte C1_Encoder1 = 3;  // Señal A del encoder 1
const byte C2_Encoder1 = 2;  // Señal B del encoder 1
const byte C1_Encoder2 = 5;  // Señal A del encoder 2
const byte C2_Encoder2 = 4;  // Señal B del encoder 2

const byte in1 = 7;
const byte in2 = 8;
const byte ena = 6;

volatile int n1 = 0; // Contador de pulsos del encoder 1
volatile int n2 = 0; // Contador de pulsos del encoder 2
volatile byte ant1 = 0; // Estado anterior de la señal A del encoder 1
volatile byte act1 = 0; // Estado actual de la señal A del encoder 1
volatile byte ant2 = 0; // Estado anterior de la señal A del encoder 2
volatile byte act2 = 0; // Estado actual de la señal A del encoder 2

double N1 = 0.0; // Revoluciones por minuto del encoder 1
double N2 = 0.0; // Revoluciones por minuto del encoder 2
int cv = 0; // Variable de control (duty cycle)

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

const int R = 500 * 4; // Resolucion del encoder R = mH*s*r.

void setup()
{
  Serial.begin(9600);

  pinMode(C1_Encoder1, INPUT);
  pinMode(C2_Encoder1, INPUT);
  pinMode(C1_Encoder2, INPUT);
  pinMode(C2_Encoder2, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, false);
  digitalWrite(in2, false);

  analogWrite(ena, cv);

  attachInterrupt(digitalPinToInterrupt(C1_Encoder1), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2_Encoder1), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C1_Encoder2), encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2_Encoder2), encoder2, CHANGE);

  Serial.print("Pulsos Encoder 1: "); Serial.println(n1);
  Serial.print("Pulsos Encoder 2: "); Serial.println(n2);

  Serial.println("Velocidad RPM");
}

void loop() {

  ////////// SI RECIBE DATOS /////////////
  if (stringComplete)
  {
    for (int i = 0; i < dataLength; i++)
    {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toInt();
      inputString = inputString.substring(index + 1);
    }

    cv = data[0];
    if (cv > 0) anticlockwise(in2, in1, ena, cv);
    else clockwise(in2, in1, ena, abs(cv));
    inputString = "";
    stringComplete = false;
  }

  if (millis() - lastTime >= sampleTime)
  {  // Se actualiza cada sampleTime (milisegundos)
    computeRpm();
    Serial.print(N1); Serial.print(","); Serial.println(N2);
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void computeRpm(void)
{
  N1 = (n1 * 60.0 * 1000.0) / ((millis() - lastTime) * R); // Calculo revoluciones por minuto del encoder 1
  N2 = (n2 * 60.0 * 1000.0) / ((millis() - lastTime) * R); // Calculo revoluciones por minuto del encoder 2
  lastTime = millis(); // Almacenar el tiempo actual.
  n1 = 0;  // Resetear los pulsos del encoder 1.
  n2 = 0;  // Resetear los pulsos del encoder 2.
}

void clockwise(int pin1, int pin2, int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  analogWrite(analogPin, pwm);
}

void anticlockwise(int pin1, int pin2, int analogPin, int pwm)
{
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
  analogWrite(analogPin, pwm);
}

// Encoder 1 - precisión cuádruple.
void encoder1(void)
{
  ant1 = act1;
  ant2 = act2;
  act1 = (digitalRead(C1_Encoder1) << 1) | digitalRead(C2_Encoder1);
  act2 = (digitalRead(C1_Encoder2) << 1) | digitalRead(C2_Encoder2);

  if (ant1 == 0 && act1 == 1) n1--;
  if (ant1 == 1 && act1 == 3) n1--;
  if (ant1 == 2 && act1 == 0) n1--;
  if (ant1 == 3 && act1 == 2) n1--;

  if (ant1 == 0 && act1 == 2) n1++;
  if (ant1 == 1 && act1 == 0) n1++;
  if (ant1 == 2 && act1 == 3) n1++;
  if (ant1 == 3 && act1 == 1) n1++;

  if (ant2 == 0 && act2 == 1) n2--;
  if (ant2 == 1 && act2 == 3) n2--;
  if (ant2 == 2 && act2 == 0) n2--;
  if (ant2 == 3 && act2 == 2) n2--;

  if (ant2 == 0 && act2 == 2) n2++;
  if (ant2 == 1 && act2 == 0) n2++;
  if (ant2 == 2 && act2 == 3) n2++;
  if (ant2 == 3 && act2 == 1) n2++;
}

// Encoder 2 - precisión cuádruple.
void encoder2(void)
{
  ant1 = act1;
  ant2 = act2;
  act1 = (digitalRead(C1_Encoder1) << 1) | digitalRead(C2_Encoder1);
  act2 = (digitalRead(C1_Encoder2) << 1) | digitalRead(C2_Encoder2);

  if (ant1 == 0 && act1 == 1) n1--;
  if (ant1 == 1 && act1 == 3) n1--;
  if (ant1 == 2 && act1 == 0) n1--;
  if (ant1 == 3 && act1 == 2) n1--;

  if (ant1 == 0 && act1 == 2) n1++;
  if (ant1 == 1 && act1 == 0) n1++;
  if (ant1 == 2 && act1 == 3) n1++;
  if (ant1 == 3 && act1 == 1) n1++;

  if (ant2 == 0 && act2 == 1) n2--;
  if (ant2 == 1 && act2 == 3) n2--;
  if (ant2 == 2 && act2 == 0) n2--;
  if (ant2 == 3 && act2 == 2) n2--;

  if (ant2 == 0 && act2 == 2) n2++;
  if (ant2 == 1 && act2 == 0) n2++;
  if (ant2 == 2 && act2 == 3) n2++;
  if (ant2 == 3 && act2 == 1) n2++;
}
