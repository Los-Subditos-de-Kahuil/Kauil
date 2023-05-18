#include <ros.h>
#include <std_msgs/Float64.h>

String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 1;
double data[dataLength]; // Valor regula ciclo de trabajo (PWM)

const byte C1 = 3; // Entrada de la señal A del encoder (Cable amarillo).
const byte C2 = 2; // Entrada de la señal B del encoder (Cable anaranjado).

const byte in1 = 7;
const byte in2 = 8;
const byte ena = 6;

volatile int n = 0;
volatile byte ant = 0;
volatile byte act = 0;

double N = 0.0; // Revoluciones por minuto calculadas.
int cv = 0;     // Variable de control (duty cycle)

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

const int R = 500 * 4; // Resolucion del encoder R = mH*s*r.

ros::NodeHandle nh;
std_msgs::Float64 msg;
ros::Publisher pub("rpm_topic", &msg);

void setup()
{
  Serial.begin(9600);

  nh.initNode();
  nh.advertise(pub);

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, false);
  digitalWrite(in2, false);

  analogWrite(ena, cv);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);
  //Serial.print("Pulsos: ");
  //Serial.println(n);

  Serial.println("Velocidad RPM");
}

void loop()
{

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
    if (cv > 0)
      anticlockwise(in2, in1, ena, cv);
    else
      clockwise(in2, in1, ena, abs(cv));
    inputString = "";
    stringComplete = false;
  }

  if (millis() - lastTime >= sampleTime)
  { // Se actualiza cada sampleTime (milisegundos)
    computeRpm();
    Serial.print("RPM: ");
    Serial.println(N);

    std_msgs::Float64 msg;
    msg.data = N;
    pub.publish(&msg);
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }
}

void computeRpm(void)
{
  N = (n * 60.0 * 1000.0) / ((millis() - lastTime) * R); // Calculo revoluciones por minuto
  lastTime = millis();                                   // Almacenar el tiempo actual.
  n = 0;                                                 // Resetear los pulsos.
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


// Encoder precisión cuádruple.
void encoder(void)
{
    ant=act;                           
    act=PIND & 12;         
                           
    if(ant==0  && act== 4)  n--;
    if(ant==4  && act==12)  n--;
    if(ant==8  && act== 0)  n--;
    if(ant==12 && act== 8)  n--;
    
    if(ant==0  && act==8)  n++; 
    if(ant==4  && act==0)  n++;
    if(ant==8  && act==12) n++;
    if(ant==12 && act==4) n++;
}
