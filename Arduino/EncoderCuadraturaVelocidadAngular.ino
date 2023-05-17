/////////////////////////// COMUNICACION SERIAL //////////////////
String inputString = "";         
bool stringComplete = false;
const char separator = ',';
const int dataLength = 1;
double data[dataLength]; // Valor regula ciclo de trabajo (PWM)

//////////////////////////// ENCONDER ///////////////////
const byte    C1 = 3; // Entrada de la señal A del encoder (Cable amarillo).
const byte    C2 = 2; // Entrada de la señal B del encoder (Cable verde).

////////////////////////// PUENTE H //////////////////////
const byte    in1 = 7;                  
const byte    in2 = 8;                  
const byte    ena = 6;                

volatile int  n = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

///////////////////// Variables Motor ////////////////

double w = 0.0;  // Velocidad angular en rad/s.
int cv = 0; // Valor regula ciclo de trabajo (PWM)

double constValue = 3.1733; //(1000*2*pi)/R ---> R = 1980 Resolucion encoder cuadruple

unsigned long lastTime = 0, sampleTime = 100;  // Tiempo de muestreo

void setup()
{
  /////////////////// CONFIGURACION PUERTO SERIAL ////////////////
  Serial.begin(9600);
  /////////////////// CONFIGURACION DE PINES ////////////////
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  pinMode(in1, OUTPUT);       
  pinMode(in2, OUTPUT);   

  //////////////////// MOTOR APAGADO //////////////////////
  digitalWrite(in1, false);       
  digitalWrite(in2, false);   

  analogWrite(ena,cv);

  ////////////////////////// INTERRUPCIONES /////////////////////
  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);
  
  lastTime = millis();
}

void loop() {
  
  ////////// SI RECIBE DATOS /////////////
  if (stringComplete) 
  {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toFloat();
      inputString = inputString.substring(index + 1);
     }
     
    cv=data[0];
    if (cv > 0) anticlockwise(in2,in1,ena,cv); else clockwise(in2,in1,ena,abs(cv));     

    inputString = "";
    stringComplete = false;
  }

  if (millis() - lastTime >= sampleTime)
  {   
      computeW(); 
      Serial.print("w: ");Serial.println(w);
      
   } 

}

void computeW(void)
{
  w =(constValue*n)/(millis()-lastTime); // Calculamos velocidad rad/s
  lastTime = millis(); // Almacenamos el tiempo actual.
  n = 0;  // Reiniciamos los pulsos.
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
 
// Encoder precisión cuádruple.
void encoder(void)
{

    ant=act;                           
    act=PIND & 12;         
                           
    if(ant==0  && act== 4)  n--;
    if(ant==4  && act==12)  n--;
    if(ant==8  && act== 0)  n--;
    if(ant==12 && act== 8)  n--;
    
    if(ant==0 && act==8)  n++; 
    if(ant==4 && act==0)  n++;
    if(ant==8 && act==12) n++;
    if(ant==12 && act==4) n++;
    
}

void clockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin,pwm);
}

void anticlockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, HIGH);  
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin,pwm);
}
