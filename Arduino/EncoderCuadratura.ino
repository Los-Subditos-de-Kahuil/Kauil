const int    C1 = 3; // Entrada de la señal A del encoder.
const int    C2 = 2; // Entrada de la señal B del encoder.

volatile int  n    = 0;
volatile byte ant  = 0;
volatile byte act  = 0;

unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

void setup()
{
  Serial.begin(9600);

  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  attachInterrupt(digitalPinToInterrupt(C1), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoder, CHANGE);
  
  Serial.println("Numero de conteos");

}

void loop() {
  if (millis() - lastTime >= sampleTime)
  {  // Se actualiza cada sampleTime (milisegundos)
      lastTime = millis();
      Serial.print("Cuentas: ");Serial.println(n);
      n=0;
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
    
    if(ant==0  && act==8)  n++; 
    if(ant==4  && act==0)  n++;
    if(ant==8  && act==12) n++;
    if(ant==12 && act==4) n++;
}