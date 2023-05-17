#include <Servo.h>

#define ENCODER_A 2 // Entrada de la señal A del encoder.
#define ENCODER_B 3 // Entrada de la señal B del encoder.

#define PI 3.1415926535897932384626433832795

// MOTOR 500 de resolucion
float resolution = 500*4;
float resEnc = 11;

// Encoder
volatile float w = 0;
float vueltas = 0;
float rp;
float pwm;
volatile int pulsosV = 0;

long tActual;
long tPasado=0;

int salidaPWM;
float volt;
int vel = 70;

int Topi = 8; // Entrada al clk del atmega.

void setup() {
  Serial.begin(9600);
  
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  
  //INTERRUPCIONES
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),leerEncoderA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B),leerEncoderB,CHANGE);

  // CODIGO TURBIO->INTERRUPCIÓN POR TIMER
  // Escribe directamente en los registros
  cli();//stop interrupts
  
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 183;// = (16*10^6) / (85*1024) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 1024 prescaler
  TCCR0B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  
  sei();//allow interrupts
}

void loop() {
   Serial.println(w);
}

// Interrupción señal A.
void leerEncoderA(){
    if(digitalRead(ENCODER_B)^digitalRead(ENCODER_A)){
      //Incremento variable global
      pulsosV++;
    }
    else{
      //Decremento variable global
      pulsosV--;
    }
    
  }
  
  // Interrupción señal B.
  void leerEncoderB(){
    if(digitalRead(ENCODER_B)^digitalRead(ENCODER_A)){
      //Incremento variable global
      pulsosV--;

    }
    else{
      //Decremento variable global
      pulsosV++;
    }
  }

  // Interrupción timer.
  ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
  //generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave - toggle high then toggle low)
  digitalWrite(Topi, !digitalRead(Topi));
  w=((pulsosV/resolution)*2*PI)*1/(0.01); // [rad/sec]
  w=pulsosV;
  //   Serial.print("velocidad angular: ");
  //  Serial.println(w);
  pulsosV=0;
}
