#include "TimerOne.h"

const int analog_input = A2;
const int control_pin = 9;
const int digital_input = 2;
const int checkLENpin = 3;


bool laststate = 0;
bool pulseflag = 0;

bool out_is_ON = 0;

unsigned long output_start_memory = 0 ;

long testval = 0;

int STOPRINT = 0;
unsigned long timelog = 0;


unsigned long durationmemo_start = 0;
unsigned long durationmemo_stop = 0;

bool printflag = 0;
bool printflag2 = 0;
bool printflag3 = 0;

bool rejectinitial = 0;

bool reject_timerfirst = 0;

unsigned long delay_memory = 0;

void PulseDetector_digital(){
  //if (STOPRINT == 0){
  if (rejectinitial){
    pulseflag = 1;
  }
  else {
    rejectinitial = 1;
  }
  //}
  
}

void CHeckPulseLen(){
  if (digitalRead(checkLENpin) == HIGH){
    durationmemo_start = micros();
    printflag = 1;
  }
  else {
    durationmemo_stop = micros();
    printflag2 = 1;
  }
}

void setup() {
  Serial.begin(500000);

  Serial.print("Timelog");
  Serial.print("\t");
  Serial.print("AnalogIN");

  // put your setup code here, to run once:
  pinMode(analog_input, INPUT);
  pinMode(control_pin, OUTPUT);
  pinMode(digital_input, INPUT_PULLUP);
  pinMode(checkLENpin, INPUT);
  attachInterrupt(digitalPinToInterrupt(digital_input), PulseDetector_digital, RISING);

  attachInterrupt(digitalPinToInterrupt(checkLENpin), CHeckPulseLen, CHANGE);

  Timer1.initialize(500);
  Timer1.stop();
  Timer1.attachInterrupt(call_gachette);
}

void call_gachette(){
  if (reject_timerfirst){
    Timer1.stop();
    delay_memory = micros() - delay_memory;
    gachette_on();
    printflag3 = 1;
    
  }
  else{
    reject_timerfirst = 1;
  }
}

void check_gachette_off(){
  if (out_is_ON && micros() - output_start_memory > 50){
    digitalWrite(control_pin, LOW);
    out_is_ON = 0;
    //STOPRINT = 2;
  }
}

void gachette_on(){
  output_start_memory = micros();
  out_is_ON = 1;
  digitalWrite(control_pin, HIGH);
  //STOPRINT = 1;
  //timelog = micros();
}

void loop() {
  check_gachette_off();
  static int checkvalue = 0;
  
  static int value = 0;
  static unsigned long memo_analogcheck = 0;
  if ( millis() - memo_analogcheck > 100){
    value = analogRead(analog_input);
    memo_analogcheck = millis();
  }  
  
  if (pulseflag){
    delay_memory = micros();
    //laststate = 1;
    pulseflag = 0;
    reject_timerfirst = 0;
    Timer1.setPeriod((value*10)-600);
    Timer1.start();
  }
//    if (STOPRINT == 1){
//    Serial.print(micros()-timelog);
//    Serial.print("\t");
//    Serial.print(value);
//    Serial.print("\t");
//    Serial.println(900);
//    }
  
  if (printflag){
    //Serial.print("A \t");
    //Serial.println(durationmemo_start-durationmemo_stop);
    printflag=0;
  }
  if (printflag2){
    Serial.print("\t");
    Serial.println(durationmemo_stop-durationmemo_start);
    printflag2=0;
  }
//  if (printflag || printflag2){
//    if (printflag == 2){
//      savedval = 1;
//    }
//    if 
//    printflag = 0;
//    Serial.print(savedval);
//    Serial.print(": \t");
//    Serial.print(micros()-durationmemo_start);
//    Serial.print("\t");
//    Serial.print(micros()-durationmemo_stop);
//    Serial.print("\t");
//    Serial.print(durationmemo_start-durationmemo_stop);
//    Serial.print("\t");
//    Serial.println(durationmemo_stop-durationmemo_start);
//        
//  }
//  if (digitalRead(checkLENpin)){
//    Serial.println("HIGH");
//  }
  
//  else {
////    if (STOPRINT == 1){
////    Serial.print(micros()-timelog);
////    Serial.print("\t");
////    Serial.print(value);
////    Serial.print("\t");
////    Serial.println(0);
////    }
//  }

  if (printflag3){
    Serial.print(delay_memory);
    printflag3 = 0;
    //Serial.print("\t");
    //Serial.println(testval);
  }

  
//  if (laststate){
//    testval = micros() - memory;
//    if ( testval > value*8){
//      if (rejectinitial){
//        Serial.print(value*8);
//        Serial.print("\t");
//        Serial.println(testval);
//        gachette_on();
//      }
//      else{
//        rejectinitial = 1;
//      }
//  
//      laststate = 0;
//    }
//  }


  
  
  // put your main code here, to run repeatedly:
  
}
