#include "TimerOne.h"
#include <FastPID.h>

float Kp=1.2, Ki=0.6, Kd=0.6, Hz=100;
int output_bits = 16;
bool output_signed = false;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

//const int offset_LUT[256] = {0, 24, 49, 74, 99, 124, 149, 174, 199, 223, 248, 273, 298, 323, 348, 373, 398, 424, 449, 474, 499, 524, 549, 575, 600, 625, 651, 676, 701, 727, 753, 778, 804, 830, 855, 881, 907, 933, 959, 985, 1011, 1037, 1064, 1090, 1116, 1143, 1170, 1196, 1223, 1250, 1277, 1304, 1331, 1358, 1386, 1413, 1441, 1469, 1496, 1524, 1552, 1581, 1609, 1638, 1666, 1695, 1724, 1753, 1782, 1812, 1841, 1871, 1901, 1931, 1962, 1992, 2023, 2054, 2085, 2117, 2149, 2180, 2213, 2245, 2278, 2311, 2345, 2378, 2412, 2447, 2482, 2517, 2552, 2588, 2625, 2662, 2699, 2737, 2775, 2814, 2854, 2894, 2935, 2976, 3018, 3062, 3105, 3150, 3196, 3243, 3291, 3340, 3391, 3443, 3497, 3552, 3610, 3670, 3733, 3799, 3868, 3942, 4021, 4107, 4202, 4309, 4436, 4601, 5000, 5398, 5563, 5690, 5797, 5892, 5978, 6057, 6131, 6200, 6266, 6329, 6389, 6447, 6502, 6556, 6608, 6659, 6708, 6756, 6803, 6849, 6894, 6937, 6981, 7023, 7064, 7105, 7145, 7185, 7224, 7262, 7300, 7337, 7374, 7411, 7447, 7482, 7517, 7552, 7587, 7621, 7654, 7688, 7721, 7754, 7786, 7819, 7850, 7882, 7914, 7945, 7976, 8007, 8037, 8068, 8098, 8128, 8158, 8187, 8217, 8246, 8275, 8304, 8333, 8361, 8390, 8418, 8447, 8475, 8503, 8530, 8558, 8586, 8613, 8641, 8668, 8695, 8722, 8749, 8776, 8803, 8829, 8856, 8883, 8909, 8935, 8962, 8988, 9014, 9040, 9066, 9092, 9118, 9144, 9169, 9195, 9221, 9246, 9272, 9298, 9323, 9348, 9374, 9399, 9424, 9450, 9475, 9500, 9525, 9550, 9575, 9601, 9626, 9651, 9676, 9701, 9726, 9751, 9776, 9800, 9825, 9850, 9875, 9900, 9925, 9950, 9975};

const int updaterate = 5;

class IRPulser{
public :
//There is 24 slots in the wheel.
//One slot spans 15°
// 800 µs for 15° = 0.02 sec per 360° wich means 50 turns per sec  (at the motor side) wich roughly corresponds to the 3600 turns per min displayed on the motor, and also to the main ac frequency.
//SPEED : 25000 seems a good low speed value, 800 is the fastest i've seen. 

  unsigned int pulses_per_turn = 24;
  float motor_ratio = 0.064;
  
  unsigned long max_delta = 25000; //min speed in microseconds
  unsigned long min_delta = 800; //max speed in microseconds

  int _input_pin = 3;// Input pin for the IR pulser

protected :

  unsigned long rpm_conversion_constant = 60000000; // number of microseconds in a minute
  unsigned long _last_pulse = 0;
  unsigned long _current_pulse = max_delta;

  unsigned long final_conversion_ratio = (unsigned long) rpm_conversion_constant / (unsigned long) (pulses_per_turn  * (float)(1 / motor_ratio));

public :

  IRPulser(int input_pin){
    _input_pin = input_pin;
    pinMode(_input_pin, INPUT);
  }

  void pulse_handler(){
    _last_pulse = _current_pulse;
    _current_pulse = micros();
  }

  unsigned long current_delta(){
    noInterrupts();
    if (micros() - _last_pulse > max_delta){
      interrupts();
      return max_delta;
    }
    unsigned long temp_delta = _current_pulse - _last_pulse;
    interrupts();
    return constrain(temp_delta, min_delta,max_delta);
  }

  unsigned int current_rpm(){
    return final_conversion_ratio / current_delta() ;
  }
};

class Gachette{
public :
  bool trigg_available = false;
  bool output_state = true;
  unsigned long trigger_start_memo = 0;
  bool enabled = false;

  int _input_pin = 2;//Pin to measure the crossing point of the 0V of the main AC line
  int _output_pin = 9;//Pïn to control the gachette of the triac

protected :

  unsigned long ac_cross_memo = 0;
  bool ac_cross_flag = false;

public :
  int value = 1000;

  Gachette(int input_pin, int output_pin){
    _input_pin = input_pin;
    _output_pin = output_pin;
    pinMode(_output_pin, OUTPUT);
    pinMode(_input_pin, INPUT_PULLUP);
  }

  void set_value(int _value){
    value = constrain(_value,0,1000);
  }

  int get_value(){
    return value;
  }

  void ac_sync_handler(){
    ac_cross_memo = micros();
    ac_cross_flag = true;
    if (enabled){
      trigg_available = false;
      Timer1.setPeriod(value);
      Timer1.start();
    }
  }

  bool ac_cross_due(){
    return ac_cross_flag;
  }

  unsigned long get_ac_cross(){
    ac_cross_flag = false;
    return ac_cross_memo;
  }
  
//    output = myPID.step(value, 200 );
//    output = constrain(output,50,1024);

  
  void trigger(){
    if (trigg_available){
      trigger_start_memo = micros();
      output_state = true;
      digitalWrite(_output_pin, output_state);
    }
    else {
      trigg_available = true;
    }
  }

  void disable(){
    if (enabled){
      enabled = false;
    }
  }

  void enable(){
    if (!enabled){
      enabled = true;
    }
  }
   
   void release_trigger(){
    if (output_state && micros() - trigger_start_memo > 50){
      output_state = false;
      digitalWrite(_output_pin, output_state);
      Timer1.stop();
    }
  }
};

class SpeedPotentiometer{
  
public :

  unsigned int value;
  unsigned int read_interval = 50;//min interval between reads, else get value returns last stored value
  unsigned int min_value = 800;
  unsigned int max_value = 25000;
  unsigned int multiplier = 27; //(25000-800 / 1024)
  unsigned int offset = -1850; 
  int _input_pin = A2;//Potentiometer for speed control 

protected :
  unsigned int pulses_per_turn = 24;
  float motor_ratio = 0.064;
  unsigned long _read_memo = 0;
  unsigned long rpm_conversion_constant = 60000000; // number of microseconds in a minute

  unsigned long final_conversion_ratio = (unsigned long) rpm_conversion_constant / (unsigned long) (pulses_per_turn  * (float)(1 / motor_ratio));

public :

  SpeedPotentiometer(int input_pin){
    _input_pin = input_pin;
    pinMode(_input_pin, INPUT);
    Serial.print("final_conversion_ratio : ");Serial.println(final_conversion_ratio);
  }

  unsigned int read_value(){
    return (unsigned int) analogRead(_input_pin);
  }

  unsigned int get_value(){
    if (millis() - _read_memo > read_interval) {
      unsigned int temp_value = read_value();
      temp_value = (temp_value * multiplier) + offset;
      value = constrain(temp_value, min_value, max_value);//outputs values in the same range as speed deltas readable by IR pulser
      _read_memo = millis();
    }
    return value;
  }

  unsigned int current_rpm(){
    return final_conversion_ratio / get_value();
  }
};

class MemoTimer{

public :

  unsigned long interval = 50;
  
protected :
  bool state = 1;
  unsigned long _time_memo = 0;

public :

  MemoTimer(unsigned long _interval){
    interval = _interval;
    start();
  }
  
  bool is_due(){
    if (state && millis() - _time_memo > interval) {
      _time_memo = millis();
      return true;
    }
    return false;
  }

  void start(){
    _time_memo = millis();
    state = 1;
  }

  void stop(){
    state = 0;
  }
};

SpeedPotentiometer Potentiometer_object(A2);
Gachette Gachette_object(2,9);
IRPulser IRPulser_object(3);

void gachette_trigger_handler(){
  Gachette_object.trigger();
}

void speed_pulse_handler(){
  IRPulser_object.pulse_handler();
}

void gachette_timer_handler(){
  Gachette_object.ac_sync_handler();
}

void print_stuff(){
  Serial.print(IRPulser_object.current_rpm());Serial.print("\t");
  Serial.print(Potentiometer_object.current_rpm());Serial.print("\t");
  //Serial.print(Gachette_object.get_value());Serial.print("\t");
  if (Gachette_object.ac_cross_due()){
    Serial.print(Gachette_object.get_ac_cross());
  }
  Serial.print("\n");
}

void setup() {  
  Serial.begin(500000);
  delay(10);
  Serial.print("Pulseur(rpm)");Serial.print("\t");
  Serial.print("Potard(rpm)");Serial.print("\t");
  Serial.println("AC_line(µs)");
//  Serial.print("\t");
//  Serial.println("OffTime(µs)");

  attachInterrupt(digitalPinToInterrupt(Gachette_object._input_pin), gachette_timer_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IRPulser_object._input_pin), speed_pulse_handler, FALLING);
  
  Timer1.initialize(500);
  Timer1.stop();
  Timer1.attachInterrupt(gachette_trigger_handler);
  Gachette_object.disable();
}

void loop() {

  static MemoTimer print_timer(10);   
  if ( print_timer.is_due() ){
    print_stuff();
  }  

//  if (localvalue < 200){
//    Gachette_object.disable();
//  }
//  else {
//    Gachette_object.enable();
//  }

  Gachette_object.release_trigger();

}
