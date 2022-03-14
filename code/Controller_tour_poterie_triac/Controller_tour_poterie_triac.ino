#include "TimerOne.h"
#include <FastPID.h>

float Kp=1.2, Ki=0.6, Kd=0.6, Hz=100;
int output_bits = 16;
bool output_signed = false;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

const uint16_t period_LUT[512] = {109, 369, 479, 563, 634, 697, 754, 806, 855, 901, 944, 985, 1025, 1063, 1099, 1134, 1169, 1202, 1234, 1265, 1296, 1326, 1355, 1383, 1411, 1439, 1466, 1492, 1518, 1544, 1569, 1593, 1618, 1642, 1665, 1689, 1712, 1734, 1757, 1779, 1801, 1823, 1844, 1865, 1886, 1907, 1928, 1948, 1968, 1988, 2008, 2028, 2047, 2066, 2085, 2104, 2123, 2142, 2160, 2179, 2197, 2215, 2233, 2251, 2269, 2286, 2304, 2321, 2339, 2356, 2373, 2390, 2407, 2424, 2440, 2457, 2473, 2490, 2506, 2522, 2538, 2555, 2571, 2586, 2602, 2618, 2634, 2649, 2665, 2680, 2696, 2711, 2726, 2742, 2757, 2772, 2787, 2802, 2817, 2832, 2846, 2861, 2876, 2890, 2905, 2919, 2934, 2948, 2963, 2977, 2991, 3006, 3020, 3034, 3048, 3062, 3076, 3090, 3104, 3118, 3132, 3145, 3159, 3173, 3186, 3200, 3214, 3227, 3241, 3254, 3268, 3281, 3295, 3308, 3321, 3335, 3348, 3361, 3374, 3387, 3401, 3414, 3427, 3440, 3453, 3466, 3479, 3492, 3505, 3518, 3530, 3543, 3556, 3569, 3582, 3594, 3607, 3620, 3633, 3645, 3658, 3670, 3683, 3696, 3708, 3721, 3733, 3746, 3758, 3771, 3783, 3796, 3808, 3820, 3833, 3845, 3857, 3870, 3882, 3894, 3907, 3919, 3931, 3943, 3955, 3968, 3980, 3992, 4004, 4016, 4028, 4040, 4053, 4065, 4077, 4089, 4101, 4113, 4125, 4137, 4149, 4161, 4173, 4185, 4197, 4209, 4221, 4233, 4245, 4256, 4268, 4280, 4292, 4304, 4316, 4328, 4340, 4352, 4363, 4375, 4387, 4399, 4411, 4422, 4434, 4446, 4458, 4470, 4481, 4493, 4505, 4517, 4529, 4540, 4552, 4564, 4576, 4587, 4599, 4611, 4622, 4634, 4646, 4658, 4669, 4681, 4693, 4705, 4716, 4728, 4740, 4751, 4763, 4775, 4787, 4798, 4810, 4822, 4833, 4845, 4857, 4869, 4880, 4892, 4904, 4915, 4927, 4939, 4951, 4962, 4974, 4986, 4997, 5009, 5021, 5033, 5044, 5056, 5068, 5080, 5091, 5103, 5115, 5127, 5139, 5150, 5162, 5174, 5186, 5198, 5209, 5221, 5233, 5245, 5257, 5269, 5280, 5292, 5304, 5316, 5328, 5340, 5352, 5364, 5376, 5387, 5399, 5411, 5423, 5435, 5447, 5459, 5471, 5483, 5495, 5507, 5519, 5531, 5544, 5556, 5568, 5580, 5592, 5604, 5616, 5628, 5641, 5653, 5665, 5677, 5689, 5702, 5714, 5726, 5739, 5751, 5763, 5776, 5788, 5800, 5813, 5825, 5838, 5850, 5862, 5875, 5887, 5900, 5913, 5925, 5938, 5950, 5963, 5976, 5988, 6001, 6014, 6026, 6039, 6052, 6065, 6078, 6091, 6103, 6116, 6129, 6142, 6155, 6168, 6181, 6194, 6208, 6221, 6234, 6247, 6260, 6274, 6287, 6300, 6314, 6327, 6340, 6354, 6367, 6381, 6394, 6408, 6422, 6435, 6449, 6463, 6477, 6490, 6504, 6518, 6532, 6546, 6560, 6574, 6589, 6603, 6617, 6631, 6645, 6660, 6674, 6689, 6703, 6718, 6732, 6747, 6762, 6777, 6791, 6806, 6821, 6836, 6851, 6867, 6882, 6897, 6912, 6928, 6943, 6959, 6974, 6990, 7006, 7022, 7038, 7054, 7070, 7086, 7102, 7118, 7135, 7151, 7168, 7185, 7201, 7218, 7235, 7252, 7270, 7287, 7304, 7322, 7339, 7357, 7375, 7393, 7411, 7429, 7448, 7466, 7485, 7504, 7523, 7542, 7561, 7581, 7600, 7620, 7640, 7660, 7681, 7701, 7722, 7743, 7764, 7785, 7807, 7829, 7851, 7874, 7896, 7919, 7943, 7966, 7990, 8015, 8040, 8065, 8090, 8116, 8143, 8169, 8197, 8225, 8253, 8282, 8312, 8343, 8374, 8406, 8440, 8474, 8509, 8546, 8583, 8623, 8664, 8708, 8753, 8802, 8854, 8911, 8974, 9045, 9129, 9239, 9500};

bool DEBUG = 1;

const int analog_input = A2;
const int control_pin = 9;
const int digital_input = 2;
const int speed_input = 3;

const int updaterate = 5;

class IRPulser{
public :
//There is 24 slots in the wheel.
//One slot spans 15°
// 800 µs for 15° = 0.02 sec per 360° wich means 50 turns per sec  (at the motor side) wich roughly corresponds to the 3600 turns per min displayed on the motor, and also to the main ac frequency.
//SPEED : 25000 seems a good low speed value, 800 is the fastest i've seen. 

  unsigned long max_delta = 25000; //min speed
  unsigned long min_delta = 800; //max speed

  unsigned long last_pulse = 0;
  unsigned long current_pulse = max_delta;

  void pulse_handler(){
    last_pulse = current_pulse;
    current_pulse = micros();
  }

  unsigned long get_value(){
    noInterrupts();
    if (micros() - last_pulse > max_delta){
      interrupts();
      return max_delta;
    }
    unsigned long current_delta = current_pulse - last_pulse;
    interrupts();
    return constrain(current_delta, min_delta,max_delta);
  }
};

class Gachette{
public :
  bool trigg_available = false;
  bool output_state = true;
  unsigned long trigger_start_memo = 0;
  bool enabled = false;

  
  int value = 1000;

  void set_value(int _value){
    value = constrain(_value,0,1000);
  }

  int get_value(){
    return value;
  }

  void ac_sync_handler(){
    if (enabled){
      trigg_available = false;
      Timer1.setPeriod(value);
      Timer1.start();
    }
  }
  
//    output = myPID.step(value, 200 );
//    output = constrain(output,50,1024);
  
  void trigger(){
    if (trigg_available){
      trigger_start_memo = micros();
      output_state = 1;
      digitalWrite(control_pin, HIGH);
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
      digitalWrite(control_pin, LOW);
      output_state = false;
      Timer1.stop();
    }
  }
};

Gachette Gachette_object;
IRPulser IRPulser_object;

void gachette_trigger_handler(){
  Gachette_object.trigger();
}

void speed_pulse_handler(){
  IRPulser_object.pulse_handler();
}

void gachette_timer_handler(){
  Gachette_object.ac_sync_handler();
}

void setup() {
  Serial.begin(500000);
  delay(10);
//  Serial.print("Speed(µs/rev)");
//  Serial.print("\t");
//  Serial.print("Potentiometer");
//  Serial.print("\t");
//  Serial.println("OffTime(µs)");

  pinMode(analog_input, INPUT);
  pinMode(speed_input, INPUT);
  pinMode(control_pin, OUTPUT);
  pinMode(digital_input, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(digital_input), gachette_timer_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(speed_input), speed_pulse_handler, FALLING);
  
  Timer1.initialize(500);
  Timer1.stop();
  Timer1.attachInterrupt(gachette_trigger_handler);
}

void loop() {
  static unsigned long memo_print = 0;
  static unsigned long memo_analog_check = 0;
  static int localvalue = 0;
    
  if ( millis() - memo_analog_check > 100){
    int temp_localvalue = analogRead(analog_input);
    if (abs(temp_localvalue - localvalue) > 30) {
      localvalue = temp_localvalue;
    }
    memo_analog_check = millis();
  }  

  if (localvalue < 200){
    Gachette_object.disable();
  }
  else {
    Gachette_object.enable();
  }

  Gachette_object.release_trigger();

  if (millis() - memo_print > updaterate ){
    Serial.print(IRPulser_object.get_value());
    //Serial.print("\t");
    //Serial.println(Gachette_object.get_value());
    Serial.print("\t");
    Serial.println(localvalue);
    memo_print = millis();
  }
}
