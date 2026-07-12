
// Include Libraries
#include "Arduino.h"
#include "ssd1306.h"
#include "HX711.h"
#include <EEPROM.h>

#include "RotaryEncoder.h"
#include "TimerOne.h"


// Pin Definitions
#define SCALE_PIN_DAT	9//2
#define SCALE_PIN_CLK	8//3

#define ENCODERA 11
#define ENCODERB 10
#define ENC_SWITCH 12
#define RELAIS 7//13

#define FLUSH_BUTTON A1//


// Global variables and defines
// Constants
#define MAX_RUNTIME 90
#define MAX_SHOTTIME 90
#define SCALE_CALIB_FAC 2.44 * 1.012
#define calibration_factor 2280 //This value is obtained using the SparkFun_HX711_Calibration sketch https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide?_ga=2.77038550.2126325781.1526891300-303225217.1493631967
#define SCALE_CHECK_FREQ_BASE 500
#define SCALE_CHECK_FREQ_RUNNING 30
#define ENC_CHECK_FREQ 20
#define DISP_UPDATE_FREQ_BASE 1000
#define DISP_UPDATE_FREQ_RUNNING 150
#define MASS_ESTIMATION_WINDOW 3
#define FLUSH_LONGPRESS_TIME 500 // ms - short tap (fixed flush) vs. long hold (manual valve control)
#define FLUSH_DURATION_MS 3000 // ms - fixed flush duration for a short tap
#define ENC_LONGPRESS_TIME 600 // ms - hold time on the encoder button to enter the pre-infusion menu


// object initialization

HX711 scale(SCALE_PIN_DAT, SCALE_PIN_CLK);

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(ENCODERB, ENCODERA);


// define vars for testing menu
int address_mass = 0;
int target_mass = 0;
//int target_init = 0;
int address_preinfusion = address_mass + sizeof(address_mass);
uint8_t preinfusion_waiting_time = 3; // Preinfusion waiting time in seconds
int address_preinfusion_valve = address_preinfusion + sizeof(preinfusion_waiting_time);
uint8_t preinfusion_valve_deciseconds = 20; // Preinfusion valve-open (pressure) time, in 0.1s units; default 2.0s
int16_t enc_value = 0;
int16_t enc_last_value = 0;
int disp_update_freq = DISP_UPDATE_FREQ_BASE;
int scale_check_freq = SCALE_CHECK_FREQ_BASE;

bool is_running = false;
bool switch_happened = false;
bool count_switch = true;

bool flush_button_held = false;
bool flush_valve_open = false;
bool flush_pulse_active = false;
unsigned long flush_pulse_end = 0;

bool enc_button_held = false;
unsigned long enc_press_start = 0;
bool enc_long_press_fired = false;
bool in_preinfusion_menu = false;
int preinfusion_menu_field = 0; // 0 = valve-open time, 1 = wait time

int rotary_pos = 0;

float curr_runtime = 0;
float curr_shottime = 0;
float curr_mass = 0;
float mass_array[MASS_ESTIMATION_WINDOW];
int mass_step = 0;


unsigned long time_run_start = 0;
unsigned long time_shot_start = 0;
unsigned long time_button_press = 0;


unsigned long last_disp_update = 0;
unsigned long last_scale_check = 0;
unsigned long last_enc_check = 0;

void check_scale(bool reset=false);

// Setup the essentials for your circuit to work. It runs first every time your circuit is powered with electricity.
void setup() 
{
    Serial.begin(9600);
    
    // initialize the lcd
    ssd1306_setFixedFont(ssd1306xled_font8x16);
    ssd1306_128x64_i2c_init();
    ssd1306_clearScreen();
    scale.set_scale(calibration_factor / SCALE_CALIB_FAC); 
    tare_scale(); //Assuming there is no weight on the scale at start up, reset the scale to 0
    check_scale(true);
    target_mass = eepromReadInt(address_mass);
//    target_mass = target_init;
    preinfusion_waiting_time = EEPROM.read(address_preinfusion);
    preinfusion_valve_deciseconds = EEPROM.read(address_preinfusion_valve);


    pinMode(ENC_SWITCH, INPUT);
    pinMode(RELAIS, OUTPUT);
    pinMode(FLUSH_BUTTON, INPUT);
}

// Main logic of your circuit. It defines the interaction between the components you selected. After setup, it runs over and over again, in an eternal loop.
void loop() 
{
    check_encoder();
    check_flush_button();
    check_scale();    
    control_loop();
    update_display(false);    
}


///////////////////
///////////////////
void control_loop(){
  
  if(is_running==true){      
      if(switch_happened==true){
        disp_update_freq = DISP_UPDATE_FREQ_RUNNING;
        scale_check_freq = SCALE_CHECK_FREQ_RUNNING;
        curr_runtime = 0.0;
        delay(100);
        tare_scale(); // tare while the valve is still closed, so its blocking HX711 reads don't eat into the preinfusion timing
        digitalWrite(RELAIS, HIGH);
        time_run_start = millis();
        time_shot_start = millis();

        switch_happened = false;
      }

      //PREINFUSION LOGIC
      if (preinfusion_waiting_time >0)
      {
        unsigned long preinfusion_valve_ms = (unsigned long)preinfusion_valve_deciseconds * 100;
        if (millis() - time_run_start < preinfusion_valve_ms) digitalWrite(RELAIS, HIGH);  //Keep flowing for pressure time
        else if (millis() - time_run_start < preinfusion_valve_ms + preinfusion_waiting_time * 1000UL) digitalWrite(RELAIS, LOW); //Pause for waiting time
        else digitalWrite(RELAIS, HIGH);
      }

        
      if(curr_mass < 1.5){
        time_shot_start = millis();
      }
      curr_shottime = float( (millis() - time_shot_start)/100)/10.0; //Divide by 1000, trying for numerical stability
      curr_runtime = float( (millis() - time_run_start)/100)/10.0; //Divide by 1000, trying for numerical stability
      
    }//is_running==true
   if (is_running == true && (curr_mass >= target_mass || curr_shottime >= MAX_SHOTTIME || curr_runtime >= MAX_RUNTIME) ){
    is_running = false; 
    switch_happened = true;
   }
   if (is_running == false && switch_happened==true){    
      switch_happened = false;      
      disp_update_freq = DISP_UPDATE_FREQ_BASE;
      scale_check_freq = SCALE_CHECK_FREQ_BASE;
      digitalWrite(RELAIS, LOW);
      delay(100);
      update_display(true);
//      curr_runtime = 0.0;  
//      delay(5000);          
   } 
  
}

/////////////////////////
/////////////////////////
// Short tap: fixed FLUSH_DURATION_MS flush pulse.
// Long hold: valve stays open exactly as long as the button is held.
// Ignored entirely while a shot is running, so it can't fight the
// shot/preinfusion relay logic in control_loop().
void check_flush_button(){

  if(is_running){
    flush_button_held = false;
    flush_pulse_active = false;
    if(flush_valve_open){
      digitalWrite(RELAIS, LOW);
      flush_valve_open = false;
      update_display(true);
    }
    return;
  }

  bool pressed = (digitalRead(FLUSH_BUTTON) == HIGH);
  if(pressed){
    delay(5); // Check for ghost readings
    pressed = (digitalRead(FLUSH_BUTTON) == HIGH);
  }

  if(pressed){
    if(flush_button_held == false){
      flush_button_held = true;
      time_button_press = millis();
    }
    if(!flush_valve_open && millis() - time_button_press >= FLUSH_LONGPRESS_TIME){
      digitalWrite(RELAIS, HIGH); // long press: valve mirrors the button
      flush_valve_open = true;
      update_display(true);
    }
  } else if(flush_button_held){ // just released
    flush_button_held = false;
    unsigned long time_pressed = millis() - time_button_press;
    if(time_pressed < FLUSH_LONGPRESS_TIME){
      flush_pulse_active = true; // short press: start a fixed, non-blocking flush pulse
      flush_pulse_end = millis() + FLUSH_DURATION_MS;
      digitalWrite(RELAIS, HIGH);
      flush_valve_open = true;
      update_display(true);
    } else {
      digitalWrite(RELAIS, LOW); // long press released: close the valve
      flush_valve_open = false;
      update_display(true);
    }
  }

  if(flush_pulse_active && millis() >= flush_pulse_end){
    digitalWrite(RELAIS, LOW);
    flush_valve_open = false;
    flush_pulse_active = false;
    update_display(true);
  }
}

/////////////////////////
/////////////////////////
void check_scale(bool reset){
  if (millis() - last_scale_check > scale_check_freq || reset==true){
    last_scale_check = millis();
    float tmp_mass = -scale.get_units(); //scale.get_units() returns a float
    if(reset==true){
        for(int i=0; i<MASS_ESTIMATION_WINDOW;++i){
          mass_array[i] = tmp_mass;
        }
      }
     mass_array[mass_step] = tmp_mass;
     mass_step = (mass_step+1)%MASS_ESTIMATION_WINDOW;
     curr_mass = 0.0;
     for(int i=0; i<MASS_ESTIMATION_WINDOW;++i){
        curr_mass += mass_array[i] / float(MASS_ESTIMATION_WINDOW);
     }//for
 
    }//if
}

/////////////////////////
/////////////////////////
void update_display(bool force_update){
  static String message;
  if ( (millis() - last_disp_update) < disp_update_freq && force_update==false)
    return;

  last_disp_update = millis();

  if(flush_valve_open){
    message = "FLUSHING       ";
    ssd1306_printFixed(0,  4, message.c_str(), STYLE_BOLD);
    message = "               ";
  } else if(in_preinfusion_menu){
    if(preinfusion_menu_field == 0){
      message = "PREINF VALVE   ";
      ssd1306_printFixed(0,  4, message.c_str(), STYLE_BOLD);
      message = String( String(preinfusion_valve_deciseconds/10.0) + "s      " );
    } else {
      message = "PREINF WAIT    ";
      ssd1306_printFixed(0,  4, message.c_str(), STYLE_BOLD);
      message = String( String(preinfusion_waiting_time) + "s      " );
    }
  } else {
    message = String(
              String(curr_mass) +
              "/" +
              String(target_mass)
              + "g     " );
    ssd1306_printFixed(0,  4, message.c_str(), STYLE_BOLD);

    message = String(
              String(preinfusion_waiting_time) +
              "|" +
              String(curr_shottime) +
              "|" +
              String(curr_runtime) +
  //            ")/" +
  //            String(MAX_SHOTTIME)
              + "s     " );
  }
  ssd1306_printFixed(0,  32, message.c_str(), STYLE_BOLD);


}

/////////////////////////
/////////////////////////
// Encoder push-button (outside the menu): short click starts/stops the
// shot; holding past ENC_LONGPRESS_TIME enters the pre-infusion menu
// (only while idle, i.e. not mid-shot).
// Encoder push-button (inside the menu): short click cycles between the
// two editable fields (valve-open time, wait time); a long click exits
// the menu.
// Rotary knob: adjusts target mass, or whichever menu field is selected
// while the menu is open.
void check_encoder() {
  encoder.tick();

  bool enc_pressed = (digitalRead(ENC_SWITCH) == LOW); // active-low: idle HIGH, pressed pulls LOW
  if(enc_pressed){
    delay(5); // Check for ghost readings
    enc_pressed = (digitalRead(ENC_SWITCH) == LOW);
  }

  if(enc_pressed){
    if(enc_button_held == false){
      enc_button_held = true;
      enc_press_start = millis();
      enc_long_press_fired = false;
    }
    if(enc_long_press_fired == false && is_running == false
       && millis() - enc_press_start >= ENC_LONGPRESS_TIME){
      enc_long_press_fired = true;
      if(in_preinfusion_menu){
        in_preinfusion_menu = false; // long click exits the menu
        ssd1306_positiveMode();
      } else {
        in_preinfusion_menu = true; // long click enters the menu
        preinfusion_menu_field = 0;
        ssd1306_negativeMode(); // inverted display marks the pre-infusion menu
      }
      update_display(true);
    }
  } else if(enc_button_held){ // just released
    enc_button_held = false;
    if(enc_long_press_fired){
      enc_long_press_fired = false; // menu enter/exit already happened during this hold
    } else if(is_running){
      is_running = false;
      switch_happened = true;
    } else if(in_preinfusion_menu){
      preinfusion_menu_field = (preinfusion_menu_field + 1) % 2; // short click switches field
      update_display(true);
    } else {
      is_running = true;
      switch_happened = true;
    }
  }

  if ( (millis() - last_enc_check) < ENC_CHECK_FREQ)
    return;
  last_enc_check = millis();

  int newPos = encoder.getPosition();
  if (rotary_pos != newPos) {
    rotary_pos = newPos;
    int step = (int(encoder.getDirection()) < 0) ? 1 : -1;
    if(in_preinfusion_menu){
      if(preinfusion_menu_field == 0){ // valve-open time
        preinfusion_valve_deciseconds += step;
        EEPROM.write(address_preinfusion_valve, preinfusion_valve_deciseconds);
      } else { // wait time
        preinfusion_waiting_time += step;
        EEPROM.write(address_preinfusion, preinfusion_waiting_time);
      }
    } else {
      target_mass += step;
      eepromWriteInt(address_mass, target_mass);
    }
    update_display(true);
  }
}

/////////////////////////
/////////////////////////
void tare_scale(){
  static String message;
  ssd1306_clearScreen( );
  ssd1306_printFixed(0, 4, "Taring Scale", STYLE_BOLD);
  
  scale.tare(5);
  delay(100);
  scale.tare(15);
  check_scale(true);
  ssd1306_printFixed(0, 32, String(curr_mass).c_str(), STYLE_BOLD);
  
  ssd1306_clearScreen( );
}

/////////////////////////
/////////////////////////
void eepromWriteInt(int adr, int value) {
  byte low, high;
  low = value & 0xFF;
  high = (value >> 8) & 0xFF;
  EEPROM.write(adr, low); // dauert 3,3ms
  EEPROM.write(adr + 1, high);
  return;
} //eepromWriteInt

/////////////////////////
/////////////////////////
int eepromReadInt(int adr) {
  byte low, high;
  low = EEPROM.read(adr);
  high = EEPROM.read(adr + 1);
  return low + ((high << 8) & 0xFF00);
} //eepromReadInt
