
// Include Libraries
#include "Arduino.h"
#include "ssd1306.h"
#include "HX711.h"
#include "Button.h"
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
#define MAX_RUNTIME 50
#define MAX_SHOTTIME 35
#define SCALE_CALIB_FAC 2.44 * 1.012
#define calibration_factor 2280 //This value is obtained using the SparkFun_HX711_Calibration sketch https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide?_ga=2.77038550.2126325781.1526891300-303225217.1493631967
#define SCALE_CHECK_FREQ_BASE 500
#define SCALE_CHECK_FREQ_RUNNING 30
#define ENC_CHECK_FREQ 20
#define DISP_UPDATE_FREQ_BASE 1000
#define DISP_UPDATE_FREQ_RUNNING 150
#define MASS_ESTIMATION_WINDOW 3

// object initialization

HX711 scale(SCALE_PIN_DAT, SCALE_PIN_CLK);

Button encButton(ENC_SWITCH);

// Setup a RoraryEncoder for pins A2 and A3:
RotaryEncoder encoder(ENCODERB, ENCODERA);


// define vars for testing menu
int address = 0;
int target_mass = 0;
int target_init = 0;
int16_t enc_value = 0;
int16_t enc_last_value = 0;
int disp_update_freq = DISP_UPDATE_FREQ_BASE;
int scale_check_freq = SCALE_CHECK_FREQ_BASE;

bool is_running = false;
bool switch_happened = false;
bool count_switch = true;

int rotary_pos = 0;

float curr_runtime = 0;
float curr_shottime = 0;
float curr_mass = 0;
float mass_array[MASS_ESTIMATION_WINDOW];
int mass_step = 0;


unsigned long time_run_start = 0;
unsigned long time_shot_start = 0;


unsigned long last_disp_update = 0;
unsigned long last_scale_check = 0; 
unsigned long last_enc_check = 0;
unsigned long check_button_next = 0;

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
    target_init = eepromReadInt(address);
    target_mass = target_init;

    pinMode(ENC_SWITCH, INPUT);
    pinMode(RELAIS, OUTPUT);
    pinMode(FLUSH_BUTTON, INPUT);
    
//    pushButton.init();
    encButton.init();
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
        digitalWrite(RELAIS, HIGH);  
        time_run_start = millis();
        time_shot_start = millis();
        delay(50);
        tare_scale();        
        switch_happened = false;        
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
      delay(5000);          
   } 
  
}

/////////////////////////
/////////////////////////
void check_flush_button(){
  
  if(digitalRead(FLUSH_BUTTON) == HIGH){
    delay(5);
    if(digitalRead(FLUSH_BUTTON) == HIGH){
      digitalWrite(RELAIS, HIGH);
      delay(3000);
      digitalWrite(RELAIS, LOW);
      delay(100);
    }
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

  message = String( 
            String(curr_mass) + 
            "/" + 
            String(target_mass) 
            + "g     " );
  ssd1306_printFixed(0,  4, message.c_str(), STYLE_BOLD);

  message = String( 
            String(curr_shottime) + 
            "(" + 
            String(curr_runtime) + 
            ")/" + 
            String(MAX_SHOTTIME) 
            + "s     " );
  ssd1306_printFixed(0,  32, message.c_str(), STYLE_BOLD);  

  
}

/////////////////////////
/////////////////////////
void check_encoder() {  
  encoder.tick();
  if ( (millis() - last_enc_check) < ENC_CHECK_FREQ) 
    return;
  last_enc_check = millis(); 
    
  switch (check_button_next%3){
//    case 0: 
//      if (pushButton.onRelease())
//      {
//        tare_scale();
//      }
//      break; 
    
    case 1: 
//      Serial.print("Check Enc switch: ");
//      Serial.println(digitalRead(ENC_SWITCH));
      if (encButton.onRelease())
      {
        is_running = !is_running;
        switch_happened = true;
      }//encButton   
      break; 

    case 2: 
//      Serial.println("Check rotary: ");
      int newPos = encoder.getPosition();
      if (rotary_pos != newPos) {      
        rotary_pos = newPos;
        if (int(encoder.getDirection())<0)
        {
          target_mass++;
          eepromWriteInt(address, target_mass);
        }else{
          target_mass--;
          eepromWriteInt(address, target_mass);
        }
        update_display(true);
      }
      break;
  }//switch
  check_button_next++;
    
}

/////////////////////////
/////////////////////////
void tare_scale(){
  static String message;
  ssd1306_clearScreen( );
  ssd1306_printFixed(0, 4, "Taring Scale", STYLE_BOLD);
  
  scale.tare(15);
  check_scale(true);
  ssd1306_printFixed(0, 32, String(curr_mass).c_str(), STYLE_BOLD);
//  int taring_counter = 0;
//  while (
//    ((curr_mass>1) || (curr_mass <-1)) 
//     && (taring_counter< 2) ){
//    scale.tare();
//    check_scale(true);
//    taring_counter++;
//    ssd1306_printFixed(64, 32, String(curr_mass).c_str(), STYLE_BOLD);
//  }
  
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
