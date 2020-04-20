#include "HID-Project.h"
#include <IRremote.h>

// Configuration:
//#define IR_SCAN_MODE        // Enable to trace the OR button codes on Serial.
#define IR_RECEIVER_PIN 8   // The pin number on which the IR receiver's output is connected.

#define IR_REPEATER_IN_PIN 7   // The pin number on which the IR Repeater input - IR receiver is connected.
#define IR_REPEATER_OUT_PIN 9  // The pin number on which the IR Repeater output - a LED is connected. - Warning - do not change!!!!

#define IR_NO_CODE     0x00000000ul
#define IR_REPEAT_CODE 0xFFFFFFFFul

typedef struct {
  uint32_t ir_code;
  KeyboardKeycode  kbd_key;
} ir_to_kbd_t;  

// IR buttons codes to keyboard keys Mapping:
#define IR_TV_AV 0xE8A24030ul

ir_to_kbd_t g_ir_to_kbd_map[] =  {
  /* IR CODE - KBD KEY */
  {0x8877A956, KEY_VOLUME_MUTE}, /*IR_MUTE*/
  {0x887753AC, KEY_UP_ARROW},    /*IR_UP*/
  {0x88774BB4, KEY_DOWN_ARROW},  /*IR_DOWN*/
  {0x88779966, KEY_LEFT_ARROW},  /*IR_LEFT*/
  {0x8877837C, KEY_RIGHT_ARROW}, /*IR_RIGHT*/
  {0x887709F6, KEY_BACKSPACE},   /*IR_BACK*/
  {0x8877738C, KEY_ENTER},       /*IR_OK*/
  {0x8877B946, KEY_M},           /*IR_MENU*/
  {0x887719E6, KEY_VOLUME_UP},   /*IR_NEXT*/
  {0x8877F30C, KEY_VOLUME_DOWN}, /*IR_PREVIOUS*/
  {0x88776B94, KEY_F},           /*IR_FF*/
  {0x88775BA4, KEY_R},           /*IR_RWND*/
  {0x8877C33C, KEY_SPACE},       /*IR_PLAY_PAUSE*/
  {0x88778B74, KEY_T},           /*IR_LANG*/
  {0x8877BB44, KEY_C},           /*IR_SEARCH*/
  {IR_NO_CODE, KEY_ESC}          /*IR_NONE*/
};

#define IR_TO_KBD_MAP_SIZE (sizeof(g_ir_to_kbd_map)/sizeof(ir_to_kbd_t))

typedef enum 
{
  TV = 0,
  AUX
} tv_mode_t;

tv_mode_t g_tv_mode = TV;

IRrecv irrecv(IR_RECEIVER_PIN);
decode_results g_ir_result;

void ir_repeater_init(void);
void ir_repeater_pwm_on(void);
void ir_repeater_pwm_off(void);
void ir_repeater_isr(void);


void setup()
{
#ifdef IR_SCAN_MODE
  SerialUSB.begin(9600);
  delay(3000);
  SerialUSB.println("Test IR remote:");
#endif
  Keyboard.begin();       //Init keyboard emulation
  Keyboard.releaseAll();
  irrecv.enableIRIn();    // Start the receiver
  ir_repeater_init();
}

void loop() {
  //Wait for an IR code
  if (irrecv.decode(&g_ir_result)) 
  {
#ifdef IR_SCAN_MODE
    SerialUSB.print("PROTOCOL: ");
    SerialUSB.print(g_ir_result.decode_type);
    SerialUSB.print("  VALUE: 0x");
    SerialUSB.println(g_ir_result.value, HEX);        
#endif

    if (IR_TV_AV == g_ir_result.value)
    {
      if (TV == g_tv_mode)
      {
        g_tv_mode = AUX;
      }
      else
      {        
        g_tv_mode = TV;
      }
    }

    
    if (NEC == g_ir_result.decode_type)
    {
      if (AUX == g_tv_mode)
      {          
        if (IR_REPEAT_CODE != g_ir_result.value) //If first press.
        {
          // Search for the mapped KBD KEY
          for (uint8_t index = 0u; index < IR_TO_KBD_MAP_SIZE; index++)
          {
            if (g_ir_to_kbd_map[index].ir_code == g_ir_result.value)
            {
              //Send the KBD KEY
              Keyboard.press(g_ir_to_kbd_map[index].kbd_key);
              Keyboard.releaseAll();
            }
          }
        }
      }
    }
    
    irrecv.resume(); // Wait for another IR code
  }
  delay(100);
}

void ir_repeater_init(void)
{ 
  pinMode(IR_REPEATER_OUT_PIN, OUTPUT);
  pinMode(IR_REPEATER_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_REPEATER_IN_PIN), ir_repeater_isr, CHANGE);   
}

void ir_repeater_pwm_on(void)
{
  pinMode(IR_REPEATER_OUT_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);                // Enable PWM outputs for OC1A on digital pins
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);     // Set fast PWM and no prescaler on timer 1
  ICR1 = 420;                                       // Set the PWM frequency to 38kHz = (16MHz / (420))
  OCR1A = 210;                                      // Set duty-cycle to 50% on D9
}

void ir_repeater_pwm_off(void)
{
  TCCR1A = 0;     // Enable PWM outputs for OC1A on digital pins
  TCCR1B = 0;     // Disable fast PWM and no prescaler on timer 1
  ICR1 = 420;     // Set the PWM frequency to 38kHz = (16MHz / (420))
  OCR1A = 0;      // Set duty-cycle to 0% on D9
  digitalWrite(IR_REPEATER_OUT_PIN, LOW);
}

void ir_repeater_isr(void)
{
  static bool b_pwm_on = false;
  
  if (HIGH == digitalRead(IR_REPEATER_IN_PIN))
  {    
    b_pwm_on = false;
    ir_repeater_pwm_off();
  }
  else
  {
    if (TV == g_tv_mode)
    {
      if (false == b_pwm_on)
      {
        ir_repeater_pwm_on();        
        b_pwm_on = true;
      }
    }    
  }
}
