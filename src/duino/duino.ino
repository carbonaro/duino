#include <LCD4884.h>
#include <Servo.h>

//keypad debounce parameter
#define DEBOUNCE_MAX 15
#define DEBOUNCE_ON  10
#define DEBOUNCE_OFF 3 

#define NUM_KEYS 5

// joystick number
#define LEFT_KEY 0
#define CENTER_KEY 1
#define DOWN_KEY 2
#define RIGHT_KEY 3
#define UP_KEY 4

#define MENU_X	0		// 0-83
#define MENU_Y	0		// 0-5

int  adc_key_val[5] ={
  50, 200, 400, 600, 800 };

// debounce counters
  byte button_count[NUM_KEYS];
// button status - pressed/released
  byte button_status[NUM_KEYS];
// button on flags for user program 
  byte button_flag[NUM_KEYS];

/* Timer2 reload value, globally available */  
/* cf. http://http://popdevelop.com/2010/04/mastering-timer-interrupts-on-the-arduino/ */
  unsigned int tcnt2;

  char messageBuffer[10 + 84], cmd[3], pin[3], val[4], aux[4], msg[84 + 1];
  int msg_length = 0;
  unsigned char lcd_x = 0;
  unsigned char lcd_y = 0;
  char lcd_mode = MENU_NORMAL;
  boolean debug = false;
  int index = 0;
  Servo servo;

  void setup() {
    setup_lcd();
    Serial.begin(115200);
  }

  void loop() {
  /**
  * Waiting for commands
  */
  while(Serial.available() > 0) {
    char x = Serial.read();
    if (x == '!') index = 0;      // start
    else if (x == '.') process(); // end
    else messageBuffer[index++] = x;
  }
}

/**
* Deal with a full message and determine function to call
*/
void process() {
  index = 0;
  char x[3], y[2], m[2];

  strncpy(cmd, messageBuffer, 2);
  cmd[2] = '\0';
  strncpy(pin, messageBuffer + 2, 2);
  pin[2] = '\0';
  strncpy(val, messageBuffer + 4, 3);
  val[3] = '\0';
  strncpy(aux, messageBuffer + 7, 3);
  aux[3] = '\0';
  if (debug) {
    Serial.println(messageBuffer); }
    Serial.println("zob");
    Serial.println(cmd);
    int cmdid = atoi(cmd);

    switch(cmdid) {
      case 0:  sm(pin,val);               break;
      case 1:  dw(pin,val);               break;
      case 2:  dr(pin);                   break;
      case 3:  aw(pin,val);               break;
      case 4:  ar(pin);                   break;
      case 90: autoReply();               break;
      case 96:
        // Lcd message structure
        //
        // X:       2 bytes
        // Y:       1 byte
        // Mode:    1 byte
        // Payload: up to 84 bytes
        strncpy(x, messageBuffer + 10, 2);
        x[2] = '\n';
        strncpy(y, messageBuffer + 12, 1);
        y[1] = '\n';
        strncpy(m, messageBuffer + 13, 1);
        m[1] = '\n';
        msg_length = atoi(aux);
        Serial.print("x:");
        Serial.print(x);
        Serial.print(", y:");
        Serial.print(y);
        Serial.print(", mode:");
        Serial.print(m);
        Serial.print(", message (");
        Serial.print(msg_length);
        Serial.print("):");
        Serial.println(msg);
        strncpy(msg, messageBuffer + 14, msg_length);
        msg[msg_length] = '\0';
        handleLcd(val, msg_length, atoi(x), atoi(y), atoi(m), msg);
        break;
      case 98: handleServo(pin,val,aux);  break;
      case 99: toggleDebug(val);          break;
      default:                            break;
    }
  }

/**
* Toggle debug mode
* @param char val value for enabling or disabling debugger (0 = false, 1 = true)
*/
void toggleDebug(char *val) {
  if (atoi(val) == 0) {
    debug = false;
    Serial.println("goodbye");
  } else {
    debug = true;
    Serial.println("hello");
  }
}

void autoReply() {
  Serial.println('Is Dave there?'); 
}

/**
* Set pin mode
* @param char pin identifier for pin
* @param char val set pit to OUTPUT or INPUT
*/
void sm(char *pin, char *val) {
  if (debug) {
    Serial.println("sm"); }

    int p = getPin(pin);
    if (p == -1 && debug) {
      Serial.println("badpin"); 
    } else {  
      if (atoi(val) == 0) {
        pinMode(p, OUTPUT);
      } else {
        pinMode(p, INPUT);
      }
    }
  }

/**
* Digital write
* @param char pin identifier for pin
* @param char val set pin to HIGH or LOW
*/
void dw(char *pin, char *val) {
  if (debug) {
    Serial.println("dw"); }

    int p = getPin(pin);
    if (p == -1 && debug) {
      Serial.println("badpin"); 
    } else {  
      pinMode(p, OUTPUT);
      if (atoi(val) == 0) {
        digitalWrite(p, LOW);
      } else {
        digitalWrite(p, HIGH);
      }
    }
  }

/**
* Digital read
* @param char pin pin identifier
*/
void dr(char *pin) {
  if (debug) {
    Serial.println("dr"); }

    int p = getPin(pin);
    if (p == -1 && debug) {
      Serial.println("badpin"); 
    } else {
      pinMode(p, INPUT);
      int oraw = digitalRead(p);
      char m[7];
      sprintf(m, "%02d::%02d", p,oraw);
      Serial.println(m);
    }
  }

/**
* Analog read
* @param char pin pin identifier
*/
void ar(char *pin) {
  if (debug) {
    Serial.println("ar"); }

    int p = getPin(pin);
    if (p == -1 && debug) {
      Serial.println("badpin"); 
    } else {
      pinMode(p, INPUT); // don't want to sw
      int rval = analogRead(p);
      char m[8];
      sprintf(m, "%s::%03d", pin, rval);
      Serial.println(m);
    }
  }  

/*
* Analog write
* @param char pin pin identifier
* @param char val value to write 
*/
void aw(char *pin, char *val) {
  if (debug) {
    Serial.println("aw"); }

    int p = getPin(pin);
    if (p == -1 && debug) {
      Serial.println("badpin"); 
    } else {
      pinMode(p, OUTPUT);
      analogWrite(p, atoi(val));
    }
  }

  int getPin(char *pin) { //Converts to A0-A5, and returns -1 on error
    int ret = -1;
    if (pin[0] == 'A' || pin[0] == 'a') {
      switch(pin[1]) {
        case '0': ret = A0; break;
        case '1': ret = A1; break;
        case '2': ret = A2; break;
        case '3': ret = A3; break;
        case '4': ret = A4; break;
        case '5': ret = A5; break;
        default:            break;
      }
    } else {
      ret = atoi(pin);
      if (ret == 0 && (pin[0] != '0' || pin[1] != '0')) {
        ret = -1; }
      }

      return ret;
    }

// handleLcd(val, msg_length, lcd_x, lcd_y, lcd_mode, msg);
void handleLcd(char *val, int len, unsigned char x, unsigned char y, char mode, char *msg) {
  char line[15];
  char remaining_chars=0, printed_chars=0, nb_chars=0;
  remaining_chars = len;
  Serial.print("got lcd message of ");
  Serial.print(len);
  Serial.print(" characters: ");
  Serial.println(msg);
  switch(atoi(val)) {
    case 0: // LCD_clear
      lcd.LCD_clear();
      break;
    case 1: // LCD_write_string
       lcd.LCD_clear();
       for (int i=y ; i < 6 ; i++) {
         if (i == y) {
           nb_chars = min(len, 14-x);
           strncpy(line, msg, nb_chars);
           line[nb_chars] = '\n';
           lcd.LCD_write_string(x*6, i, line, mode);
         } else {
           nb_chars = min(remaining_chars, 14);
           strncpy(line, msg + printed_chars, nb_chars);
           line[nb_chars] = '\n';
           lcd.LCD_write_string(0, i, line, mode);
         }
         printed_chars += nb_chars;
         remaining_chars -= nb_chars;
       }
       break;
    case 2: // LCD_write_string_big
      // TDB
      break;
    case 3: // Writeln
      nb_chars = min(len, 14-x);
      strncpy(line, msg, nb_chars);
      line[nb_chars] = '\n';
      lcd.LCD_write_string(x*6, y, line, mode);
      break;
    case 98:  // backlight(OFF)
      lcd.backlight(OFF);
      break;
    case 99: // backlight(ON)
      lcd.backlight(ON);
      break;
    default:
      // Noop
      break;
  }
}

/*
* Handle Servo commands
* attach, detach, write, read, writeMicroseconds, attached
*/
void handleServo(char *pin, char *val, char *aux) {
  if (debug) {
    Serial.println("ss"); }

    int p = getPin(pin);
    if (p == -1 && debug) {
      Serial.println("badpin"); 
    } else {
      Serial.println("got signal");
      if (atoi(val) == 0) {
        servo.detach();
      } else if (atoi(val) == 1) {
        servo.attach(p);
        Serial.println("attached");
      } else if (atoi(val) == 2) {
        Serial.println("writing to servo");
        Serial.println(atoi(aux));
        servo.write(atoi(aux));
      }  
    }
  }

/*
* Initialize LCD4884 shield
*
*/
void setup_lcd() {
  for(byte i=0; i<NUM_KEYS; i++){
    button_count[i]=0;
    button_status[i]=0;
    button_flag[i]=0;
  }

  /* First disable the timer overflow interrupt while we're configuring */  
  TIMSK2 &= ~(1<<TOIE2);  

  /* Configure timer2 in normal mode (pure counting, no PWM etc.) */  
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));  
  TCCR2B &= ~(1<<WGM22);  

  /* Select clock source: internal I/O clock */  
  ASSR &= ~(1<<AS2);  

  /* Disable Compare Match A interrupt enable (only want overflow) */  
  TIMSK2 &= ~(1<<OCIE2A);  

  /* Now configure the prescaler to CPU clock divided by 256 */  
  TCCR2B |= (1<<CS22)  | (1<<CS21); // Set bits  
  TCCR2B &= ~(1<<CS21);             // Clear bit  

  /* We need to calculate a proper value to load the timer counter. 
  * The following loads the value 6 into the Timer 2 counter register 
  * The math behind this is: 
  * (CPU frequency) / (prescaler value) = 62500 Hz = 16us. 
  * (desired period) / 16us = 250 
  * MAX(uint8) + 1 - 250 = 6; 
  */  
  /* Save value globally for later reload in ISR */  
  tcnt2 = 6;   

  /* Finally load end enable the timer */  
  TCNT2 = tcnt2;  
  TIMSK2 |= (1<<TOIE2);

  SREG |= 1<<SREG_I;

  lcd.LCD_init();
  lcd.LCD_clear();
  lcd.backlight(OFF);
}

// The followinging are interrupt-driven keypad reading functions
// which includes DEBOUNCE ON/OFF mechanism, and continuous pressing detection

/*
* Convert ADC value to key number
* int input key indentifier
*/
char get_key(unsigned int input)
{
  char k;

  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
      return k;
  }

  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed

  return k;
}

void update_adc_key(){
  int adc_key_in;
  char key_in;
  byte i;

  adc_key_in = analogRead(0);
  key_in = get_key(adc_key_in);
  for(i=0; i<NUM_KEYS; i++)
  {
    if(key_in==i)  //one key is pressed 
    { 
      if(button_count[i]<DEBOUNCE_MAX)
      {
        button_count[i]++;
        if(button_count[i]>DEBOUNCE_ON)
        {
          if(button_status[i] == 0)
          {
            button_flag[i] = 1;
            button_status[i] = 1; //button debounced to 'pressed' status
          }

        }
      }

    }
    else // no button pressed
    {
      if (button_count[i] > 0)
      {  
        button_flag[i] = 0;	
        button_count[i]--;
        if (button_count[i] < DEBOUNCE_OFF) {
          button_status[i] = 0;   //button debounced to 'released' status
        }
      }
    }

  }
}

/*
* Timer Interrupt routine
*
* 1/(160000000/256/(256-6)) = 4ms interval
*/
ISR(TIMER2_OVF_vect) {  
  TCNT2  = tcnt2;
  update_adc_key();
}


