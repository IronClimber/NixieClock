#define F_CPU 8000000UL

#define TEST_INDICATOR

#define LED_START    7
#define LED_STOP     22

#define TRAIN_START  2
#define TRAIN_STOP   4

#define BUZZER_START 7
#define BUZZER_STOP  20
#define BUZZER_TIMER 20

#define NEON_TIMER   20 
#define BLINK_TIME   10

#define TRAIN_TIMER  20
#define TRAIN_PERIOD 1

#define DATE_TIMEOUT 200

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#define SET_PIN    0x02  // Set button
#define MODE_PIN   0x01  // Mode button

#define BUZZER_PIN PD6

#define LED_PIN    PD2  // LEDs

#define NEON_PIN   PD5  // Neon light

#define A0_PIN     PB3
#define A1_PIN     PB1
#define A2_PIN     PB0
#define A3_PIN     PB2

#define AL1_PIN    PB7
#define AL2_PIN    PB6
#define AL3_PIN    PB5
#define AL4_PIN    PB4

#define I2C_SCL_PIN    PA1
#define I2C_SDA_PIN    PA0


// ------------------ Flags ------------------------------------------------

#define INDI0_BLINK_MASK 0x01
#define INDI1_BLINK_MASK 0x02
#define INDI2_BLINK_MASK 0x04
#define INDI3_BLINK_MASK 0x08
#define INDI_BLINK_MASK  0x0F
#define INDI01_BLINK_MASK 0x03
#define INDI23_BLINK_MASK 0x0C

#define BLINK_STATE_MASK            0x80
#define NEON_BLINK_STATE_MASK       0x40
#define NEON_STATE_MASK             0x02

#define TIME_MASK                   0x10
#define DATE_MASK                   0x20

#define BUZZER_ONCE_MASK            0x08

#define LAMP_TRAINING_MODE_MASK     0x04

/*
1 - indi 0
2 - indi 1
3 - indi 2
4 - indi 3
5 - indi 0 state
6 - indi 1 state
7 - indi 2 state
8 - indi 3 state

1 -
2 - neon_state
3 - lamp_training_mode
4 - buzzer_once
5 - time
6 - date
7 - neon_blinking_state
8 - blink_state
*/

#define SET_FLAG           0x01
#define MODE_FLAG          0x02
#define SET_STATE          0x04
#define MODE_STATE         0x08

#define MODE_FIRST_PRESSED 0x80

//button_flags description:
//0- set_button_flag = 0;
//1- mode_button_flag = 0;
//2- set_button_state = 0;
//3- mode_button_state = 0;
//7 -first_mode_pressed = 1; 

struct Flags {
    uint8_t button;
    uint8_t indi;
    uint8_t common;
};

struct Flags flags;

// ------------------------- I2C (TWI library) ------------------------------
#define I2C_DELAY        12
#define I2C_DELAY_HALF   6

void __attribute__((noinline)) i2c_delay_half() { _delay_us(I2C_DELAY_HALF); }
void __attribute__((noinline)) i2c_delay()      { _delay_us(I2C_DELAY); }

void i2c_start() {
    
    PORTA |= (1 << I2C_SDA_PIN);
    PORTA |= (1 << I2C_SCL_PIN);
    i2c_delay_half();    
    PORTA &= ~(1 << I2C_SDA_PIN); // Переводим SDA=0 пока SCL=1
    i2c_delay_half();      
    PORTA &= ~(1 << I2C_SCL_PIN); // Переводим SCL=0  
    i2c_delay_half();        
        
}

void i2c_stop() {
    
    i2c_delay();
    PORTA |= (1 << I2C_SCL_PIN);
    i2c_delay();
    PORTA |= (1 << I2C_SDA_PIN);
    i2c_delay();

}
    
void i2c_write(uint8_t byte) {
    
    // Send a byte
    for (uint8_t i = 0; i < 8; i++) {
      if (byte & 0x80) { PORTA |= (1 << I2C_SDA_PIN); } // Set SDA high (bit is 1)
      else { PORTA &= ~(1 << I2C_SDA_PIN); } // Set SDA low (bit is 0)
      i2c_delay();
      byte <<= 1;  // Shift byte to the left
      PORTA |= (1 << I2C_SCL_PIN); // Set SCL high to clock in the bit
      i2c_delay();
      PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low
    }

    DDRA &= ~(1 << I2C_SDA_PIN); // Set SDA as input ??? DDRA?
    i2c_delay();
    PORTA |= (1 << I2C_SCL_PIN);  // Set SCL high
    i2c_delay();
    
    // Generate an ACK: DELETED

    PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low    
    DDRA |= (1 << I2C_SDA_PIN); // Set SDA as output ??? DDRA?
    PORTA &= ~(1 << I2C_SDA_PIN);

}

uint8_t i2c_read(uint8_t ack) {
    
    uint8_t byte = 0;

    DDRA &= ~(1 << I2C_SDA_PIN);
    
    for (uint8_t i = 0; i < 8; i++) {
        i2c_delay();
        PORTA |= (1 << I2C_SCL_PIN);
        i2c_delay_half();
        byte <<= 1;
        if (PINA & (1 << I2C_SDA_PIN)) { byte |= 0x01; }
        i2c_delay_half();
        PORTA &= ~(1 << I2C_SCL_PIN);        
    }

    DDRA |= (1 << I2C_SDA_PIN);
	
    if (ack) { PORTA |= (1 << I2C_SDA_PIN); }
    else {     PORTA &= ~(1 << I2C_SDA_PIN); }
    i2c_delay();
    PORTA |= (1 << I2C_SCL_PIN);
    i2c_delay();
    PORTA &= ~(1 << I2C_SCL_PIN);

    return byte;

}

// ---------------------------- RTC --------------------------------
#define RTC_WADDR 0xD0
#define RTC_RADDR 0xD1

#define RTC_CONTROL_REGISTER 0x0E

uint8_t sec = 0;
uint8_t min = 0;
uint8_t hour = 0;
uint8_t day = 0;
uint8_t month = 0;
uint8_t year = 0;

// BCD -> DEC
uint8_t bcd (uint8_t data) {
    return (((data & 0xF0) >> 4) * 10) + (data & 0x0F);
}

// DEC -> BCD
uint8_t bin(uint8_t num) {
    return ((num % 10) | ((num / 10) % 10) << 4);
}

void rtc3231a_read_time() {
   
    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_stop();
    i2c_start();
    i2c_write(RTC_RADDR);
    sec = bcd(i2c_read(0));
    min = bcd(i2c_read(0));
    hour = bcd(i2c_read(1));
    i2c_stop();

}

void rtc3231a_read_date() {

    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x04);
    i2c_stop();
    i2c_start();
    i2c_write(RTC_RADDR);
    day = bcd(i2c_read(0));
    month = bcd(i2c_read(0));
    year = bcd(i2c_read(1));
    i2c_stop();

}

void rtc3231a_write_time() {

    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x01);
    i2c_write(bin(min));
    i2c_write(bin(hour));
   
}

void rtc3231a_write_date() {

    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x04);
    i2c_write(bin(day));
    i2c_write(bin(month));
    i2c_write(bin(year));
   
}

// -------------------------- EEPROM ------------------------------
#define INIT_VALUE 0xCA

#define INIT_VALUE_ADDRESS            0x00
#define TIME_FORMAT_ADDRESS           0x01
#define LED_MODE_ADDRESS              0x02
#define BUZZER_MODE_ADDRESS           0x03
#define LAMP_TRAINING_MODE_ADDRESS    0x04
#define CALENDAR_MODE_ADDRESS         0x05

// ------------------------- Params -------------------------------
#define TIME_FORMAT_DEFAULT           0
#define LED_MODE_DEFAULT              0
#define BUZZER_MODE_DEFAULT           0
#define LAMP_TRAINING_MODE_DEFAULT    0
#define CALENDAR_MODE_DEFAULT         0

#define TIME_FORMAT_MAX               1
#define LED_MODE_MAX                  2
#define BUZZER_MODE_MAX               2
#define LAMP_TRAINING_MODE_MAX        1
#define CALENDAR_MODE_MAX             1

struct Param {
    uint8_t init_value;
    uint8_t time_format;        // 0 - 24,  1 - 12
    uint8_t led_mode;           // 0 - off, 1 - on (7-22), 2 - on
    uint8_t buzzer_mode;        // 0 - off, 1 - on (7-20), 2 - on
    uint8_t lamp_training_mode; // 0 - off, 1 - on (2-4)
    uint8_t calendar_mode;      // 0 - off, 1 - on
};

struct Param param;

/*
uint8_t time_format       = TIME_FORMAT_DEFAULT;        // 0 - 12,  1 - 24
uint8_t led_mode          = LED_MODE_DEFAULT;   // 0 - off, 1 - on (7-22), 2 - on
uint8_t buzzer_mode       = BUZZER_MODE_DEFAULT;// 0 - off, 1 - on (7-20), 2 - on
uint8_t lamp_training_mode= LAMP_TRAINING_MODE_DEFAULT; // 0 - off, 1 - on (2-4)
uint8_t calendar_mode     = CALENDAR_MODE_DEFAULT;      // 0 - off, 1 - om
*/

void update_params() {
    //eeprom_update_byte(INIT_VALUE_ADDRESS, INIT_VALUE);
    //eeprom_update_byte((uint8_t*)TIME_FORMAT_ADDRESS, time_format);
    //eeprom_update_byte((uint8_t*)LED_MODE_ADDRESS, led_mode);
    //eeprom_update_byte((uint8_t*)BUZZER_MODE_ADDRESS, buzzer_mode);
    //eeprom_update_byte((uint8_t*)LAMP_TRAINING_MODE_ADDRESS, lamp_training_mode);
    //eeprom_update_byte((uint8_t*)CALENDAR_MODE_ADDRESS, calendar_mode);
    eeprom_update_block(&param, INIT_VALUE_ADDRESS, 6);
}

// --------------------------- Mode -------------------------------
#define CURRENT_TIME      0
#define CURRENT_DATE      1
#define SET_TIME          2
#define SET_HOURS         3
#define SET_MINUTES       4
#define SET_12_24         5
#define SET_LED           6
#define SET_BUZZER        7
#define SET_LAMP_TRAIN    8
#define SET_YEAR          9
#define SET_MONTH         10
#define SET_DAY           11
#define SET_CALENDAR      12
#define LAMP_TRAINING     13

uint8_t mode = CURRENT_TIME;
    
// --------------------------- Display ----------------------------
uint8_t indi[4] = { 0x00 };
uint8_t digit = 0;
uint8_t counter = 0;

const uint8_t digits[] = {0x0A, 0x01, 0x03, 0x09, 0x02, 0x08, 0x0C, 0x00, 0x04, 0x0B };
 /*get_digit(uint8_t v) {
    switch(v) {
    case 0: return 0x0A;
    case 1: return 0x01;
    case 2: return 0x03;
    case 3: return 0x09;
    case 4: return 0x02;
    case 5: return 0x08;
    case 6: return 0x0C;
    case 7: return 0x00;
    case 8: return 0x04;
    case 9: return 0x0B;
    default: return 0x0A;
    }
}*/
void __attribute__((noinline)) set_indicatorN(uint8_t n, uint8_t value) {
    indi[n] = digits[value%10] | (1 << (7-n));
}

#define PAIR_MASK0 0x00
#define PAIR_MASK1 0x02

void set_pair_indicator(uint8_t pair, uint8_t value, uint8_t mask) {
    flags.indi = mask;
    set_indicatorN(pair,value/10);
    set_indicatorN(++pair,value%10);
}

/*void set_indicator(uint8_t fp, uint8_t sp, uint8_t mask) {
    flags.indi = mask;
    set_pair_indicator(PAIR_MASK0, fp, mask);
    set_pair_indicator(PAIR_MASK1, sp, mask);
}*/

void set_indicator_all(uint8_t v) {
    int i = 3;
    do { set_indicatorN(i, v); } while (i--);
}

// --------------------------- Timer ------------------------------
uint8_t blink_timer = 0;
uint8_t buzzer_timer = 0;
uint8_t mode_hold_timer = 0;
    
// --------------------------- Buttons ----------------------------
// SET_PIN    PD1
// MODE_PIN   PD0

uint8_t is_set_pressed() {
    if (flags.button & SET_FLAG && flags.button & SET_STATE) { 
        flags.button &= ~SET_FLAG;
        return 1; 
    }
    return 0;
}
 
/*
uint8_t is_mode_pressed() {
    if (flags.button & MODE_FLAG && flags.button & MODE_STATE) { 
        flags.button &= ~MODE_FLAG;
        return 1; 
    }
    return 0;
} 
*/

void process_button(uint8_t pin, uint8_t state, uint8_t flag) {
       if (!(flags.button & state))  { flags.button |= flag; }
       if (PIND & pin) { flags.button &= ~state; }
       else { flags.button |= state; }
}

void next_value(uint8_t* vp, uint8_t max_value, uint8_t min_value) {
    if (is_set_pressed()) {
        if (*vp == max_value) { *vp = min_value; }
        else { (*vp)++; }
    }
}

void next_mode(uint8_t m/*, uint8_t p, uint8_t a*/) {
    if (flags.button & MODE_FLAG && flags.button & MODE_STATE) {
        flags.button &= ~MODE_FLAG;
        rtc3231a_write_time();
        rtc3231a_write_date();
        update_params();
        mode = m;
    }
}

void process_mode(uint8_t i, uint8_t* param, uint8_t mask, uint8_t max, uint8_t min, uint8_t nm) {
   flags.indi = mask;
   set_indicatorN(i,*param);
   next_value(param, max, min);
   next_mode(nm);
}

/*
void process_mode_p(uint8_t i, uint8_t* param, uint8_t mask, uint8_t max, uint8_t min, uint8_t nm) {
           set_pair_indicator(i, *param, mask);
           next_value(param, max, min);
           next_mode(nm);
}
*/

// --------------------------- Main -------------------------------


int main(void) {

    DDRD |= (1 << LED_PIN) | (1 << NEON_PIN) | (1 << BUZZER_PIN);
    DDRD &= ~(SET_PIN | MODE_PIN);
    
    // Init lamps
    DDRB = 0xFF;
    PORTB = 0x00;

    // -------------------- Timer 0 --------------------------------    
    TCCR0A |= (1 << WGM01);               // CTC mode
    TCCR0B |= (1 << CS02) | (1 << CS00);  // Prescaler
    OCR0A = 194;                          // Compare value
    TIMSK |= (1 << OCIE0A);               // Allow interrupts
    
    // -------------------- Timer 1 --------------------------------    
    TCCR1B = (1 << WGM12);  // CTC mode
    TCCR1B |= (1 << CS12);  // Prescaler
    OCR1A = 110;            // Compare value
    TIMSK |= (1 << OCIE1A); // Allow interrupts
        
    // -------------------- INIT_I2C -------------------------------
    DDRA |= (1 << I2C_SDA_PIN); //SDA as output
    DDRA |= (1 << I2C_SCL_PIN); //SCL as output
        
    PORTA |= (1 << I2C_SDA_PIN); //SDA high
    PORTA |= (1 << I2C_SCL_PIN); //SCL high
    
    // -------------- RTC INIT -------------------------------------
    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(RTC_CONTROL_REGISTER);
    i2c_write(0x20);
    i2c_write(0x08);
    i2c_stop();
    
    // -------------- Restore params from EEPROM -------------------
    eeprom_read_block(&param, (const void*) INIT_VALUE_ADDRESS, 6);

    if (param.init_value != INIT_VALUE) {
        //PORTD |= (1 << LED_PIN);
        param.init_value        = INIT_VALUE;
        param.time_format       = TIME_FORMAT_DEFAULT; //0-12, 1-24
        param.led_mode          = LED_MODE_DEFAULT;    //0-off, 1-on(7-22), 2 - on
        param.buzzer_mode       = BUZZER_MODE_DEFAULT; //0-off, 1-on(7-20), 2-on
        param.lamp_training_mode= LAMP_TRAINING_MODE_DEFAULT; //0-off, 1-on (2-4)
        param.calendar_mode     = CALENDAR_MODE_DEFAULT; 
        ///uint8_t eeprom_init_value = eeprom_read_byte((uint8_t*)INIT_VALUE_ADDRESS);
        /*
        time_format = eeprom_read_byte((uint8_t*)TIME_FORMAT_ADDRESS);
        led_mode = eeprom_read_byte((uint8_t*)LED_MODE_ADDRESS);
        buzzer_mode = eeprom_read_byte((uint8_t*)BUZZER_MODE_ADDRESS);
        lamp_training_mode = eeprom_read_byte((uint8_t*)LAMP_TRAINING_MODE_ADDRESS);
        calendar_mode = eeprom_read_byte((uint8_t*)CALENDAR_MODE_ADDRESS);
        */        
    }  

    // --------------------- Set Flags -------------------------
    flags.button = 0x80;
    flags.indi   = 0xF0;
    flags.common = 0xF8;

#ifdef TEST_INDICATOR    
  int i = 9;
  do { set_indicator_all(i); } while (i--);
  //for (int i = 0; i < 10; ++i) {
  //    set_indicator_all(i, 0xF0);
  //    _delay_ms(1000);
  //  }
#endif
    
    uint8_t blink_time = NEON_TIMER;
    
    sei();   // Start interrupts

    for(;;) {
        
       if (flags.common & TIME_MASK) { rtc3231a_read_time(); }
       if (flags.common & DATE_MASK) { rtc3231a_read_date(); }
       
       // ------------------- READ BUTTONS ---------------------
       process_button(SET_PIN,  SET_STATE,  SET_FLAG);
       process_button(MODE_PIN, MODE_STATE, MODE_FLAG);
 
       // ----------------- STATE MACHINE ---------------
       switch(mode) {
       
     /*  case LAMP_TRAINING:
           flags.indi = 0xF0;
           flags.common &= ~NEON_BLINK_STATE_MASK;
           flags.common |= NEON_STATE_MASK;
           set_indicator_all(counter);
       */
       //INDICATOR 0-3
       //MODE 3s hold -> Menu
       //SET press    -> Date
       case CURRENT_TIME:
           if (param.calendar_mode && sec > 55) { 
               mode = CURRENT_DATE;
               break; 
           }
           blink_time = NEON_TIMER;
           flags.common |= NEON_BLINK_STATE_MASK;
           uint8_t h = hour;
           if (param.time_format) { h = (hour < 13) ? hour : (hour - 12); }
           set_pair_indicator(PAIR_MASK0, h,   0xF0);
           set_pair_indicator(PAIR_MASK1, min, 0xF0);
           //set_indicator(h, min, 0xF0);
                      
           if (is_set_pressed()) { mode = CURRENT_DATE; }
           if (flags.button & MODE_STATE) {
               if (flags.button & MODE_FIRST_PRESSED) {
                   flags.button &= ~MODE_FIRST_PRESSED;
                   mode_hold_timer = 0;
               }
               else if (mode_hold_timer > 120) {    
                   flags.button &= ~MODE_FLAG;               
                   PORTD |= (1 << BUZZER_PIN);
                   buzzer_timer = 0;                  
                   mode = SET_TIME;
               }           
           }
           else { flags.button |= MODE_FIRST_PRESSED; }
           break;
           
       //INDICATOR 0-3
       //Waiting 5seconds and display time
       case CURRENT_DATE:
                  
           //flags.indi = 0xF0;
           /*
           uint8_t current_date[6] = {day/10, day%10, month/10, month%10, year/10, year%10};
           for (uint8_t i = 5; i < 23; i++) {
               for (uint8_t j = 0; j < 4; j++) {
                   uint8_t value = (i+j)%8;
                   if (value > 5) { indi[j] = 0x00; }
                   else { set_indicatorN(j, current_date[value]); }
               }
               _delay_ms(250);
           }
           */
           
           set_pair_indicator(PAIR_MASK0, day,   0xF0);
           set_pair_indicator(PAIR_MASK1, month, 0xF0);
           _delay_ms(2000);
           mode = CURRENT_TIME;
           /*
           if (time_format) { 
               set_pair_indicator(PAIR_MASK0, month);
               set_pair_indicator(PAIR_MASK1, day);
           }
           else {
               set_pair_indicator(PAIR_MASK0, day);
               set_pair_indicator(PAIR_MASK1, month);
           }
           if (date_timer > DATE_TIMEOUT) { mode = CURRENT_TIME; }*/
           //while(next_step()) {_delay_ms(250); }
           break;
       
       //------------------------    
       //INDICATOR 0-3 (blinking)
       //MENU pressed -> 12-24 set
       //SET  pressed -> set hours
       case SET_TIME:
           flags.indi = 0xFF;
           blink_time = BLINK_TIME;
           if (is_set_pressed()) { mode = SET_HOURS; }
           next_mode(SET_12_24);
           break;
       
       // --------------------------
       // INDICATOR 0-3. INDICATOR 0-1 (blinking)    
       case SET_HOURS:
           flags.common &= ~(NEON_BLINK_STATE_MASK | TIME_MASK | NEON_STATE_MASK);
           set_pair_indicator(PAIR_MASK0, hour, 0xF3);
           next_value(&hour, 23, 0);
           next_mode(SET_MINUTES);
           //process_mode_p(PAIR_MASK0, &hour, 0xF3, 23, 0, SET_MINUTES);
           break;
           
       case SET_MINUTES:
           set_pair_indicator(PAIR_MASK1, min, 0xFC);
           next_value(&min, 59, 0);
           next_mode(SET_12_24);
           //process_mode_p(PAIR_MASK1, &min, 0xFC, 59, 0, SET_12_24);
           break;
           
       // ----------------------------    
       // INDICATOR 0-1 (not blinking)
       
       case SET_12_24:
           flags.common &= ~NEON_BLINK_STATE_MASK;
           flags.common |= (TIME_MASK | NEON_STATE_MASK);
           process_mode(0, &param.time_format, 0x10, TIME_FORMAT_MAX, 0, SET_LED);
           break;
       
       // -------------------------    
       // INDICATOR 3 (not blinking)    
       case SET_LED:
           process_mode(3, &param.led_mode, 0x80, LED_MODE_MAX, 0, SET_BUZZER);
           break;
           
       // INDICATOR 2 (not blinking)
       case SET_BUZZER:       
           process_mode(2, &param.buzzer_mode, 0x40, BUZZER_MODE_MAX, 0, SET_LAMP_TRAIN);
           break;
           
       // INDICATOR 1 (not blinking)
       case SET_LAMP_TRAIN:
           process_mode(1, &param.lamp_training_mode, 0x20, LAMP_TRAINING_MODE_MAX, 0, SET_YEAR);
           break;
           
       // INDICATOR 2-3 (blinking)
       case SET_YEAR:
           flags.common &= ~(DATE_MASK | NEON_STATE_MASK);
           set_pair_indicator(PAIR_MASK1, year, 0xCC);
           next_value(&year,99, 0);
           next_mode(SET_MONTH);
           //process_mode_p(PAIR_MASK1, &year, 0xCC, 99, 0, SET_MONTH);
           break;
           
       case SET_MONTH:
           //set_indicator(day, month, 0xFF);
           set_pair_indicator(PAIR_MASK0, day,   0xFC);
           set_pair_indicator(PAIR_MASK1, month, 0xFC);
           next_value(&month,12, 1); 
           next_mode(SET_DAY);
           break;
           
       case SET_DAY:
           
           flags.common;          
           uint8_t max_day = 31;
           //Если высокосный год февраль -2 - (29, иначе 28)
           //1, 3, 5, 7, 8, 10, 12 -31
           //4,6,9,11
           if (month == 4 || month == 6 || month == 9 || month == 11) {
               max_day = 30; 
           }
           else if (month == 2) { max_day = (year%4) ? 28 : 29; }
           
           set_pair_indicator(PAIR_MASK0, day, 0xF3);
           next_value(&day, max_day, 1);
           next_mode(SET_CALENDAR);
           //process_mode_p(PAIR_MASK0, &day, 0xF3, max_day, 1, SET_CALENDAR);
           break;
           
       case SET_CALENDAR:
           process_mode(0, &param.calendar_mode, 0x10, CALENDAR_MODE_MAX, 0, CURRENT_TIME);
           //mode_hold_timer = 0;
           break;
           
       default:
           break;
       }
      
      /* -------------------------- Buzzer ------------------------ */
      
      if (min == 0) {
          if (param.buzzer_mode == 2 || (param.buzzer_mode == 1 && (hour > BUZZER_START) && (hour < BUZZER_STOP))) {
              if (flags.common & BUZZER_ONCE_MASK) {
                  PORTD |= (1 << BUZZER_PIN);
                  buzzer_timer = 0;
                  flags.common &= ~BUZZER_ONCE_MASK;
              }  
          }
      }
      else { flags.common |= BUZZER_ONCE_MASK; }
      
      if (PORTD & (1 << BUZZER_PIN) && buzzer_timer > BUZZER_TIMER) { 
          PORTD &= ~(1 << BUZZER_PIN); 
      }
      
      // ----------------- LED ---------------------
      PORTD &= ~(1 << LED_PIN);
      if (param.led_mode == 2 || (param.led_mode == 1 && (hour >= LED_START) && (hour < LED_STOP))) { 
          PORTD |= (1 << LED_PIN);
      }

      // ---------------- Blink controller ---------
      if (blink_timer > blink_time) {
          blink_timer = 0;
          flags.common ^= BLINK_STATE_MASK;
          counter++;
          if (counter > 9) { counter = 0; }          
      }
      
      // ----------------- NEON --------------------
      PORTD &= ~(1 << NEON_PIN);
      if (flags.common & NEON_BLINK_STATE_MASK) {
          if (flags.common & BLINK_STATE_MASK) { PORTD |= (1 << NEON_PIN); }
      } else if (flags.common & NEON_STATE_MASK) { PORTD |= (1 << NEON_PIN); }
        
      
      // --------------- LAMP TRAINING ----------------------------
/*      
      if (param.lamp_training_mode == 1 && (hour >= TRAIN_START) && (hour < TRAIN_STOP)) { 
          flags.common |= LAMP_TRAINING_MODE_MASK;
          
      } else { flags.common &= ~(LAMP_TRAINING_MODE_MASK); }
      */
      
      // ----------------- Calendar --------------------------------
      
      _delay_ms(50);
    }
    
    return 0;
}

// ----------------------- Interrupts ---------------------------
ISR(TIMER0_COMPA_vect) {
    blink_timer++;
    buzzer_timer++;
    mode_hold_timer++;
}

ISR(TIMER1_COMPA_vect) {
    
    uint8_t t = indi[digit];
    //PORTB = 0x00;
    
    if (flags.indi & (0x10 << digit)) {
        if (flags.indi & (0x01 << digit)) {
            if (flags.common & BLINK_STATE_MASK) { PORTB = t; }
            else { PORTB = 0x00; }
        }
        else  { PORTB = t; }
    }
    else { PORTB = 0x00; }
/*
    
    if ((flags.indi & (0x10 << digit)) && !(flags & BLINK_STATE_MASK) && (flags.indi & (1 << digit))) { PORTB = indi[digit]; }
    */
    digit++;
    if (digit > 3) { digit = 0; }
}
