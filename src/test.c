#define F_CPU 8000000UL

#define TEST_INDICATOR

#define LED_START    7
#define LED_STOP     22

#define BUZZER_START 7
#define BUZZER_STOP  20
#define BUZZER_TIMER 20

#define NEON_TIMER   20 
#define BLINK_TIME   10

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define SET_PIN    0x01  // Set button
#define MODE_PIN   0x02  // Mode button

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
#define NEON_BLINK_STATE_TYPE_MASK  0x01


#define TIME_MASK                   0x10
#define DATE_MASK                   0x20

#define BUZZER_ONCE_MASK            0x08

/*
1 - indi 0
2 - indi 1
3 - indi 2
4 - indi 3
5 - indi 0 state
6 - indi 1 state
7 - indi 2 state
8 - indi 3 state

1 - neon_blink_state_type
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


register uint8_t flag_button asm("r2");
register uint8_t flag_indi asm("r3");
register uint8_t flag_common asm("r4");


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
      byte <<= 1;  
      PORTA |= (1 << I2C_SCL_PIN); // Set SCL high to clock in the bit
      i2c_delay();
      PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low
    }

    DDRA &= ~(1 << I2C_SDA_PIN); // Set SDA as input
    i2c_delay();
    PORTA |= (1 << I2C_SCL_PIN);  // Set SCL high
    i2c_delay();
    
    // Generate an ACK: DELETED

    PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low    
    DDRA |= (1 << I2C_SDA_PIN);   // Set SDA as output
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
/*
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
*/
void rtc3231a_read() {

    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_stop();
    i2c_start();
    i2c_write(RTC_RADDR);
    sec = bcd(i2c_read(0));
    min = bcd(i2c_read(0));
    hour = bcd(i2c_read(0));
    i2c_read(0);
    day = bcd(i2c_read(0));
    month = bcd(i2c_read(0));
    year = bcd(i2c_read(1));
    i2c_stop();

}

inline void rtc3231a_write_time() {

    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_write(0x00);
    i2c_write(bin(min));
    i2c_write(bin(hour));
   
}

inline void rtc3231a_write_date() {

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

// ------------------------- Params -------------------------------
#define TIME_FORMAT_DEFAULT           0
#define LED_MODE_DEFAULT              0
#define BUZZER_MODE_DEFAULT           0
#define CALENDAR_MODE_DEFAULT         0
#define NEON_MODE_DEFAULT             0

#define TIME_FORMAT_MAX               1
#define LED_MODE_MAX                  2
#define BUZZER_MODE_MAX               2
#define CALENDAR_MODE_MAX             1
#define NEON_MODE_MAX                 2

struct Param {
    uint8_t init_value;
    uint8_t time_format;        // 0 - 24,  1 - 12
    uint8_t led_mode;           // 0 - off, 1 - on (7-22), 2 - on
    uint8_t buzzer_mode;        // 0 - off, 1 - on (7-20), 2 - on
    uint8_t calendar_mode;      // 0 - off, 1 - on
    uint8_t neon_mode;          // 0 - 1Hz, 1 - 1/3, 2 - off                
};

struct Param param;

//void eeprom_read_struct() {
    //while (EECR & (1 << EEPE));
    /*
    uint8_t* p = &param.init_value;
    uint8_t i = 0;
    do {
        EEAR=i;
        EECR |= (1 << EERE);
        *(p+i) = EEDR;
        i++;
    } while (i < 4);
    */
    /*
    uint8_t* p = &param.init_value;
    EEAR = 0x04;
    uint8_t i = 4;
    do {
        i--;
        EECR |= (1 << EERE);
        EEAR = i;        
        *(p+i) = EEDR;
    } while (i);      
    */    
        
    /*
    EECR |= (1 << EERE);
    param.time_format = EEDR;
    EEAR++;
    EECR |= (1 << EERE);
    param.led_mode = EEDR;
    EEAR++;
    EECR |= (1 << EERE);
    param.buzzer_mode = EEDR;
    */
    /*
    for (uint8_t i = 0; i < 4; i++) {
        *((uint8_t*)p + i) = EEDR;
        EEAR++;
    }
    */
//}

// --------------------------- Mode -------------------------------
#define CURRENT_TIME      0
#define CURRENT_DATE      1
#define SET_TIME          2
#define SET_HOURS         3
#define SET_MINUTES       4
#define SET_12_24         5
#define SET_LED           6
#define SET_BUZZER        7
#define SET_YEAR          8
#define SET_MONTH         9
#define SET_DAY           10
#define SET_CALENDAR      11
#define SET_NEON          12

uint8_t mode = CURRENT_TIME;
    
// --------------------------- Display ----------------------------
uint8_t indi[4] = { 0x00 };
register uint8_t digit asm("r5");;
//uint8_t counter = 0;

const uint8_t digits[] = {0x0A, 0x01, 0x03, 0x09, 0x02, 0x08, 0x0C, 0x00, 0x04, 0x0B };

void __attribute__((noinline)) set_indicatorN(uint8_t n, uint8_t value) {
    indi[n] = digits[value] | (1 << (7-n)); //value%10?
}

#define PAIR_MASK0 0x00
#define PAIR_MASK1 0x02

void set_pair_indicator(uint8_t pair, uint8_t value, uint8_t mask) {
    flag_indi = mask;
    set_indicatorN(pair,value/10);
    set_indicatorN(++pair,value%10);
}

void __attribute__((noinline)) delay250()      { _delay_ms(250); }
void __attribute__((noinline)) delay100()      { _delay_ms(150); }

void train_lamps() {
    uint8_t i = 9;
    do {
        uint8_t j = 3;
        do { set_indicatorN(j, i); } while (j--);
        delay250();     
    } while (i--);   
}

// --------------------------- Timer ------------------------------
uint8_t blink_timer = 0;
//uint8_t buzzer_timer = 0;
uint8_t mode_hold_timer = 0;
/*
void reset_blink_timer() {
    blink_timer = 0;
}
*/
// --------------------------- Buttons ----------------------------
// SET_PIN    PD1
// MODE_PIN   PD0

uint8_t is_set_pressed() {
    if (flag_button & SET_FLAG && flag_button & SET_STATE) { 
        flag_button &= ~SET_FLAG;
        return 1; 
    }
    return 0;
}
 
void process_button(uint8_t pin, uint8_t state, uint8_t flag) {
       if (!(flag_button & state))  { flag_button |= flag; }
       if (PIND & pin) { flag_button &= ~state; }
       else { flag_button |= state; }
}

void next_value(uint8_t* vp, uint8_t max_value, uint8_t min_value) {
    if (is_set_pressed()) {
        if (*vp == max_value) { *vp = min_value; }
        else { (*vp)++; }
    }
}
/*
void eeprom_write_byte_my(uint8_t *addr, uint8_t value) {
    // Wait for completion of previous write
    while (EECR & (1 << EEPE));
    // Set up address register
    EEAR = (uint16_t)addr;
    // Set the EEMPE bit to enable write
    EECR |= (1 << EEMPE);
    // Write the data to data register
    EEDR = value;
    // Set the EEPE bit to start write
    EECR |= (1 << EEPE);
}
*/
void next_mode(uint8_t m) {
    if (flag_button & MODE_FLAG && flag_button & MODE_STATE) {
        flag_button &= ~MODE_FLAG;
        //rtc3231a_write_time();
        //rtc3231a_write_date();
        //eeprom_update_block(&param, INIT_VALUE_ADDRESS, 4);
        
        uint8_t* p = &param.init_value;
        uint8_t i = 0;
        
        do {
            uint8_t v = *(p+i);
            while (EECR & (1 << EEPE));
            EEAR=i;
            EECR |= (1 << EERE);
            uint8_t eeprom_data = EEDR;
            if (eeprom_data != v) {
                while (EECR & (1 << EEPE));
                EEAR=(uint16_t)i;
                EECR |= (1 << EEMPE);
                EEDR = v;
                EECR |= (1 << EEPE);
            }
            i++;
        } while (i < 6); //Number of params
        
        mode = m;
    }
}

void process_mode(uint8_t i, uint8_t* param, uint8_t mask, uint8_t max, uint8_t min, uint8_t nm) {
   flag_indi = mask;
   set_indicatorN(i,*param);
   next_value(param, max, min);
   next_mode(nm);
}

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
    
    //eeprom_read_block(&param, (const void*) INIT_VALUE_ADDRESS, 4);

    uint8_t* p = &param.init_value;
    uint8_t i = 0;
    do {
        EEAR=i;
        EECR |= (1 << EERE);
        *(p+i) = EEDR;
        i++;
    } while (i < 6); //Number of params
    
    //zero_reg??
    if (param.init_value != INIT_VALUE) {
        param.init_value        = INIT_VALUE;
        param.time_format       = TIME_FORMAT_DEFAULT;   //0-12, 1-24
        param.led_mode          = LED_MODE_DEFAULT;      //0-off, 1-on(7-22), 2 - on
        param.buzzer_mode       = BUZZER_MODE_DEFAULT;   //0-off, 1-on(7-20), 2-on
        param.calendar_mode     = CALENDAR_MODE_DEFAULT; //0-off, 1-on
        param.neon_mode         = NEON_MODE_DEFAULT;     //0 - 1Hz, 1 - 1/3, 2 - off    
    }  
    
    // --------------------- Set Flags -------------------------
    flag_button = 0x80;
    flag_indi   = 0xF0;
    flag_common = 0xF8;

    uint8_t blink_time = NEON_TIMER;
    uint8_t counter = 0;
    digit = 0;
    sei();   // Start interrupts

#ifdef TEST_INDICATOR    
    train_lamps();
#endif
    
    for(;;) {
        
       if (flag_common & TIME_MASK) { rtc3231a_read(); }
       
       // ------------------- READ BUTTONS ---------------------
       process_button(SET_PIN,  SET_STATE,  SET_FLAG);
       process_button(MODE_PIN, MODE_STATE, MODE_FLAG);
 
       // ----------------- STATE MACHINE ---------------
       switch(mode) {

       //INDICATOR 0-3
       //MODE 3s hold -> Menu
       //SET press    -> Date
       case CURRENT_TIME:
           if (is_set_pressed() || (!(min%5) && sec > 55 && param.calendar_mode)) { 
               mode = CURRENT_DATE;
               break; 
           }
           
           blink_time = NEON_TIMER;
           flag_common |= (NEON_BLINK_STATE_MASK | TIME_MASK);
           uint8_t h = hour;
           if (param.time_format) { h = (hour < 13) ? hour : (hour - 12); }
           set_pair_indicator(PAIR_MASK0, h,   0xF0);
           set_pair_indicator(PAIR_MASK1, min, 0xF0);
                      
           //if (is_set_pressed()) { mode = CURRENT_DATE; }
           if (flag_button & MODE_STATE) {
               if (flag_button & MODE_FIRST_PRESSED) {
                   flag_button &= ~MODE_FIRST_PRESSED;
                   mode_hold_timer = 0;
               }
               else if (mode_hold_timer > 120) {    
                   flag_button &= ~MODE_FLAG;               
                   PORTD |= (1 << BUZZER_PIN);
                   _delay_ms(500);
                   PORTD &= ~(1 << BUZZER_PIN); 
                   //buzzer_timer = 0;                  
                   mode = SET_TIME;
               }           
           }
           else { flag_button |= MODE_FIRST_PRESSED; }
           break;
           
       //INDICATOR 0-3
       //Waiting 5seconds and display time
       case CURRENT_DATE:
           
           PORTD &= ~(1 << NEON_PIN);
                  
           train_lamps();
           uint8_t current_date[8] = {day/10, day%10, 0xFF, month/10, month%10, 0xFF, year/10, year%10};
           for (uint8_t i = 7; i < 27; i++) {
               for (uint8_t j = 0; j < 4; j++) {
                   uint8_t s = i+j;
                   uint8_t value = s%10;
                   if (value > 7 || (s<8) || (s>27) || current_date[value] == 0xFF) { indi[j] = 0x00; } 
                   else { set_indicatorN(j, current_date[value]); }
               }
               delay250();
           }
           train_lamps();
           mode = CURRENT_TIME;

           break;
       
       //------------------------    
       //INDICATOR 0-3 (blinking)
       //MENU pressed -> 12-24 set
       //SET  pressed -> set hours
       case SET_TIME:
           flag_indi = 0xFF;
           blink_time = BLINK_TIME;
           if (is_set_pressed()) { mode = SET_HOURS; }
           next_mode(SET_12_24);
           break;
       
       // --------------------------
       // INDICATOR 0-3. INDICATOR 0-1 (blinking)    
       case SET_HOURS:
           flag_common &= ~(NEON_BLINK_STATE_MASK | TIME_MASK | NEON_STATE_MASK);
           set_pair_indicator(PAIR_MASK0, hour, 0xF3);
           next_value(&hour, 23, 0);
           next_mode(SET_MINUTES);
           break;
           
       case SET_MINUTES:
           set_pair_indicator(PAIR_MASK1, min, 0xFC);
           next_value(&min, 59, 0);
           next_mode(SET_12_24);
           rtc3231a_write_time();
           break;
           
       // ----------------------------    
       // INDICATOR 0-1 (not blinking)
       case SET_12_24:
           flag_common &= ~NEON_BLINK_STATE_MASK;
           flag_common |= (TIME_MASK | NEON_STATE_MASK);
           if (param.time_format) { set_pair_indicator(PAIR_MASK0, 12, 0x30); }
           else { set_pair_indicator(PAIR_MASK0, 24, 0x30); }
           next_value(&param.time_format, TIME_FORMAT_MAX, 0);
           next_mode(SET_LED);
           break;
       
       // -------------------------    
       // INDICATOR 3 (not blinking)    
       case SET_LED:
           process_mode(3, &param.led_mode, 0x80, LED_MODE_MAX, 0, SET_BUZZER);
           break;
           
       // INDICATOR 2 (not blinking)
       case SET_BUZZER:       
           process_mode(2, &param.buzzer_mode, 0x40, BUZZER_MODE_MAX, 0, SET_CALENDAR);
           break;
           
       // INDICATOR 1 (not blinking)    
       case SET_CALENDAR:
           process_mode(1, &param.calendar_mode, 0x20, CALENDAR_MODE_MAX, 0, SET_NEON);
           break;
           
       // INDICATOR 0 (not blinking)    
       case SET_NEON:
           process_mode(0, &param.neon_mode, 0x10, NEON_MODE_MAX, 0, SET_YEAR);
           break;
           
       // INDICATOR 2-3 (blinking)
       case SET_YEAR:
           flag_common &= ~(TIME_MASK | NEON_STATE_MASK);
           set_pair_indicator(PAIR_MASK1, year, 0xCC);
           next_value(&year,99, 0);
           next_mode(SET_MONTH);
           break;
           
       case SET_MONTH:
           set_pair_indicator(PAIR_MASK0, day,   0xFC);
           set_pair_indicator(PAIR_MASK1, month, 0xFC);
           next_value(&month,12, 1); 
           next_mode(SET_DAY);
           break;
           
       case SET_DAY:
           
           flag_common;       
              
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
           next_mode(CURRENT_TIME);
           rtc3231a_write_date();
           mode_hold_timer = 0;
           
           break;
           
       default:
           break;
       }
      
      /* -------------------------- Buzzer ------------------------ */
      if (min == 0) {
          if (param.buzzer_mode == 2 || (param.buzzer_mode == 1 && (hour >= BUZZER_START) && (hour <= BUZZER_STOP))) {
              if (flag_common & BUZZER_ONCE_MASK) {
                  //PORTD |= (1 << BUZZER_PIN);
                  //buzzer_timer = 0;
                  PORTD |= (1 << BUZZER_PIN);
                  delay100();
                  PORTD &= ~(1 << BUZZER_PIN);
                  delay100();
                  PORTD |= (1 << BUZZER_PIN);
                  delay100();
                  PORTD &= ~(1 << BUZZER_PIN);
                  flag_common &= ~BUZZER_ONCE_MASK;
              
              }  
          }
      }
      else { flag_common |= BUZZER_ONCE_MASK; }
      /*
      if (PORTD & (1 << BUZZER_PIN) && buzzer_timer > BUZZER_TIMER) { 
          PORTD &= ~(1 << BUZZER_PIN); 
      }*/
      
      // ----------------- LED ---------------------
      PORTD &= ~(1 << LED_PIN);
      if (param.led_mode == 2 || (param.led_mode == 1 && (hour >= LED_START) && (hour < LED_STOP))) { 
          PORTD |= (1 << LED_PIN);
      }

      // ---------------- Blink controller ---------
      
      if (blink_timer > blink_time) {
          //reset_blink_timer();
          //toogle_flag();
          counter+=2;
          if (counter >=6) { counter = 0; }
          blink_timer = 0;
          flag_common ^= BLINK_STATE_MASK;
      }
      
      // ----------------- NEON --------------------
      /*PORTD &= ~(1 << NEON_PIN);
      if (flag_common & (NEON_BLINK_STATE_MASK | BLINK_STATE_MASK)) {
          PORTD |= (1 << NEON_PIN);
      } 
      else if (flag_common & NEON_STATE_MASK) { PORTD |= (1 << NEON_PIN); }*/
      
      if (flag_common & NEON_BLINK_STATE_MASK) {
          if ((flag_common & BLINK_STATE_MASK) || 
               (param.neon_mode == 1 && counter > 0)) {
              PORTD |= (1 << NEON_PIN); 
              
          }
          else { 
              PORTD &= ~(1 << NEON_PIN); 
              /*if (param.neon_mode == 1 && counter) {
                  PORTD |= (1 << NEON_PIN); 
              }*/
              if (param.neon_mode == 2) {
                  PORTD |= (1 << NEON_PIN); 
              } 
          }
      } else {
          if (flag_common & NEON_STATE_MASK) { PORTD |= (1 << NEON_PIN); }
          else  { PORTD &= ~(1 << NEON_PIN); }
      }
        
      
    }
    
    return 0;
}

// ----------------------- Interrupts ---------------------------
ISR(TIMER0_COMPA_vect) {
    blink_timer++;
    //buzzer_timer++;
    mode_hold_timer++;
}

ISR(TIMER1_COMPA_vect) {
    
    uint8_t t = indi[digit];
    
    if (flag_indi & (0x10 << digit)) {
        if (flag_indi & (0x01 << digit)) {
            if (flag_common & BLINK_STATE_MASK) { PORTB = t; }
            else { PORTB = 0x00; }
        }
        else  { PORTB = t; }
    }
    else { PORTB = 0x00; }

    digit++;
    if (digit > 3) { digit = 0; }
}
