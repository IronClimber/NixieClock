#define F_CPU 8000000UL

//#define TEST_INDICATOR

#define LED_START    7
#define LED_STOP     22

#define TRAIN_START  2
#define TRAIN_STOP   4

#define BUZZER_START 7
#define BUZZER_STOP  20

#define NEON_TIMER   20 

#define TRAIN_TIMER  20
#define TRAIN_PERIOD 1

#define BLINK_TIME   10

#define DATE_TIMEOUT 200

//#define

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define SET_PIN    PD1  // Set button
#define MODE_PIN   PD0  // Mode button

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

//Indicator
uint8_t buf[4] = { 0x00 }; 
uint8_t indi[4] = { 0x00 };

uint8_t digit = 0;
uint8_t counter = 0;
uint8_t digits[] = {
 0x8A, 0x81, 0x83, 0x89, 0x82, 0x88, 0x8C, 0x80, 0x84, 0x8B,
 0x4A, 0x41, 0x43, 0x49, 0x42, 0x48, 0x4C, 0x40, 0x44, 0x4B,
 0x2A, 0x21, 0x23, 0x29, 0x22, 0x28, 0x2C, 0x20, 0x24, 0x2B,
 0x1A, 0x11, 0x13, 0x19, 0x12, 0x18, 0x1C, 0x10, 0x14, 0x1B
};


// ------------------ Flags ------------------------------------------------
uint8_t neon_blink_state = 0; // 0 - blink 1Hz, 1 - on, 2 - off, 3 - blink 2Hz
uint8_t indi_blink[4] = { 0x00 };
uint8_t blink_state = 1;


// ------------------------- I2C (TWI library) ------------------------------
#define I2C_DELAY        12
#define I2C_DELAY_HALF   6

void i2c_start() {
    
    PORTA |= (1 << I2C_SDA_PIN); // Выставляем исходное состояние SDA=1 на всякий случай
    PORTA |= (1 << I2C_SCL_PIN); // Выставляем исходное состояние SCL=1 на всякий случай
    
    _delay_us(I2C_DELAY_HALF);	
    PORTA &= ~(1 << I2C_SDA_PIN); // Переводим SDA=0 пока SCL=1
    
    _delay_us(I2C_DELAY_HALF);
    PORTA &= ~(1 << I2C_SCL_PIN); // Переводим SCL=0
        
    _delay_us(I2C_DELAY_HALF);
        
}

void i2c_stop() {
    
    _delay_us(I2C_DELAY);

    PORTA |= (1 << I2C_SCL_PIN);
    
    _delay_us(I2C_DELAY);
    PORTA |= (1 << I2C_SDA_PIN);
    
    _delay_us(I2C_DELAY);

}
    
void i2c_write(uint8_t byte) {
    
    // Send a byte
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            PORTA |= (1 << I2C_SDA_PIN); // Set SDA high (bit is 1)
        } else {
            PORTA &= ~(1 << I2C_SDA_PIN); // Set SDA low (bit is 0)
        }
	_delay_us(I2C_DELAY);
        
        byte <<= 1;  // Shift byte to the left
        PORTA |= (1 << I2C_SCL_PIN); // Set SCL high to clock in the bit
        
        _delay_us(I2C_DELAY);
        PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low
        
    }

    DDRA &= ~(1 << I2C_SDA_PIN); // Set SDA as input ??? DDRA?
    _delay_us(I2C_DELAY);
    PORTA |= (1 << I2C_SCL_PIN);  // Set SCL high
    _delay_us(I2C_DELAY);
    
    // Generate an ACK: DELETED


    PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low
    // Set SDA as output ??? DDRA?
    DDRA |= (1 << I2C_SDA_PIN);
    PORTA &= ~(1 << I2C_SDA_PIN);

}

uint8_t i2c_read(uint8_t ack) {
    
    uint8_t byte = 0;

    DDRA &= ~(1 << I2C_SDA_PIN);
    
    for (uint8_t i = 0; i < 8; i++) {
        _delay_us(I2C_DELAY);
        PORTA |= (1 << I2C_SCL_PIN);
        _delay_us(I2C_DELAY_HALF);
        byte <<= 1;
        if (PINA & (1 << I2C_SDA_PIN)) {
            byte |= 0x01;
        }
        else {
        }
        _delay_us(I2C_DELAY_HALF);
        PORTA &= ~(1 << I2C_SCL_PIN);
        
    }

    DDRA |= (1 << I2C_SDA_PIN);
	
    if (ack) {
        PORTA |= (1 << I2C_SDA_PIN);
    }
    else {
        PORTA &= ~(1 << I2C_SDA_PIN);
    }
    _delay_us(I2C_DELAY);
    PORTA |= (1 << I2C_SCL_PIN);
    _delay_us(I2C_DELAY);
    PORTA &= ~(1 << I2C_SCL_PIN);

    return byte;

}

// ---------------------------- RTC --------------------------------
#define RTC_WADDR 0xD0
#define RTC_RADDR 0xD1

#define RTC_CONTROL_REGISTER 0x0E

uint8_t min = 0;
uint8_t hour = 0;
uint8_t day = 0;
uint8_t month = 0;
uint8_t year = 0;

uint8_t bcd (uint8_t data) {
    uint8_t bc;
    bc = ((((data & (1 << 6)) | (data & (1 << 5)) | (data & (1 << 4)))*0x0A) >> 4)
	+ ((data & (1 << 3))|(data & (1 << 2))|(data & (1 << 1))|(data & 0x01));
    return bc;
}
/*
static uint8_t bin(uint8_t dec) {
	uint8_t bcd;
	uint8_t n, dig, num, count;

	num = dec;
	count = 0;
	bcd = 0;

	for (n = 0; n < 4; n++) {
		dig = num % 10;
		num = num / 10;
		bcd = (dig << count) | bcd;
		count += 4;
	}
	
	return bcd;
}
*/

void rtc3231a_init(void) {
       
    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(RTC_CONTROL_REGISTER);
    i2c_write(0x20);
    i2c_write(0x08);
    i2c_stop();

}

void rtc3231a_read_time() {
   
    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x01);
    i2c_stop();
    i2c_start();
    i2c_write(RTC_RADDR);
    //time->sec = bcd(i2c_read(0));
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
/*
uint8_t rtc3231a_write_date() {

    i2c_start();
    if (i2c_write(RTC_WADDR)) { return 1; }
    if (i2c_write(0x04)) { return 2; }
    if (i2c_write(bin(17))) { return 3; }
    if (i2c_write(bin(8))) { return 4; }
    if (i2c_write(bin(97))) { return 5; }
   
    return 0;
}
*/
// -------------------------- EEPROM ------------------------------

#define INIT_VALUE 0xCE

#define INIT_VALUE_ADDRESS            0x00
#define TIME_FORMAT_ADDRESS           0x01
#define LED_MODE_ADDRESS              2
#define BUZZER_MODE_ADDRESS           3
#define LAMP_TRAINING_MODE_ADDRESS    4
#define CALENDAR_MODE_ADDRESS         5

#define TIME_FORMAT_DEFAULT           1
#define LED_MODE_DEFAULT              2
#define BUZZER_MODE_DEFAULT           3
#define LAMP_TRAINING_MODE_DEFAULT    4
#define CALENDAR_MODE_DEFAULT         5


typedef struct {
    uint8_t time_format;         // 0 - 12, 1 - 24
    uint8_t led_mode;            // 0 - off, 1 - on (7-22), 2 - on
    uint8_t buzzer_mode;         // 0 - off, 1- on (7-20), 2 - on
    uint8_t lamp_training_mode;  // 0 - off, 1 - on (2-4)
    uint8_t calendar_mode;       // 0 - off, 1 - om
} ClockParams;

ClockParams cp;
ClockParams t = { TIME_FORMAT_DEFAULT,
                LED_MODE_DEFAULT, 
                BUZZER_MODE_DEFAULT, 
                LAMP_TRAINING_MODE_DEFAULT,
                CALENDAR_MODE_DEFAULT };
                
void eeprom_write_default_params() {
    eeprom_write_block((const void*)&t, (void*)TIME_FORMAT_ADDRESS, sizeof(t));
}

void eeprom_read_params() {
    eeprom_read_block((void*)&cp, (const void*)TIME_FORMAT_ADDRESS, sizeof(cp));
}

// ToDo!!!!!
void eeprom_update_params() {
    //ClockParams cp_temp = 
}


// --------------------------- Mode -------------------------------
typedef enum {
    CURRENT_TIME,
    CURRENT_DATE,
    SET_TIME,
    SET_HOURS,
    SET_MINUTES,
    SET_12_24,
    SET_LED,
    SET_BUZZER,
    SET_LAMP_TRAIN,
    SET_YEAR,
    SET_MONTH,
    SET_DAY,
    SET_CALENDAR
} State;

State mode = CURRENT_TIME;
    
// --------------------------- Display ----------------------------

void set_indicator(uint8_t n, uint8_t value) {
    indi[n] = digits[value+n*10];
}

void display_time() {
    set_indicator(0,hour/10);
    set_indicator(1,hour%10);
    set_indicator(2,min/10);
    set_indicator(3,min%10);
}

void display_date() {
    set_indicator(0,day/10);
    set_indicator(1,day%10);
    set_indicator(2,month/10);
    set_indicator(3,month%10);
}
/*

void display_time() {
    [hour/10];
    indi[1] = digits[hour%10+10];
    indi[2] = digits[min/10+20];
    indi[3] = digits[min%10+30];
}
*/
/*
void display_time_ms() {
    indi[0] = digits[(uint8_t)min/10];
    indi[1] = digits[(uint8_t)min%10+10];
    indi[2] = digits[(uint8_t)sec/10+20];
    indi[3] = digits[(uint8_t)sec%10+30];
}
*/

// --------------------------- Timer ------------------------------
    uint8_t neon_timer = 0;
    uint8_t blink_timer = 0;
    uint8_t buzzer_timer = 0;
    uint8_t date_timer = 0;
    uint8_t mode_hold_timer = 0;
    
// --------------------------- Buzzer -----------------------------
/*
void buzzer(uint8_t time) {
    buzzer_timer = ticks + time;
}
*/

// --------------------------- Buttons ----------------------------
// SET_PIN    PD1
// MODE_PIN   PD0

uint8_t set_button_flag = 0;
uint8_t mode_button_flag = 0;
uint8_t set_button_state = 0;
uint8_t mode_button_state = 0;

uint8_t first_mode_pressed = 1; 

uint8_t is_set_pressed() {
    if (set_button_state && set_button_flag) { return 1; }
    return 0;
} 

uint8_t is_mode_pressed() {
    if (mode_button_state && mode_button_flag) {
        mode_button_flag = 0;
        return 1; 
    }
    return 0;
} 

// ----------------------------
// --------------------------- Main -------------------------------

int main(void) {

    // Настройка пина светодиода как выхода
    DDRD |= (1 << LED_PIN);
    // Настройка пина
    DDRD |= (1 << NEON_PIN);
    // Настройка пина Баззера
    DDRD |= (1 << BUZZER_PIN);
    
    // Init buttons
    DDRD &= ~((1 << SET_PIN) | (1 << MODE_PIN));
    //PORTD = 0x00;
    
    // Init lamps
    DDRB = 0xFF;
    PORTB = 0x00;
     
    //PORTD^= (1 << NEON_PIN);

    // -------------------- Timer 0 --------------------------------
    // Установка режима CTC (Clear Timer on Compare Match)
    TCCR0A |= (1 << WGM01);
    
    // Установка предделителя (например, 64 для периода в 1 миллисекунду при тактовой частоте 8 МГц)
    TCCR0B |= (1 << CS02) | (1 << CS00);
    
    // Устанавливаем значение, при котором будет генерироваться прерывание (OCR0)
    OCR0A = 196;  // (125 - 1)Для периода в 1 миллисекунду при тактовой частоте 8 МГц
    
    // Разрешение прерывания по совпадению с OCR0
    TIMSK |= (1 << OCIE0A);
    
    // -------------------- Timer 1 --------------------------------
    //Set timer 1 to CTC mode (Clear Timer on Compare
    TCCR1B = (1 << WGM12);
    
    // Установка предделителя таймера 1 в 256 (
    TCCR1B |= (1 << CS12);

    //OCR - Output Compare Register
    OCR1A = 110;

    // Разрешение прерывания по переполнению таймера 0
    TIMSK |= (1 << OCIE1A);    
    // --------------------------------------------------------------
    
    sei();
    
    // ---------- INIT_I2C -------------------
    DDRA |= (1 << I2C_SDA_PIN); //SDA as output
    DDRA |= (1 << I2C_SCL_PIN); //SCL as output
        
    PORTA |= (1 << I2C_SDA_PIN); //SDA high
    PORTA |= (1 << I2C_SCL_PIN); //SCL high
    
    // -------------- RTC INIT -------------------
    rtc3231a_init();
    
    uint8_t eeprom_init_value = eeprom_read_byte((uint8_t*)INIT_VALUE_ADDRESS);
    if (eeprom_init_value != INIT_VALUE) {
        eeprom_write_default_params();
    }
    eeprom_read_params();

#ifdef TEST_INDICATOR    
    for (int i = 0; i < 10; ++i) {
      indi[0] = digits[i];
      indi[1] = digits[i+10];
      indi[2] = digits[i+20];
      indi[3] = digits[i+30];
      _delay_ms(100);
    }
#endif

    neon_timer = 0;
    blink_timer = 0;
    buzzer_timer = 0;
    date_timer = 0;

    while (1) {
        
       // ------------------- READ BUTTONS ---------------------
       
       set_button_state = !(PIND & (1 << SET_PIN));
       mode_button_state = !(PIND & (1 << MODE_PIN));
       
       
 
       // ----------------- NEON --------------------
       if (neon_timer > NEON_TIMER) {
           neon_timer = 0;
           if (neon_blink_state == 0) {
               PORTD ^= (1 << NEON_PIN); //Toggle neon
           }
           rtc3231a_read_time();
           rtc3231a_read_date();
      
       }
       // ----------------- STATE MACHINE ---------------
       switch(mode) {
       case CURRENT_TIME:
           display_time();
           if (!(PIND & (1 << SET_PIN))) {
               date_timer = 0;
               mode = CURRENT_DATE;
           }
           /*
           { PORTD |= (1 << LED_PIN); }
           else { PORTD &= ~(1 << LED_PIN); }
           */
           if (mode_button_state) {
               if (first_mode_pressed) {
                   first_mode_pressed = 0; 
                   mode_hold_timer = 0;
               }
               if (mode_hold_timer > 120) {
                   mode_button_flag = 0;
                   indi_blink[0] = 1;
                   indi_blink[1] = 1;
                   indi_blink[2] = 1;
                   indi_blink[3] = 1;
                   neon_blink_state = 3;
                   //buzzer(20);
                   
                   mode = SET_TIME;
               }           
           }
           else {
               first_mode_pressed = 1;
           }
           break;
       case CURRENT_DATE:
           display_date();
           if (date_timer > DATE_TIMEOUT) {
               mode = CURRENT_TIME;
           }
           break;
       case SET_TIME:
           display_time();
           if (!(PIND & (1 << SET_PIN))) {
               indi_blink[2] = 0;
               indi_blink[3] = 0;
               neon_blink_state = 2;
               mode = SET_HOURS;
           }
           else if (is_mode_pressed()) {
               mode = SET_12_24;
               
           }
           break;
       case SET_HOURS:
           if (is_mode_pressed()) {
               indi_blink[0] = 0;
               indi_blink[1] = 0;
               indi_blink[2] = 1;
               indi_blink[3] = 1;
               mode = SET_MINUTES;
           }
           break;
       case SET_MINUTES:
           if (is_mode_pressed()) {
               //set time
               mode = SET_12_24;
           }
           break;
       case SET_12_24:
           indi[2] = 0x00;
           indi[3] = 0x00;
           indi_blink[0] = 0;
           indi_blink[1] = 0;
           indi_blink[2] = 0;
           indi_blink[3] = 0;
           neon_blink_state = 1;
           set_indicator(0,1);
           set_indicator(1,2);
           if (is_mode_pressed()) {
               indi[0] = 0x00;
               indi[1] = 0x00;
               mode = SET_LED;
           }
           break;
       case SET_LED:
           set_indicator(3,cp.led_mode);
           if (is_set_pressed()) {
              // if (cp.led_mode == 2) { cp.led_mode = 0; }
              // else { cp.led_mode++; }
           }
           else if (is_mode_pressed()) {
               indi[3] = 0x00;
               mode = SET_BUZZER;
           }
           break;
       case SET_BUZZER:
           set_indicator(2,cp.buzzer_mode);
           if (is_mode_pressed()) {
               indi[2] = 0x00;
               mode = SET_LAMP_TRAIN;
           }
           break;
       case SET_LAMP_TRAIN:
           set_indicator(1,cp.lamp_training_mode);
           if (is_mode_pressed()) {
               indi[1] = 0x00;
               indi_blink[2] = 1;
               indi_blink[3] = 1;
               mode = SET_YEAR;
           }
           break;
       case SET_YEAR:
           set_indicator(2,year/10);
           set_indicator(3,year%10);
           if (is_mode_pressed()) {
               mode = SET_MONTH;
           }
           break;
       case SET_MONTH:
           set_indicator(0,day/10);
           set_indicator(1,day%10);
           set_indicator(2,month/10);
           set_indicator(3,month%10);
           
           if (is_mode_pressed()) {
               indi_blink[0] = 1;
               indi_blink[1] = 1;
               indi_blink[2] = 0;
               indi_blink[3] = 0;
               mode = SET_DAY;
           }
           break;
       case SET_DAY:
           if (is_mode_pressed()) {
               indi_blink[0] = 0;
               indi_blink[1] = 0;
               indi[1] = 0x00;
               indi[2] = 0x00;
               indi[3] = 0x00;
               mode = SET_CALENDAR;
           }
           break;
       case SET_CALENDAR:
           set_indicator(0,cp.calendar_mode);
           if (is_mode_pressed()) {
               mode_hold_timer = 0;
               mode = CURRENT_TIME;
           }
           break;
       default:
           break;
       }

      
      /* -------------------------- Buzzer ------------------------ */
      /* todo: timer overflow */
      /*if (buzzer_timer > ticks) { PORTD |= (1 << BUZZER_PIN); }
      else  { PORTD &= ~(1 << BUZZER_PIN); }*/
         
      if (blink_timer > BLINK_TIME) {
          blink_timer = 0;
          blink_state = !blink_state;
      }
      
      switch (neon_blink_state) {
      case 2:
          PORTD |= (1 << NEON_PIN);
          break;
      case 1:
          PORTD &= ~(1 << NEON_PIN);
          break;
      case 3:
          if (!blink_state) { PORTD |= (1 << NEON_PIN); }
          else  { PORTD &= ~(1 << NEON_PIN); }
          break;
      default:
          break;
      }
      
      /* -------------------------- Display ----------------------- */
      for (int i = 0; i < 4; ++i) {
          if (!blink_state || !indi_blink[i]) { buf[i] = indi[i]; }
          else { buf[i] = 0x00; }
      }

      if (!set_button_state) { set_button_flag = 1; }
      if (!mode_button_state) { mode_button_flag = 1; }

      _delay_us(50);

    }
    
    return 0;
}

// ----------------------- Interrupts ---------------------------
ISR(TIMER0_COMPA_vect) {
    neon_timer++;
    blink_timer++;
    buzzer_timer++;
    date_timer++;
    mode_hold_timer++;
}

ISR(TIMER1_COMPA_vect) {
    PORTB = 0x00;
    PORTB = buf[digit];
    digit++;
    if (digit > 3) { digit = 0; }
}
