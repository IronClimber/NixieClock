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
uint8_t blink_time = NEON_TIMER;

#define INDI0_BLINK_MASK 0x01
#define INDI1_BLINK_MASK 0x02
#define INDI2_BLINK_MASK 0x04
#define INDI3_BLINK_MASK 0x08
#define INDI_BLINK_MASK  0x0F
#define INDI01_BLINK_MASK 0x03
#define INDI23_BLINK_MASK 0x0C

#define BLINK_STATE_MASK      0x80
#define NEON_BLINK_STATE_MASK 0x40
#define NEON_STATE            0x20

#define TIME_MASK  0x10
#define DATE_MASK  0x20
/*
1 - indi 0
2 - indi 1
3 - indi 2
4 - indi 3
5 - time
6 - date
7 - neon_blinking_state
8 - blink_state
*/
uint8_t flags = 0xF0;


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

uint8_t bin(uint8_t num) {
return ((num % 10) | ((num / 10) % 10) << 4);
/*
	uint8_t bcd;
	uint8_t n, dig, count;

	count = 0;
	bcd = 0;

	for (n = 0; n < 4; n++) {
		dig = num % 10;
		num = num / 10;
		bcd = (dig << count) | bcd;
		count += 4;
	}
	
	return bcd;
	*/
}


void rtc3231a_read_time() {
   
    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(0x01);
    i2c_stop();
    i2c_start();
    i2c_write(RTC_RADDR);
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
#define INIT_VALUE 0xCE

#define INIT_VALUE_ADDRESS            0x00
#define TIME_FORMAT_ADDRESS           0x01
#define LED_MODE_ADDRESS              0x02
#define BUZZER_MODE_ADDRESS           0x03
#define LAMP_TRAINING_MODE_ADDRESS    0x04
#define CALENDAR_MODE_ADDRESS         0x05

// ------------------------- Params -------------------------------
#define TIME_FORMAT_DEFAULT           1
#define LED_MODE_DEFAULT              0
#define BUZZER_MODE_DEFAULT           0
#define LAMP_TRAINING_MODE_DEFAULT    1
#define CALENDAR_MODE_DEFAULT         0

#define TIME_FORMAT_MAX               1
#define LED_MODE_MAX                  2
#define BUZZER_MODE_MAX               2
#define LAMP_TRAINING_MODE_MAX        1
#define CALENDAR_MODE_MAX             1


uint8_t time_format;              // 0 - 12, 1 - 24
uint8_t led_mode;                    // 0 - off, 1 - on (7-22), 2 - on
uint8_t buzzer_mode;              // 0 - off, 1- on (7-20), 2 - on
uint8_t lamp_training_mode;// 0 - off, 1 - on (2-4)
uint8_t calendar_mode;          // 0 - off, 1 - om


/*
7-CHECK_BIT
6-TIME_FORMAT
4-5-LED_MODE
2-3 - BUZZER_MODE
1 - LAMP_TRAINING_MODE
0 - CALENDAR_MODE
*/
/*
#define INIT_VALUE_MASK            0x80
#define TIME_FORMAT_MASK           0x50
#define LED_MODE_MASK              0x30
#define BUZZER_MODE_MASK           0x0C
#define LAMP_TRAINING_MODE_MASK    0x02
#define CALENDAR_MODE_MASK         0x01

#define INIT_VALUE_SHIFT            7
#define TIME_FORMAT_SHIFT           6
#define LED_MODE_SHIFT              4
#define BUZZER_MODE_SHIFT           2
#define LAMP_TRAINING_MODE_SHIFT    1
#define CALENDAR_MODE_SHIFT         0

uint8_t params = 0;

#define UNPACK(mask, shift) ( (params & mask) >> shift )
#define SET_PARAMS()        ( INIT_VALUE_MASK | (time_format << TIME_FORMAT_SHIFT) | (led_mode << LED_MODE_SHIFT) | (buzzer_mode << BUZZER_MODE_SHIFT) | (lamp_training_mode << LAMP_TRAINING_MODE_SHIFT) | (calendar_mode << CALENDAR_MODE_SHIFT) )
*/

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

uint8_t mode = CURRENT_TIME;
    
// --------------------------- Display ----------------------------
uint8_t indi[4] = { 0x00 };

uint8_t digit = 0;
uint8_t counter = 0;

/*
uint8_t digits[] = {
 0x8A, 0x81, 0x83, 0x89, 0x82, 0x88, 0x8C, 0x80, 0x84, 0x8B,
 0x4A, 0x41, 0x43, 0x49, 0x42, 0x48, 0x4C, 0x40, 0x44, 0x4B,
 0x2A, 0x21, 0x23, 0x29, 0x22, 0x28, 0x2C, 0x20, 0x24, 0x2B,
 0x1A, 0x11, 0x13, 0x19, 0x12, 0x18, 0x1C, 0x10, 0x14, 0x1B
};
*/

uint8_t digits[] = {0x0A, 0x01, 0x03, 0x09, 0x02, 0x08, 0x0C, 0x00, 0x04, 0x0B };

void set_indicator(uint8_t n, uint8_t value) {
    /*
    switch (n) {
    case 0: indi[0] = digits[value] + 0X80; break;
    case 1: indi[1] = digits[value] + 0X40; break;
    case 2: indi[2] = digits[value] + 0X20; break;
    case 3: indi[3] = digits[value] + 0X10; break;
    default: indi[n] = 0x00; break;
    }
    */
    
    indi[n] = digits[value] | (1 << (7-n));
}

#define PAIR_MASK0 0x00
#define PAIR_MASK1 0x02

void set_pair_indicator(uint8_t pair, uint8_t value) {
    set_indicator(0+pair,value/10);
    set_indicator(1+pair,value%10);
}

// --------------------------- Timer ------------------------------
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

#define SET_PRESSED_MASK   0x05
#define MODE_PRESSED_MASK  0x0A

#define SET_FLAG           0x01
#define MODE_FLAG          0x02
#define SET_STATE          0x04
#define MODE_STATE         0x08

#define MODE_FIRST_PRESSED 0x80

uint8_t button_flags = 0x80;

//0- uint8_t set_button_flag = 0;
//1- uint8_t mode_button_flag = 0;
//2- uint8_t set_button_state = 0;
//3- uint8_t mode_button_state = 0;


//7 - uint8_t first_mode_pressed = 1; 

uint8_t is_set_pressed() {
    //if (set_button_state && set_button_flag) { 
    if (button_flags & SET_FLAG && button_flags & SET_STATE) { 
        //set_button_flag = 0;
        button_flags &= ~SET_FLAG;
        return 1; 
    }
    return 0;
} 

uint8_t is_mode_pressed() {
    //if (mode_button_state && mode_button_flag) {
    if (button_flags & MODE_FLAG && button_flags & MODE_STATE) { 
        //mode_button_flag = 0;
        button_flags &= ~MODE_FLAG;
        return 1; 
    }
    return 0;
} 

void next_value(uint8_t* vp, uint8_t max_value, uint8_t min_value) {
    if (is_set_pressed()) {
        if (*vp == max_value) { *vp = min_value; }
        else { (*vp)++; }
    }
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
    DDRD &= ~(SET_PIN | MODE_PIN);
    //PORTD = 0x00;
    
    // Init lamps
    DDRB = 0xFF;
    PORTB = 0x00;
     

    // -------------------- Timer 0 --------------------------------
    // Установка режима CTC (Clear Timer on Compare Match)
    TCCR0A |= (1 << WGM01);
    
    // Установка предделителя (например, 64 для периода в 1 миллисекунду при тактовой частоте 8 МГц)
    TCCR0B |= (1 << CS02) | (1 << CS00);
    
    // Устанавливаем значение, при котором будет генерироваться прерывание (OCR0)
    OCR0A = 194;  // (195-1 - 1)Для периода в 1 миллисекунду при тактовой частоте 8 МГц
    
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

    i2c_start();
    i2c_write(RTC_WADDR);
    i2c_write(RTC_CONTROL_REGISTER);
    i2c_write(0x20);
    i2c_write(0x08);
    i2c_stop();
    
    // -------------- Restore params from EEPROM -
    //uint8_t eeprom_init_value = eeprom_read_byte((uint8_t*)INIT_VALUE_ADDRESS);
    //params = eeprom_read_byte((uint8_t*)INIT_VALUE_ADDRESS);
    /*if (eeprom_init_value != INIT_VALUE) {
        eeprom_update_byte(INIT_VALUE_ADDRESS, INIT_VALUE);
        eeprom_update_byte((uint8_t*)TIME_FORMAT_ADDRESS, TIME_FORMAT_DEFAULT);
        eeprom_update_byte((uint8_t*)LED_MODE_ADDRESS, LED_MODE_DEFAULT);
        eeprom_update_byte((uint8_t*)BUZZER_MODE_ADDRESS, BUZZER_MODE_DEFAULT);
        eeprom_update_byte((uint8_t*)LAMP_TRAINING_MODE_ADDRESS, LAMP_TRAINING_MODE_DEFAULT);
        eeprom_update_byte((uint8_t*)CALENDAR_MODE_ADDRESS, CALENDAR_MODE_DEFAULT);
    }*/
    
    /*if (!params) { 
        params = SET_PARAMS(); 
        eeprom_update_byte((uint8_t*)INIT_VALUE_ADDRESS, params);
    }*/
    
    time_format = eeprom_read_byte((uint8_t*)TIME_FORMAT_ADDRESS);
    led_mode = eeprom_read_byte((uint8_t*)LED_MODE_ADDRESS);
    buzzer_mode = eeprom_read_byte((uint8_t*)BUZZER_MODE_ADDRESS);
    lamp_training_mode = eeprom_read_byte((uint8_t*)LAMP_TRAINING_MODE_ADDRESS);
    calendar_mode = eeprom_read_byte((uint8_t*)CALENDAR_MODE_ADDRESS);
    
    /*time_format = UNPACK(TIME_FORMAT_MASK, TIME_FORMAT_SHIFT);
    led_mode = UNPACK(LED_MODE_MASK, LED_MODE_SHIFT);
    buzzer_mode = UNPACK(BUZZER_MODE_MASK, BUZZER_MODE_SHIFT);
    lamp_training_mode = UNPACK(LAMP_TRAINING_MODE_MASK, LAMP_TRAINING_MODE_SHIFT);
    calendar_mode = UNPACK(CALENDAR_MODE_MASK, CALENDAR_MODE_SHIFT);*/


#ifdef TEST_INDICATOR    
    for (int i = 0; i < 10; ++i) {
      indi[0] = digits[i];
      indi[1] = digits[i+10];
      indi[2] = digits[i+20];
      indi[3] = digits[i+30];
      _delay_ms(100);
    }
#endif

    blink_timer = 0;
    buzzer_timer = 0;
    date_timer = 0;

    while (1) {
        
       if (flags & TIME_MASK) { rtc3231a_read_time(); }
       if (flags & DATE_MASK) { rtc3231a_read_date(); }
       
       // ------------------- READ BUTTONS ---------------------
       if (PIND & SET_PIN) { button_flags &= ~SET_STATE; }
       else { button_flags |= SET_STATE; }
       if (PIND & MODE_PIN) { button_flags &= ~MODE_STATE; }
       else { button_flags |= MODE_STATE; }
 
       // ----------------- STATE MACHINE ---------------
       switch(mode) {
       
       //INDICATOR 0-3
       //MODE 3s hold -> Menu
       //SET press    -> Date
       case CURRENT_TIME:
           set_pair_indicator(PAIR_MASK0, hour);
           set_pair_indicator(PAIR_MASK1, min);
           if (is_set_pressed()) {
               date_timer = 0;
               mode = CURRENT_DATE;
           }
           if (button_flags & MODE_STATE) {
               if (button_flags & MODE_FIRST_PRESSED) {
                   button_flags &= ~MODE_FIRST_PRESSED;
                   mode_hold_timer = 0;
               }
               if (mode_hold_timer > 120) {    
                   button_flags &= ~MODE_FLAG;               
                   flags |= INDI_BLINK_MASK;  //0-3 - blink
                   blink_timer = BLINK_TIME;
                   //buzzer(20);                                      
                   mode = SET_TIME;
               }           
           }
           else { button_flags |= MODE_FIRST_PRESSED; }
           break;
           
       //INDICATOR 0-3
       //Waiting 5seconds and display time
       case CURRENT_DATE:
           set_pair_indicator(PAIR_MASK0, day);
           set_pair_indicator(PAIR_MASK1, month);
           if (date_timer > DATE_TIMEOUT) { mode = CURRENT_TIME; }
           break;
       
       //------------------------    
       //INDICATOR 0-3 (blinking)
       //MENU pressed -> 12-24 set
       //SET  pressed -> set hours
       case SET_TIME:
           if (is_set_pressed()) {
               PORTD &= ~(1 << NEON_PIN);
               flags &= ~(NEON_BLINK_STATE_MASK);
               flags &= ~INDI23_BLINK_MASK;
               flags &= ~TIME_MASK;
               mode = SET_HOURS;
           }
           else if (is_mode_pressed()) {
               indi[2] = 0x00;
               indi[3] = 0x00;
               PORTD |= (1 << NEON_PIN);
               flags &= ~NEON_BLINK_STATE_MASK;
               flags &= ~INDI_BLINK_MASK;
               mode = SET_12_24;
           }
           break;
       
       // --------------------------
       // INDICATOR 0-3. INDICATOR 0-1 (blinking)    
       case SET_HOURS:
           set_pair_indicator(PAIR_MASK0, hour);
           next_value(&hour, 23, 0);
           if (is_mode_pressed()) {
               flags &= ~INDI01_BLINK_MASK;
               flags |= INDI23_BLINK_MASK;
               mode = SET_MINUTES;
           }
           break;
           
       case SET_MINUTES:
           set_pair_indicator(PAIR_MASK1, min);
           next_value(&min, 59, 0);
           if (is_mode_pressed()) {
               rtc3231a_write_time();
               indi[2] = 0x00;
               indi[3] = 0x00;
               PORTD |= (1 << NEON_PIN);
               flags &= ~NEON_BLINK_STATE_MASK;
               flags &= ~INDI_BLINK_MASK;
               flags |= TIME_MASK;
               mode = SET_12_24;
           }
           break;
           
       // ----------------------------    
       // INDICATOR 0-1 (not blinking)
       
       case SET_12_24:
           if (time_format) { set_pair_indicator(PAIR_MASK0, 24); }
           else { set_pair_indicator(PAIR_MASK0, 12); }
           next_value(&time_format, TIME_FORMAT_MAX, 0);
           if (is_mode_pressed()) {
               indi[0] = 0x00;
               indi[1] = 0x00;
               mode = SET_LED;
           }
           break;
       
       // -------------------------    
       // INDICATOR 3 (not blinking)    
       case SET_LED:
           set_indicator(3,led_mode);
           next_value(&led_mode, LED_MODE_MAX, 0);
           if (is_mode_pressed()) {
               eeprom_update_byte((uint8_t*)LED_MODE_ADDRESS, led_mode);
               mode = SET_BUZZER;
           }
           break;
           
       // INDICATOR 2 (not blinking)
       case SET_BUZZER:              
           indi[3] = 0x00;
           set_indicator(2,buzzer_mode);
           next_value(&buzzer_mode, BUZZER_MODE_MAX, 0);
           
           if (is_mode_pressed()) {
               indi[2] = 0x00;
               mode = SET_LAMP_TRAIN;
           }
           break;
           
       // INDICATOR 1 (not blinking)
       case SET_LAMP_TRAIN:
           set_indicator(1,lamp_training_mode);
           next_value(&lamp_training_mode, LAMP_TRAINING_MODE_MAX, 0);
           if (is_mode_pressed()) {
               indi[1] = 0x00;
               flags &= ~DATE_MASK;
               PORTD &= ~(1 << NEON_PIN);
               flags |= INDI23_BLINK_MASK;
               mode = SET_YEAR;
           }
           break;
           
       // INDICATOR 2-3 (blinking)
       case SET_YEAR:
           set_pair_indicator(PAIR_MASK1, year);
           next_value(&year,99, 0);
           if (is_mode_pressed()) { mode = SET_MONTH; }
           break;
       case SET_MONTH:
           set_pair_indicator(PAIR_MASK0, day);
           set_pair_indicator(PAIR_MASK1, month);
           next_value(&month,12, 1); 
           if (is_mode_pressed()) {
               flags |= INDI01_BLINK_MASK;
               flags &= ~INDI23_BLINK_MASK;
               mode = SET_DAY;
           }
           break;
           
       case SET_DAY:
           set_pair_indicator(PAIR_MASK0, day);
           uint8_t max_day = 31;
           //Если высокосный год февраль -2 - (29, иначе 28)
           //1, 3, 5, 7, 8, 10, 12 -31
           //4,6,9,11
           if (month == 4 || month == 6 || month == 9 || month == 11) {
               max_day = 30; 
           }
           if (month == 2) { max_day = (year%4) ? 28 : 29; }
           next_value(&day, max_day, 1);
           if (is_mode_pressed()) {
               rtc3231a_write_date();
               flags |= DATE_MASK;
               flags &= ~INDI01_BLINK_MASK;
               indi[1] = 0x00;
               indi[2] = 0x00;
               indi[3] = 0x00;
               PORTD |= (1 << NEON_PIN);               
               mode = SET_CALENDAR;
           }
           break;
           
       case SET_CALENDAR:
           set_indicator(0,calendar_mode);
           next_value(&calendar_mode, CALENDAR_MODE_MAX, 0);
           if (is_mode_pressed()) {
               mode_hold_timer = 0;
               flags |= NEON_BLINK_STATE_MASK;               
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
         
      // ----------------- NEON --------------------
       
      if (blink_timer > blink_time) {
          blink_timer = 0;
          flags ^= BLINK_STATE_MASK;          
      }
      
      if (flags & NEON_BLINK_STATE_MASK) {
          if (flags & BLINK_STATE_MASK) { PORTD &= ~(1 << NEON_PIN); }
          else  { PORTD |= (1 << NEON_PIN); }
      }
      
      
      /* -------------------------- Display ----------------------- */
      /*for (int i = 0; i < 4; ++i) {
          if (!(flags & BLINK_STATE_MASK) || !(flags & (1 << i))) { buf[i] = indi[i]; }
          else { buf[i] = 0x00; }
      }*/

      if (!(button_flags & SET_STATE))  { button_flags |= SET_FLAG; }
      if (!(button_flags & MODE_STATE)) { button_flags |= MODE_FLAG; }

      _delay_us(50);

    }
    
    return 0;
}

// ----------------------- Interrupts ---------------------------
ISR(TIMER0_COMPA_vect) {
    blink_timer++;
    buzzer_timer++;
    date_timer++;
    mode_hold_timer++;
}

ISR(TIMER1_COMPA_vect) {
    PORTB = 0x00;
    if (!(flags & BLINK_STATE_MASK) || !(flags & (1 << digit))) { 
        PORTB = indi[digit]; 
    }
    //PORTB = buf[digit];
    digit++;
    if (digit > 3) { digit = 0; }
}
