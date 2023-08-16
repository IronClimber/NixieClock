#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#define SET_PIN  PD1  // Set button
#define MODE_PIN PD0  // Mode button

#define LED_PIN  PD2  // LEDs

#define NEON_PIN PD5  // Neon light

#define A0_PIN   PB3
#define A1_PIN   PB1
#define A2_PIN   PB0
#define A3_PIN   PB2

#define AL1_PIN  PB7
#define AL2_PIN  PB6
#define AL3_PIN  PB5
#define AL4_PIN  PB4

//System functions

//Interrupt timer init
void timer1_init() {
    //Set timer 1 to CTC mode (Clear Timer on Compare
    TCCR1B = (1 << WGM12);
    
    // Установка предделителя таймера 1 в 256 (
    TCCR1B |= (1 << CS12);

    // Установка начального значения счетчика таймера 0 (для 1 секунды при F_CPU=8МГц)
    //TCNT1 = 65536 - 60000; // 256 - (8000000 / 1024 / 2)

    //OCR - Output Compare Register
    OCR1A = 110;

    // Разрешение прерывания по переполнению таймера 0
    TIMSK = (1 << OCIE1A);
}

//I2C (TWI library)
void i2c_init() {
    USICR = (1 << USIWM1) | (1 << USICS1); //2-wire mode
}
void i2c_write(uint8_t data) {
    USIDR = data;
    USISR = (1 << USIOIF); // Сброс флага прерывания USI
    while (!(USISR & (1 << USIOIF))) {
        USICR |= (1 << USITC);
    }
}

uint8_t i2c_read() {
    USISR = (1 << USIOIF); // Сброс флага прерывания USI
    while (!(USISR & (1 << USIOIF))) {
        USICR |= (1 << USITC);
    }
    return USIDR;
}

// RTC
#define RTC_WADDR 0b11010000
#define RTC_RADDR 0b11010001

#define RTC_CONTROL_REGISTER 0x0E


struct rtc_time
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
};

struct rtc_date
{
	uint8_t wday;
	uint8_t day;
	uint8_t month;
	uint8_t year;
};

static uint8_t bcd (uint8_t data) {
    uint8_t bc;
    bc = ((((data & (1 << 6)) | (data & (1 << 5)) | (data & (1 << 4)))*0x0A) >> 4)
	+ ((data & (1 << 3))|(data & (1 << 2))|(data & (1 << 1))|(data & 0x01));
    return bc;
}

//rtc_time realtime;
//rtc_date realdate;

void rtc3231a_init(void) {
    i2c_write(RTC_WADDR);
    i2c_write(RTC_CONTROL_REGISTER);
    i2c_write(0x20);
    i2c_write(0x08);
}

void rtc3231a_read_time(struct rtc_time *time) {
    i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_write(RTC_RADDR);
    time->sec = bcd(i2c_read());
    time->min = bcd(i2c_read());
    time->hour = bcd(i2c_read());
}

void rtc3231a_read_date(struct rtc_date *date) {
    i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_write(RTC_RADDR);
    bcd(i2c_read());
    bcd(i2c_read());
    bcd(i2c_read());
    date->wday = bcd(i2c_read());
    date->day = bcd(i2c_read());
    date->month = bcd(i2c_read());
    date->year = bcd(i2c_read());
}


//Indicator
uint8_t indi[4] = { 0x00 };

uint8_t digit = 0;
uint8_t counter = 0;
uint8_t digits[] = {
 0x8A, 0x81, 0x83, 0x89, 0x82, 0x88, 0x8C, 0x80, 0x84, 0x8B,
 0x4A, 0x41, 0x43, 0x49, 0x42, 0x48, 0x4C, 0x40, 0x44, 0x4B,
 0x2A, 0x21, 0x23, 0x29, 0x22, 0x28, 0x2C, 0x20, 0x24, 0x2B,
 0x1A, 0x11, 0x13, 0x19, 0x12, 0x18, 0x1C, 0x10, 0x14, 0x1B
};

int main(void) {

    // Настройка пина светодиода как выхода
    DDRD |= (1 << LED_PIN);
    // Настройка пина
    DDRD |= (1 << NEON_PIN);
    // Init buttons
    PORTD = 0x00;
    // Init lamps
    DDRB = 0xFF;
    PORTB = 0x00;
    
    //i2c_init();
    
    timer1_init();
    
    sei();
    
    indi[0] = digits[8];
    indi[1] = digits[8+10];
    indi[2] = digits[9+20];
    indi[3] = digits[1+30];
    
    while (1) {
    	/*if (!(PIND & 0x02)) { //Не нажата ли кнопка SET
    		PORTD |= 0x07;  //Включить LED
    	}
    	if (!(PIND & 0x01)) { //Не нажата ли кнопка MODE
    		PORTD &= 0xFB;  //Выключить LED
    	} */
       
      //PORTD^= (1 << LED_PIN);
      //PORTD^= (1 << NEON_PIN);
        // Пауза в 1 секунду
      //_delay_ms(1000);
    }

    return 0;
}

ISR(TIMER1_COMPA_vect) {
    
    PORTB = 0x00;
    PORTB = indi[digit];
    digit++;
    if (digit > 3) { digit = 0; }

    

     
    //PORTB = 0x21;
    
    //PORTB^=(1 << AL1_PIN);
    //
    //PORTB^=(1 << A1_PIN);
    //PORTB^=(1 << A2_PIN);
    //PORTB^=(1 << A3_PIN);

  
}
