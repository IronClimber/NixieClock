#define F_CPU 8000000UL

#define TEST_INDICATOR

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>

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
uint8_t indi[4] = { 0x00 };

uint8_t digit = 0;
uint8_t counter = 0;
uint8_t digits[] = {
 0x8A, 0x81, 0x83, 0x89, 0x82, 0x88, 0x8C, 0x80, 0x84, 0x8B,
 0x4A, 0x41, 0x43, 0x49, 0x42, 0x48, 0x4C, 0x40, 0x44, 0x4B,
 0x2A, 0x21, 0x23, 0x29, 0x22, 0x28, 0x2C, 0x20, 0x24, 0x2B,
 0x1A, 0x11, 0x13, 0x19, 0x12, 0x18, 0x1C, 0x10, 0x14, 0x1B
};

//Interrupt timer init
void timer1_init() {
    //Set timer 1 to CTC mode (Clear Timer on Compare
    TCCR1B = (1 << WGM12);
    
    // Установка предделителя таймера 1 в 256 (
    TCCR1B |= (1 << CS12);

    //OCR - Output Compare Register
    OCR1A = 110;

    // Разрешение прерывания по переполнению таймера 0
    TIMSK = (1 << OCIE1A);
}

//I2C (TWI library)
void view_state() {
    if (PORTA & (1 << I2C_SCL_PIN)) { indi[1] = digits[1+10]; }
    else { indi[1] = digits[0+10]; }

    if (PORTA & (1 << I2C_SDA_PIN)) { indi[3] = digits[1+30]; }
    else { indi[3] = digits[0+30]; }

    _delay_ms(2000);

    indi[1] = 0x00;
    indi[3] = 0x00;
    _delay_ms(1000);

}

void i2c_delay() {
    _delay_us(12);
}

void i2c_delay_half() {
    _delay_us(6);
}

void i2c_init() {

    DDRA |= (1 << I2C_SDA_PIN); //SDA as output
    DDRA |= (1 << I2C_SCL_PIN); //SCL as output
        
    PORTA |= (1 << I2C_SDA_PIN); //SDA high
    PORTA |= (1 << I2C_SCL_PIN); //SCL high
}

void i2c_start() {
    

    PORTA |= (1 << I2C_SDA_PIN); // Выставляем исходное состояние SDA=1 на всякий случай
    PORTA |= (1 << I2C_SCL_PIN); // Выставляем исходное состояние SCL=1 на всякий случай
    
    

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
    
uint8_t i2c_write(uint8_t byte) {
    

    // Send a byte
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            PORTA |= (1 << I2C_SDA_PIN); // Set SDA high (bit is 1)
        } else {
            PORTA &= ~(1 << I2C_SDA_PIN); // Set SDA low (bit is 0)
        }
	i2c_delay();
        

        byte <<= 1;  // Shift byte to the left
        PORTA |= (1 << I2C_SCL_PIN); // Set SCL high to clock in the bit
        
        

        i2c_delay();
        PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low

        
        
    }

    DDRA &= ~(1 << I2C_SDA_PIN);
    //PORTA &= ~(1 << I2C_SDA_PIN); // Set SDA as input ??? DDRA?
    i2c_delay();
    PORTA |= (1 << I2C_SCL_PIN);  // Set SCL high
    i2c_delay_half();
    
    
    

     // Generate an ACK
    uint8_t ack = 1;
    if (PINA & (1 << I2C_SDA_PIN)) {
        ack = 1; // NACK received
    } else {
        ack = 0; // ACK received
    }

    
    i2c_delay_half();

    
    PORTA &= ~(1 << I2C_SCL_PIN); // Set SCL low
    //PORTA |= (1 << I2C_SDA_PIN);  // Set SDA as output ??? DDRA?
    DDRA |= (1 << I2C_SDA_PIN);
    PORTA &= ~(1 << I2C_SDA_PIN);
    

    return ack;
}

uint8_t i2c_read(uint8_t ack) {
    
    uint8_t byte = 0;

    //PORTA &= ~(1 << I2C_SDA_PIN);
    DDRA &= ~(1 << I2C_SDA_PIN);
    
    for (uint8_t i = 0; i < 8; i++) {
        i2c_delay();
        PORTA |= (1 << I2C_SCL_PIN);
        i2c_delay_half();
        byte <<= 1;
        if (PINA & (1 << I2C_SDA_PIN)) {
            byte |= 0x01;
            //indi[3] = digits[1+30];
        }
        else {
            //indi[3] = digits[0+30];
        }
        i2c_delay_half();
        PORTA &= ~(1 << I2C_SCL_PIN);
        
    }

    DDRA |= (1 << I2C_SDA_PIN);
	
    if (ack) {
        PORTA |= (1 << I2C_SDA_PIN);
    }
    else {
        PORTA &= ~(1 << I2C_SDA_PIN);
    }
    i2c_delay();
    PORTA |= (1 << I2C_SCL_PIN);
    i2c_delay();
    PORTA &= ~(1 << I2C_SCL_PIN);

    return byte;

}

// RTC
#define RTC_WADDR 0xD0
#define RTC_RADDR 0xD1

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
struct rtc_time realtime;
struct rtc_date realdate;

uint8_t rtc3231a_init(void) {
       
    i2c_start();
    if (i2c_write(RTC_WADDR)) {return 1; }
    if (i2c_write(RTC_CONTROL_REGISTER)) { return 2; }
    if (i2c_write(0x20)) { return 3; }
    if (i2c_write(0x08)) { return 4; }
    i2c_stop();

    return 0;
    //_delay_ms(1500); 
    //PORTD^= (1 << LED_PIN);

}

uint8_t rtc3231a_read_time(struct rtc_time *time) {
   
    i2c_start();
    if (i2c_write(RTC_WADDR)) { return 1; }
    if (i2c_write(0x00)) { return 2; }
    i2c_stop();
    i2c_start();
    if (i2c_write(RTC_RADDR)) { return 3; }
    time->sec = bcd(i2c_read(0));
    time->min = bcd(i2c_read(0));
    time->hour = bcd(i2c_read(1));
    i2c_stop();

    return 0;
}

void rtc3231a_read_date(struct rtc_date *date) {
    /*i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_write(RTC_RADDR);
    bcd(i2c_read());
    bcd(i2c_read());
    bcd(i2c_read());
    date->wday = bcd(i2c_read());
    date->day = bcd(i2c_read());
    date->month = bcd(i2c_read());
    date->year = bcd(i2c_read());*/
}




int main(void) {

    // Настройка пина светодиода как выхода
    DDRD |= (1 << LED_PIN);
    // Настройка пина
    DDRD |= (1 << NEON_PIN);
    // Настройка пина Баззера
    DDRD |= (1 << BUZZER_PIN);
    
    // Init buttons
    PORTD = 0x00;
    
    // Init lamps
    DDRB = 0xFF;
    PORTB = 0x00;
 

    
    //PORTD^= (1 << NEON_PIN);

    timer1_init();
    sei();
    _delay_ms(3000);
    i2c_init();
    rtc3231a_init();
    //indi[3] = digits[rtc3231a_init() + 30];
    _delay_ms(50);
    //rtc3231a_read_time(&realtime);

    while (1) {
    
#ifdef TEST_INDICATOR    
    indi[0] = digits[counter];
    indi[1] = digits[counter+10];
    indi[2] = digits[counter+20];
    indi[3] = digits[counter+30];
    counter++;
    if (counter>9) { counter = 0; }

#else
    //indi[1] = digits[rtc3231a_read_time(&realtime)+10];
    //indi[3] = digits[5+30];
       
       rtc3231a_read_time(&realtime);
       //rtc3231a_read_date(&realdate);
        
    	/*if (!(PIND & 0x02)) { //Не нажата ли кнопка SET
    		PORTD |= 0x07;  //Включить LED
    	}
    	if (!(PIND & 0x01)) { //Не нажата ли кнопка MODE
    		PORTD &= 0xFB;  //Выключить LED
    	} */
    //rtc3231a_read_time(&realtime);
      //PORTD^= (1 << LED_PIN);
      PORTD^= (1 << NEON_PIN);
      //PORTD ^= (1 << BUZZER_PIN);
        // Пауза в 1 секунду
      
      indi[0] = digits[(uint8_t)realtime.min/10];
      indi[1] = digits[(uint8_t)realtime.min%10+10];
      indi[2] = digits[(uint8_t)realtime.sec/10+20];
      indi[3] = digits[(uint8_t)realtime.sec%10+30];
      
#endif
      _delay_ms(500);
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
