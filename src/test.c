#define F_CPU 8000000UL

//#define TEST_INDICATOR

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

#define I2C_SCL_PIN    PA0
#define I2C_SDA_PIN    PA1

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

//System functions
#define nop() asm volatile ("nop")

void dummyloop(unsigned int timetoloop)
{ 
    //PORTD ^= (1 << LED_PIN);
    _delay_us(5);
  	/*while (timetoloop>0) 
	{
	nop();
	timetoloop--;
  	}*/
}

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

#define USI_DELAY (50)
#define USISR_8BIT 0xF0
#define USISR_1BIT 0xFE

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

uint8_t get_counter() {
    uint8_t counter = 0x0F & USISR;
    return counter;
}
void i2c_init() {
    //USICR = (1 << USIWM1) | (1 << USICS1); //2-wire mode & external positive edge
    
    //SDA
    DDRA |= (1 << I2C_SDA_PIN); //SDA as output
      
    //SCL 
    DDRA |= (1 << I2C_SCL_PIN); //SCL as output
        
    USICR = (0 << USISIE) | (0 << USIOIE) |
            (1 << USIWM1) | (0 << USIWM0) |
            (1 << USICS1) | (0 << USICS0) |
            (1 << USICLK) | (0 << USITC);
            
    USISR = (1 << USISIF) | (1 << USIOIF) |
            (1 << USIPF)  | (1 << USIDC)  |
            (0x0 << USICNT0);

    PORTA |= (1 << I2C_SDA_PIN); //SDA high
    PORTA |= (1 << I2C_SCL_PIN); //SCL high
}

void i2c_start() {
    
    PORTA |= (1 << I2C_SDA_PIN); // Выставляем исходное состояние SDA=1 на всякий случай
    PORTA |= (1 << I2C_SCL_PIN); // Выставляем исходное состояние SCL=1 на всякий случай
    
    DDRA  &= ~(1 << I2C_SCL_PIN); // Отключаем SCL от выходного буфера интерфейса, тоесть переводим его в input
    
    USISR = (1 << USISIF) | (1 << USIOIF) | // Сбрасываем USISR
            (1 << USIPF)  | (1 << USIDC)  |
            (0x0 << USICNT0);
    
    PORTA &= ~(1 << I2C_SDA_PIN); // Переводим SDA=0 пока SCL=1
    
    dummyloop(USI_DELAY);	
    
    DDRA  |= (1 << I2C_SCL_PIN); // Подключаем SCL обратно к выходному буферу интерфейса
    PORTA &= ~(1 << I2C_SCL_PIN); // Переводим SCL=0
    PORTA |= (1 << I2C_SDA_PIN); // SDA=1, Освобождаем линию данных для последующего приёма, передачи даных
    
    dummyloop(USI_DELAY);
    
}

void i2c_stop() {
    
    PORTA &= ~(1 << I2C_SCL_PIN);
    dummyloop(USI_DELAY);

    PORTA &= ~(1 << I2C_SDA_PIN);
    dummyloop(USI_DELAY);

    PORTA |= (1 << I2C_SDA_PIN);
    dummyloop(USI_DELAY);

    PORTA |= (1 << I2C_SCL_PIN);
    dummyloop(USI_DELAY);

    USISR |= (1 << USIPF);
}
    
uint8_t i2c_transfer(uint8_t tr) {
    
    USISR = tr;
    
    tr = (0 << USISIE) | (0 << USIOIE) |  //Заготовка для USICR
         (1 << USIWM1) | (0 << USIWM0) |
         (1 << USICS1) | (0 << USICS0) |
         (1 << USICLK) | (1 << USITC);    //USITC=1 чтобы сдвинуть регистр и перевернуть SCL
    
    

    do {
        dummyloop(USI_DELAY);
        USICR=tr;                        // Поидее это должно перевернуть SCL 
        indi[1] = digits[0x0F & USISR + 10];
        while(bit_is_clear(PINA,I2C_SCL_PIN)) { } //Wait for setting SCL
        view_state();
        dummyloop(USI_DELAY);
        USICR=tr;                        // Это должно перевернуть SCL
        
    } while (!(USISR&(1<<USIOIF))); // Пока не переполнился счётчик
    
    dummyloop(USI_DELAY);
    
    tr = USIDR;
    USIDR=0xff;
    DDRA |= (1 << I2C_SDA_PIN);
    
    return tr;    
    
}
    
uint8_t i2c_write(uint8_t *data, uint8_t data_size) {
    
    //if (PINA & (1 << I2C_SCL_PIN)) {return 1;}
    if (bit_is_set(PINA, I2C_SCL_PIN)) { return 1;}    

    do {
        USIDR=(*(data++));   // Предварительно загружаем регистр данными
        
        i2c_transfer(USISR_8BIT);  // Отправляем данные
        
        DDRA &= ~(1 << I2C_SDA_PIN);         //Switch to input
        if (i2c_transfer(USISR_1BIT)&0x01) {return 2;}  // Получаем подтверждение о полученнии
        data_size--;
    } while (data_size>0);

    return 0;
    
    //USIDR = data;
    //USISR = (1 << USIOIF); // Сброс флага прерывания USI
    //while (!(USISR & (1 << USIOIF))) {
    //    USICR |= (1 << USITC);
    //}
}

uint8_t i2c_read(uint8_t *data, uint8_t data_size) {
    
    if (PINA & (1 << I2C_SCL_PIN)) {return 1;}
    do {
        DDRA |= (1 << I2C_SDA_PIN);
        *(data++) = i2c_transfer(USISR_8BIT);
        
        if (data_size==1) { USIDR = 0xFF; }
        else { USIDR = 0x00; }
        i2c_transfer(USISR_1BIT);
        data_size--;
    } while (data_size>0);
    return 0;
    //USISR = (1 << USIOIF); // Сброс флага прерывания USI
    //while (!(USISR & (1 << USIOIF))) {
    //    USICR |= (1 << USITC);
    //}
    //return USIDR;
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
    
    
    uint8_t write_buf[4];
    write_buf[0] = RTC_WADDR;
    write_buf[1] = RTC_CONTROL_REGISTER;
    write_buf[2] = 0x20;
    write_buf[3] = 0x08;
    
    i2c_start();
    
    dummyloop(USI_DELAY);

    switch (i2c_write(write_buf,4)) {

        case 1: {		// Error SCL
            return 1;
        }
        case 2: {		// NASC
            return 2;
        };
        default : {		// Ok
            break;
        };
    };
    
    dummyloop(USI_DELAY);

    i2c_stop();

    return 0;
    //_delay_ms(1500); 
    //PORTD^= (1 << LED_PIN);

}

uint8_t rtc3231a_read_time(struct rtc_time *time) {
    uint8_t write_buf[3];
    write_buf[0] = RTC_WADDR;
    write_buf[1] = 0x00;
    
    //Check if SDA == 0 and SCL == 0, then continue
    //if (!(PINA & (1 << I2C_SCL_PIN)) || !(PINA & (1 << I2C_SDA_PIN))) {return 1;}
    view_state();
    if ((bit_is_clear(PINA,I2C_SCL_PIN))||(bit_is_clear(PINA, I2C_SDA_PIN))) {return 1;}
    view_state();
    i2c_start();
    view_state();
    dummyloop(USI_DELAY);
    view_state();
    switch (i2c_write(write_buf,2)) {
    
	case 1: {
		return (2); // Error SCL
	}
	case 2: {
		return (3); // NASC
	}
	default: {
		break; // Ok
	}
    }	
    write_buf[0] = RTC_RADDR;
    dummyloop(USI_DELAY);
    view_state();
    i2c_start();
    view_state();
    switch (i2c_write(write_buf,1)) {

	case 1: {
		return(4); //Error SCL
	}
	case 2: {
		return(5); // NASC
	}
	default: {
		break; // Ok
	}
    }
    dummyloop(USI_DELAY);
    view_state();
    switch (i2c_read(write_buf, 3)) {

	case 1: {
		return (6);
	}
	default: {
		break;
	}
    }
    dummyloop(USI_DELAY);
    view_state();
    i2c_stop();
view_state();
    
    time->sec = bcd(write_buf[0]);
    time->min = bcd(write_buf[1]);
    time->hour = bcd(write_buf[2]);
    
    return(0);

    /*i2c_write(RTC_WADDR);
    i2c_write(0x00);
    i2c_write(RTC_RADDR);
    time->sec = bcd(i2c_read());
    time->min = bcd(i2c_read());
    time->hour = bcd(i2c_read());*/
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

    i2c_init();
    rtc3231a_init();
    
    while (1) {
    
#ifdef TEST_INDICATOR    
    indi[0] = digits[counter];
    indi[1] = digits[counter+10];
    indi[2] = digits[counter+20];
    indi[3] = digits[counter+30];
    counter++;
    if (counter>9) { counter = 0; }
#endif
    //indi[3] = digits[rtc3231a_read_time(&realtime)+30];
    indi[3] = digits[5+30];
    
       //rtc3231a_read_date(&realdate);
        
    	/*if (!(PIND & 0x02)) { //Не нажата ли кнопка SET
    		PORTD |= 0x07;  //Включить LED
    	}
    	if (!(PIND & 0x01)) { //Не нажата ли кнопка MODE
    		PORTD &= 0xFB;  //Выключить LED
    	} */
       
      //PORTD^= (1 << LED_PIN);
      PORTD^= (1 << NEON_PIN);
      //PORTD ^= (1 << BUZZER_PIN);
        // Пауза в 1 секунду
      /*
      indi[0] = (uint8_t)realtime.min/10;
      indi[1] = (uint8_t)realtime.min%10;
      indi[2] = (uint8_t)realtime.sec/10;
      indi[3] = (uint8_t)realtime.sec%10;
      */
      _delay_ms(1000);
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
