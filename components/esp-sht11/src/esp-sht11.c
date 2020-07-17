#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/rtc_io.h"
#include "esp-sht11.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOW               0x0
#define HIGH              0x1

//GPIO FUNCTIONS
#define INPUT             0x01
#define OUTPUT            0x02
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define SPECIAL           0xF0
#define FUNCTION_1        0x00
#define FUNCTION_2        0x20
#define FUNCTION_3        0x40
#define FUNCTION_4        0x60
#define FUNCTION_5        0x80
#define FUNCTION_6        0xA0

#define ESP_REG(addr) *((volatile uint32_t *)(addr))

#ifndef FCPU80
#define FCPU80 80000000L
#endif

#define noACK 0
#define ACK   1
                             //adr   command       r/w
#define STATUS_REG_W  0x06   //000    0011          0
#define STATUS_REG_R  0x07   //000    0011          1
#define MEASURE_TEMP  0x03   //000    0001          1
#define MEASURE_HUMI  0x05   //000    0010          1
#define RESET         0x1E   //000    1111          0

enum {
    TEMP,
    HUMI
};

const uint8_t pin_to_mux[40] = { 0x44, 0x88, 0x40, 0x84, 0x48, 0x6c, 0x60, 0x64, 0x68, 0x54, 0x58, 0x5c, 0x34, 0x38, 0x30, 0x3c, 0x4c, 0x50, 0x70, 0x74, 0x78, 0x7c, 0x80, 0x8c, 0, 0x24, 0x28, 0x2c, 0, 0, 0, 0, 0x1c, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0c, 0x10};
static const char *TAG = "sht11";

static uint8_t sht11_dcount = 18;
static uint8_t sht11_sda, sht11_scl;
static bool do_log = false;

static void pinMode(uint8_t pin, uint8_t mode)
{
    if(pin >= 40) {
        return;
    }

    uint32_t rtc_reg = rtc_gpio_desc[pin].reg;

    //RTC pins PULL settings
    if(rtc_reg) {
        //lock rtc
        ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].mux);
        if(mode & PULLUP) {
            ESP_REG(rtc_reg) = (ESP_REG(rtc_reg) | rtc_gpio_desc[pin].pullup) & ~(rtc_gpio_desc[pin].pulldown);
        } else if(mode & PULLDOWN) {
            ESP_REG(rtc_reg) = (ESP_REG(rtc_reg) | rtc_gpio_desc[pin].pulldown) & ~(rtc_gpio_desc[pin].pullup);
        } else {
            ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].pullup | rtc_gpio_desc[pin].pulldown);
        }
        //unlock rtc
    }

    uint32_t pinFunction = 0, pinControl = 0;

    //lock gpio
    if(mode & INPUT) {
        if(pin < 32) {
            GPIO.enable_w1tc = BIT(pin);
        } else {
            GPIO.enable1_w1tc.val = BIT(pin - 32);
        }
    } else if(mode & OUTPUT) {
        if(pin > 33) {
            //unlock gpio
            return;//pins above 33 can be only inputs
        } else if(pin < 32) {
            GPIO.enable_w1ts = BIT(pin);
        } else {
            GPIO.enable1_w1ts.val = BIT(pin - 32);
        }
    }

    if(mode & PULLUP) {
        pinFunction |= FUN_PU;
    } else if(mode & PULLDOWN) {
        pinFunction |= FUN_PD;
    }

    pinFunction |= ((uint32_t)2 << FUN_DRV_S);//what are the drivers?
    pinFunction |= FUN_IE;//input enable but required for output as well?

    if(mode & (INPUT | OUTPUT)) {
        pinFunction |= ((uint32_t)2 << MCU_SEL_S);
    } else if(mode == SPECIAL) {
        pinFunction |= ((uint32_t)(((pin)==1||(pin)==3)?0:1) << MCU_SEL_S);
    } else {
        pinFunction |= ((uint32_t)(mode >> 5) << MCU_SEL_S);
    }

    ESP_REG(DR_REG_IO_MUX_BASE + pin_to_mux[pin]) = pinFunction;

    if(mode & OPEN_DRAIN) {
        pinControl = (1 << GPIO_PIN0_PAD_DRIVER_S);
    }

    GPIO.pin[pin].val = pinControl;
    //unlock gpio
}

static void digitalWrite(uint8_t pin, uint8_t val)
{
    if(val) {
        if(pin < 32) {
            GPIO.out_w1ts = BIT(pin);
        } else if(pin < 34) {
            GPIO.out1_w1ts.val = BIT(pin - 32);
        }
    } else {
        if(pin < 32) {
            GPIO.out_w1tc = BIT(pin);
        } else if(pin < 34) {
            GPIO.out1_w1tc.val = BIT(pin - 32);
        }
    }
}



static inline void SDA_LOW()
{
    //Enable SDA (becomes output and since GPO is 0 for the pin,
    // it will pull the line low)
    if (sht11_sda < 32) {
        GPIO.enable_w1ts = BIT(sht11_sda);
    } else {
        GPIO.enable1_w1ts.val = BIT(sht11_sda - 32);
    }
}

static inline void SDA_HIGH()
{
    //Disable SDA (becomes input and since it has pullup it will go high)
    if (sht11_sda < 32) {
        GPIO.enable_w1tc = BIT(sht11_sda);
    } else {
        GPIO.enable1_w1tc.val = BIT(sht11_sda - 32);
    }
}

static inline uint32_t SDA_READ()
{
    if (sht11_sda < 32) {
        return (GPIO.in & BIT(sht11_sda)) != 0;
    } else {
        return (GPIO.in1.val & BIT(sht11_sda - 32)) != 0;
    }
}

static void SCL_LOW()
{
    if (sht11_scl < 32) {
        GPIO.enable_w1ts = BIT(sht11_scl);
    } else {
        GPIO.enable1_w1ts.val = BIT(sht11_scl - 32);
    }
}

static void SCL_HIGH()
{
    if (sht11_scl < 32) {
        GPIO.enable_w1tc = BIT(sht11_scl);
    } else {
        GPIO.enable1_w1tc.val = BIT(sht11_scl - 32);
    }
}

static uint32_t SCL_READ()
{
    if (sht11_scl < 32) {
        return (GPIO.in & BIT(sht11_scl)) != 0;
    } else {
        return (GPIO.in1.val & BIT(sht11_scl - 32)) != 0;
    }
}




void sht11_setClock(unsigned int freq)
{
#if F_CPU == FCPU80
    if(freq <= 100000) {
        sht11_dcount = 19;    //about 100KHz
    } else if(freq <= 200000) {
        sht11_dcount = 8;    //about 200KHz
    } else if(freq <= 300000) {
        sht11_dcount = 3;    //about 300KHz
    } else if(freq <= 400000) {
        sht11_dcount = 1;    //about 400KHz
    } else {
        sht11_dcount = 1;    //about 400KHz
    }
#else
    if(freq <= 100000) {
        sht11_dcount = 64;    //about 100KHz
    } else if(freq <= 200000) {
        sht11_dcount = 28;    //about 200KHz
    } else if(freq <= 300000) {
        sht11_dcount = 16;    //about 300KHz
    } else if(freq <= 400000) {
        sht11_dcount = 10;    //about 400KHz
    } else if(freq <= 500000) {
        sht11_dcount = 6;    //about 500KHz
    } else if(freq <= 600000) {
        sht11_dcount = 4;    //about 600KHz
    } else {
        sht11_dcount = 2;    //about 700KHz
    }
#endif
}

static void sht11_pins_init(uint8_t sda, uint8_t scl)
{
    sht11_sda = sda;
    sht11_scl = scl;
    pinMode(sht11_sda, OUTPUT);
    pinMode(sht11_scl, OUTPUT);

    digitalWrite(sht11_sda, 0);
    digitalWrite(sht11_scl, 0);

    pinMode(sht11_sda, INPUT_PULLUP);
    pinMode(sht11_scl, INPUT_PULLUP);
    sht11_setClock(200000);
    SCL_HIGH();
    SDA_HIGH();
}

static void sht11_pins_stop(void)
{
    pinMode(sht11_sda, INPUT);
    pinMode(sht11_scl, INPUT);
}

static void sht11_delay(uint8_t v)
{
    unsigned int i;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    unsigned int reg;
    for(i=0; i<v; i++) {
        reg = REG_READ(GPIO_IN_REG);
    }
#pragma GCC diagnostic pop
}

static void sht11_sleep(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

//----------------------------------------------------------------------------------
static char sht11_write_byte(uint8_t value)
//----------------------------------------------------------------------------------
// writes a byte on the Sensibus and checks the acknowledge
{
    uint8_t i, error=0;

    for (i = 0x80; i > 0; i /= 2) {            //shift bit for masking
        if (i & value)
            SDA_HIGH();                        //masking value with i , write to SENSI-BUS
        else
            SDA_LOW();
        sht11_delay(sht11_dcount);
        SCL_HIGH();                            //clk for SENSI-BUS
        sht11_delay(sht11_dcount);             //pulswith approx. 5 us
        SCL_LOW();
    }
    SDA_HIGH();                               //release DATA-line
    sht11_delay(sht11_dcount);
    SCL_HIGH();                               //clk #9 for ack
    sht11_delay(sht11_dcount);
    error = SDA_READ();                       //check ack (DATA will be pulled down by SHT11)
    SCL_LOW();
    sht11_delay(sht11_dcount);

    return error;                             //error=1 in case of no acknowledge
}
//----------------------------------------------------------------------------------
static char sht11_read_byte(uint8_t ack)
//----------------------------------------------------------------------------------
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
{
    uint8_t i,val=0;

    SDA_HIGH();                               //release DATA-line
    for (i = 0x80; i > 0; i /= 2)             //shift bit for masking
    {
        SCL_HIGH();                           //clk for SENSI-BUS
        sht11_delay(sht11_dcount);
        if (SDA_READ())
            val=(val | i);                    //read bit
        SCL_LOW();
        sht11_delay(sht11_dcount);
    }
    //in case of "ack==1" pull down DATA-Line
    if(!ack)
        SDA_HIGH();
    else
        SDA_LOW();
    SCL_HIGH();                            //clk #9 for ack
    sht11_delay(sht11_dcount);             //pulswith approx. 5 us
    SCL_LOW();
    sht11_delay(sht11_dcount);
    SDA_HIGH();                           //release DATA-line
    sht11_delay(sht11_dcount);
    return val;
}

//----------------------------------------------------------------------------------
static void sht11_trans_start(void)
//----------------------------------------------------------------------------------
// generates a transmission start
//
//       _____          ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
{
    SDA_HIGH();
    SCL_LOW();          //Initial state
    sht11_delay(sht11_dcount);
    SCL_HIGH();
    sht11_delay(sht11_dcount);
    SDA_LOW();
    sht11_delay(sht11_dcount);
    SCL_LOW();
    sht11_delay(sht11_dcount * 2);
     SCL_HIGH();
    sht11_delay(sht11_dcount);
    SDA_HIGH();
    sht11_delay(sht11_dcount);
    SCL_LOW();
}
//----------------------------------------------------------------------------------
static void sht11_connection_reset(void)
//----------------------------------------------------------------------------------
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________        ________
// DATA:                                                      |_______|

//         _    _    _    _    _    _    _    _    _        ___     ___
// SCK :__| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______

{
    uint8_t i;

    SDA_HIGH();
    SCL_LOW();                          //Initial state
    sht11_delay(sht11_dcount);
    for(i = 0; i < 9; i++) {             //9 SCK cycles
        SCL_HIGH();
        sht11_delay(sht11_dcount);
        SCL_LOW();
        sht11_delay(sht11_dcount);
    }
    sht11_trans_start();               //transmission start
}
//----------------------------------------------------------------------------------
static uint8_t sht11_soft_reset(void)
//----------------------------------------------------------------------------------
// resets the sensor by a softreset
{
    uint8_t error = 0;

    sht11_connection_reset();              //reset communication
    error += sht11_write_byte(RESET);      //send RESET-command to sensor

    return error;                          //error=1 in case of no response form the sensor
}
//----------------------------------------------------------------------------------
static uint8_t sht11_read_status_reg(uint8_t *p_value, uint8_t *p_checksum)
//----------------------------------------------------------------------------------
// reads the status register with checksum (8-bit)
{
    uint8_t error = 0;

    sht11_trans_start();                    //transmission start
    error = sht11_write_byte(STATUS_REG_R); //send command to sensor
    *p_value = sht11_read_byte(ACK);        //read status register (8-bit)
    *p_checksum = sht11_read_byte(noACK);   //read checksum (8-bit)

    return error;                     //error=1 in case of no response form the sensor
}
//----------------------------------------------------------------------------------
static uint8_t sht11_write_status_reg(uint8_t *p_value)
//----------------------------------------------------------------------------------
// writes the status register with checksum (8-bit)
{
    uint8_t error=0;

    sht11_trans_start();                     //transmission start
    error += sht11_write_byte(STATUS_REG_W); //send command to sensor
    error += sht11_write_byte(*p_value);     //send value of status register

    return error;                            //error>=1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
static uint8_t sht11_measure(int16_t *p_value, uint8_t *p_checksum, uint8_t mode)
//----------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
{
    uint8_t error = 0, value_msb = 0, value_lsb = 0;
    uint32_t i;

    sht11_trans_start();                   //transmission start
    switch(mode) {                         //send command to sensor
        case TEMP:
            error += sht11_write_byte(MEASURE_TEMP);
            break;
        case HUMI:
            error += sht11_write_byte(MEASURE_HUMI);
            break;
        default:
            break;
    }
    for (i = 0; i < 50; i++) {
        if (SDA_READ() == 0) {
            break;  //wait until sensor has finished the measurement
        }
        sht11_sleep(5);
    }
    if (SDA_READ()) {
        error += 1;                                   // or timeout (~2 sec.) is reached
    }
    value_msb = sht11_read_byte(ACK);                 //read the first byte (MSB)
    value_lsb = sht11_read_byte(ACK);                 //read the second byte (LSB)
    *p_value = (value_msb << 8) + value_lsb;
    *p_checksum = sht11_read_byte(noACK);             //read checksum

    return error;
}
//----------------------------------------------------------------------------------------
static void calc_sth11(float *p_humidity ,float *p_temperature)
//----------------------------------------------------------------------------------------
// calculates temperature and humidity [%RH]
// input :  humi [Ticks] (12 bit)
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp
{
    const float C1 = -4.0;              // for 12 Bit
    const float C2 = +0.0405;           // for 12 Bit
    const float C3 = -0.0000028;        // for 12 Bit
    const float T1 = +0.01;             // for 14 Bit @ 5V
    const float T2 = +0.00008;          // for 14 Bit @ 5V
    float rh = *p_humidity;             // rh:      Humidity [Ticks] 12 Bit
    float t = *p_temperature;           // t:       Temperature [Ticks] 14 Bit
    float rh_lin;                       // rh_lin:  Humidity linear
    float rh_true;                      // rh_true: Temperature compensated humidity
    float t_C;                          // t_C   :  Temperature
    t_C = t * 0.01 - 40;                      //calc. temperature from ticks to
    rh_lin = C3 * rh * rh + C2 * rh + C1;     //calc. humidity from ticks to [%RH]
    rh_true = (t_C - 25) * (T1 + T2 * rh) + rh_lin;   //calc. temperature compensated humidity [%RH]
    if (rh_true > 100)
        rh_true = 100;       //cut if the value is outside of
    if (rh_true < 0.1)
        rh_true = 0.1;       //the physical possible range
    *p_temperature = t_C;    //return temperature
    *p_humidity = rh_true;   //return humidity[%RH]
}


//--------------------------------------------------------------------
static float calc_dewpoint(float h, float t)
//--------------------------------------------------------------------
// calculates dew point
// input:   humidity [%RH], temperature
// output:  dew point
{
    float logEx, dew_point;

    logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
    dew_point = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);

    return dew_point;
}


void esp_sht11_init(int sda, int scl)
{
    uint8_t status, checksum;

    sht11_pins_init(sda, scl);
    sht11_soft_reset();
    sht11_sleep(11);

    sht11_connection_reset();
    sht11_read_status_reg(&status, &checksum);
    ESP_LOGI(TAG, "get status value 0x%02x, checksum 0x%02x", status, checksum);
    if (status & 0x01) {
        status = 0x00;
        sht11_write_status_reg(&status);
    }

}

void esp_sht11_stop()
{
    sht11_pins_stop();
}

//----------------------------------------------------------------------------------
void sht11_get_value(float *humidity, float *temperature)
//----------------------------------------------------------------------------------
// sample program that shows how to use SHT11 functions
// 1. connection reset
// 2. measure humidity [ticks](12 bit) and temperature [ticks](14 bit)
// 3. calculate humidity [%RH] and temperature
// 4. calculate dew point
// 5. print temperature, humidity, dew point
{
    uint8_t error, checksum;
    int16_t humidity_i, temperature_i;

    sht11_connection_reset();

    error = 0;
    error += sht11_measure(&humidity_i, &checksum, HUMI);  //measure humidity
    ESP_LOGD(TAG, "get humidity value 0x%04x, checksum 0x%02x, error %d", humidity_i, checksum, error);
    error += sht11_measure(&temperature_i, &checksum, TEMP);  //measure temperature
    ESP_LOGD(TAG, "get temperature value 0x%04x, checksum 0x%02x, error %d", temperature_i, checksum, error);
    if(error != 0) {
        ESP_LOGE(TAG, "get value error %d", error);
        sht11_connection_reset();                 //in case of an error: connection reset
    } else {
        *humidity = (float)humidity_i;
        *temperature = (float)temperature_i;
        calc_sth11(humidity, temperature);            //calculate humidity, temperature

        ESP_LOGI(TAG, "humi %f, temp %f", *humidity, *temperature);
    }
}

float sht11_calc_dewpoint(float h, float t)
{
    return calc_dewpoint(h, t);
}