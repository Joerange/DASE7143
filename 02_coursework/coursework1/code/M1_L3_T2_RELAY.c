#include "mraa_beaglebone_pinmap.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

mraa_i2c_context I2Chandle;
#define Slave_Addr 0x60

#define STATUS     0x00
#define OUT_P_MSB  0x01
#define OUT_P_CSB  0x02
#define OUT_P_LSB  0x03
#define CTRL_REG1  0x26
#define PT_DATA_CFG 0x13
#define LCD_ADDR 		 0x3E
mraa_i2c_context i2cp;

#define PRESSURE_THRESHOLD 1010.0f
#define PRESSURE_READY_ATTEMPTS 1000

void home_LCD (void)
{
    uint8_t buf[2] = {0x00,0x02};
    mraa_i2c_write(i2cp, buf, 2);  
}

void home2_LCD (void)
{
    uint8_t buf[] = {0x00,0xC0};
    mraa_i2c_write(i2cp, buf, 2);   
}

void LCD_Print2 (char* str)
{
    uint8_t buf[80]={0};  
    int i = 0, strl;      
    home2_LCD ();
    buf[i] = 0x40;  
    i++;
    strl = strlen((char*)str);
    for (int j = 0; j < strl; j++)
    {
        buf[i] = str[j];
        i++;
    }
    mraa_i2c_write(i2cp, buf, i);
}

void LCD_Print (uint8_t* str)
{
    uint8_t buf[80]={0};   
    uint8_t buf1[2]={0x00,0x80};	
    int32_t i = 0, strl, j=0; 
    buf[i] = 0x40;  
    i++;
    strl = strlen((char*)str);
    for (j = 0; j < strl; j++)
    {
        buf[i] = str[j];
        i++;
    }
    
    mraa_i2c_write(i2cp, buf1, 2);
    mraa_i2c_write(i2cp, buf, i);
}

void LCD_init (void)
{
    uint8_t init1[2] = {0x00,0x38};
    uint8_t init2[8] = {0x00, 0x39, 0x14,0x74,0x54,0x6f,0x0c,0x01};
    mraa_i2c_write(i2cp, init1, 2);
    mraa_i2c_write(i2cp, init2,8);
    usleep(2000);  
}

void clear_LCD (void)
{
    uint8_t buf[2] = {0x00,0x01};
    mraa_i2c_write(i2cp, buf, 2);
    usleep(2000);  
}

void toggleOneShot(void)
{
    unsigned char Value = mraa_i2c_read_byte_data(I2Chandle, CTRL_REG1); //Read current settings
    Value &= ~(1 << 1); //Clear OST bit
    mraa_i2c_write_byte_data(I2Chandle, Value, CTRL_REG1);

    Value = mraa_i2c_read_byte_data(I2Chandle, CTRL_REG1); //Read current settings to be safe
    Value |= (1 << 1); //Set OST bit
    mraa_i2c_write_byte_data(I2Chandle, Value, CTRL_REG1);
}

float ReadPressure(void)
{
    unsigned char msb =0, csb=0, lsb=0;
    
    if ((mraa_i2c_read_byte_data(I2Chandle, STATUS) & (1 << 2)) == 0) 
    {
        toggleOneShot();					//Toggle the OST bit causing the sensor to immediately take another reading
    }

    // Wait at most about one second for new pressure data.
    int attempts = 0;
    while ((mraa_i2c_read_byte_data(I2Chandle, STATUS) & (1 << 2)) == 0)
    {
        if (++attempts >= PRESSURE_READY_ATTEMPTS)
            return -1.0f;
        usleep(1000);
    }

    
    msb = mraa_i2c_read_byte_data(I2Chandle, OUT_P_MSB);
    csb = mraa_i2c_read_byte_data(I2Chandle, OUT_P_CSB);
    lsb = mraa_i2c_read_byte_data(I2Chandle, OUT_P_LSB);
    
    toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading

    // Pressure comes back as a left shifted 20 bit number
    long pressure_whole = (long)msb << 16 | (long)csb << 8 | (long)lsb;
    pressure_whole >>= 6;

    lsb &= 0x30; //Bits 5/4 represent the fractional component
    lsb >>= 4; //Get it right aligned
    float pressure_decimal = (float)lsb / 4.0; //Turn it into fraction

    float pressure = (float)pressure_whole + pressure_decimal;

    return(pressure);
}

void SetupPressureSensor(void)
{
    unsigned char tempVal;
    
    mraa_i2c_write_byte_data(I2Chandle, 0x07, PT_DATA_CFG); // Enable all three pressure and temp event flags 
    
    tempVal =  mraa_i2c_read_byte_data(I2Chandle, CTRL_REG1);
    
    // Activate Barometer mode 
    tempVal &= ~(1<<7); 		//Clear ALT bit
    tempVal |= (1<<0); 			//Set SBYB bit for Active mode
    mraa_i2c_write_byte_data(I2Chandle, tempVal, CTRL_REG1);
}

// Control the status of relay
void ControlRelay(mraa_gpio_context relay, float pressure_mbar)
{
    if (pressure_mbar < PRESSURE_THRESHOLD) {
        mraa_gpio_write(relay, 1);   // Below 1010 mBar: closed (CLO)
    } else {
        mraa_gpio_write(relay, 0);   // At or above 1010 mBar: open (OPE)
    }
}

int main()
{
    char buf[20];
    float pressure = 0;
    int status;
    mraa_gpio_context relay;
    
    if (mraa_init() != MRAA_SUCCESS)
        return EXIT_FAILURE;
    
    // Initialize the I2C LCD Bus
    i2cp = mraa_i2c_init_raw(I2CP_BUS);
    if (i2cp == NULL) {
        fprintf(stderr, "Cannot initialize LCD I2C bus.\n");
        mraa_deinit();
        return EXIT_FAILURE;
    }
    mraa_i2c_frequency(i2cp, MRAA_I2C_STD);
    mraa_i2c_address(i2cp, LCD_ADDR);
    
    // Initialize the pressure sensor
    I2Chandle = mraa_i2c_init_raw(I2CS_BUS);
    if (I2Chandle == NULL) {
        fprintf(stderr, "Cannot initialize pressure I2C bus.\n");
        mraa_i2c_stop(i2cp);
        mraa_deinit();
        return EXIT_FAILURE;
    }
    mraa_i2c_frequency(I2Chandle, MRAA_I2C_STD);
    mraa_i2c_address(I2Chandle, Slave_Addr);
    status = mraa_i2c_read_byte(I2Chandle);
    if (status < 0){
        printf("Failed to Initialize Pressure Sensor -> Exiting program\n");
        mraa_i2c_stop(I2Chandle);
        mraa_i2c_stop(i2cp);
        mraa_deinit();
        return EXIT_FAILURE;
    }
    SetupPressureSensor();
    
    // Initialize the Relay
    relay = mraa_gpio_init(RELAY_PIN);
    if (relay == NULL) {
        printf("MRAA couldn't initialize GPIO %d, exiting", RELAY_PIN);
        mraa_i2c_stop(I2Chandle);
        mraa_i2c_stop(i2cp);
        mraa_deinit();
        return MRAA_ERROR_UNSPECIFIED;
    }
    
    sleep(1); 
    
    // Set the relay pin as output
    if (mraa_gpio_dir(relay, MRAA_GPIO_OUT) != MRAA_SUCCESS) {
        printf("Can't set digital pin %d as output, exiting", RELAY_PIN);
        mraa_gpio_close(relay);
        mraa_i2c_stop(I2Chandle);
        mraa_i2c_stop(i2cp);
        mraa_deinit();
        return MRAA_ERROR_UNSPECIFIED;
    }
    
    // Initialize the LCD
    LCD_init();
    clear_LCD();
    
    
    mraa_gpio_write(relay, 0);  // Open until the first valid pressure reading.
    
    while(1)
    {
        // Read the pressure
        pressure = ReadPressure();
        if (pressure < 0.0f) {
            mraa_gpio_write(relay, 0);
            LCD_Print((uint8_t*)"Pressure error  ");
            LCD_Print2("Relay: OPE      ");
            fprintf(stderr, "Pressure data timeout; relay set to OPE.\n");
            sleep(1);
            continue;
        }
        float pressure_mbar = pressure / 100.0;
        
        ControlRelay(relay, pressure_mbar);
        int relay_state = mraa_gpio_read(relay);
        const char* relay_status = (relay_state == 0) ? "OPE" : "CLO";
        snprintf(buf, sizeof(buf), "Pressure        ");
        LCD_Print((uint8_t*)buf);
        snprintf(buf, sizeof(buf), "%6.2f mBar %s ", pressure_mbar, relay_status);
        LCD_Print2(buf);
        printf("Pressure: %.4f mBar, Relay: %s\n", pressure_mbar, relay_status);
        fflush(stdout);
        
        sleep(1);
    }
    
    // Exit and Clean Up Resources
    mraa_gpio_close(relay);
    mraa_i2c_stop(I2Chandle);
    mraa_i2c_stop(i2cp);
    mraa_deinit();
    
    return 0;
}
