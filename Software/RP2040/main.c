// Capstone Mainboard Power Supply Code V0.3
// Jordan Harris-Toovy, May 2024

/* Libraries */
#include <stdio.h>
#include <time.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>

/* Turn dev mode on or off */
#define DEV_MODE true           // TODO: REMOVE: CONV TO WHEN USB CONN

/* GPIO Pin declarations */
// Communications
static const uint8_t I2C_0_SDA_PIN          = 16;           // I2C-0 (PMICs & Temp Sensor) Interface data pin [pulled up externally]
static const uint8_t I2C_0_SCL_PIN          = 17;           // I2C-0 (PMICs & Temp Sensor) Interface clock pin [pulled up externally]

// SMPS ICs
static const uint8_t PIMC_1V0_EN            = 8;            // Enable singal for the 1.0V core SMPS [pulled down externally]
static const uint8_t PIMC_1V8_EN            = 10;           // Enable singal for the 1.8V SMPS [pulled down externally]
static const uint8_t PIMC_2V5_EN            = 11;           // Enable singal for the 2.5V SMPS [pulled down externally]
static const uint8_t PIMC_3V3_EN            = 12;           // Enable singal for the 3.3V SMPS [pulled down externally]
static const uint8_t PIMC_1V0_PG            = 2;            // Power good signal from the 1.0V core SMPS [pulled up externally]
static const uint8_t PIMC_1V8_PG            = 3;            // Power good signal from the 1.8V SMPS [pulled up externally]
static const uint8_t PIMC_2V5_PG            = 4;            // Power good signal from the 2.5V SMPS [pulled up externally]
static const uint8_t PIMC_3V3_PG            = 5;            // Power good signal from the 3.3V SMPS [pulled up externally]

// Linear Reg ICs
static const uint8_t PIMC_3V3AUX_PG         = 1;            // Power good signal from the 3.3V auxiliary rail linear reg [pulled up externally]
static const uint8_t PIMC_1V0GTX_EN         = 13;           // Enable singal for the 1.0V GTX transceiver linear reg [pulled down externally]
static const uint8_t PIMC_1V2GTX_EN         = 14;           // Enable singal for the 1.2V GTX transceiver linear reg [pulled down externally]
static const uint8_t PIMC_1V8GTX_EN         = 15;           // Enable singal for the 1.8V GTX transceiver linear reg [pulled down externally]
static const uint8_t PIMC_1V0GTX_PG         = 6;            // Power good signal from the 1.0V GTX transceiver linear reg [pulled up externally]
static const uint8_t PIMC_1V2GTX_PG         = 7;            // Power good signal from the 1.2V GTX transceiver linear reg [pulled up externally]

// FPGA Control
static const uint8_t FPGA_CONFDONE          = 19;           // FPGA Configuration done indicator: high when finished !!! TODO: VALIDATE THIS !!!
static const uint8_t FPGA_INIT_CRCERR       = 20;           // FPGA Initialization done or CRC error signal: !!! TODO: ADD THIS !!!
static const uint8_t FPGA_NRESET            = 21;           // FPGA Reset (active low) signal: !!! TODO: ADD THIS !!!

// Indication
static const uint8_t IND_1_GREEN            = 23;           // TODO: Change names and add descriptions <------------------------------------------------------------
static const uint8_t IND_1_ORANGE           = 24;           // 
static const uint8_t IND_2_GREEN            = 25;           // 
static const uint8_t IND_2_ORANGE           = 26;           // 
static const uint8_t IND_3_GREEN            = 27;           // 
static const uint8_t IND_3_ORANGE           = 28;           // 

// Misc
static const uint8_t MASTER_PWR_GOOD        = 0;            // Global power status output [pulled up externally]
static const uint8_t UNUSED_PIN             = 9;            // Unused [pulled down externally]
static const uint8_t DIAG_USB_CONN          = 18;           // USB connection indicator: high when connected
static const uint8_t PWR_IN_MOD_RESERVED    = 22;           // Reserved for use with future power input module
static const uint8_t PWR_INPUT_SENSE        = 29;           // Analog signal for monitoring input voltage: proportional to unprotected input divided by 2

/* Communication parameters */
// General I2C parameters
static const uint32_t INIT_SERIAL_DELAY     = 5000;         // Delay before serial communication starts (in ms)
static const uint16_t I2C_TIMEOUT_PERIOD    = 250;          // i2c Communication hang timeout period (in ms)
static const uint32_t I2C_0_FREQ            = 100;          // I2C-0 Communication frequency (in kHz)
static const uint8_t I2C_0_DATA_BUF_LEN     = 6;            // I2C-0 Data buffer size

// PIMC I2C Addresses
static const uint8_t PMIC_1V0_ADDR          = 0x40;         // TPS62872QWRXSRQ1 PMIC address for the 1.0V rail
static const uint8_t PMIC_1V8_ADDR          = 0x41;         // TPS62871QWRXSRQ1 PMIC address for the 1.8V rail
static const uint8_t PMIC_2V5_ADDR          = 0x43;         // TPS62871QWRXSRQ1 PMIC address for the 2.5V rail
static const uint8_t PMIC_3V3_ADDR          = 0x42;         // TPS62870QWRXSRQ1 PMIC address for the 3.3V rail

// Temperature Sensor I2C Addresses
static const uint8_t TEMP_SEN_1_ADDR        = 0x90;         // Temperature sensor address for the GTX linear reg region
static const uint8_t TEMP_SEN_2_ADDR        = 0x91;         // Temperature sensor address for the 1V8 & 2V5 SMPS region
static const uint8_t TEMP_SEN_3_ADDR        = 0x92;         // Temperature sensor address for the 1V0 & 3V3 SMPS region

// PIMC I2C Design Set Values (Device Specific)
static const uint8_t PMIC_1V0_VSET_SET      = 0xF0;         // 1V0 PMIC VSET register set value
static const uint8_t PMIC_1V8_VSET_SET      = 0x64;         // 1V8 PMIC VSET register set value
static const uint8_t PMIC_2V5_VSET_SET      = 0xD2;         // 2V5 PMIC VSET register set value
static const uint8_t PMIC_3V3_VSET_SET      = 0xFA;         // 3V3 PMIC VSET register set value
static const uint8_t PMIC_1V0_CTRL2_SET     = 0b00000101;   // 1V0 PMIC CONTROL2 register set value
static const uint8_t PMIC_1V8_CTRL2_SET     = 0b00001101;   // 1V8 PMIC CONTROL2 register set value
static const uint8_t PMIC_2V5_CTRL2_SET     = 0b00001101;   // 2V5 PMIC CONTROL2 register set value
static const uint8_t PMIC_3V3_CTRL2_SET     = 0b00001101;   // 3V3 PMIC CONTROL2 register set value

// PIMC I2C Default Register Contents
static const uint8_t TPS6287X_CTRL1_DEF     = 0b00101010;   // TPS6287XQXXXXXQ1 family default CONTROL1 register value
static const uint8_t TPS6287X_CTRL1_DEF_RST = 0b10101010;   // TPS6287XQXXXXXQ1 family default CONTROL1 register value with reset bit high (use to reset PIMC)
static const uint8_t TPS6287X_CTRL2_DEF     = 0b00001001;   // TPS6287XQXXXXXQ1 family default CONTROL2 register value
static const uint8_t TPS6287X_CTRL3_DEF     = 0b00000000;   // TPS6287XQXXXXXQ1 family default CONTROL3 register value
static const uint8_t TPS6287X_STATUS_INI    = 0b00000010;   // TPS6287XQXXXXXQ1 family default/initial STATUS register value (cleared on read)

// PIMC I2C Register Offset Addresses
static const uint8_t TPS6287X_VSET_OA       = 0x00;         // TPS6287XQXXXXXQ1 family VSET register offset
static const uint8_t TPS6287X_CTRL1_OA      = 0x01;         // TPS6287XQXXXXXQ1 family CONTROL1 register offset
static const uint8_t TPS6287X_CTRL2_OA      = 0x02;         // TPS6287XQXXXXXQ1 family CONTROL2 register offset
static const uint8_t TPS6287X_CTRL3_OA      = 0x03;         // TPS6287XQXXXXXQ1 family CONTROL2 register offset
static const uint8_t TPS6287X_STATUS_OA     = 0x04;         // TPS6287XQXXXXXQ1 family STATUS register offset

// PIMC I2C Design Set Values (Family)
static const uint8_t TPS6287X_CTRL1_SET_EN  = 0b01101000;   // Onboard TPS6287X CONTROL1 register set value with SEN bit high
static const uint8_t TPS6287X_CTRL1_SET_DIS = 0b01001000;   // Onboard TPS6287X CONTROL1 register set value with SEN bit low
static const uint8_t TPS6287X_CTRL3_SET     = 0b00000010;   // Onboard TPS6287X CONTROL3 register set value

//Timing constants
static const uint16_t PMIC_SEQUENCING_DELAY = 100;          // WIP: sequencing param

/* Functions */

/* WARNING: Possible bug in multi-byte writes - use with caution!
Write up to 127 bytes to target address at provided offset. Returns the number of bytes written, or negative values on error
WARNING: This function is non-blocking */
int8_t write_i2c (i2c_inst_t *i2c, const uint8_t address, const uint8_t offset, uint8_t *buffer, const uint8_t num_bytes) {

    int8_t bytes_written = 0;
    uint8_t num_bytes_clamped = 0;

    // Clamp num_bytes to valid range
    if (num_bytes < 1) {
        num_bytes_clamped = 1;
    }
    else if (num_bytes > I2C_0_DATA_BUF_LEN)
    {
        num_bytes_clamped = I2C_0_DATA_BUF_LEN;
    }
    else {
        num_bytes_clamped = num_bytes;
    }

    // Asign new message array
    uint8_t message[num_bytes_clamped + 1];

    // Fill new array with the message and offset address
    message[0] = offset;
    for (uint8_t index = 1; index < (num_bytes_clamped + 1); index++) {
        message[index] = buffer[index - 1];
    }

    // Send out filled message, retun negative values if an error occurs
    bytes_written = i2c_write_timeout_us(i2c, address, message, (num_bytes_clamped + 1), false, (I2C_TIMEOUT_PERIOD * 1000));
    if (bytes_written == PICO_ERROR_TIMEOUT) {
        return (-1);
    }
    else if (bytes_written == PICO_ERROR_GENERIC) {
        return (-2);
    }
    else {
        return (bytes_written - 1);
    }
}

/* Read up to 127 bytes from target address at provided offset. Returns the number of bytes read, or negative values on error
WARNING: This function is non-blocking */
int8_t read_i2c(i2c_inst_t *i2c, const uint8_t address, const uint8_t offset, uint8_t *buffer, const uint8_t num_bytes) {

    int8_t bytes_read = 0;
    int8_t bytes_written = 0;
    uint8_t num_bytes_clamped = 0;

    // Clamp num_bytes to valid range
    if (num_bytes < 1) {
        num_bytes_clamped = 1;
    }
    else if (num_bytes > I2C_0_DATA_BUF_LEN)
    {
        num_bytes_clamped = I2C_0_DATA_BUF_LEN;
    }
    else {
        num_bytes_clamped = num_bytes;
    }
    
    // Send a read request to the device, retun negative values if an error occurs
    bytes_written = i2c_write_timeout_us(i2c, address, &offset, 1, true, (I2C_TIMEOUT_PERIOD * 1000));
    if (bytes_written == PICO_ERROR_TIMEOUT) {
        return (-1);
    }
    else if (bytes_written == PICO_ERROR_GENERIC) {
        return (-2);
    }

    // Readback information from the target device, retun negative values if an error occurs
    bytes_read = i2c_read_timeout_us(i2c, address, buffer, num_bytes_clamped, false, (I2C_TIMEOUT_PERIOD * 1000));
    if (bytes_read == PICO_ERROR_TIMEOUT) {
        return (-3);
    }
    else if (bytes_read == PICO_ERROR_GENERIC) {
        return (-4);
    }
    else {
        return (bytes_read);
    }
}

// Scans the I2C bus for devices
void scan_i2c (i2c_inst_t *i2c, uint8_t *buffer) {

    int8_t bytes_read = 0;
    bool device_found = false;

    for (uint8_t test_address = 0x00; test_address < 128; test_address++) {

        sleep_ms(5);

        printf("Testing address %03d (0x%x)\t", test_address, test_address);

        bytes_read = read_i2c(i2c, test_address, 0x00, buffer, 1);

        if (bytes_read > 0) {
            printf("Device found!\n");
            device_found = true;
        }
        else if (bytes_read == -2) {
            printf("Communication Error (Write)\n");
        }
        else if (bytes_read == -4) {
            printf("Communication Error (Read)\n");
        }
        else {
            printf("No response\n");
        }
    }

    if (!device_found) {
        printf("WARNING: No devices found on bus\n");
    }
}

/* Main program */

int main(void) {

    uint64_t timestamp_A_us             = time_us_64();     // 64-Bit timestamp in us. WARNING: Requires multiple clock cycles to process and could be malformed by an interrupt
    uint64_t timestamp_B_us             = 0;                // 64-Bit timestamp in us. WARNING: Requires multiple clock cycles to process and could be malformed by an interrupt
    uint8_t i2c_error_state             = 0;
    uint8_t program_retry_count         = 0;
    int8_t i2c_bytes_read               = 0;

    // Setup GPIO pins
    gpio_init(PIMC_1V0_EN);
    gpio_init(PIMC_1V2_EN);
    gpio_init(PIMC_1V8_EN);
    gpio_init(PIMC_3V3_EN);
    gpio_init(PIMC_1V25REF_EN);
    gpio_set_dir(PIMC_1V0_EN, GPIO_OUT);
    gpio_set_dir(PIMC_1V2_EN, GPIO_OUT);
    gpio_set_dir(PIMC_1V8_EN, GPIO_OUT);
    gpio_set_dir(PIMC_3V3_EN, GPIO_OUT);
    gpio_set_dir(PIMC_1V25REF_EN, GPIO_OUT);

    // Interface definitions
    i2c_inst_t *i2c_0 = i2c0;                               // I2C-0 object creation

    i2c_init(i2c_0, ((uint32_t)1000 * I2C_0_FREQ));         // I2C-0 object activation
    gpio_set_function(I2C_0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_0_SCL_PIN, GPIO_FUNC_I2C);

    uint8_t i2c_0_data_buffer[I2C_0_DATA_BUF_LEN];          // I2C-0 Data buffer

    // Initialize the serial port
    stdio_init_all();

    // Set all enable pins low
    gpio_put(PIMC_1V0_EN, false);
    gpio_put(PIMC_1V2_EN, false);
    gpio_put(PIMC_1V8_EN, false);
    gpio_put(PIMC_3V3_EN, false);
    gpio_put(PIMC_1V25REF_EN, false);

    // Clear I2C-0 Data buffer
    for (uint8_t index = 0; index < I2C_0_DATA_BUF_LEN; index++) {
        i2c_0_data_buffer[index] = 0x00;
    }

    // Sleep before starting serial communication
    sleep_ms(INIT_SERIAL_DELAY);

    do {

    printf("\nScanning for I2C PMICs\n");

    // Read regs from the 3V3 PMIC
    i2c_bytes_read = read_i2c(i2c_0, PMIC_3V3_ADDR, TPS6287X_VSET_OA, i2c_0_data_buffer, 5);

    // Parse the reg values from the 3V3 PMIC
    if (i2c_bytes_read <= 0) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 1;
        printf("ERROR: 3V3 PMIC did not respond\n");
    }
    else if (i2c_0_data_buffer[1] != TPS6287X_CTRL1_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 2;
        printf("ERROR: Non-default readback value from 3V3 PMIC CONTROL1 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL1_DEF, i2c_0_data_buffer[1]);
    }
    else if (i2c_0_data_buffer[2] != TPS6287X_CTRL2_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 3;
        printf("ERROR: Non-default readback value from 3V3 PMIC CONTROL2 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL2_DEF, i2c_0_data_buffer[2]);
    }
    else if (i2c_0_data_buffer[3] != TPS6287X_CTRL3_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 4;
        printf("ERROR: Non-default readback value from 3V3 PMIC CONTROL3 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL3_DEF, i2c_0_data_buffer[3]);
    }
    else if (i2c_0_data_buffer[4] != TPS6287X_STATUS_INI) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 5;
        printf("ERROR: Non-default readback value from 3V3 PMIC STATUS register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_STATUS_INI, i2c_0_data_buffer[4]);
    }
    else {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        printf("3V3 PMIC register readback successful\n");
    }

    // Read regs from the 1V8 PMIC
    i2c_bytes_read = read_i2c(i2c_0, PMIC_1V8_ADDR, TPS6287X_VSET_OA, i2c_0_data_buffer, 5);

    // Parse the reg values from the 1V8 PMIC
    if (i2c_bytes_read <= 0) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 11;
        printf("ERROR: 1V8 PMIC did not respond\n");
    }
    else if (i2c_0_data_buffer[1] != TPS6287X_CTRL1_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 12;
        printf("ERROR: Non-default readback value from 1V8 PMIC CONTROL1 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL1_DEF, i2c_0_data_buffer[1]);
    }
    else if (i2c_0_data_buffer[2] != TPS6287X_CTRL2_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 13;
        printf("ERROR: Non-default readback value from 1V8 PMIC CONTROL2 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL2_DEF, i2c_0_data_buffer[2]);
    }
    else if (i2c_0_data_buffer[3] != TPS6287X_CTRL3_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 14;
        printf("ERROR: Non-default readback value from 1V8 PMIC CONTROL3 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL3_DEF, i2c_0_data_buffer[3]);
    }
    else if (i2c_0_data_buffer[4] != TPS6287X_STATUS_INI) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 15;
        printf("ERROR: Non-default readback value from 1V8 PMIC STATUS register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_STATUS_INI, i2c_0_data_buffer[4]);
    }
    else {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        printf("1V8 PMIC register readback successful\n");
    }

    // Read regs from the 1V0 PMIC
    i2c_bytes_read = read_i2c(i2c_0, PMIC_1V0_ADDR, TPS6287X_VSET_OA, i2c_0_data_buffer, 5);

    // Parse the reg values from the 1V0 PMIC
    if (i2c_bytes_read <= 0) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 21;
        printf("ERROR: 1V0 PMIC did not respond\n");
    }
    else if (i2c_0_data_buffer[1] != TPS6287X_CTRL1_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 22;
        printf("ERROR: Non-default readback value from 1V0 PMIC CONTROL1 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL1_DEF, i2c_0_data_buffer[1]);
    }
    else if (i2c_0_data_buffer[2] != TPS6287X_CTRL2_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 23;
        printf("ERROR: Non-default readback value from 1V0 PMIC CONTROL2 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL2_DEF, i2c_0_data_buffer[2]);
    }
    else if (i2c_0_data_buffer[3] != TPS6287X_CTRL3_DEF) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 24;
        printf("ERROR: Non-default readback value from 1V0 PMIC CONTROL3 register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_CTRL3_DEF, i2c_0_data_buffer[3]);
    }
    else if (i2c_0_data_buffer[4] != TPS6287X_STATUS_INI) {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        i2c_error_state = 25;
        printf("ERROR: Non-default readback value from 1V0 PMIC STATUS register\n");
        printf("Expected: %x\t Received: %x\n", TPS6287X_STATUS_INI, i2c_0_data_buffer[4]);
    }
    else {
        if (DEV_MODE) {
            sleep_ms(200);
        }
        printf("1V0 PMIC register readback successful\n");
    }

    // Check if an error has occured
    if ((i2c_error_state > 0) && (program_retry_count == 0)) {
        printf("Errors detected - reseting PMICS\n");
        program_retry_count++;

        // Write high reset bit to each PMIC
        i2c_0_data_buffer[0] = TPS6287X_CTRL1_DEF_RST;
        write_i2c(i2c_0, PMIC_3V3_ADDR, TPS6287X_CTRL1_OA, i2c_0_data_buffer, 1);
        write_i2c(i2c_0, PMIC_1V8_ADDR, TPS6287X_CTRL1_OA, i2c_0_data_buffer, 1);
        write_i2c(i2c_0, PMIC_1V0_ADDR, TPS6287X_CTRL1_OA, i2c_0_data_buffer, 1);
    }
    else if (i2c_error_state > 0) {
        printf("Persistent errors detected - last error: %d\nAborting startup\n", i2c_error_state);

        // TODO: TURN ON FAULT INDICATOR HERE                                                           <-------------------------------------- TODO

        // Go into inf sleep
        while (true) {
            sleep_ms(10000);
            printf("Startup aborted (last code %d) - awaiting reset\n", i2c_error_state);
        }
    }

    } while (i2c_error_state > 0);

    // Write to the registers in the 3V3 PMIC
    i2c_0_data_buffer[0] = TPS6287X_CTRL1_SET_EN;
    write_i2c(i2c_0, PMIC_3V3_ADDR, TPS6287X_CTRL1_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = PMIC_3V3_CTRL2_SET;
    write_i2c(i2c_0, PMIC_3V3_ADDR, TPS6287X_CTRL2_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = TPS6287X_CTRL3_SET;
    write_i2c(i2c_0, PMIC_3V3_ADDR, TPS6287X_CTRL3_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = PMIC_3V3_VSET_SET;
    write_i2c(i2c_0, PMIC_3V3_ADDR, TPS6287X_VSET_OA, i2c_0_data_buffer, 1);
    sleep_ms(2);
    
    // Write to the registers in the 1V8 PMIC
    i2c_0_data_buffer[0] = TPS6287X_CTRL1_SET_EN;
    write_i2c(i2c_0, PMIC_1V8_ADDR, TPS6287X_CTRL1_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = PMIC_1V8_CTRL2_SET;
    write_i2c(i2c_0, PMIC_1V8_ADDR, TPS6287X_CTRL2_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = TPS6287X_CTRL3_SET;
    write_i2c(i2c_0, PMIC_1V8_ADDR, TPS6287X_CTRL3_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = PMIC_1V8_VSET_SET;
    write_i2c(i2c_0, PMIC_1V8_ADDR, TPS6287X_VSET_OA, i2c_0_data_buffer, 1);
    sleep_ms(2);

    // Write to the registers in the 1V0 PMIC
    i2c_0_data_buffer[0] = TPS6287X_CTRL1_SET_EN;
    write_i2c(i2c_0, PMIC_1V0_ADDR, TPS6287X_CTRL1_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = PMIC_1V0_CTRL2_SET;
    write_i2c(i2c_0, PMIC_1V0_ADDR, TPS6287X_CTRL2_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = TPS6287X_CTRL3_SET;
    write_i2c(i2c_0, PMIC_1V0_ADDR, TPS6287X_CTRL3_OA, i2c_0_data_buffer, 1);
    i2c_0_data_buffer[0] = PMIC_1V0_VSET_SET;
    write_i2c(i2c_0, PMIC_1V0_ADDR, TPS6287X_VSET_OA, i2c_0_data_buffer, 1);

    // TODO: Put readback check here                                                                          <----------------

    printf("PMICs setup\n");

    sleep_ms(2000);

    // Sequence on all PIMCs
    gpio_put(PIMC_1V0_EN, true);
    sleep_ms(PMIC_SEQUENCING_DELAY);

    // TODO: wait for PG pin, or until timeout and abort

    gpio_put(PIMC_1V2_EN, true);
    gpio_put(PIMC_1V8_EN, true);
    gpio_put(PIMC_1V25REF_EN, true);
    sleep_ms(PMIC_SEQUENCING_DELAY);

    // TODO: wait for PG pin, or until timeout and abort

    gpio_put(PIMC_3V3_EN, true);

    // TODO: wait for PG pin, or until timeout and abort

    printf("Startup successful\n");

    // Halt the program
    while (true)
    {
        sleep_ms(5000);    
    }
 
}