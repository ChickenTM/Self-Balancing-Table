#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define I2CSPEED 100000
#define PBCLK    8000000
#define ADDREAD  0xD1
#define ADDWRITE 0xD0
#define WHO_AM_I 0x75

#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define ACCEL_INT_CTRL 0x69
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
 
//initialising ports as input or output
void IO_init(void) {
    ANSELBbits.ANSB13 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB7 = 0;
    TRISDbits.TRISD1 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC15 = 0;
}

// Initialising I2C
void I2C_init(void) {
    double BRG = 0.5 * PBCLK / I2CSPEED - 1 - 0.5 * PBCLK * 130e-9;
    I2C3CON = 0;
    I2C3CONbits.DISSLW = 0; //slew rate control
    I2C3BRG = (int)BRG;
    I2C3CONbits.ON = 1; //I2C ON
}

// wait for I2C
void I2C_wait(void) {
    while (I2C3CON & 0x1F) {}
    while (I2C3STATbits.TRSTAT) {}
}

//start I2C
void I2C_start(void) {
    I2C_wait();
    I2C3CONbits.SEN = 1;
    while (I2C3CONbits.SEN == 1) {}
}

//End I2C
void I2C_stop(void) {
    I2C_wait();
    I2C3CONbits.PEN = 1;
    while (I2C3CONbits.PEN == 1) {}
}

//Restart I2C
void I2C_restart(void) {
    I2C_wait();
    I2C3CONbits.RSEN = 1;
    while (I2C3CONbits.RSEN == 1) {}
}

//I2C Acknowledge
void I2C_ack(void) {
    I2C_wait();
    I2C3CONbits.ACKDT = 0;
    I2C3CONbits.ACKEN = 1;
    while (I2C3CONbits.ACKEN == 1) {}
}

//I2C Not Acknowledge
void I2C_nack(void) {
    I2C_wait();
    I2C3CONbits.ACKDT = 1;
    I2C3CONbits.ACKEN = 1;
    while (I2C3CONbits.ACKEN == 1) {}
}

//write through I2C
void I2C_write(unsigned char data, char ack) {
    I2C_wait();
    I2C3TRN = data;
    while (I2C3STATbits.TBF == 1) {}
    I2C_wait();
    if (ack) {
        while (I2C3STATbits.ACKSTAT == 1) {}
    }
}

//read through I2C
void I2C_read(unsigned char *value, char ack_nack) {
    I2C3CONbits.RCEN = 1;
    while (I2C3CONbits.RCEN == 1) {}
    while (I2C3STATbits.RBF == 0) {}
    *value = I2C3RCV;
    if (!ack_nack) {
        I2C_ack();
    } else {
        I2C_nack();
    }
}

// Send request to sensor through I2C
void sensor_send(unsigned char address, unsigned char value) {
    I2C_start();
    I2C_write(ADDWRITE, 1);
    I2C_write(address, 1);
    I2C_write(value, 1);
    I2C_stop();
}

//Receive reading from sensor through I2C
void sensor_receive(unsigned char address, unsigned char *value) {
    I2C_start();
    I2C_write(ADDWRITE, 1);
    I2C_write(address, 1);
    I2C_restart();
    I2C_write(ADDREAD, 1);
    I2C_read(value, 1);
    I2C_stop();
}

//Initialise UART
void UART_init(void) {
    //set input and output pins
    ANSELAbits.ANSA6 = 0;
    TRISAbits.TRISA6 = 1;
    TRISCbits.TRISC12 = 0;
    
    U1BRG = 51; // baud rate 9600
    U1STA = 0; //status pin 0
    U1MODE = 0x8000; //enable UART and set mode
    U1STA = 0x1400; //enable UART
}

//return received data
char UART_readChar(void) {
    while (!U1STAbits.URXDA) {}
    return U1RXREG;
}

//send data while it is available in buffer
void UART_sendChar(char c) {
    while (U1STAbits.UTXBF) {}
    U1TXREG = c;
}

//send data as a whole string
void UART_sendString(char *str) {
    for (; *str; str++) {
        UART_sendChar(*str);
    }
}

void main(void) {
    unsigned char value, ByteLow, ByteHigh;
    char str[20];
    int accelx, accely, limit = 500;
    I2C_init();
    UART_init();
    IO_init();
    
    //configure sensor and check if response is good
    sensor_send(PWR_MGMT_1, 0x01);
    sensor_send(PWR_MGMT_2, 0x00);
    sensor_send(USER_CTRL, 0x01);
    sensor_send(CONFIG, 0x80);
    sensor_send(GYRO_CONFIG, 0x00);
    sensor_send(ACCEL_CONFIG, 0x00);
    sensor_send(ACCEL_INT_CTRL, 0x02);
    sensor_receive(WHO_AM_I, &value);
    UART_sendString(value == 0x12 ? "SENSOR OK!\n\r" : "SENSOR ERROR!\n\r");

    while (1) {
        //receive x axis acceleration and append higher and lower bytes
        sensor_receive(ACCEL_XOUT_L, &ByteLow);
        sensor_receive(ACCEL_XOUT_H, &ByteHigh);
        accelx = (ByteHigh << 8) | ByteLow;
        
        //receive y axis acceleration and append higher and lower bytes
        sensor_receive(ACCEL_YOUT_L, &ByteLow);
        sensor_receive(ACCEL_YOUT_H, &ByteHigh);
        accely = (ByteHigh << 8) | ByteLow;
        
        //adjust values to be inside range
        if (accelx > 32767) {
            accelx -= 65536;
        }
        
        if (accely > 32767) {
            accely -= 65536;
        }

        //get absolute values of acceleration
        accelx = abs(accelx);
        accely = abs(accely);
        
        //toggle LED's based on the value of acceleration in x and y axis
        LATDbits.LATD1 = (accelx < limit && accely < limit) ? 1 : 0;
        LATCbits.LATC3 = (accelx >= limit) ? 1 : 0;
        LATCbits.LATC15 = (accely >= limit) ? 1 : 0;
        
        sprintf(str, "X:%9d, Y:%9d\n\r", accelx, accely);
        
        UART_sendString(str); //send string to UART
    }
}