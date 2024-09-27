/**
  I2C1 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    i2c1.c

  @Summary
    This is the generated header file for the i2c1 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This header file provides APIs for driver for i2c1.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.4
        Device            :  dsPIC33EP256MC502

    The generated drivers are tested against the following:
        Compiler          :  XC16 v2.10
        MPLAB             :  MPLAB X v6.05
*/

/* 
 * File:   i2c.c
 * Author: axeltremaudant
 *
 * Created on November 28, 2023, 12:50 PM
 */

/*
 * The code has been designed to transmit the rotating speed of the motor on one byte.
 * To allow that, the first bit is a sign bit (0 is positive, 1 is negative),
 * the 7 next bits represent the absolute rotating speed divided by a coefficient.
 */

#include <stdio.h>
#include <stdlib.h>

#include "i2c.h"

/**
 Section: Data Types
*/

/**
  I2C Slave Driver State Enumeration

  @Summary
    Defines the different states of the i2c slave.

  @Description
    This defines the different states that the i2c slave
    used to process transactions on the i2c bus.
*/
typedef enum
{
    S_SLAVE_IDLE,
    S_SLAVE_RECEIVE_MODE,
    S_SLAVE_TRANSMIT_MODE,
} I2C_SLAVE_STATES;


/**
 Section: Macro Definitions
*/
/* defined for I2C1 */
#define I2C1_TRANSMIT_REG                       I2C1TRN	// Defines the transmit register used to send data.
#define I2C1_RECEIVE_REG                        I2C1RCV	// Defines the receive register used to receive data.

#define I2C1_MASK_REG                           I2C1MSK	// Defines the address mask register.
#define I2C1_ADDRESS_REG                        I2C1ADD	// Defines the address register. 

// The following control bits are used in the I2C state machine to manage
// the I2C module and determine next states.
#define I2C1_GENERAL_CALL_ENABLE_BIT            I2C1CONbits.GCEN	// I2C General Call enable control bit.
#define I2C1_RELEASE_SCL_CLOCK_CONTROL_BIT      I2C1CONbits.SCLREL	// I2C clock stretch/release control bit.

// The following status bits are used in the I2C state machine to determine
// the next states.

#define I2C1_READ_NOT_WRITE_STATUS_BIT          I2C1STATbits.R_W    // I2C current transaction read/write status bit.
#define I2C1_DATA_NOT_ADDRESS_STATUS_BIT        I2C1STATbits.D_A    // I2C last byte receive was data/address status bit.
#define I2C1_RECEIVE_OVERFLOW_STATUS_BIT        I2C1STATbits.I2COV	// I2C receive buffer overflow status bit.
#define I2C1_GENERAL_CALL_ADDRESS_STATUS_BIT    I2C1STATbits.GCSTAT	// I2C General Call status bit.
#define I2C1_ACKNOWLEDGE_STATUS_BIT             I2C1STATbits.ACKSTAT	// I2C ACK status bit.

/**
 Section: Local Functions
*/

static inline void __attribute__ ((always_inline)) I2C1_TransmitProcess(void);
static inline void __attribute__ ((always_inline)) I2C1_ReceiveProcess(void);
static inline void __attribute__ ((always_inline)) toggle_led_receive(void);
static inline void __attribute__ ((always_inline)) toggle_led_transmit(void);
void (*I2C_ReceiveHandler)(int) = NULL;

/**
 Section: Local Variables
*/

static I2C_SLAVE_STATES   i2c1_slave_state;
static uint8_t            i2c1_write;
static uint8_t            *p_read_speed_nb_measure;
static int                *p_read_speed_sum;
uint8_t                   i2c_motor_speed_multiplier;

/**
  Prototype:        void I2C1_Initialize(void)
  Input:            none
  Output:           none
  Description:      I2C1_Initialize is an
                    initialization routine that takes inputs from the GUI.
  Comment:          
  Usage:            I2C1_Initialize();
*/
void I2C1_Initialize(const uint16_t address, const uint8_t motor_speed_multiplier_)
{

    // initialize the hardware
    // ACKEN disabled; STREN enabled; GCEN disabled; SMEN disabled; DISSLW enabled; I2CSIDL disabled; ACKDT Sends ACK; SCLREL Holds; RSEN disabled; IPMIEN disabled; A10M 7 Bit; PEN disabled; RCEN disabled; SEN disabled; I2CEN enabled; 
    I2C1CON = 0x8040;
    // BCL disabled; P disabled; S disabled; I2COV disabled; IWCOL enabled; 
    I2C1STAT = 0x80;
    // I2CADD 87; 
    I2C1_SlaveAddressSet(address);
    // AMSK 0; 
    I2C1_SlaveAddressMaskSet(0x00);

    // make sure this is set first
    i2c1_slave_state = S_SLAVE_IDLE;
    
    I2C1_ReadPointerSet(NULL, NULL);
    i2c1_write = 0;
    
    i2c_motor_speed_multiplier = motor_speed_multiplier_;
    
    // INTERRUPTS
    //    MICI: I2C1 Master Events
    //    Priority: 7
    IPC4bits.MI2C1IP = 7;
    //    SICI: I2C1 Slave Events
    //    Priority: 7
    IPC4bits.SI2C1IP = 7;
    
    /* I2C1 Slave Events */
    // clear the master interrupt flag
    IFS1bits.SI2C1IF = 0;
    // enable the master interrupt
    IEC1bits.SI2C1IE = 1;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _SI2C1Interrupt ( void )
{
    uint8_t dummy;

    // NOTE: The slave driver will always acknowledge 
    //       any address match.


    switch (i2c1_slave_state)
    {
        case S_SLAVE_IDLE:
        case S_SLAVE_RECEIVE_MODE:

            /* When at S_SLAVE_RECEIVE_MODE this mode there
               will be two types of incoming transactions:
               1. Data sent by master
               2. A restart or start detection

               But from the point of view of the firmware, there is
               no difference between S_SLAVE_IDLE and S_SLAVE_RECEIVE_MODE
               states, since the types of incoming transactions will be
               the same so we share the code here.
             */           
                            
            if (I2C1_READ_NOT_WRITE_STATUS_BIT == 0)
            {
                // it is a write, go to receive mode 

                // Receive the data if valid
                I2C1_ReceiveProcess(); // Get address of communication
                i2c1_slave_state = S_SLAVE_RECEIVE_MODE;
            }
            else
            {
                // read the receive register only when
                // we are ready for the next transaction.
                // this one is a dummy read
                dummy = I2C1_RECEIVE_REG;

                // it is a read, go to transmit mode

                // during this portion, the master is expecting the
                // slave for a reply. So the returned status of
                // the callback at this point cannot be used to 
                // delay the reply if needed.
                // In other words, the slave has to reply to the master.
                // Therefore, the transmit will be performed.

                I2C1_TransmitProcess();

                i2c1_slave_state = S_SLAVE_TRANSMIT_MODE;
            }

            // this if statement is to make sure we only save incoming
            // data when we are truly in receiving mode
            if (i2c1_slave_state == S_SLAVE_RECEIVE_MODE)
            {
                // case of data received
                if (I2C1_DATA_NOT_ADDRESS_STATUS_BIT == 1)
                {
                    // check if we are overflowing the receive buffer
                    if (I2C1_RECEIVE_OVERFLOW_STATUS_BIT != 1)
                    {
                        I2C1_ReceiveProcess();
                        
                        int value = i2c1_write & 0b01111111; // Get the 7-bits value
                        if(i2c1_write & 0b10000000) // Get the bit sign
                            value = -value;
                        
                        I2C_ReceiveHandler(value * i2c_motor_speed_multiplier);
                    }
                    else
                    {
                        // overflow detected!
                        // read the buffer to reset the buffer full flag
                        // and clear the overflow bit
                        // then do nothing so the master
                        // will resend the data
                        dummy = I2C1_RECEIVE_REG;
                        I2C1_RECEIVE_OVERFLOW_STATUS_BIT = 0;
                    }
                }
            }

            break;

        case S_SLAVE_TRANSMIT_MODE:

            // this is the state where an ACK or NACK is expected
            // to occur after the slave has placed data to the
            // transmit register.

            // if the transaction was ACK'ed, more data needs to be sent
            // if the transaction was NACK'ed then we don't need to send
            // more data
            if (I2C1_ACKNOWLEDGE_STATUS_BIT == 0)
            {
                // transmit more data
                I2C1_TransmitProcess();
            }
            else //if (I2C1_ACKNOWLEDGE_STATUS_BIT == 1)
            {
                // no more data to be sent so we go to idle state
                i2c1_slave_state = S_SLAVE_IDLE;
            }
            break;


        default:
            // should never happen, if we ever get here stay here forever
            while(1);
            break;
    }

    I2C1_RELEASE_SCL_CLOCK_CONTROL_BIT = 1;

    // clear the slave interrupt flag
    IFS1bits.SI2C1IF = 0;
}

void I2C1_ReadPointerSet(int *sum_speed, uint8_t *measure_count)
{
    p_read_speed_nb_measure = measure_count;
    p_read_speed_sum = sum_speed;
}

void I2C1_SlaveAddressMaskSet(uint16_t mask)
{
    I2C1_MASK_REG = mask;
}

void I2C1_SlaveAddressSet(uint16_t address)
{
    i2c1_slave_state = S_SLAVE_IDLE;
    I2C1_ADDRESS_REG = address;
}

static inline void __attribute__ ((always_inline)) I2C1_TransmitProcess(void)
{
    // get the data to be transmitted
    
    // Calculate the average speed reduced
    int val = *p_read_speed_sum / *p_read_speed_nb_measure / i2c_motor_speed_multiplier;
    
    // Set the message to be sent
    uint8_t message = val < 0 ? 0b10000000 + (uint8_t) -val : (uint8_t) val;
    
    I2C1_TRANSMIT_REG = message;
    
    // Reset the values
    *p_read_speed_sum = *p_read_speed_nb_measure = 0;

    // set the SCL clock to be released
    I2C1_RELEASE_SCL_CLOCK_CONTROL_BIT = 1;
    toggle_led_transmit();
}

static inline void __attribute__ ((always_inline)) I2C1_ReceiveProcess(void)
{   
    // store the received data 

    i2c1_write = I2C1_RECEIVE_REG;
    toggle_led_receive();
}

static inline void __attribute__ ((always_inline)) toggle_led_transmit()
{
    _LATB4 = !_LATB4;
}

static inline void __attribute__ ((always_inline)) toggle_led_receive()
{
    _LATA4 = !_LATA4;
}

void I2C1_set_receive_handler(void (* interruptHandler)(int))
{
    I2C_ReceiveHandler = interruptHandler;
}
