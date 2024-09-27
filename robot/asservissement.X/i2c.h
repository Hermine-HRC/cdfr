/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File: i2c.h  
 * Author: Axel TREMAUDANT
 * Comments: This file contains an I2C API to use the I2C with the dsPIC33.
 * This API is copied from the API provided by MCC with some modification to match the context.
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef I2C_H
#define	I2C_H

#include <xc.h> 
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/**
    @Summary
        Initializes and enables the i2c slave instance : 1

    @Description
        This routine initializes the i2c slave driver instance for : 1
        index, making it ready for clients to open and use it.

    @Preconditions
        None

    @Param
        None

    @Returns
        None

    @Example
        <code>
            
            uint8_t SlaveReadBuffer;
            uint8_t SlaveWriteBuffer;

            // initialize the i2c slave driver    
            I2C1_Initialize();
    
            // set up the slave driver
 
            // initialize the location of the read buffer
            I2C1_ReadPointerSet(SlaveReadBuffer);
            // initialize the location of the write buffer
            I2C1_WritePointerSet(SlaveWriteBuffer);
  
        </code>
*/
void I2C1_Initialize(const uint16_t address, const uint8_t motor_speed_multiplier_);


/**
    @Summary
        This function sets the slave address mask.

    @Description
        This function sets the 10-bit slave address mask to be able to
        respond to multiple addresses. This function should be called
        after the initialization of the module.

    @Preconditions
        None

    @Param
        mask - The address mask to be used when filtering
            addresses from the i2c master transactions.

    @Returns
        None

    @Example
        <code>
            Refer to I2C1_SlaveAddressSet() for an example	
        </code>

*/
void I2C1_SlaveAddressMaskSet(uint16_t mask);

/**
    @Summary
        This function sets the slave address.

    @Description
        This function sets the 10-bit slave address to be used by the
        module when filtering transactions from the i2c masters in the
        bus. The function analyzes the given address and decides if
        the 10-bit or 7-bit mode will be enabled. Once the function
        returns, the given address is set for the slave module.

        This function should be called after the initialization of
        the module.

        When changing the slave address the module must be idle.

    @Preconditions
        None

    @Param
        address - The address to be used to determine if the transaction
            is intended for this slave module.

    @Returns
        None

    @Example
        <code>
            // initialize the i2c slave driver    
            I2C1_Initialize();
 
            // set the slave address and address mask if the default
            // values set in the initialize is not the desired values.
            I2C1_SlaveAddressMaskSet(0x0xF);
            I2C1_SlaveAddressSet(0x3C);
 
        </code>

*/
void I2C1_SlaveAddressSet(uint16_t address);


/**
    @Summary
        This function sets the read pointer for the slave driver.

    @Description
        This function sets the read pointer that the driver will
        need to retrieve data that will be transmitted to the master
        whenever the master requests a read.

    @Preconditions
        None

    @Param
        *p - The pointer to the read buffer, that will be used to transmit
             data to the requesting i2c master.

    @Returns
        None

    @Example
        <code>
            Refer to I2C1_Initialize() for an example	
        </code>

*/
void I2C1_ReadPointerSet(int *sum_speed, uint8_t *measure_count);

/*
 * Set the handler when data is received
 * @param interruptHandler: the callback function
 * This function must have an integer as argument
 */
void I2C1_set_receive_handler(void (* interruptHandler)(int));

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* I2C_H */

