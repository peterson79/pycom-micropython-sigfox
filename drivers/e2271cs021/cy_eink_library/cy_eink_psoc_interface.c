/******************************************************************************
* File Name: cy_eink_psoc_encapsulation.c
*
* Version: 1.00
*
* Description: This file contains functions that encapsulate PSoC Component APIs
*              or Peripheral Driver Library APIs.
*
* Hardware Dependency: CY8CKIT-028-EPD E-INK Display Shield
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/******************************************************************************
* This file contains functions that encapsulate PSoC Component APIs or 
* Peripheral Driver Library APIs. Functions defined in this file are used by the 
* cy_eink_hardware_driver.c.
*
* For the details of the E-INK display and library functions, see the code  
* example document of CE218133 - PSoC 6 MCU E-INK Display with CapSense
*
* For the details of E-INK display control and communication protocols, see the
* driver document available at the following website:
* http://www.pervasivedisplays.com/products/271
*******************************************************************************/

/* Header file includes */
#include "cy_eink_psoc_interface.h"
#include "driver/gpio.h"
#include <driver/spi_master.h>

/* Glue */
int CY_EINK_Ssel_PORT = 0;
int CY_EINK_Ssel_NUM = 2;

int CY_EINK_DispRst_PORT = 0;
int CY_EINK_DispRst_NUM = 33;

int CY_EINK_Discharge_PORT = 0;
int CY_EINK_Discharge_NUM = 26;

int CY_EINK_DispBusy_PORT = 0;
int CY_EINK_DispBusy_NUM = 36;

int CY_EINK_DispIoEn_PORT = 0;
int CY_EINK_DispIoEn_NUM = 32;

int CY_EINK_Border_PORT = 0;
int CY_EINK_Border_NUM = 25;

int CY_EINK_DispEn_PORT = 0;
int CY_EINK_DispEn_NUM = 14;

int CY_EINK_SCLK_NUM = 13;
int CY_EINK_MOSI_NUM = 22;
int CY_EINK_MISO_NUM = 37;

static spi_device_handle_t handle;

static uint8_t txBuffer[10];
static uint8_t rxBuffer[10];

static int spiSpeed = 1000000;

void Cy_EINK_GPIO_Init(){
    
    gpio_config_t gpio;
    
    //outputs
    gpio.pin_bit_mask =
        ((uint64_t)(((uint64_t)1)<<CY_EINK_Ssel_NUM)) |
        ((uint64_t)(((uint64_t)1)<<CY_EINK_DispRst_NUM)) |
        ((uint64_t)(((uint64_t)1)<<CY_EINK_Discharge_NUM)) |
        ((uint64_t)(((uint64_t)1)<<CY_EINK_DispIoEn_NUM)) |
        ((uint64_t)(((uint64_t)1)<<CY_EINK_Border_NUM)) |
        ((uint64_t)(((uint64_t)1)<<CY_EINK_DispEn_NUM));
    
    gpio.mode = GPIO_MODE_OUTPUT;
    gpio.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio.intr_type = GPIO_INTR_DISABLE;
    
    gpio_config(&gpio);
    
    //inputs
    gpio.pin_bit_mask = ((uint64_t)(((uint64_t)1)<<CY_EINK_DispBusy_NUM));
    gpio.mode = GPIO_MODE_INPUT;
    
    gpio_config(&gpio);
    
}


void Cy_GPIO_Write(int port, int pin, int value){
    ESP_ERROR_CHECK(gpio_set_level(pin, value));
}

void Cy_GPIO_Set(int port, int pin){
    ESP_ERROR_CHECK(gpio_set_level(pin, 1));
}

void Cy_GPIO_Clr(int port, int pin){
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
}

void Cy_SCB_SPI_Init(){
    spi_bus_config_t bus_config = {
    
        .sclk_io_num = CY_EINK_SCLK_NUM,
        .mosi_io_num = CY_EINK_MOSI_NUM,
        .miso_io_num = CY_EINK_MISO_NUM,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &bus_config, 1));
    
}

void Cy_SCB_SPI_Enable(){
    
    spi_device_interface_config_t dev_config = {
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=spiSpeed,
        .duty_cycle_pos=128,
        .mode=0,
        .spics_io_num=-1,
        .queue_size=3
    };
    
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &dev_config, &handle));
    
}

void Cy_SCB_SPI_Disable(){
    ESP_ERROR_CHECK(spi_bus_remove_device(handle));
    ESP_ERROR_CHECK(spi_bus_free(VSPI_HOST));
    
}

void Cy_SCB_SPI_Write(uint8_t data){
    
    //ESP_ERROR_CHECK(spi_device_ac)
    txBuffer[0] = data;
    
    spi_transaction_t trans_desc;
    //trans_desc.address = 0;
    //trans_desc.command = 0;
    trans_desc.flags = 0;
    trans_desc.length = 8;
    trans_desc.rxlength = 0;
    trans_desc.tx_buffer = txBuffer;
    trans_desc.rx_buffer = rxBuffer;
    
    ESP_ERROR_CHECK(spi_device_transmit(handle, &trans_desc));
}

void Cy_SCB_SPI_Write_Array(uint8_t *data, uint16_t length){
    
    uint8_t rxArrayBuffer[length];
    
    spi_transaction_t trans_desc;
    //trans_desc.address = 0;
    //trans_desc.command = 0;
    trans_desc.flags = 0;
    trans_desc.length = 8 * length;
    trans_desc.rxlength = 0;
    trans_desc.tx_buffer = data;
    trans_desc.rx_buffer = rxArrayBuffer;
    
    ESP_ERROR_CHECK(spi_device_transmit(handle, &trans_desc));
}

uint8_t Cy_SCB_SPI_Read(){
    
    return rxBuffer[0];
}

void Cy_Wait_For_RX(){
    

}

void Cy_Clear_SPI_Buffers(){
    
    
}

bool Cy_GPIO_Read(int port, int pin){

    
    return gpio_get_level(pin);
}

/* Function pointer for EINK delay in milliseconds */
cy_eink_delay_function_t Cy_EINK_Delay;

/*******************************************************************************
* Function Name: void Cy_EINK_RegisterDelayFunction(cy_eink_delay_function_t 
*                                                   delayFunction)
********************************************************************************
*
* Summary:
*  Registers the callback function for EINK delay
*
* Parameters:
*  cy_eink_delay_function_t:    Function pointer to a delay function
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_RegisterDelayFunction(cy_eink_delay_function_t delayFunction)
{
    /* Register the delay function */
    Cy_EINK_Delay = delayFunction;
}

/*******************************************************************************
* Function Name: void Cy_EINK_InitSPI(void)
********************************************************************************
*
* Summary:
*  Initializes the SPI block that communicates with the E-INK display.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_InitSPI(int speed)
{
    spiSpeed = speed;
    /* Start the SPI master */
    ///Cy_SCB_SPI_Init(CY_EINK_SPIM_HW, &CY_EINK_SPIM_config, &CY_EINK_SPIM_context);
    ///Cy_SCB_SPI_Enable(CY_EINK_SPIM_HW);
    
    Cy_EINK_GPIO_Init();
    Cy_SCB_SPI_Init();
    //_SCB_SPI_Enable();
    
    /* Make the chip select HIGH */
    CY_EINK_CsHigh;
}

/*******************************************************************************
* Function Name: void Cy_EINK_AttachSPI(void)
********************************************************************************
*
* Summary:
*  Attaches the SPI master to the E-INK display driver.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_AttachSPI(void)
{
    /* Make the chip select HIGH */
    CY_EINK_CsHigh;
    
    /* Start the SPI block */
    ///Cy_SCB_SPI_Enable(CY_EINK_SPIM_HW);
    Cy_SCB_SPI_Enable();
}

/*******************************************************************************
* Function Name: void Cy_EINK_DetachSPI(void)
********************************************************************************
*
* Summary:
*  Detaches the SPI master from the E-INK display driver.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_DetachSPI(void)
{
    /* Stop the SPI master */
    //Cy_SCB_SPI_Disable(CY_EINK_SPIM_HW, &CY_EINK_SPIM_context);
    Cy_SCB_SPI_Disable();
}

/*******************************************************************************
* Function Name: void Cy_EINK_WriteSPI(uint8_t data)
********************************************************************************
*
* Summary:
*  Send a byte of data to the E-INK display driver via SPI.
*
* Parameters:
*  uint8_t data : data byte that need to be transmitted
*
* Return:
*  None
*
* Side Effects:
*  None
*******************************************************************************/
void Cy_EINK_WriteSPI(uint8_t data)
{
    /* Send one byte of data */
    ///Cy_SCB_SPI_Write(CY_EINK_SPIM_HW, data);
    Cy_SCB_SPI_Write(data);
    
    /* Wait for RX buffer to get filled with the dummy data from E-INK driver */
    ///while ( CY_SCB_SPI_RX_NOT_EMPTY != (Cy_SCB_SPI_GetRxFifoStatus
    ///       (CY_EINK_SPIM_HW) & CY_SCB_SPI_RX_NOT_EMPTY))
    ///{
    ///}
    
    Cy_Wait_For_RX();
    
    /* Clear the TX and RX buffers */
    ///Cy_SCB_SPI_ClearTxFifo(CY_EINK_SPIM_HW);
    ///Cy_SCB_SPI_ClearRxFifo(CY_EINK_SPIM_HW);
    ///Cy_SCB_SPI_ClearRxFifoStatus(CY_EINK_SPIM_HW, CY_SCB_SPI_RX_NOT_EMPTY);
    Cy_Clear_SPI_Buffers();
}

void Cy_EINK_WriteSPI_Array(uint8_t *data, uint16_t length)
{
    /* Send one byte of data */
    ///Cy_SCB_SPI_Write(CY_EINK_SPIM_HW, data);
    Cy_SCB_SPI_Write_Array(data, length);
    
    /* Wait for RX buffer to get filled with the dummy data from E-INK driver */
    ///while ( CY_SCB_SPI_RX_NOT_EMPTY != (Cy_SCB_SPI_GetRxFifoStatus
    ///       (CY_EINK_SPIM_HW) & CY_SCB_SPI_RX_NOT_EMPTY))
    ///{
    ///}
    
    Cy_Wait_For_RX();
    
    /* Clear the TX and RX buffers */
    ///Cy_SCB_SPI_ClearTxFifo(CY_EINK_SPIM_HW);
    ///Cy_SCB_SPI_ClearRxFifo(CY_EINK_SPIM_HW);
    ///Cy_SCB_SPI_ClearRxFifoStatus(CY_EINK_SPIM_HW, CY_SCB_SPI_RX_NOT_EMPTY);
    Cy_Clear_SPI_Buffers();

}

/*******************************************************************************
* Function Name: Cy_EINK_ReadSPI(uint8_t data)
********************************************************************************
*
* Summary:
*  Read a byte of data from the E-INK display driver via SPI.
*
* Parameters:
*  uint8_t data : command that need to be transmitted
*
* Return:
*  uint8_t : received data
*
* Side Effects:
*  None
*******************************************************************************/
uint8_t Cy_EINK_ReadSPI(uint8_t data)
{
    /* Variable used to store the return data*/
    uint8_t readData;
    
    /* Send a command to the E-INK driver */
    //Cy_SCB_SPI_Write(CY_EINK_SPIM_HW, data);
    Cy_SCB_SPI_Write(data);
    
    /* Wait for RX buffer to get filled with a byte of data */
    ///while ( CY_SCB_SPI_RX_NOT_EMPTY != (Cy_SCB_SPI_GetRxFifoStatus
    ///       (CY_EINK_SPIM_HW) & CY_SCB_SPI_RX_NOT_EMPTY))
    ///{
   /// }
    
    Cy_Wait_For_RX();
    /* Read one byte of the RX data */
    //readData = Cy_SCB_SPI_Read(CY_EINK_SPIM_HW);
    readData = Cy_SCB_SPI_Read();
    
    /* Clear the RX and TX buffers */
    ///Cy_SCB_SPI_ClearTxFifo(CY_EINK_SPIM_HW);
    ///Cy_SCB_SPI_ClearRxFifo(CY_EINK_SPIM_HW);
    ///Cy_SCB_SPI_ClearRxFifoStatus(CY_EINK_SPIM_HW, CY_SCB_SPI_RX_NOT_EMPTY);
    
    Cy_Clear_SPI_Buffers();
    
    /* Return received byte */
    return(readData);
}

/*******************************************************************************
* Function Name: bool CY_EINK_IsBusy(void)
********************************************************************************
*
* Summary:
*  Check if the E-INK display is busy.
*
* Parameters:
*  None
*
* Return:
*  bool : True if the E-INK display is buy, False otherwise
*
* Side Effects:
*  None
*******************************************************************************/
bool Cy_EINK_IsBusy(void)
{
    /* Return the status of CY_EINK_DispBusy pin */
    if (Cy_GPIO_Read(CY_EINK_DispBusy_PORT, CY_EINK_DispBusy_NUM))
    {
        return(true);
    }
    else
    {
        return(false);
    }
}

/* [] END OF FILE */
