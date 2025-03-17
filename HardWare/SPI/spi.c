#include "spi.h"
#include "stm32f10x_spi.h"
#include "control.h"

// Global variables
uint8_t SPI_RxBuffer[SPI_RX_BUFFER_SIZE];
uint8_t SPI_TxBuffer[SPI_TX_BUFFER_SIZE];
uint8_t SPI_RxComplete = 0;

/**
 * @brief  Initialize SPI2 in slave mode for communication with Raspberry Pi
 * @param  None
 * @retval None
 */
void SPI_Slave_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable SPI2 and GPIO clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // Configure SPI2 pins: SCK, MISO and MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure SPI2 NSS pin (PB12)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // SPI2 configuration
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // Not used in slave mode
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    // Enable SPI2 RXNE interrupt
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);

    // Configure NVIC for SPI2 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable SPI2
    SPI_Cmd(SPI2, ENABLE);

    // Initialize buffers
    for (int i = 0; i < SPI_RX_BUFFER_SIZE; i++) {
        SPI_RxBuffer[i] = 0;
    }
    for (int i = 0; i < SPI_TX_BUFFER_SIZE; i++) {
        SPI_TxBuffer[i] = 0;
    }
}

/**
 * @brief  SPI2 interrupt handler
 * @param  None
 * @retval None
 */
void SPI2_IRQHandler(void)
{
    static uint8_t rx_index = 0;
    
    // Check if RXNE flag is set (data received)
    if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
    {
        // Read received data
        uint8_t data = SPI_I2S_ReceiveData(SPI2);
        
        // Store in receive buffer if there's space
        if (rx_index < SPI_RX_BUFFER_SIZE)
        {
            SPI_RxBuffer[rx_index++] = data;
        }
        
        // Check if we've received a complete command
        if (rx_index >= 5) // Motor control command is 5 bytes
        {
            rx_index = 0;
            SPI_RxComplete = 1;
        }
        
        // Send response byte (if any)
        if (rx_index < SPI_TX_BUFFER_SIZE)
        {
            SPI_I2S_SendData(SPI2, SPI_TxBuffer[rx_index]);
        }
        else
        {
            SPI_I2S_SendData(SPI2, 0); // Send dummy byte
        }
    }
}

/**
 * @brief  Process received SPI command
 * @param  None
 * @retval None
 */
void SPI_Process_Command(void)
{
    if (!SPI_RxComplete)
        return;
    
    // Reset flag
    SPI_RxComplete = 0;
    
    // Check command type
    switch (SPI_RxBuffer[0])
    {
        case SPI_CMD_MOTOR_CONTROL:
            // Validate checksum
            if (SPI_Calculate_Checksum(SPI_RxBuffer, 4) == SPI_RxBuffer[4])
            {
                // Extract joystick values
                int8_t joystick_x = SPI_RxBuffer[1];
                int8_t joystick_y = SPI_RxBuffer[2];
                uint8_t speed = SPI_RxBuffer[3];
                
                // Set speed multiplier (1-6)
                if (speed >= 1 && speed <= 6)
                {
                    Flag_Direction = '0' + speed; // Convert to character '1'-'6'
                }
                
                // Map joystick values to movement
                // X: -100 to 100 (left to right)
                // Y: -100 to 100 (backward to forward)
                
                // Set Move_X and Move_Y based on joystick values
                // Scale to appropriate range for the robot
                Move_X = joystick_x / 4; // Scale down to match existing control values
                Move_Y = joystick_y / 4; // Scale down to match existing control values
                
                // Calculate rotation (Move_Z) based on joystick X for turning
                if (abs(joystick_y) < 20) { // Only apply rotation when not moving much forward/backward
                    Move_Z = -joystick_x / 4; // Negative because right rotation is negative in the code
                } else {
                    Move_Z = 0;
                }
                
                // Apply kinematic analysis to calculate motor targets
                Kinematic_Analysis(Move_X, Move_Y, Move_Z);
                
                // Prepare success response
                SPI_TxBuffer[0] = SPI_RESP_OK;
            }
            else
            {
                // Checksum error
                SPI_TxBuffer[0] = SPI_RESP_ERROR;
            }
            break;
            
        case SPI_CMD_SENSOR_READ:
            // Implement sensor data reading if needed
            // For now, just acknowledge the command
            SPI_TxBuffer[0] = SPI_RESP_OK;
            break;
            
        default:
            // Unknown command
            SPI_TxBuffer[0] = SPI_RESP_ERROR;
            break;
    }
}

/**
 * @brief  Calculate simple checksum for data validation
 * @param  data: Pointer to data buffer
 * @param  length: Length of data to calculate checksum for
 * @retval Calculated checksum
 */
uint8_t SPI_Calculate_Checksum(uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;
    
    for (uint8_t i = 0; i < length; i++)
    {
        checksum ^= data[i]; // Simple XOR checksum
    }
    
    return checksum;
}

/**
 * @brief  Send response data over SPI
 * @param  data: Pointer to data buffer to send
 * @param  length: Length of data to send
 * @retval None
 */
void SPI_Send_Response(uint8_t* data, uint8_t length)
{
    // Copy data to transmit buffer
    for (uint8_t i = 0; i < length && i < SPI_TX_BUFFER_SIZE; i++)
    {
        SPI_TxBuffer[i] = data[i];
    }
} 