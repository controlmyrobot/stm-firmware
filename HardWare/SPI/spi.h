#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

// SPI Command Protocol
#define SPI_CMD_MOTOR_CONTROL    0x01  // Command for motor control
#define SPI_CMD_SENSOR_READ      0x02  // Command for sensor data read

// SPI Response Codes
#define SPI_RESP_OK              0xAA  // Response OK
#define SPI_RESP_ERROR           0xEE  // Response Error

// SPI Buffer Sizes
#define SPI_RX_BUFFER_SIZE       8     // Size of SPI receive buffer
#define SPI_TX_BUFFER_SIZE       8     // Size of SPI transmit buffer

// SPI Data Structure for Motor Control
typedef struct {
    uint8_t command;      // Command byte (SPI_CMD_MOTOR_CONTROL)
    int8_t joystick_x;    // X position of joystick (-100 to 100)
    int8_t joystick_y;    // Y position of joystick (-100 to 100)
    uint8_t speed;        // Speed multiplier (1-6)
    uint8_t checksum;     // Simple checksum for validation
} SPI_MotorControl_t;

// SPI Data Structure for Sensor Data
typedef struct {
    uint8_t command;      // Command byte (SPI_CMD_SENSOR_READ)
    uint8_t sensor_mask;  // Bit mask for which sensors to read
    uint8_t reserved[2];  // Reserved for future use
    uint8_t checksum;     // Simple checksum for validation
} SPI_SensorRead_t;

// Function Prototypes
void SPI_Slave_Init(void);
void SPI_Process_Command(void);
uint8_t SPI_Calculate_Checksum(uint8_t* data, uint8_t length);
void SPI_Send_Response(uint8_t* data, uint8_t length);

// External Variables
extern uint8_t SPI_RxBuffer[SPI_RX_BUFFER_SIZE];
extern uint8_t SPI_TxBuffer[SPI_TX_BUFFER_SIZE];
extern uint8_t SPI_RxComplete;

#endif /* __SPI_H */ 