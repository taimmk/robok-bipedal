/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- DYNAMIXEL PROTOCOL 1.0 MACROS ---
#define DXL_HEADER_1           0xFF
#define DXL_HEADER_2           0xFF
#define DXL_BROADCAST_ID       0xFE

#define DXL_INST_READ_DATA     0x02
#define DXL_INST_WRITE_DATA    0x03

#define DXL_REG_LED            0x19
#define DXL_REG_GOAL_POSITION  0x1E
#define DXL_REG_PRESENT_POS    0x24
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// --- SYSTEM FLAGS ---
volatile bool imu_data_ready = false;

// --- NON-BLOCKING GLOBAL BUFFERS ---
uint8_t dxl_tx_buffer[64];      // Holds data safely while the background interrupt sends it
uint8_t dxl_usart1_echo[64];    // Catches the junk TTL echo in the background
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// --- UNIFIED MULTI-BUS DYNAMIXEL FUNCTIONS ---
void Dynamixel_SendPacket(uint8_t* packet, uint16_t length);
uint8_t Calculate_Dxl_Checksum(uint8_t *packet, uint8_t total_packet_length);
void Dxl_WriteRegister(uint8_t id, uint8_t address, uint8_t *data, uint8_t data_length);
void Dynamixel_ChangeID(uint8_t current_id, uint8_t new_id);
void Dynamixel_SetPosition(uint8_t motor_id, uint16_t position);

bool Dxl_ReadRegister(UART_HandleTypeDef *huart, uint8_t id, uint8_t address, uint8_t bytes_to_read, uint8_t *out_data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USB_OTG_FS_USB_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	uint8_t led_on = 1;
	uint8_t led_off = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // -----------------------------------------------------------------
	  // 1. IMU INTERRUPT PROCESSING TASK
	  // -----------------------------------------------------------------
	  if (imu_data_ready == true)
	  {
		  imu_data_ready = false; // Acknowledge and clear the flag

	  }

	  // -----------------------------------------------------------------
	  // 2. DYNAMIXEL PILOT CONTROL ROUTINE
	  // -----------------------------------------------------------------
	  // Alternate state commands across both separate physical networks

	  // Turn LEDs ON for Motor ID 1 (TTL Bus) and Motor ID 2 (RS485 Bus)
	  	  uint8_t id = 2;
		  // Example A: Change a motor from broadcast (any ID) to ID 2
		  //Dynamixel_ChangeID(0xFE, id);


	  	  Dxl_WriteRegister(id, DXL_REG_LED, &led_on, 1);
		  HAL_Delay(250);
		  // 1. HEARTBEAT: Toggle the onboard LED (PC13)
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  // Turn LEDs OFF
		  Dxl_WriteRegister(id, DXL_REG_LED, &led_off, 1);
		  HAL_Delay(250);

		  Dynamixel_SetPosition(id, 512);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ==============================================================================
// READ REGISTER FUNCTION (PROTOCOL 1.0)
// ==============================================================================
bool Dxl_ReadRegister(UART_HandleTypeDef *huart, uint8_t id, uint8_t address, uint8_t bytes_to_read, uint8_t *out_data)
{
    // 1. Construct the Read Instruction Packet
    uint8_t tx_packet[8];
    tx_packet[0] = 0xFF;          // Header 1
    tx_packet[1] = 0xFF;          // Header 2
    tx_packet[2] = id;            // Target ID
    tx_packet[3] = 0x04;          // Length: Always 4 for a READ instruction
    tx_packet[4] = 0x02;          // Instruction: READ_DATA (0x02)
    tx_packet[5] = address;       // Starting Memory Address
    tx_packet[6] = bytes_to_read; // How many bytes to read

    uint32_t checksum_sum = tx_packet[2] + tx_packet[3] + tx_packet[4] + tx_packet[5] + tx_packet[6];
    tx_packet[7] = (uint8_t)(~checksum_sum); // Bitwise NOT of the lowest byte

    // 2. Clear any lingering junk in the UART hardware RX buffer before transmitting
    __HAL_UART_FLUSH_DRREGISTER(huart);

    // 3. Transmit the Request based on the Bus Type
    if (huart->Instance == USART2)
    {
        // RS485 Bus: Assert Transmit Mode
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_UART_Transmit(huart, tx_packet, 8, 10);

        // Wait for shift register to empty
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) {}

        // CRITICAL: Drop PA9 to listen mode IMMEDIATELY. Do not put a HAL_Delay() here!
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    }
    else if (huart->Instance == USART1)
    {
        // TTL Bus
        uint8_t dummy_echo[8];
        HAL_UART_Transmit(huart, tx_packet, 8, 10);

        // Drain the local echo of our own TX packet before listening for the real response
        HAL_UART_Receive(huart, dummy_echo, 8, 5);
    }

    // 4. Receive the Motor's Reply
    // Expected RX size: 6 framing bytes (Header, Header, ID, Length, Error, Checksum) + Payload
    uint8_t expected_rx_len = 6 + bytes_to_read;
    uint8_t rx_buffer[16] = {0}; // Generous buffer size for most reads

    // Listen with a strict 5ms timeout. If the motor is unplugged, the code won't freeze here.
    if (HAL_UART_Receive(huart, rx_buffer, expected_rx_len, 5) == HAL_OK)
    {
        // Validate the packet header and ID
        if (rx_buffer[0] == 0xFF && rx_buffer[1] == 0xFF && rx_buffer[2] == id)
        {
            // Extract the requested data (starts at index 5 in the status packet)
            for (uint8_t i = 0; i < bytes_to_read; i++)
            {
                out_data[i] = rx_buffer[5 + i];
            }
            return true; // Success!
        }
    }

    return false; // Read failed, timeout, or bad packet
}
void Dynamixel_SetPosition(uint8_t motor_id, uint16_t position)
{
    // Restrict the position parameter to the maximum allowed limit
    if (position > 1023) {
        position = 1023;
    }

    // Split the 16-bit position into two 8-bit registers (Low Byte and High Byte)
    uint8_t pos_low  = (uint8_t)(position & 0xFF);
    uint8_t pos_high = (uint8_t)((position >> 8) & 0xFF);

    // A standard 2-byte write packet has exactly 9 bytes
    uint8_t packet[9];

    packet[0] = 0xFF;         // Header 1
    packet[1] = 0xFF;         // Header 2
    packet[2] = motor_id;     // Target Motor ID
    packet[3] = 0x05;         // Length: 3 + number of parameters (Address + 2 Data Bytes)
    packet[4] = 0x03;         // Instruction: WRITE
    packet[5] = 0x1E;         // Control Table Start Address: 0x1E (Goal Position Reg)
    packet[6] = pos_low;      // Parameter 1: Goal Position Low Byte
    packet[7] = pos_high;     // Parameter 2: Goal Position High Byte

    // Calculate Checksum: ~ (ID + Length + Instruction + Address + Param1 + Param2)
    uint32_t checksum_sum = packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7];
    packet[8] = (uint8_t)(~checksum_sum); // Bitwise NOT of the lowest byte

    Dynamixel_SendPacket(packet, sizeof(packet));

}
void Dynamixel_ChangeID(uint8_t current_id, uint8_t new_id)
{
    // A Dynamixel WRITE packet to the ID register has exactly 8 bytes
    uint8_t packet[8];

    packet[0] = 0xFF;         // Header 1
    packet[1] = 0xFF;         // Header 2
    packet[2] = current_id;   // Target ID (or 0xFE Broadcast)
    packet[3] = 0x04;         // Length: 3 + number of parameters (Address + Value)
    packet[4] = 0x03;         // Instruction: WRITE
    packet[5] = 0x03;         // Control Table Address: 0x03 is the ID register
    packet[6] = new_id;       // Parameter: The new ID value

    // Calculate the Checksum: ~ (ID + Length + Instruction + Address + Value)
    uint32_t checksum_sum = packet[2] + packet[3] + packet[4] + packet[5] + packet[6];
    packet[7] = (uint8_t)(~checksum_sum); // Take the bitwise NOT of the lowest byte


    // 2. Transmit the packet over your UART instance
    Dynamixel_SendPacket(packet, sizeof(packet));
}
// ==============================================================================
// HARDWARE INTERRUPT ROUTINE (EXTI)
// ==============================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Capture transitions on PC14 or PC15 (IMU Hardware Interrupt Output Lines)
    if(GPIO_Pin == GPIO_PIN_14 || GPIO_Pin == GPIO_PIN_15)
    {
        imu_data_ready = true; // Signal main thread context to handle communication
    }
}


// ==============================================================================
// DYNAMIXEL PACKET FORMATTING LAYER (PROTOCOL 1.0)
// ==============================================================================
uint8_t Calculate_Dxl_Checksum(uint8_t *packet, uint8_t total_packet_length)
{
    uint32_t sum = 0;
    for(uint8_t i = 2; i < (total_packet_length - 1); i++)
    {
        sum += packet[i];
    }
    return (uint8_t)(~sum & 0xFF);
}

void Dxl_WriteRegister( uint8_t id, uint8_t address, uint8_t *data, uint8_t data_length)
{
    uint8_t tx_buffer[32];
    uint8_t total_packet_size = data_length + 7;

    tx_buffer[0] = DXL_HEADER_1;
    tx_buffer[1] = DXL_HEADER_2;
    tx_buffer[2] = id;
    tx_buffer[3] = data_length + 3; // Length parameter field
    tx_buffer[4] = DXL_INST_WRITE_DATA;
    tx_buffer[5] = address;

    for(uint8_t i = 0; i < data_length; i++)
    {
        tx_buffer[6 + i] = data[i];
    }

    tx_buffer[total_packet_size - 1] = Calculate_Dxl_Checksum(tx_buffer, total_packet_size);

    Dynamixel_SendPacket(tx_buffer, total_packet_size);
}
// ==============================================================================
// BROADCAST TO ALL BUSES
// ==============================================================================
void Dynamixel_SendPacket(uint8_t* packet, uint16_t length)
{
    // --- 1. SEND TO RS485 BUS (USART2) ---
    // Assert Driver Enable High to put MAX485 in transmit mode
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

    // Send the data
    HAL_UART_Transmit(&huart2, packet, length, 100);

    // Wait for the hardware shift register to empty completely
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {}
    HAL_Delay(1); // Brief timing stabilization cushion

    // Assert Driver Enable Low to return MAX485 to listening mode
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);


    // --- 2. SEND TO TTL BUS (USART1) ---
    uint8_t echoBuffer[255];

    // Send the data natively
    if (HAL_UART_Transmit(&huart1, packet, length, 100) == HAL_OK)
    {
        // Drain the physical local loopback echo immediately from the RX buffer
        HAL_UART_Receive(&huart1, echoBuffer, length, 10);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
