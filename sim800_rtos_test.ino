#include <stdlib.h>
#include <HardwareSerial.h>
#include <string.h>



// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Creating objects for serials

HardwareSerial Sim800(2);   //  Hardware UART2
HardwareSerial DummyRf(1);  // Hardware UART1



// ****************************************************************************
// Macros
#define LED_PIN 0   // For blink task
#define LED2_PIN 4  // To test something

#define MAX_SIZE_UART_RX_BUFFER 256    // For uart
#define MAX_SIZE_UART_TX_BUFFER 256    // For uart
#define MAX_SIZE_RF_RX_BUFFER 256      // For uart
#define MAX_SIZE_SERIAL_RX_BUFFER 256  // For uart





// *****************************************************************************
// Global variables

char gsm_rx_buffer[MAX_SIZE_UART_RX_BUFFER];
char gsm_tx_buffer[MAX_SIZE_UART_TX_BUFFER];

char rf_rx_buffer[MAX_SIZE_RF_RX_BUFFER];  // Uart rx buffer for dummy rf module
char serial_rx_buffer[MAX_SIZE_SERIAL_RX_BUFFER];


char balance[150];
char http_parsed_data[150];

volatile uint16_t gsm_rx_index = 0;
volatile char gsm_incoming_byte;

volatile uint16_t rf_rx_index = 0;
volatile char rf_incoming_byte;

volatile uint16_t serial_rx_index = 0;
volatile char serial_incoming_byte;

volatile uint8_t flag_OK_Status = 0;
volatile uint8_t flag_HTTP_Status_200 = 0;
volatile uint8_t flag_HTTP_Get_New_Data = 0;
volatile uint8_t flag_HTTP_Unread_Data = 0;

// ****************************************************************************
// Global Functions

uint8_t gsm_end_of_data(unsigned char data)  // detect "end of line" for sim800
{
  volatile static uint8_t cr_flag = 0;
  volatile static uint8_t data_step = 0;

  data_step++;

  if (data == '\r') {
    cr_flag = 1;
    data_step = 1;
  }

  if ((data_step == 2) && (cr_flag = 1) && (data == '\n'))  //CRLF came and there is nothing between them
  {
    data_step = 0;
    cr_flag = 0;
    return 1;
  }


  else {
    return 0;
  }
}


void sim800_init() {

  uint8_t attempts_count = 0;
  uint8_t dont_continue = 0;

  vTaskDelay(7000 / portTICK_PERIOD_MS);


  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("ATE1\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during ATE1");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+CMGF=1\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during AT");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+CNMI=1,2,0,0,0\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during CNMI");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+SAPBR=3,1,\"Contype\", \"GPRS\"\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during SAPBR");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;



  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+SAPBR=3,1,\"APN\",\"internet\"\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during SAPBR");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  // No need to check
  Sim800.print("AT+SAPBR=1,1\r\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  flag_OK_Status = 0;



  flag_OK_Status = 0;
  if (dont_continue != 1) {
    Sim800.print("AT+SAPBR=2,1\r\n");

    do {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 50) {
        dont_continue = 1;
        Serial.println("Error during getting IP");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  if (dont_continue == 1) {
    Serial.println("Failed to init sim800");
  }
}


//-------
void http_get_request(char *buffer, char *card_number) {
  char url[150] = { "https://www.azadvending.com/NestleJo/api/rfid/verifycard/" };

  uint8_t attempts_count = 0;
  uint8_t dont_continue = 0;

  strcat(url, card_number);

  Sim800.print("AT+HTTPTERM\r\n");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+HTTPINIT\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during HTTP Init");
        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;



  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+HTTPSSL=1\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        dont_continue = 1;
        Serial.println("Error during HTTP SSL");

        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;



  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+HTTPPARA=\"CID\",1\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        Serial.println("Error during HTTP Parameter");

        dont_continue = 1;
        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;



  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+HTTPPARA=\"URL\",\"");
      Sim800.print(url);
      Sim800.print("\"\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        Serial.println("Error during HTTP URL Parameter");

        dont_continue = 1;
        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;



  if (dont_continue != 1) {
    flag_HTTP_Status_200 = 0;
    Sim800.print("AT+HTTPACTION=0\r\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    flag_OK_Status = 0;

    do {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        Serial.println("Error during HTTP Action");

        dont_continue = 1;
        break;
      }
    } while (flag_HTTP_Status_200 == 0);
  }
  attempts_count = 0;



  flag_OK_Status = 0;
  if (dont_continue != 1) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Sim800.print("AT+HTTPREAD\r\n");
    do {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 100) {
        Serial.println("Error during HTTP READ");

        dont_continue = 1;
        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  flag_OK_Status = 0;
  if (dont_continue != 1) {
    do {
      Sim800.print("AT+HTTPTERM\r\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      attempts_count++;
      if (attempts_count >= 10) {
        Serial.println("Error during HTTP Term");

        dont_continue = 1;
        break;
      }
    } while (flag_OK_Status == 0);
  }
  attempts_count = 0;


  if (dont_continue == 1) {
    Serial.println("HTTP Request Error");
  }

  else {
    strcpy(buffer, http_parsed_data);
  }
}




// ****************************************************************************
//*****************************************************************************
// Tasks

// Blink LED Task to test the tasks' stability
void toggleLED(void *parameter) {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void checkSim800(void *parameters) {

  while (1) {

    if (Sim800.available() > 0) {
      gsm_incoming_byte = Sim800.read();

      if (gsm_rx_index >= MAX_SIZE_UART_RX_BUFFER - 1)  // Prevent buffer overflow
      {
        gsm_rx_index = 0;
      }

      if (gsm_end_of_data(gsm_incoming_byte)) {

        if (flag_HTTP_Get_New_Data == 1) {
          strcpy(http_parsed_data, gsm_rx_buffer);
          flag_HTTP_Get_New_Data = 0;
          flag_HTTP_Unread_Data = 1;
        }

        if (strstr((char *)gsm_rx_buffer, "OK")) {
          flag_OK_Status = 1;
          digitalWrite(LED2_PIN, HIGH);
          gsm_rx_index = 0;
          memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
        }

        if (strstr((char *)gsm_rx_buffer, "HTTPACTION: 0,200")) {
          flag_HTTP_Status_200 = 1;
          digitalWrite(LED2_PIN, HIGH);
          gsm_rx_index = 0;
          memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
        }

        if (strstr((char *)gsm_rx_buffer, "+HTTPREAD:")) {
          flag_HTTP_Get_New_Data = 1;
          gsm_rx_index = 0;
          memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));
        }

        // If the end of line came but no meaningful data
        gsm_rx_index = 0;
        memset((char *)gsm_rx_buffer, '\0', sizeof(gsm_rx_buffer));


      }
      // If it isn't the end of line, keep adding to buffer
      else {
        gsm_rx_buffer[gsm_rx_index] = gsm_incoming_byte;
        gsm_rx_index++;
      }
    }
  }
}


void checkRfModule(void *parameters) {

  while (1) {

    if (DummyRf.available() > 0) {
      rf_incoming_byte = DummyRf.read();

      if (rf_rx_index >= MAX_SIZE_RF_RX_BUFFER - 1)  // Prevent buffer overflow
      {
        rf_rx_index = 0;
      }

      if (rf_incoming_byte == '\n') {



        rf_rx_index = 0;
        memset((char *)rf_rx_buffer, '\0', sizeof(rf_rx_buffer));


      }
      // If it isn't end of line, keep adding to buffer
      else {
        rf_rx_buffer[rf_rx_index] = rf_incoming_byte;
        rf_rx_index++;
      }
    }
  }
}


void checkSerial(void *parameters) {

  while (1) {

    if (Serial.available() > 0) {
      serial_incoming_byte = Serial.read();

      if (serial_rx_index >= MAX_SIZE_SERIAL_RX_BUFFER - 1)  // Prevent buffer overflow
      {
        serial_rx_index = 0;
      }

      if (serial_incoming_byte == '\n') {  

        if (strstr((char *)serial_rx_buffer, "trigger")) {
          Serial.println("http get request trigger");
          digitalWrite(LED2_PIN, LOW);
          http_get_request(balance, "123456");

          if (flag_HTTP_Unread_Data == 1) {

            Serial.println("New data has been read");
            Serial.println(balance);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            flag_HTTP_Unread_Data = 0;
          }

          serial_rx_index = 0;
          memset((char *)serial_rx_buffer, '\0', sizeof(serial_rx_buffer));
        }

        serial_rx_index = 0;
        memset((char *)serial_rx_buffer, '\0', sizeof(serial_rx_buffer));


      }
      // If it isn't end of line, keep adding to buffer
      else {
        serial_rx_buffer[serial_rx_index] = serial_incoming_byte;
        serial_rx_index++;
      }
    }
  }
}



//*****************************************************************************
// Main

void setup() {

  // Configure pin
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);


  // Configure serials and wait a second
  Serial.begin(115200);
  Sim800.begin(9600, SERIAL_8N1, 16, 17);   // RX - TX
  DummyRf.begin(9600, SERIAL_8N1, 32, 33);  // RX - TX
  // BaudRate values can be changed.

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Sim800 FreeRTOS Demo");


  xTaskCreatePinnedToCore(
    toggleLED,     // Function to be called
    "Toggle LED",  // Name of task
    1024,          // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,          // Parameter to pass
    1,             // Task priority
    NULL,          // Task handle
    app_cpu);      // Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(
    checkSim800,     // Function to be called
    "Check Sim800",  // Name of task
    2048,            // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,            // Parameter to pass
    1,               // Task priority (must be same to prevent lockup)
    NULL,            // Task handle
    app_cpu);        // Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(
    checkRfModule,  // Function to be called
    "Check Rf",     // Name of task
    2048,           // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,           // Parameter to pass
    1,              // Task priority (must be same to prevent lockup)
    NULL,           // Task handle
    app_cpu);       // Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(
    checkSerial,  // Function to be called
    "Check Serial",    // Name of task
    2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass
    1,            // Task priority (must be same to prevent lockup)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)


  sim800_init();

  vTaskDelay(100 / portTICK_PERIOD_MS);


  // Delete "setup and loop" task
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!! This is really important !!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  //      ||
  //      \/
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}