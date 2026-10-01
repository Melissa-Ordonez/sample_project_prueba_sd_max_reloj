#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "MAIN_SYSTEM";

// ==========================================

// 1. PINOUT Y CONFIGURACIÓN HARDWARE

// ==========================================

// Bus I2C (DS3231)

#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define DS3231_I2C_ADDR  0x68

// Bus SPI (MAX6675)

#define PIN_NUM_MISO     GPIO_NUM_19
#define PIN_NUM_CLK      GPIO_NUM_18g
#define PIN_NUM_MAX_CS   GPIO_NUM_4

// Pines LCD 16x2 Paralela (Modo 4 bits)

#define LCD_RS           GPIO_NUM_25
#define LCD_E            GPIO_NUM_26
#define LCD_D4           GPIO_NUM_27
#define LCD_D5           GPIO_NUM_14
#define LCD_D6           GPIO_NUM_16
#define LCD_D7           GPIO_NUM_13

// Botones de selección de perfil
#define PIN_BTN_1          GPIO_NUM_34  // Perfil 1: 196°C / 11 min
#define PIN_BTN_2          GPIO_NUM_35  // Perfil 2: 210°C / 13 min
#define PIN_BTN_3          GPIO_NUM_32  // Perfil 3: 231°C / 15 min

// Pin del Relé
#define PIN_HEATER_RELAY   GPIO_NUM_33
#define HYSTERESIS         2.0f

// ==========================================

// 2. ESTRUCTURAS Y HANDLES GLOBALES

// ==========================================

typedef struct {

    char time_str[16];   // "HH:MM:SS"
    char date_str[16];   // "DD/MM/YYYY"
    float temperatura;
    bool tc_error;
    bool heater_on;
    
    // Estado del proceso
    bool en_proceso;       // true: ejecutando perfil, false: esperando selección
    int perfil_seleccionado; // 1, 2 o 3
    float temp_target;     // 196, 210 o 231 °C
    int tiempo_restante_sec; // Segundos restantes del proceso

} SensorData_t;

QueueHandle_t sensorQueue;
i2c_master_dev_handle_t rtc_dev_handle;
spi_device_handle_t max6675_spi_handle;

// ==========================================

// 3. AUXILIARES RTC DS3231 (I2C)

// ==========================================

static inline uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
static inline uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

static int parse_month(const char *month_str) {
    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", 
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++) {
        if (strncmp(month_str, months[i], 3) == 0) return i + 1;
    }
    return 1;
}

// Programa la fecha/hora de la PC en el DS3231

// Verifica si el oscilador se detuvo (bit OSF) y solo reprograma si es necesario
esp_err_t ds3231_init_or_update(i2c_master_dev_handle_t dev_handle, bool force_reset) {
    uint8_t reg_status = 0x0F;
    uint8_t status_val = 0;

    // Leer el registro de estado (0x0F)
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_status, 1, &status_val, 1, -1);
    if (ret != ESP_OK) return ret;

    // El bit 7 (OSF) indica si el oscilador se detuvo por falta de energía
    bool osf_flag = (status_val & 0x80) != 0;

    // Solo escribe la hora de compilación si forzamos el reset O si el RTC perdió energía
    if (force_reset || osf_flag) {
        char month_buf[4];
        int day, year, hour, min, sec;
        
        sscanf(__DATE__, "%s %d %d", month_buf, &day, &year);
        sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec);

        uint8_t month = parse_month(month_buf);
        uint8_t write_buf[8];
        
        write_buf[0] = 0x00; // Registro inicial
        write_buf[1] = dec2bcd(sec);
        write_buf[2] = dec2bcd(min);
        write_buf[3] = dec2bcd(hour);
        write_buf[4] = 1;
        write_buf[5] = dec2bcd(day);
        write_buf[6] = dec2bcd(month);
        write_buf[7] = dec2bcd(year % 100);

        ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
        if (ret != ESP_OK) return ret;

        // Limpiar el bit OSF (Escribir 0 en el registro 0x0F)
        uint8_t clear_osf[2] = {0x0F, 0x00};
        return i2c_master_transmit(dev_handle, clear_osf, sizeof(clear_osf), -1);
    }

    return ESP_OK;
}

esp_err_t ds3231_get_time(i2c_master_dev_handle_t dev_handle, struct tm *timeinfo) {
    uint8_t reg_addr = 0x00;
    uint8_t data[7];

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, sizeof(data), -1);
    if (ret != ESP_OK) return ret;


    timeinfo->tm_sec  = bcd2dec(data[0] & 0x7F);
    timeinfo->tm_min  = bcd2dec(data[1] & 0x7F);
    timeinfo->tm_hour = bcd2dec(data[2] & 0x3F);
    timeinfo->tm_mday = bcd2dec(data[4] & 0x3F);
    timeinfo->tm_mon  = bcd2dec(data[5] & 0x1F) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100;

    return ESP_OK;

}

// ==========================================

// 4. CONTROLADOR PARALELO LCD 16x2 (4 BITS)

// ==========================================

void lcd_pulse_enable(void) {

    gpio_set_level(LCD_E, 1);
    esp_rom_delay_us(50);
    gpio_set_level(LCD_E, 0);
    esp_rom_delay_us(100);

}

void lcd_write_nibble(uint8_t nibble) {

    gpio_set_level(LCD_D4, (nibble >> 0) & 0x01);
    gpio_set_level(LCD_D5, (nibble >> 1) & 0x01);
    gpio_set_level(LCD_D6, (nibble >> 2) & 0x01);
    gpio_set_level(LCD_D7, (nibble >> 3) & 0x01);
    esp_rom_delay_us(10);
    lcd_pulse_enable();

}

void lcd_send_byte(uint8_t val, uint8_t is_data) {

    gpio_set_level(LCD_RS, is_data);
    esp_rom_delay_us(10);
    
    lcd_write_nibble(val >> 4);   // Nibble alto
    lcd_write_nibble(val & 0x0F); // Nibble bajo
}

void lcd_send_cmd(uint8_t cmd) {

    lcd_send_byte(cmd, 0);

    if (cmd == 0x01 || cmd == 0x02) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Tiempo extra para Clear Display / Return Home
    }
    else {
        esp_rom_delay_us(200);
    }

}

void lcd_send_char(char c) {

    lcd_send_byte(c, 1);
    esp_rom_delay_us(100);

}

void lcd_init_parallel(void) {

    gpio_config_t io_conf = {

        .pin_bit_mask = (1ULL<<LCD_RS) | (1ULL<<LCD_E) |

                        (1ULL<<LCD_D4) | (1ULL<<LCD_D5) |

                        (1ULL<<LCD_D6) | (1ULL<<LCD_D7),

        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE

    };

    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RS, 0);
    gpio_set_level(LCD_E, 0);

    // Secuencia de inicialización modo 4 bits

    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_nibble(0x03);
    esp_rom_delay_us(500);

    lcd_write_nibble(0x02); // Establecer oficialmente el modo de 4 bits
    vTaskDelay(pdMS_TO_TICKS(5));

    // Configuración de pantalla

    lcd_send_cmd(0x28); // 4 bits, 2 líneas, fuente 5x8
    lcd_send_cmd(0x08); // Apagar pantalla
    lcd_send_cmd(0x01); // Limpiar pantalla
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_cmd(0x06); // Incremento de cursor a la derecha
    lcd_send_cmd(0x0C); // Encender pantalla sin cursor

}

void lcd_set_cursor(uint8_t col, uint8_t row) {

    uint8_t offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_send_cmd(0x80 | (col + offsets[row]));

}

void lcd_print_string(const char *str) {

    while (*str) {
        lcd_send_char(*str++);
    }

}

// ==========================================

// 5. LECTURA DE MAX6675 (SPI)

// ==========================================

bool max6675_read_temp(float *temp_out) {

    uint16_t raw_data = 0;
    spi_transaction_t t = {

        .length = 16,
        .rx_buffer = &raw_data,
    };

    esp_err_t ret = spi_device_transmit(max6675_spi_handle, &t);

    if (ret == ESP_OK) {

        raw_data = __builtin_bswap16(raw_data);

        if (raw_data & 0x04) { // Bit 2: Termocupla desconectada
            return false;

        }
        *temp_out = (raw_data >> 3) * 0.25f;
        return true;
    }

    return false;

}

// ==========================================

// 6. TAREAS FREERTOS

// ==========================================

void vTaskSampling(void *pvParameters) {
    SensorData_t sample = {0};
    struct tm timeinfo;
    
    int tiempo_total_sec = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 1. Obtener Hora del RTC
        if (ds3231_get_time(rtc_dev_handle, &timeinfo) == ESP_OK) {
            strftime(sample.time_str, sizeof(sample.time_str), "%H:%M:%S", &timeinfo);
        } else {
            snprintf(sample.time_str, sizeof(sample.time_str), "RTC ERR");
        }

        // 2. Leer Termocupla
        if (max6675_read_temp(&sample.temperatura)) {
            sample.tc_error = false;
        } else {
            sample.tc_error = true;
            sample.temperatura = 0.0f;
        }

        // 3. Lógica de Selección por Botones (si no hay proceso activo)
        if (!sample.en_proceso) {
            gpio_set_level(PIN_HEATER_RELAY, 0); // Mantener apagado el calentador
            sample.heater_on = false;

            if (gpio_get_level(PIN_BTN_1) == 0) { // Presionado (Active LOW)
                sample.en_proceso = true;
                sample.perfil_seleccionado = 1;
                sample.temp_target = 40.0f;
                tiempo_total_sec = 2 * 60;
            } else if (gpio_get_level(PIN_BTN_2) == 0) {
                sample.en_proceso = true;
                sample.perfil_seleccionado = 2;
                sample.temp_target = 50.0f;
                tiempo_total_sec = 2 * 60;
            } else if (gpio_get_level(PIN_BTN_3) == 0) {
                sample.en_proceso = true;
                sample.perfil_seleccionado = 3;
                sample.temp_target = 60.0f;
                tiempo_total_sec = 2 * 60;
            }
            sample.tiempo_restante_sec = tiempo_total_sec;
        } 
        
        // 4. Lógica durante el proceso activo (Temporizador y Control de Temperatura)
        if (sample.en_proceso) {
            // Control ON/OFF con Histéresis
            if (!sample.tc_error) {
                if (sample.temperatura < (sample.temp_target - HYSTERESIS)) {
                    gpio_set_level(PIN_HEATER_RELAY, 1);
                    sample.heater_on = true;
                } else if (sample.temperatura > (sample.temp_target + HYSTERESIS)) {
                    gpio_set_level(PIN_HEATER_RELAY, 0);
                    sample.heater_on = false;
                }
            } else {
                gpio_set_level(PIN_HEATER_RELAY, 0); // Apagar si falla sensor
                sample.heater_on = false;
            }

            // Descontar 1 segundo
            if (tiempo_total_sec > 0) {
                tiempo_total_sec--;
                sample.tiempo_restante_sec = tiempo_total_sec;
            } else {
                // Proceso Finalizado
                sample.en_proceso = false;
                gpio_set_level(PIN_HEATER_RELAY, 0);
                sample.heater_on = false;
            }
        }

        xQueueSend(sensorQueue, &sample, pdMS_TO_TICKS(50));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // Periodo exacto de 1 segundo
    }
}

//-------------------------------------------------------------------------

void vTaskLCDDisplay(void *pvParameters) {
    SensorData_t data;
    char buffer[24];

    lcd_init_parallel();

    while (1) {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE) {
            if (!data.en_proceso) {
                // Modo Selección: Muestra mensaje y la Hora actual
                lcd_set_cursor(0, 0);
                snprintf(buffer, sizeof(buffer), "Hora: %s  ", data.time_str);
                lcd_print_string(buffer);

                lcd_set_cursor(0, 1);
                lcd_print_string("Elija P1, P2 o P3");
            } else {
                // Modo Proceso Activo: Muestra Temperatura, Tiempo restante y Hora
                int min_rest = data.tiempo_restante_sec / 60;
                int sec_rest = data.tiempo_restante_sec % 60;

                // Línea 1: Temp actual y Tiempo Restante (ej: "195C 10:45m  *")
                lcd_set_cursor(0, 0);
                if (data.tc_error) {
                    lcd_print_string("Err Termocupla  ");
                } else {
                    snprintf(buffer, sizeof(buffer), "%3.0fC %02d:%02dm %s", 
                             data.temperatura, min_rest, sec_rest, data.heater_on ? "*" : " ");
                    lcd_print_string(buffer);
                }

                // Línea 2: Perfil y Hora (ej: "P1:196C  14:30:15")
                lcd_set_cursor(0, 1);
                snprintf(buffer, sizeof(buffer), "P%d:%3.0fC %s", 
                         data.perfil_seleccionado, data.temp_target, data.time_str);
                lcd_print_string(buffer);
            }
        }
    }
}

// ==========================================

// 7. INICIALIZACIÓN GENERAL

// ==========================================

void app_main(void) {

    ESP_LOGI(TAG, "Iniciando sistema con LCD Paralela...");

    sensorQueue = xQueueCreate(5, sizeof(SensorData_t));

    // A. I2C para DS3231 ---------------------------------------------------

    i2c_master_bus_config_t i2c_bus_cfg = {

        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,

    };

    i2c_master_bus_handle_t i2c_bus;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    i2c_device_config_t rtc_dev_cfg = {

        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_I2C_ADDR,
        .scl_speed_hz = 100000,

    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &rtc_dev_cfg, &rtc_dev_handle));

    // B. SPI para MAX6675  -------------------------------------------------------------

    spi_bus_config_t spi_bus_cfg = {

        .mosi_io_num = -1,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32

    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t max6675_cfg = {

        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_MAX_CS,
        .queue_size = 1,

    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &max6675_cfg, &max6675_spi_handle));

    ESP_ERROR_CHECK(ds3231_init_or_update(rtc_dev_handle, false));


    // Configuración de Pines de Botones (Pull-up interno activado)
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_1) | (1ULL << PIN_BTN_2) | (1ULL << PIN_BTN_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    // Configuración de Pin del Relé
    gpio_config_t relay_conf = {
        .pin_bit_mask = (1ULL << PIN_HEATER_RELAY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&relay_conf);
    gpio_set_level(PIN_HEATER_RELAY, 0); // Apagado por seguridad al inicio

    // C. Tareas

    xTaskCreate(vTaskSampling, "sampling_task", 3072, NULL, 5, NULL);
    xTaskCreate(vTaskLCDDisplay, "lcd_task", 3072, NULL, 4, NULL);

} 

