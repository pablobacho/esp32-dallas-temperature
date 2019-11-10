/**
 * dallas_temperature.h
 *
 * (C) 2019 - Pablo Bacho <pablo@pablobacho.com>
 * This code is licensed under the MIT License.
 */

#ifndef _DALLAS_TEMPERATURE_H_
#define _DALLAS_TEMPERATURE_H_

#include <string.h>

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ds18b20.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DALLAS_TEMPERATURE_SEMAPHORE_MAX_WAIT   10000
#define DALLAS_TEMPERATURE_ERROR_COUNT      3   /*!< Number of failed reading attempts before throwing an error and start looking for a new sensor */
#define DALLAS_TEMPERATURE_ROM_CODE_STRING_SIZE  16 /*!< Length of human-readable string containing ROM code (excluding trailing '\0') */

#define DALLAS_TEMPERATURE_GPIO         ((gpio_num_t) 32)
#define DALLAS_TEMPERATURE_HELPER_GPIO  ((gpio_num_t) 33)

/*!< Sensor struct. Stores sensor information and latest temperature read from it. */
typedef struct {
    float temperature;                  // Temperature output
    uint32_t sampling_period;           // Sampling period in seconds
    DS18B20_Info info;                  // DS18B20 structure
    char rom_code_string[DALLAS_TEMPERATURE_ROM_CODE_STRING_SIZE +1]; // Sensor ROM code in string format for quick access
    UBaseType_t priority;               // Sensor task priority
    TaskHandle_t task;                  // Sensor task handle
    SemaphoreHandle_t bus_semaphore;    // OWB semaphore
    esp_event_loop_handle_t event_loop; // Event loop for sensor events
} dallas_temperature_sensor_t;

#define DALLAS_TEMPERATURE_SENSOR_MAX   2
#define DALLAS_TEMPERATURE_SENSOR_RANGE_MIN (-55)
#define DALLAS_TEMPERATURE_SENSOR_RANGE_MAX (125)

/*!< Component configuration struct */
typedef struct {
    gpio_num_t bus_gpio;                // GPIO used for the OWB
    gpio_num_t pullup_helper;           // Connect this pin in parallel to weaken the bus pullup (when using internal pullups)
    uint32_t search_period;             // Search period in seconds to find update the sensors list
    uint32_t sampling_period;           // Default sampling period in seconds for all sensors
    bool enable_crc;                    // Enable CRC on the OWB
    DS18B20_RESOLUTION resolution;      // Default sensor resolution
    UBaseType_t main_task_priority;     // Component main task priority
    uint32_t main_task_stack_size;
    UBaseType_t sensor_task_priority;   // Sensor task priority
    uint32_t sensor_task_stack_size;
    BaseType_t core_affinity;           // Tasks core affinity
    bool enable_events;                 // Enable sensor event functionality
} dallas_temperature_config_t;

/*!< Component struct */
typedef struct {
    dallas_temperature_sensor_t * sensor[DALLAS_TEMPERATURE_SENSOR_MAX];    // Buffer with pointers to allocated sensors
    dallas_temperature_config_t config; // Component configuration
    TaskHandle_t task;                  // Component main task handle
    SemaphoreHandle_t bus_semaphore;    // OWB semaphore
    esp_event_loop_handle_t event_loop; // Component event loop
} dallas_temperature_t;

/*!< Default component configuration */
#define DALLAS_TEMPERATURE_DEFAULT() { \
        .config = { \
            .bus_gpio = 0, \
            .pullup_helper = 0, \
            .search_period = 60, \
            .sampling_period = 5, \
            .enable_crc = true, \
            .resolution = DS18B20_RESOLUTION_12_BIT, \
            .main_task_priority = 10, \
            .main_task_stack_size = 2048, \
            .sensor_task_stack_size = 2048, \
            .sensor_task_priority = 5, \
            .enable_events = true, \
            .core_affinity = tskNO_AFFINITY \
        } \
    }

ESP_EVENT_DECLARE_BASE(DALLAS_TEMPERATURE_EVENT_BASE)

/**
 * List of possible events this module can trigger
 */
typedef enum {
    DALLAS_TEMPERATURE_SEARCH_STARTED_EVENT = 0,        /*!< Main task started searching for sensors */
    DALLAS_TEMPERATURE_SEARCH_FINISHED_EVENT,           /*!< Main task finished searching for sensors */
    DALLAS_TEMPERATURE_SEARCH_ERROR_EVENT,              /*!< Error encountered during the search process */
    DALLAS_TEMPERATURE_SENSOR_REGISTERED_EVENT,         /*!< New sensor found & registered */
    DALLAS_TEMPERATURE_SENSOR_SAMPLE_EVENT,             /*!< Sensor took a new measurement of temperature */
    DALLAS_TEMPERATURE_SENSOR_TEMPERATURE_CHANGE_EVENT, /*!< Temperature changed */
    DALLAS_TEMPERATURE_SENSOR_ERROR_EVENT,              /*!< Error reading sensor */
    DALLAS_TEMPERATURE_SENSOR_LOST_EVENT,               /*!< Sensor was unregistered because of too many errors trying to read it */
    DALLAS_TEMPERATURE_EVENT_MAX
} dallas_temperature_event_id_t;

extern const char * dallas_temperature_event_id_string[DALLAS_TEMPERATURE_EVENT_MAX +1]; /*!< Human-readable strings to print event IDs */

/**
 * @brief   Start component
 *
 *          Initializes the event loop and starts the main task. User program cannot subscribe to events without calling this function first.
 *
 * @param   handle Address of a valid struct of dallas_temperature_t type
 * @return  ESP_OK success
 *          ESP_FAIL error
 */
esp_err_t dallas_temperature_start(dallas_temperature_t * handle);

/**
 * @brief   Stop component and free resources
 *
 * @param   handle Address of a valid struct of dallas_temperature_t type
 */
void dallas_temperature_stop(dallas_temperature_t * handle);

/**
 * @brief   Main task
 *
 *          This task initializes the component and searches the OneWire Bus in the background for new sensors. When a sensor is found,
 *          memory is allocated for it and a new task is started for it.
 *
 * @param   pvParameter Pointer to a valid dallas_temperature_t structure
 */
void dallas_temperature_task(void * pvParameter);

/**
 * @brief   Sensor task
 *
 *          This task is initialized by the main task whenever a new sensor is found and registered. This task polls the sensor for new
 *          readings and posts related events to the event loop. One task per sensor found is started, so multiple instances of this
 *          task are possible.
 *
 * @param   pvParameter Pointer to a valid and unique dallas_temperature_sensor_t structure
 */
void dallas_temperature_sensor_task(void * pvParameter);

/**
 * @brief   Post event to the event loop
 *
 *          This function is internally used by the component to simplify the code for posting events. It is not meant to be used by
 *          the user program. The user program registers for events posted using this function.
 *
 * @param   event_loop Component's event loop handle
 * @param   event_id Event ID as described in dallas_temperature.h
 * @param   sensor Pointer to the sensor originating the event or NULL for component-wide events.
 */
void dallas_temperature_post_event(esp_event_loop_handle_t event_loop, int32_t event_id, dallas_temperature_sensor_t * sensor);

#ifdef __cplusplus
}
#endif

#endif // _DALLAS_TEMPERATURE_H_