/**
 * dallas_temperature.c
 *
 * (C) 2019 - Pablo Bacho <pablo@pablobacho.com>
 * This code is licensed under the MIT License.
 */

 #include "dallas_temperature.h"

static const char * TAG = "dallas_temperature";

ESP_EVENT_DEFINE_BASE(DALLAS_TEMPERATURE_EVENT_BASE)
ESP_EVENT_DEFINE_BASE(DALLAS_TEMPERATURE_SENSOR_EVENT_BASE)

const char * dallas_temperature_event_id_string[] = {
    "SEARCH_STARTED",
    "SEARCH_FINISHED",
    "SEARCH_ERROR",
    "SENSOR_REGISTERED",
    "SENSOR_SAMPLE",
    "SENSOR_TEMPERATURE_CHANGE",
    "SENSOR_ERROR",
    "SENSOR_LOST",
    "UNKNOWN EVENT"
};

esp_err_t dallas_temperature_start(dallas_temperature_t * handle)
{
    // Start event loop
    if(handle->config.enable_events == true) {
        esp_event_loop_args_t event_loop_args = {   // FIXME magic numbers
            .queue_size = 15,
            .task_name = "dallas_events",
            .task_priority = 10,
            .task_stack_size = 4096,
            .task_core_id = handle->config.core_affinity
        };
        if(esp_event_loop_create(&event_loop_args, &handle->event_loop) == ESP_OK) {
            ESP_LOGD(TAG, "[DBG ] Event loop created");
        } else {
            ESP_LOGE(TAG, "[FAIL] Error creating event loop");
            return ESP_FAIL;
        }
    }

    // Start main task
    BaseType_t e = xTaskCreatePinnedToCore(
        &dallas_temperature_task,           // pvTaskCode
        "dallas_temperature",               // pcName
        handle->config.main_task_stack_size,// usStackDepth
        handle,                             // pvParameters
        handle->config.main_task_priority,  // uxPriority
        &handle->task,                      // pvCreatedTask
        handle->config.core_affinity        // xCoreID
    );
    if(e == pdPASS) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

void dallas_temperature_stop(dallas_temperature_t * handle)
{
    // Stop main task
    vTaskDelete(handle->task);

    // Delete sensors
    for(uint8_t i=0; i < DALLAS_TEMPERATURE_SENSOR_MAX; ++i) {
        if(handle->sensor[i] != NULL) {
            if(handle->sensor[i]->info.init == true) {
                // Stop sensor task
                vTaskDelete(handle->sensor[i]->task);
            }
            // Free sensor memory
            free(handle->sensor[i]);
            handle->sensor[i] = NULL;
        }
    }

    // Delete semaphore
    vSemaphoreDelete(handle->bus_semaphore);
    handle->bus_semaphore = NULL;

    // Stop event loop
    esp_event_loop_delete(handle->event_loop);
    handle->event_loop = NULL;
}

void dallas_temperature_task(void * pvParameter)
{
    dallas_temperature_t * handle = (dallas_temperature_t *) pvParameter;
    if(handle == NULL) {
        ESP_LOGE(TAG, "[FAIL] Invalid arg");
        vTaskDelete(NULL);
    }

    handle->bus_semaphore = xSemaphoreCreateMutex();
    if(handle->bus_semaphore == NULL) {
        ESP_LOGE(TAG, "[FAIL] Error creating owb semaphore: not enough memory");
        vTaskDelete(NULL);
    }

    // Initialize pullup helper pin
    if(handle->config.pullup_helper != -1) {
        gpio_set_direction(handle->config.pullup_helper, GPIO_MODE_INPUT);
        gpio_set_pull_mode(handle->config.pullup_helper, GPIO_PULLUP_ONLY);
        gpio_pullup_en(handle->config.pullup_helper);
    }

    owb_rmt_driver_info rmt_driver_info;
    OneWireBus * owb = owb_rmt_initialize(&rmt_driver_info, handle->config.bus_gpio, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, handle->config.enable_crc);  // enable CRC check for ROM code

    while(1) {
        OneWireBus_SearchState search_state = {0};
        bool found = 0;

        // Housekeeping
        // Lost sensors get un-initialized (init = false) and their task terminated.
        // The main task is responsible for cleaning up the slot in the buffer.
        for(uint8_t i=0; i < DALLAS_TEMPERATURE_SENSOR_MAX; ++i) {
            if(handle->sensor[i] != NULL) {
                if(handle->sensor[i]->info.init == false) {
                    ESP_LOGD(TAG, "[DBG ] Deleting sensor %s", handle->sensor[i]->rom_code_string);
                    free(handle->sensor[i]);
                    handle->sensor[i] = NULL;
                }
            }
        }

        if(xSemaphoreTake(handle->bus_semaphore, portMAX_DELAY)) {
            if(handle->config.enable_events == true) {
                dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SEARCH_STARTED_EVENT, NULL);
            }
            owb_status s = owb_search_first(owb, &search_state, &found);
            if(s == OWB_STATUS_OK) {
                while(found == true) {
                    uint8_t i;
                    for(i=0; i < DALLAS_TEMPERATURE_SENSOR_MAX; i++) { // Find if sensor already allocated
                        if(handle->sensor[i] != NULL) {
                            if(!memcmp(search_state.rom_code.bytes, handle->sensor[i]->info.rom_code.bytes, 8)) {
                                ESP_LOGD(TAG, "[DBG ] Sensor already allocated");
                                break;
                            }
                        }
                    }
                    if(i == DALLAS_TEMPERATURE_SENSOR_MAX) { // Does not exist
                        ESP_LOGD(TAG, "[DBG ] Sensor not allocated yet");
                        for(i=0; i < DALLAS_TEMPERATURE_SENSOR_MAX; i++) {
                            // Find an empty pointer to register new sensor
                            if(handle->sensor[i] == NULL) {
                                ESP_LOGD(TAG, "[DBG ] Allocating sensor");
                                handle->sensor[i] = malloc(sizeof(dallas_temperature_sensor_t));
                                if(handle->sensor[i] != NULL) {
                                    // Copy the found ROM code to the new sensor
                                    memcpy(&handle->sensor[i]->info.rom_code, &search_state.rom_code, sizeof(OneWireBus_ROMCode));
                                    // Turn it into a string for quick human-readable access
                                    owb_string_from_rom_code(search_state.rom_code, handle->sensor[i]->rom_code_string, DALLAS_TEMPERATURE_ROM_CODE_STRING_SIZE);
                                    // Initialize sensor
                                    ds18b20_init(&handle->sensor[i]->info, owb, handle->sensor[i]->info.rom_code);
                                    ds18b20_use_crc(&handle->sensor[i]->info, handle->config.enable_crc);
                                    ds18b20_set_resolution(&handle->sensor[i]->info, handle->config.resolution);
                                    handle->sensor[i]->sampling_period = handle->config.sampling_period;
                                    // Copy bus semaphore handle
                                    handle->sensor[i]->bus_semaphore = handle->bus_semaphore;
                                    // Assign event loop
                                    if(handle->config.enable_events == true) {
                                        handle->sensor[i]->event_loop = handle->event_loop;
                                    }
                                    // Launch sensor task
                                    xTaskCreatePinnedToCore(
                                        dallas_temperature_sensor_task,         // pvTaskCode
                                        "dallas_sensor",                        // pcName
                                        handle->config.sensor_task_stack_size,  // usStackDepth
                                        handle->sensor[i],                      // pvParameters
                                        handle->config.sensor_task_priority,    // uxPriority
                                        &handle->sensor[i]->task,               // pvCreatedTask
                                        handle->config.core_affinity            // xCoreID
                                    );
                                    if(handle->config.enable_events == true) {
                                        dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SENSOR_REGISTERED_EVENT, handle->sensor[i]);
                                    }
                                    ESP_LOGD(TAG, "[DBG ] Sensor allocated with rom code %s", handle->sensor[i]->rom_code_string);
                                } else {
                                    if(handle->config.enable_events == true) {
                                        dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SEARCH_ERROR_EVENT, NULL);
                                    }
                                    ESP_LOGE(TAG, "[FAIL] Error allocating sensor: Not enough memory");
                                }
                                break;
                            }
                        }
                    }
                    owb_status s = owb_search_next(owb, &search_state, &found);
                    if(s != OWB_STATUS_OK) {
                        if(handle->config.enable_events == true) {
                            dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SEARCH_ERROR_EVENT, NULL);
                        }
                        ESP_LOGE(TAG, "[FAIL] OWB status error searching more devices: %d", s);
                        break;
                    }
                }
            } else {
                if(handle->config.enable_events == true) {
                    dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SEARCH_ERROR_EVENT, NULL);
                }
                ESP_LOGE(TAG, "[FAIL] OWB status error searching the first device: %d", s);
            }
            xSemaphoreGive(handle->bus_semaphore); // Give semaphore
            if(handle->config.enable_events == true) {
                dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SEARCH_FINISHED_EVENT, NULL);
            }
        } else { // Semaphore timed out
            if(handle->config.enable_events == true) {
                dallas_temperature_post_event(handle->event_loop, DALLAS_TEMPERATURE_SEARCH_ERROR_EVENT, NULL);
            }
            ESP_LOGE(TAG, "[FAIL] Semaphore timed out");
        }
        ESP_LOGD(TAG, "[DBG ] Sleeping for %d seconds", handle->config.search_period);
        ESP_LOGD(TAG, "[DBG ] Main - Free stack: %d", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(handle->config.search_period * 1000 / portTICK_PERIOD_MS);
    } // task loop
}

void dallas_temperature_sensor_task(void * pvParameter)
{
    dallas_temperature_sensor_t * sensor = (dallas_temperature_sensor_t *) pvParameter;
    if(sensor == NULL) {
        ESP_LOGE(TAG, "[FAIL] Invalid arg");
        vTaskDelete(NULL);
    }

    DS18B20_ERROR e;
    uint8_t error_count = 0;
    while(error_count < DALLAS_TEMPERATURE_ERROR_COUNT) {
        TickType_t last_sample_time = xTaskGetTickCount();
        if(xSemaphoreTake(sensor->bus_semaphore, sensor->sampling_period*1000/portTICK_PERIOD_MS) == pdTRUE) {
            bool convert_result = ds18b20_convert(&sensor->info);
            xSemaphoreGive(sensor->bus_semaphore);
            if(convert_result == true) { // Conversion requested
                ds18b20_wait_for_conversion(&sensor->info);
                if(xSemaphoreTake(sensor->bus_semaphore, sensor->sampling_period*1000/portTICK_PERIOD_MS) == pdTRUE) {
                    float next_temperature, prev_temperature;
                    e = ds18b20_read_temp(&sensor->info, &next_temperature);
                    xSemaphoreGive(sensor->bus_semaphore);
                    if(e == DS18B20_OK && (next_temperature >= DALLAS_TEMPERATURE_SENSOR_RANGE_MIN && next_temperature <= DALLAS_TEMPERATURE_SENSOR_RANGE_MAX)) { // Temperature read successfully
                        error_count = 0;
                        prev_temperature = sensor->temperature;
                        sensor->temperature = next_temperature;
                        if(sensor->event_loop != NULL) {
                            dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_SAMPLE_EVENT, sensor);
                            if(prev_temperature != sensor->temperature) {
                                dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_TEMPERATURE_CHANGE_EVENT, sensor);
                            }
                        }
                        ESP_LOGI(TAG, "[INFO] Temperature: %f Sensor: %s", sensor->temperature, sensor->rom_code_string);
                    } else {
                        ++error_count;
                        if(sensor->event_loop != NULL) {
                            dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_ERROR_EVENT, sensor);
                        }
                        ESP_LOGE(TAG, "[FAIL] Error reading temperature from %s (error_count = %d)", sensor->rom_code_string, error_count);
                    }
                } else {
                    ++error_count;
                    if(sensor->event_loop != NULL) {
                        dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_ERROR_EVENT, sensor);
                    }
                    ESP_LOGE(TAG, "[FAIL] Error reading temperature from %s: semaphore timed out (error_count = %d)", sensor->rom_code_string, error_count);
                }
            } else {
                ++error_count;
                if(sensor->event_loop != NULL) {
                    dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_ERROR_EVENT, sensor);
                }
                ESP_LOGE(TAG, "[FAIL] Error requesting conversion from %s (error_count = %d)", sensor->rom_code_string, error_count);
            }
        } else {
            ++error_count;
            if(sensor->event_loop != NULL) {
                dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_ERROR_EVENT, sensor);
            }
            ESP_LOGE(TAG, "[FAIL] Error starting conversion from %s: semaphore timed out (error_count = %d)", sensor->rom_code_string, error_count);
        }
        ESP_LOGD(TAG, "[DBG ] Sleeping %d seconds", sensor->sampling_period);
        ESP_LOGD(TAG, "[DBG ] Sensor - Free stack: %d", uxTaskGetStackHighWaterMark(NULL));
        vTaskDelayUntil(&last_sample_time, sensor->sampling_period * 1000 / portTICK_PERIOD_MS);
    }
    if(sensor->event_loop != NULL) {
        dallas_temperature_post_event(sensor->event_loop, DALLAS_TEMPERATURE_SENSOR_LOST_EVENT, sensor);
    }
    ESP_LOGE(TAG, "[FAIL] Sensor %s lost", sensor->rom_code_string);
    sensor->info.init = false; // mark as un-initialized
    vTaskDelete(NULL);
}

void dallas_temperature_post_event(esp_event_loop_handle_t event_loop, int32_t event_id, dallas_temperature_sensor_t * sensor)
{
    if(event_loop == NULL) {
        ESP_LOGE(TAG, "[FAIL] post_event invalid arg");
        return;
    }

    if(esp_event_post_to(event_loop, DALLAS_TEMPERATURE_EVENT_BASE, event_id, &sensor, sizeof(dallas_temperature_sensor_t *), 0) == ESP_OK) {
        ESP_LOGD(TAG, "[DBG ] %s posted to event queue", dallas_temperature_event_id_string[(event_id < DALLAS_TEMPERATURE_EVENT_MAX) ? event_id : DALLAS_TEMPERATURE_EVENT_MAX]);
    } else {
        ESP_LOGE(TAG, "[FAIL] Error posting %s to event queue", dallas_temperature_event_id_string[(event_id < DALLAS_TEMPERATURE_EVENT_MAX) ? event_id : DALLAS_TEMPERATURE_EVENT_MAX]);
    }
}