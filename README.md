# esp32-dallas-temperature

**esp32-dallas-temperature** is an automated background service for ESP32 to read DS18B20 digital temperature sensors in ESP-IDF.

## Features

- Multi-sensor
- Auto-detect sensors
- Hot-plug & Un-plug
- Event-based

## Usage

### Example project

Check out the example project at [https://github.com/pablobacho/esp32-dallas-temperature-example](https://github.com/pablobacho/esp32-dallas-temperature-example)

### Setup workspace

This is an ESP-IDF component. To include it in your project, you can clone this repository into the *components* folder in your project.

    git clone --recurse-submodules https://github.com/pablobacho/esp32-dallas-temperature.git

Notice the `--recurse-submodules` option. This component depends on [esp32-ds18b20](https://github.com/DavidAntliff/esp32-ds18b20) and [esp32-owb](https://github.com/DavidAntliff/esp32-owb) by [DavidAntliff](https://github.com/DavidAntliff).

Include `dallas_temperature.h` in your source files to use it.

    #include "dallas_temperature.h"

### Setting up

This component data is stored in a `dallas_temperature_t` structure. Create an instance of this type in your program and populate it with default values:

    dallas_temperature_t dallas_temperature = DALLAS_TEMPERATURE_DEFAULT();

After creating the structure you can customize its configuration:

    dallas_temperature.config.bus_gpio = (gpio_num_t) 32;
    dallas_temperature.config.enable_crc = true;
    dallas_temperature.config.sampling_period = 5;

For a full list of fields check out its declaration in `dallas_temperature.h`.

Start `dallas_temperature` service:

    dallas_temperature_start(&dallas_temperature);

And register for events:

    esp_event_handler_register_with(dallas_temperature.event_loop, DALLAS_TEMPERATURE_EVENT_BASE, ESP_EVENT_ANY_ID, event_handler, NULL);

Where the parameter `event_handler` is the name of the function you create to handle events. The list of possible events can be found in `dallas_temperature.h`. Check out ESP-IDF documentation on the Event Library for more information.

## Contributing

If you find a bug, improve documentation, add a feature, or anything else, I encourage you to open an issue and/or make a pull request.

When contributing, please follow [Espressif IoT Development Framework Style Guide](https://docs.espressif.com/projects/esp-idf/en/latest/contribute/style-guide.html) and their [Documenting Code Guide](https://docs.espressif.com/projects/esp-idf/en/latest/contribute/documenting-code.html)

Licensed under the MIT License.

Referenced sub-modules are property of its creator(s) and published under their own license. Check out their respective repositories for more information.