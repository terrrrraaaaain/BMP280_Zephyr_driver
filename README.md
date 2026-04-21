# BMP280 driver for Zephyr RTOS
Driver for temperature and pressure sensor Bosch BMP280.
Compatible with Zephyr RTOS
## Current Functionality
- I2C communitacion
- Fully customizable via Kconfig, Device Tree and runtime via  ```sensor_attr_set```
- Supports Zephyr Sensor API
- Supports Bosch Sensortec Compensation Formula
## Quick Start
1. Copy this repo to your project
2. Add to main `CMakeLists.txt` file this declaration:
    ```CMake
    get_filename_component(BMP280_LIB_PATH "FULL/PATH/TO/DRIVER" ABSOLUTE)
    set(ZEPHYR_EXTRA_MODULES ${BMP280_LIB_PATH})
    ```
3.  Add to `prj.conf` following lines to enable usage of driver:
    ```kconfig
    CONFIG_SENSOR=y
    CONFIG_BMP280=y
    ```
4. Define node in `app.overlay` like this:
    ```
    #include <dt-bindings/sensor/custom_bmp280.h> // include predefined constants

    &i2c0 {
        status = "okay";
        pinctrl-0 = <&i2c0_alt>;
        pinctrl-names = "default";
        clock-frequency = <100000>; 
    
        bmp280: bmp280@76 {
            compatible = "custom,bmp280-i2c";
            reg = <0x76>;
            status = "okay";
            # set some configs (can be done also using sensor_attr_set in code)  
            mode = <BMP280_MODE_NORMAL>;
            temperature-oversampling = <BMP280_OVERSAMPLING_X16>;
            pressure-oversampling = <BMP280_OVERSAMPLING_X16>;
            iir-filter = <BMP280_FILTER_X8>;
            t-standby = <BMP280_T_STANDBY_0_5MS>;
        };
    
    };
    ```
## Example
```c
    #include <zephyr/kernel.h>
    #include <zephyr/drivers/sensor.h>
    #include <zephyr/drivers/sensor/custom_bmp280.h>  // include predefined constatnts for sensor_attr_set

    int main()
    {
        const struct device *bmp = DEVICE_DT_GET_ANY(custom_bmp280_i2c);
        if (!device_is_ready(bmp)){
            printf("Device not ready");
            return -ENODEV;
        }
        /* set custom attribute MODE to NORMAL.*/
        sensor_attr_set(bmp, SENSOR_CHAN_ALL, BMP280_ATTR_MODE, &BMP280_MODE_NORMAL);

        struct sensor_value temp, pres;

        while(1){
            sensor_sample_fetch(bmp);
            sensor_channel_get(bmp, SENSOR_CHAN_PRESS, &pres);
            sensor_channel_get(bmp, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            printk("Temp: %d C, %duC\n", temp.val1, temp.val2);
            printk("Press: %dk Pa, %dmPa\n", pres.val1, pres.val2);
            k_msleep(1000);
        }
        return 0;
    }
```
## Further Development
- [ ] SPI communication
- [ ] Read and Decode API

### Licence
Released under Apache-2.0 licence
### Author
Franciszek Trzeciak