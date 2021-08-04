# esp8266-rtos-sdk-i2c-bme280
`BME280` (temperature, pressure & humiditty) and `BMP280` (temperature & pressure) sensors `I2C` driver port of [RyAndrew driver](https://github.com/RyAndrew/esp8266_i2c_bme280) for ESP8266 RTOS SDK.  
Configured for `msys2` toolchain, with VSCode IDE on Windows: [Standard Setup of Toolchain for Windows](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/windows-setup.html)  

## Toolchain setup guide
* Clone git repository `https://github.com/espressif/ESP8266_RTOS_SDK.git` into `C:\Espressif` folder. Alternative path can be used.  
Checkout lastest release branch. In my case `origin/release/v3.4` worked fine.  
Add system environment variable `IDF_PATH` with `C:\Espressif\ESP8266_RTOS_SDK` value.  

* Download [esp32_win32_msys2_environment_and_toolchain-20181001.zip](https://dl.espressif.com/dl/esp32_win32_msys2_environment_and_toolchain-20181001.zip)  
Unzip into `C:\` (Toolchain should be located at `C:\msys32`, otherwise you will need to correct the `"terminal.integrated.shell.windows"` path in `.vscode\settings.json`)  
Add system environment variable `MSYS2` with `C:\msys32` value. 

* Download toolchain for the ESP8266 [xtensa-lx106-elf-gcc8_4_0-esp-2020r3-win32.zip](https://dl.espressif.com/dl/xtensa-lx106-elf-gcc8_4_0-esp-2020r3-win32.zip)  
Unzip into `C:\msys32\opt`

* To provide **xtensa-lx106 toolchain** and **python** paths for environment,  
create `esp8266_toolchain.sh` file in the `C:\msys32\etc\profile.d` folder with:

      export PATH="$PATH:/opt/xtensa-lx106-elf/bin"  
      export PATH="$PATH:/mingw32/bin"

## Build and Flash
Check for `CONFIG_ESPTOOLPY_PORT` value in `sdkconfig`.  

`VSCode` -> `Ctrl + Shift + B`:  
* **Build** - to build project  
* **Flash** - to flash project
