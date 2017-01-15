# adis16350

## Contents:

1. Arduino source code (SPI communication with adis16350 and UART communication with PC)
2. Calculated acceleration, angular velocity and orientation (using complementary filter) and publishing on topic
3. Services for settings registers on Adis16350 (calibration, bias, sample rate, filters, range)
4. Diagnostic - periodical reading status from Adis16350
5. URDF model and TF broadcaster for visualization and debug
6. Yaml file for configuration initial parameters
7. Documentation. See [wiki](https://github.com/MichalDobis/adis16350/wiki)

## Running example:
    roslaunch adis16350 display.launch
or
    roslaunch adis16350 adis_driver.launch
