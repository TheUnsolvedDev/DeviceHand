MPU6050 PIN Config.

| **Pin** | **Name**                     | **Description**                                                                              |
|---------|------------------------------|----------------------------------------------------------------------------------------------|
| 1       | Vcc                          | Provides power for the module, can be +3V to +5V. Typically +5V is used                      |
| 2       | Ground                       | Connected to Ground of system                                                                |
| 3       | Serial Clock (SCL)           | Used for providing clock pulse for I2C Communication                                         |
| 4       | Serial Data (SDA)            | Used for transferring Data through I2C communication                                         |
| 5       | Auxiliary Serial Data (XDA)  | Can be used to interface other I2C modules with MPU6050. It is optional                      |
| 6       | Auxiliary Serial Clock (XCL) | Can be used to interface other I2C modules with MPU6050. It is optional                      |
| 7       | AD0                          | If more than one MPU6050 is used a single MCU, then this pin can be used to vary the address |
| 8       | Interrupt (INT)              | Interrupt pin to indicate that data is available for MCU to read.                            |