#ifndef BRD_CONFIG_HG
#define BRD_CONFIG_HG

//***********************************************************************************
// include files
//***********************************************************************************
// system include files


// silicon Labs include files
#include "em_cmu.h"
#include "em_gpio.h"

// developer included files


//***********************************************************************************
// defined macros
//***********************************************************************************
// I2C peripheral configuration
// compiler directive to use application specific I2C0 peripheral
#define APP_I2C0

#ifdef APP_I2C0
  #define APP_I2Cn              I2C0                        // I2C0 peripheral
  #define APP_I2Cn_SDA_ROUTE    I2C_ROUTELOC0_SDALOC_LOC15  // SDA PC10: route location #15 for I2C0 (TRM 6.3 pg 78)
  #define APP_I2Cn_SCL_ROUTE    I2C_ROUTELOC0_SCLLOC_LOC15  // SCL PC11: route location #15 for I2C0 (TRM 6.3 pg 75)
#else
  #define APP_I2Cn              I2C1                        // I2C1 peripheral
  #define APP_I2Cn_SDA_ROUTE    I2C_ROUTELOC0_SDALOC_LOC19  // SDA PC10: route location #19 for I2C1 (TRM 6.3 pg 78)
  #define APP_I2Cn_SCL_ROUTE    I2C_ROUTELOC0_SCLLOC_LOC19  // SCL PC11: route location #19 for I2C1 (TRM 6.3 pg 75)
#endif

// Si7021 configuration
#define SI7021_SCL_PORT         gpioPortC               // port c (UG257 6.4)
#define SI7021_SCL_PIN          11u                     // Pin 11 (UG257 6.4)
#define SI7021_SDA_PORT         gpioPortC               // port c (UG257 6.4)
#define SI7021_SDA_PIN          10u                     // pin 10 (UG257 6.4)
#define SI7021_SENSOR_EN_PORT   gpioPortB               // port b (UG257 6.4)
#define SI7021_SENSOR_EN_PIN    10u                     // pin 10 (UG257 6.4)
#define SI7021_SENSOR_CONFIG    gpioModePushPull        // push-pull output (TRM 31.5.2 MODE10 pg 1111)
#define SI7021_DRIVE_STRENGTH   gpioDriveStrengthWeak   // drive strength weak (TRM 31.5.1)
#define SI7021_WIREDAND         gpioModeWiredAnd        // WiredAnd configuration (TRM 31.3.1)
#define SI7021_DEFAULT_0        0                       // I2C lines are isolated, sensor is not powered (UG257 6.4)
#define SI7021_DEFAULT_1        1                       // Sensor is powered and connected (UG257 6.4)

// SHTC3 configuration (EFM32 User Guide: UG257 4.2)
#define SHTC3_SDA_ROUTE_LOC     I2C_ROUTELOC0_SDALOC_LOC6   // Connected to EXP Header Pin 7
#define SHTC3_SDA_PORT          gpioPortB                   // GPIO Port B
#define SHTC3_SDA_PIN           6u                          // GPIO Pin 6
#define SHTC3_SCL_ROUTE_LOC     I2C_ROUTELOC0_SCLLOC_LOC6   // Connected to EXP Header Pin 9
#define SHTC3_SCL_PORT          gpioPortB                   // GPIO Port B
#define SHTC3_SCL_PIN           7u                          // GPIO Pin 7
#define SHTC3_DRIVE_STRENGTH    gpioDriveStrengthWeak       // 1mA drive current
#define SHTC3_WIREDAND          gpioModeWiredAnd            // open-drain output
#define SHTC3_SENSOR_GPIO_MODE  gpioModePushPull            // push-pull output
#define SHTC3_SENSOR_EN_PORT    gpioPortB                   // Port enabled
#define SHTC3_LINE_DEFAULT      1                           // Line is powered and connected

// GPIO pin configuration
#define STRONG_DRIVE

#ifdef STRONG_DRIVE
  #define LED0_DRIVE_STRENGTH   gpioDriveStrengthStrongAlternateStrong
  #define LED1_DRIVE_STRENGTH   gpioDriveStrengthStrongAlternateStrong
#else
  #define LED0_DRIVE_STRENGTH   gpioDriveStrengthWeakAlternateWeak
  #define LED1_DRIVE_STRENGTH   gpioDriveStrengthWeakAlternateWeak
#endif

// LED 0 pin configuration
#define LED0_PORT               gpioPortF         // Port F
#define LED0_PIN                04u               // Pin 4
#define LED0_DEFAULT            false             // Default false (0) = off, true (1) = on
#define LED0_GPIOMODE           gpioModePushPull  // Push -pull mode

// LED 1 pin configuration
#define LED1_PORT               gpioPortF         // Port F
#define LED1_PIN                05u               // Pin 5
#define LED1_DEFAULT            false             // Default false (0) = off, true (1) = on
#define LED1_GPIOMODE           gpioModePushPull  // Push-pull mode


// System Clock setup
#define MCU_HFXO_FREQ           cmuHFRCOFreq_32M0Hz   // Configure HFRC to 32MHz


// LETIMER PWM Configuration
#define PWM_ROUTE_0             28  // PWM route location value
#define PWM_ROUTE_1             29  // PWM route location value


//***********************************************************************************
// enums
//***********************************************************************************


//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************

#endif
