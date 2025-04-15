/**
 * GY85.h - Biblioteka do obsługi modułu czujników 9-osiowego GY-85 dla STM32 z HAL
 * Zawiera adresy, stałe i funkcje dla czujników:
 * - ITG3205 (żyroskop)
 * - ADXL345 (akcelerometr)
 * - HMC5883L (magnetometr)
 * - BMP085 (barometr/czujnik ciśnienia)
 */

#ifndef GY85_H
#define GY85_H

#include "stm32f4xx_hal.h" // Zastąp odpowiednim nagłówkiem twojego mikrokontrolera (np. stm32f4xx_hal.h)
#include <stdint.h>
#include <stdio.h>

// Adresy I2C urządzeń (Uwaga: adresy są przesunięte o 1 bit w lewo dla HAL)
#define GY85_ITG3205_ADDR (0x68 << 1)  // Adres I2C żyroskopu ITG3205
#define GY85_ADXL345_ADDR (0x53 << 1)  // Adres I2C akcelerometru ADXL345
#define GY85_HMC5883L_ADDR (0x1E << 1) // Adres I2C magnetometru HMC5883L
#define GY85_BMP085_ADDR (0x77 << 1)   // Adres I2C barometru BMP085

// ********** Rejestry ITG3205 (Żyroskop) **********
#define ITG3205_WHO_AM_I 0x00    // Rejestr identyfikacyjny
#define ITG3205_SMPLRT_DIV 0x15  // Dzielnik częstotliwości próbkowania
#define ITG3205_DLPF_FS 0x16     // Konfiguracja filtru dolnoprzepustowego i zakresu
#define ITG3205_INT_CFG 0x17     // Konfiguracja przerwań
#define ITG3205_INT_STATUS 0x1A  // Status przerwań
#define ITG3205_TEMP_OUT_H 0x1B  // Temperatura - bajt górny
#define ITG3205_TEMP_OUT_L 0x1C  // Temperatura - bajt dolny
#define ITG3205_GYRO_XOUT_H 0x1D // Żyroskop oś X - bajt górny
#define ITG3205_GYRO_XOUT_L 0x1E // Żyroskop oś X - bajt dolny
#define ITG3205_GYRO_YOUT_H 0x1F // Żyroskop oś Y - bajt górny
#define ITG3205_GYRO_YOUT_L 0x20 // Żyroskop oś Y - bajt dolny
#define ITG3205_GYRO_ZOUT_H 0x21 // Żyroskop oś Z - bajt górny
#define ITG3205_GYRO_ZOUT_L 0x22 // Żyroskop oś Z - bajt dolny
#define ITG3205_PWR_MGM 0x3E     // Zarządzanie zasilaniem

// Konfiguracja ITG3205
#define ITG3205_DLPF_FS_FS_SEL 0x18         // Pełna skala +/-2000 stopni/s
#define ITG3205_DLPF_FS_DLPF_CFG_10HZ 0x05  // Filtr dolnoprzepustowy 10Hz
#define ITG3205_DLPF_FS_DLPF_CFG_20HZ 0x04  // Filtr dolnoprzepustowy 20Hz
#define ITG3205_DLPF_FS_DLPF_CFG_42HZ 0x03  // Filtr dolnoprzepustowy 42Hz
#define ITG3205_DLPF_FS_DLPF_CFG_98HZ 0x02  // Filtr dolnoprzepustowy 98Hz
#define ITG3205_DLPF_FS_DLPF_CFG_188HZ 0x01 // Filtr dolnoprzepustowy 188Hz
#define ITG3205_DLPF_FS_DLPF_CFG_256HZ 0x00 // Filtr dolnoprzepustowy 256Hz

#define ITG3205_PWR_MGM_CLK_SEL_INTERNAL 0x00 // Wewnętrzny oscylator
#define ITG3205_PWR_MGM_CLK_SEL_X_GYRO 0x01   // Referencja PLL do żyroskopu X
#define ITG3205_PWR_MGM_CLK_SEL_Y_GYRO 0x02   // Referencja PLL do żyroskopu Y
#define ITG3205_PWR_MGM_CLK_SEL_Z_GYRO 0x03   // Referencja PLL do żyroskopu Z
#define ITG3205_PWR_MGM_STBY_ZG 0x08          // Tryb czuwania osi Z
#define ITG3205_PWR_MGM_STBY_YG 0x10          // Tryb czuwania osi Y
#define ITG3205_PWR_MGM_STBY_XG 0x20          // Tryb czuwania osi X
#define ITG3205_PWR_MGM_SLEEP 0x40            // Tryb uśpienia
#define ITG3205_PWR_MGM_H_RESET 0x80          // Reset urządzenia

// Skala dla żyroskopu ITG3205 (stopnie na sekundę / LSB)
#define ITG3205_SCALE_FACTOR 14.375f // LSB na stopień/s

// ********** Rejestry ADXL345 (Akcelerometr) **********
#define ADXL345_DEVID 0x00          // Identyfikator urządzenia
#define ADXL345_THRESH_TAP 0x1D     // Próg stuknięcia
#define ADXL345_OFSX 0x1E           // Offset osi X
#define ADXL345_OFSY 0x1F           // Offset osi Y
#define ADXL345_OFSZ 0x20           // Offset osi Z
#define ADXL345_DUR 0x21            // Czas trwania stuknięcia
#define ADXL345_LATENT 0x22         // Okres ukryty stuknięcia
#define ADXL345_WINDOW 0x23         // Okno stuknięcia
#define ADXL345_THRESH_ACT 0x24     // Próg aktywności
#define ADXL345_THRESH_INACT 0x25   // Próg nieaktywności
#define ADXL345_TIME_INACT 0x26     // Czas nieaktywności
#define ADXL345_ACT_INACT_CTL 0x27  // Kontrola osi dla aktywności/nieaktywności
#define ADXL345_THRESH_FF 0x28      // Próg swobodnego spadania
#define ADXL345_TIME_FF 0x29        // Czas swobodnego spadania
#define ADXL345_TAP_AXES 0x2A       // Stuknięcie w osie
#define ADXL345_ACT_TAP_STATUS 0x2B // Status aktywności/stuknięcia
#define ADXL345_BW_RATE 0x2C        // Szybkość danych i moc
#define ADXL345_POWER_CTL 0x2D      // Kontrola zasilania
#define ADXL345_INT_ENABLE 0x2E     // Włączenie przerwania
#define ADXL345_INT_MAP 0x2F        // Mapowanie przerwania
#define ADXL345_INT_SOURCE 0x30     // Źródło przerwania
#define ADXL345_DATA_FORMAT 0x31    // Format danych
#define ADXL345_DATAX0 0x32         // Dane osi X - bajt 0
#define ADXL345_DATAX1 0x33         // Dane osi X - bajt 1
#define ADXL345_DATAY0 0x34         // Dane osi Y - bajt 0
#define ADXL345_DATAY1 0x35         // Dane osi Y - bajt 1
#define ADXL345_DATAZ0 0x36         // Dane osi Z - bajt 0
#define ADXL345_DATAZ1 0x37         // Dane osi Z - bajt 1
#define ADXL345_FIFO_CTL 0x38       // Kontrola FIFO
#define ADXL345_FIFO_STATUS 0x39    // Status FIFO

// Konfiguracja ADXL345
#define ADXL345_POWER_CTL_MEASURE 0x08    // Tryb pomiaru
#define ADXL345_POWER_CTL_STANDBY 0x00    // Tryb czuwania
#define ADXL345_POWER_CTL_SLEEP 0x04      // Tryb uśpienia
#define ADXL345_POWER_CTL_WAKEUP_8HZ 0x00 // Częstotliwość budzenia 8Hz
#define ADXL345_POWER_CTL_WAKEUP_4HZ 0x01 // Częstotliwość budzenia 4Hz
#define ADXL345_POWER_CTL_WAKEUP_2HZ 0x02 // Częstotliwość budzenia 2Hz
#define ADXL345_POWER_CTL_WAKEUP_1HZ 0x03 // Częstotliwość budzenia 1Hz

#define ADXL345_DATA_FORMAT_RANGE_2G 0x00  // Zakres ±2g
#define ADXL345_DATA_FORMAT_RANGE_4G 0x01  // Zakres ±4g
#define ADXL345_DATA_FORMAT_RANGE_8G 0x02  // Zakres ±8g
#define ADXL345_DATA_FORMAT_RANGE_16G 0x03 // Zakres ±16g
#define ADXL345_DATA_FORMAT_FULL_RES 0x08  // Pełna rozdzielczość
#define ADXL345_DATA_FORMAT_JUSTIFY 0x04   // Wyrównanie do prawej (bez znaku)
#define ADXL345_DATA_FORMAT_SELF_TEST 0x80 // Tryb autotestu

// Skala dla akcelerometru ADXL345 (g / LSB)
#define ADXL345_SCALE_FACTOR_2G 0.0039f  // 3.9mg/LSB
#define ADXL345_SCALE_FACTOR_4G 0.0078f  // 7.8mg/LSB
#define ADXL345_SCALE_FACTOR_8G 0.0156f  // 15.6mg/LSB
#define ADXL345_SCALE_FACTOR_16G 0.0312f // 31.2mg/LSB

// ********** Rejestry HMC5883L (Magnetometr) **********
#define HMC5883L_CONFIG_A 0x00 // Rejestr konfiguracyjny A
#define HMC5883L_CONFIG_B 0x01 // Rejestr konfiguracyjny B
#define HMC5883L_MODE 0x02     // Rejestr trybu
#define HMC5883L_DATAX_H 0x03  // Dane osi X - bajt górny
#define HMC5883L_DATAX_L 0x04  // Dane osi X - bajt dolny
#define HMC5883L_DATAZ_H 0x05  // Dane osi Z - bajt górny
#define HMC5883L_DATAZ_L 0x06  // Dane osi Z - bajt dolny
#define HMC5883L_DATAY_H 0x07  // Dane osi Y - bajt górny
#define HMC5883L_DATAY_L 0x08  // Dane osi Y - bajt dolny
#define HMC5883L_STATUS 0x09   // Rejestr statusu
#define HMC5883L_ID_A 0x0A     // Rejestr identyfikacyjny A
#define HMC5883L_ID_B 0x0B     // Rejestr identyfikacyjny B
#define HMC5883L_ID_C 0x0C     // Rejestr identyfikacyjny C

// Konfiguracja HMC5883L
#define HMC5883L_CONFIG_A_SAMPLES_1 0x00 // 1 próbka na pomiar
#define HMC5883L_CONFIG_A_SAMPLES_2 0x20 // 2 próbki na pomiar
#define HMC5883L_CONFIG_A_SAMPLES_4 0x40 // 4 próbki na pomiar
#define HMC5883L_CONFIG_A_SAMPLES_8 0x60 // 8 próbek na pomiar
#define HMC5883L_CONFIG_A_RATE_0_75 0x00 // 0.75 Hz
#define HMC5883L_CONFIG_A_RATE_1_5 0x04  // 1.5 Hz
#define HMC5883L_CONFIG_A_RATE_3 0x08    // 3 Hz
#define HMC5883L_CONFIG_A_RATE_7_5 0x0C  // 7.5 Hz
#define HMC5883L_CONFIG_A_RATE_15 0x10   // 15 Hz
#define HMC5883L_CONFIG_A_RATE_30 0x14   // 30 Hz
#define HMC5883L_CONFIG_A_RATE_75 0x18   // 75 Hz
#define HMC5883L_CONFIG_A_NORMAL 0x00    // Normalny tryb pomiaru
#define HMC5883L_CONFIG_A_POSITIVE 0x01  // Pozytywny bias
#define HMC5883L_CONFIG_A_NEGATIVE 0x02  // Negatywny bias

#define HMC5883L_CONFIG_B_GAIN_0_88 0x00 // Wzmocnienie 0.88 Ga
#define HMC5883L_CONFIG_B_GAIN_1_3 0x20  // Wzmocnienie 1.3 Ga
#define HMC5883L_CONFIG_B_GAIN_1_9 0x40  // Wzmocnienie 1.9 Ga
#define HMC5883L_CONFIG_B_GAIN_2_5 0x60  // Wzmocnienie 2.5 Ga
#define HMC5883L_CONFIG_B_GAIN_4_0 0x80  // Wzmocnienie 4.0 Ga
#define HMC5883L_CONFIG_B_GAIN_4_7 0xA0  // Wzmocnienie 4.7 Ga
#define HMC5883L_CONFIG_B_GAIN_5_6 0xC0  // Wzmocnienie 5.6 Ga
#define HMC5883L_CONFIG_B_GAIN_8_1 0xE0  // Wzmocnienie 8.1 Ga

#define HMC5883L_MODE_CONTINUOUS 0x00 // Tryb ciągły
#define HMC5883L_MODE_SINGLE 0x01     // Tryb pojedynczego pomiaru
#define HMC5883L_MODE_IDLE 0x02       // Tryb bezczynności
#define HMC5883L_MODE_SLEEP 0x03      // Tryb uśpienia

// Skala dla magnetometru HMC5883L (Gauss / LSB)
#define HMC5883L_SCALE_0_88 0.73f // 0.73 mG/LSB dla 0.88 Ga
#define HMC5883L_SCALE_1_3 0.92f  // 0.92 mG/LSB dla 1.3 Ga
#define HMC5883L_SCALE_1_9 1.22f  // 1.22 mG/LSB dla 1.9 Ga
#define HMC5883L_SCALE_2_5 1.52f  // 1.52 mG/LSB dla 2.5 Ga
#define HMC5883L_SCALE_4_0 2.27f  // 2.27 mG/LSB dla 4.0 Ga
#define HMC5883L_SCALE_4_7 2.56f  // 2.56 mG/LSB dla 4.7 Ga
#define HMC5883L_SCALE_5_6 3.03f  // 3.03 mG/LSB dla 5.6 Ga
#define HMC5883L_SCALE_8_1 4.35f  // 4.35 mG/LSB dla 8.1 Ga

// ********** Rejestry BMP085 (Barometr) **********
#define BMP085_CAL_AC1 0xAA         // Współczynnik kalibracji AC1
#define BMP085_CAL_AC2 0xAC         // Współczynnik kalibracji AC2
#define BMP085_CAL_AC3 0xAE         // Współczynnik kalibracji AC3
#define BMP085_CAL_AC4 0xB0         // Współczynnik kalibracji AC4
#define BMP085_CAL_AC5 0xB2         // Współczynnik kalibracji AC5
#define BMP085_CAL_AC6 0xB4         // Współczynnik kalibracji AC6
#define BMP085_CAL_B1 0xB6          // Współczynnik kalibracji B1
#define BMP085_CAL_B2 0xB8          // Współczynnik kalibracji B2
#define BMP085_CAL_MB 0xBA          // Współczynnik kalibracji MB
#define BMP085_CAL_MC 0xBC          // Współczynnik kalibracji MC
#define BMP085_CAL_MD 0xBE          // Współczynnik kalibracji MD
#define BMP085_CONTROL 0xF4         // Rejestr kontrolny
#define BMP085_TEMPDATA 0xF6        // Odczyt danych temperatury
#define BMP085_PRESSUREDATA 0xF6    // Odczyt danych ciśnienia
#define BMP085_READTEMPCMD 0x2E     // Komenda odczytu temperatury
#define BMP085_READPRESSURECMD 0x34 // Komenda odczytu ciśnienia

// Tryby nadpróbkowania BMP085
#define BMP085_ULTRALOWPOWER 0x00 // Niskie zużycie energii
#define BMP085_STANDARD 0x01      // Standardowa konfiguracja
#define BMP085_HIGHRES 0x02       // Wysoka rozdzielczość
#define BMP085_ULTRAHIGHRES 0x03  // Ultra wysoka rozdzielczość

// Opóźnienia dla BMP085 (w ms)
#define BMP085_TEMP_DELAY 5          // Opóźnienie odczytu temperatury
#define BMP085_PRESSURE_DELAY_ULP 5  // Opóźnienie odczytu ciśnienia (ultra-low power)
#define BMP085_PRESSURE_DELAY_STD 8  // Opóźnienie odczytu ciśnienia (standard)
#define BMP085_PRESSURE_DELAY_HR 14  // Opóźnienie odczytu ciśnienia (high-res)
#define BMP085_PRESSURE_DELAY_UHR 26 // Opóźnienie odczytu ciśnienia (ultra-high-res)

// Struktura do przechowywania danych z żyroskopu
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    float temperature;
} ITG3205_Data_t;

// Struktura do przechowywania danych z akcelerometru
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ADXL345_Data_t;

// Struktura do przechowywania danych z magnetometru
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883L_Data_t;

// Struktura do przechowywania danych z barometru
typedef struct
{
    int32_t temperature; // w 0.1°C
    int32_t pressure;    // w Pa
    float altitude;      // w metrach
} BMP085_Data_t;

// Struktura do przechowywania współczynników kalibracyjnych BMP085
typedef struct
{
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} BMP085_Calibration_t;

// Struktura główna dla całego modułu GY-85
typedef struct
{
    I2C_HandleTypeDef *hi2c;      // Uchwyt do I2C
    ITG3205_Data_t gyro;          // Dane z żyroskopu
    ADXL345_Data_t accel;         // Dane z akcelerometru
    HMC5883L_Data_t mag;          // Dane z magnetometru
    BMP085_Data_t baro;           // Dane z barometru
    BMP085_Calibration_t bmp_cal; // Dane kalibracyjne barometru
    uint8_t bmp_oss;              // Tryb nadpróbkowania BMP085
} GY85_t;

// Kody błędów
#define GY85_OK 0
#define GY85_ERROR 1
#define GY85_TIMEOUT_ERROR 2
#define GY85_DEVICE_NOT_FOUND 3

// Prototypy funkcji dla całego modułu
HAL_StatusTypeDef GY85_Init(GY85_t *gy85, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef GY85_ReadAllSensors(GY85_t *gy85);

// Prototypy funkcji dla żyroskopu ITG3205
HAL_StatusTypeDef ITG3205_Init(GY85_t *gy85);
HAL_StatusTypeDef ITG3205_ReadData(GY85_t *gy85);
float ITG3205_ConvertTemp(int16_t raw_temp);
float ITG3205_ConvertGyro(int16_t raw_gyro);

// Prototypy funkcji dla akcelerometru ADXL345
HAL_StatusTypeDef ADXL345_Init(GY85_t *gy85);
HAL_StatusTypeDef ADXL345_ReadData(GY85_t *gy85);
float ADXL345_ConvertAccel(int16_t raw_accel, uint8_t range);

// Prototypy funkcji dla magnetometru HMC5883L
HAL_StatusTypeDef HMC5883L_Init(GY85_t *gy85);
HAL_StatusTypeDef HMC5883L_ReadData(GY85_t *gy85);
float HMC5883L_ConvertMag(int16_t raw_mag, uint8_t gain);
float HMC5883L_CalculateHeading(GY85_t *gy85);

// Prototypy funkcji dla barometru BMP085
HAL_StatusTypeDef BMP085_Init(GY85_t *gy85, uint8_t oss);
HAL_StatusTypeDef BMP085_ReadCalibration(GY85_t *gy85);
HAL_StatusTypeDef BMP085_ReadTemperature(GY85_t *gy85);
HAL_StatusTypeDef BMP085_ReadPressure(GY85_t *gy85);
float BMP085_CalculateAltitude(int32_t pressure);

// Funkcje pomocnicze
HAL_StatusTypeDef GY85_WriteReg(GY85_t *gy85, uint8_t device_addr, uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef GY85_ReadReg(GY85_t *gy85, uint8_t device_addr, uint8_t reg_addr, uint8_t *data);
HAL_StatusTypeDef GY85_ReadMulti(GY85_t *gy85, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);

#endif // GY85_H