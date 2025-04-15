/**
 * GY85.c - Implementacja biblioteki do obsługi modułu czujników 9-osiowego GY-85 dla STM32 z HAL
 */

#include "gy85.h"
#include <math.h>

/**
 * @brief Inicjalizacja całego modułu GY-85
 * @param gy85: Wskaźnik na strukturę GY85
 * @param hi2c: Wskaźnik na strukturę uchwytu I2C
 * @return HAL_StatusTypeDef: Status operacji
 */
HAL_StatusTypeDef GY85_Init(GY85_t *gy85, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    // Zapisanie uchwytu I2C
    gy85->hi2c = hi2c;

    // Inicjalizacja żyroskopu
    status = ITG3205_Init(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Inicjalizacja akcelerometru
    status = ADXL345_Init(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Inicjalizacja magnetometru
    status = HMC5883L_Init(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Inicjalizacja barometru
    status = BMP085_Init(gy85, BMP085_STANDARD);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief Odczyt danych ze wszystkich czujników
 * @param gy85: Wskaźnik na strukturę GY85
 * @return HAL_StatusTypeDef: Status operacji
 */
HAL_StatusTypeDef GY85_ReadAllSensors(GY85_t *gy85)
{
    HAL_StatusTypeDef status;

    // Odczyt danych z żyroskopu
    status = ITG3205_ReadData(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Odczyt danych z akcelerometru
    status = ADXL345_ReadData(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Odczyt danych z magnetometru
    status = HMC5883L_ReadData(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Odczyt danych z barometru
    status = BMP085_ReadTemperature(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BMP085_ReadPressure(gy85);
    if (status != HAL_OK)
    {
        return status;
    }

    // Obliczenie wysokości na podstawie ciśnienia
    gy85->baro.altitude = BMP085_CalculateAltitude(gy85->baro.pressure);

    return HAL_OK;
}

// @TODO: implement rest functions