#ifndef _SIG_GEN_CONFIG_H_
#define _SIG_GEN_CONFIG_H_

/*
Общий объем памяти у f407 - 192kb sram

Прерывание таймера семплирования для генераторов, которые работают не по DMA,
должны иметь более высокий приоритет!
*/

#define FP_TYPE float

#define POINTS_PER_PERIOD_NUM 16 // количество точек на минимальный период несущего сигнала
#define CARRIER_FREQ_MAX_HZ 1000 // максимальная частота несущего сигнала

// Параметры генератора DMA
#define DMA_GEN_TOTAL_NUM 4 // общее количество генераторов DMA
#define BUF_DATA_TYPE int32_t // тип данных для DMA
#define DMA_BUF_SIZE (2 * POINTS_PER_PERIOD_NUM) // размер буфера на один генератор

// Параметры генератора без DMA
#define IT_CARRIER_FREQ_MIN_HZ 1 // минимальная частота несущего сигнала
#define IT_BUF_DATA_TYPE int8_t
#define IT_BUF_SIZE ((CARRIER_FREQ_MAX_HZ - IT_CARRIER_FREQ_MIN_HZ) / IT_CARRIER_FREQ_MIN_HZ * POINTS_PER_PERIOD_NUM)
#define SAMPLE_RATE (POINTS_PER_PERIOD_NUM * CARRIER_FREQ_MAX_HZ) // частота семплирования таймера без DMA / частота вывода DMA

// Параметры задач FreeRTOS
#define RTOS_TASK_STACK_SIZE 1024

#endif // #ifndef _SIG_GEN_CONFIG_H_
