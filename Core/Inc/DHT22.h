
#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "main.h"

/* Структура возвращаемых датчиком данных */
typedef struct {
    float hum;
    float temp;
} DHT_data;
/* Тип используемого датчика */
typedef enum {
    DHT11,
    DHT22
} DHT_type;

/* Настройки */
#define DHT_Port GPIOA //Группа линии данных
#define DHT_Pin GPIO_PIN_5 //Пин линии данных
#define DHT_PullUp 0 //Нужно ли включать внутреннюю подтяжку пина к питанию
#define DHT_timeout 10000 //Количество итераций, после которых функция вернёт пустые значения

/* Прототипы функций */
DHT_data DHT_getData(DHT_type t); //Получить данные с датчика

#endif /* INC_DHT22_H_ */
