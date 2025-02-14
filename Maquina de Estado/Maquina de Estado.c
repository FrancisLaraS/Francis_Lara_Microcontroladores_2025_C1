#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definición de los estados posibles
typedef enum {
    ABIERTO,
    CERRADO,
    RESET,
    DESCONOCIDO,
    FALLA
} Estado;

// Variable global para el estado actual
Estado estado_actual = DESCONOCIDO;

// Máquina de estados
void maquina_de_estados() {
    switch (estado_actual) {
        case ABIERTO:
            printf("Estado: ABIERTO\n");
            vTaskDelay(50 / portTICK_PERIOD_MS);
            estado_actual = CERRADO; // Ejemplo de transición
            break;
        
        case CERRADO:
            printf("Estado: CERRADO\n");
            vTaskDelay(50 / portTICK_PERIOD_MS);
            estado_actual = RESET;
            break;
        
        case RESET:
            printf("Estado: RESET\n");
            vTaskDelay(50 / portTICK_PERIOD_MS);
            estado_actual = FALLA;
            break;
        
        case DESCONOCIDO:
            printf("Estado: DESCONOCIDO\n");
            vTaskDelay(50 / portTICK_PERIOD_MS);
            estado_actual = ABIERTO;
            break;
        
        case FALLA:
            printf("Estado: FALLA\n");
            vTaskDelay(50 / portTICK_PERIOD_MS);
            estado_actual = DESCONOCIDO;
            break;
        
        default:
            printf("Estado desconocido! Reiniciando...\n");
            estado_actual = DESCONOCIDO;
            break;
    }
}

// Función principal en FreeRTOS para ESP32
void app_main() {
    while (1) {
        maquina_de_estados();
    }
}
