//probando
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/_intsup.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "pagina_html.h"
#include "driver/timer.h"
#include "cJSON.h" // Asegúrate de incluir esta librería
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

#define DEFAULT_VREF    1100        // Valor de referencia en mV
#define NO_OF_SAMPLES   64
#define ADC_CHANNEL     ADC1_CHANNEL_6   // GPIO34
#define R_MAX_OHMS      10000.0     // Potenciómetro de 10kΩ
#define MAX_PWM 8191

static esp_adc_cal_characteristics_t *adc_chars;
float resistencia_estim = 0.0;

#define PWM_PIN 2
#define BT_PIN 16

#define ESTADO_INIT         0
#define ESTADO_ESPERA		1
#define ESTADO_ASTABLE      2
#define ESTADO_MONOSTABLE	3
#define ESTADO_PWM         	4
#define ESTADO_ERROR		5
#define ESTADO_DETENER		6

int Func_ESTADO_INIT();
int Func_ESTADO_ESPERA();
int Func_ESTADO_ASTABLE();
int Func_ESTADO_MONOSTABLE();
int Func_ESTADO_PWM();
int Func_ESTADO_ERROR();
int Func_ESTADO_DETENER();

volatile int ESTADO_ACTUAL = ESTADO_INIT;
volatile int ESTADO_SIGUIENTE = ESTADO_INIT;
volatile int ESTADO_ANTERIOR = ESTADO_INIT;
volatile int ESTADO_EJECUTADO = ESTADO_INIT;

volatile bool VALIDAR_MONOSTABLE = false;
//Definicion de la variable utilizada como handle
TaskHandle_t PUNTERO_MONOSTABLE_TASK = NULL;


static TimerHandle_t ASTABLE_TIMER = NULL;
static bool estado_alto = true;  // true = ON, false = OFF

static void ASTABLE_TIMER_callback(TimerHandle_t xTimer);

char modo_string[20], r1_string[20], r2_string[20], c_string[20];

double tiempo_pulso_monostable = 0;
static int t_alto_ms = 0;
static int t_bajo_ms = 0;


void calcular_astable(const char *r1_str, const char *r2_str, const char *c_str);
void iniciar_pwm(double frecuencia, double duty_percent);
void iniciar_pwm_fijo(const char *r2_str);
void calcular_monostable(const char *r1_str, const char *c_str);

static const char *TAG = "555_Emulador";

void CREAR_TIMER(void);

/* Handler para la raíz (/) */
esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, pagina_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_pwm_handler(httpd_req_t *req) {
	
    if (ESTADO_EJECUTADO == ESTADO_PWM) {
 
 		char resp[100];
	    snprintf(resp, sizeof(resp), "{\"resistencia\": %d, \"pwm\": %d}", (int)resistencia_estim, (int)resistencia_estim);
	    httpd_resp_set_type(req, "application/json");
	    httpd_resp_sendstr(req, resp);
    }else {
		
		httpd_resp_send_404(req);
        return ESP_FAIL;
	}

    
    return ESP_OK;
}


/* Handler para /set */
esp_err_t set_post_handler(httpd_req_t *req) {
    char content[200]; // Ajusta el tamaño según tu payload esperado
    int total_len = req->content_len;

    if (total_len >= sizeof(content)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int received = httpd_req_recv(req, content, total_len);
    if (received <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    content[received] = '\0';  // Null-terminate

    ESP_LOGI(TAG, "JSON recibido: %s", content);

    // Parsear JSON
    cJSON *json = cJSON_Parse(content);
    if (!json) {
        ESP_LOGE(TAG, "Error parseando JSON");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    const cJSON *modo = cJSON_GetObjectItem(json, "tipo");
    const cJSON *r1 = cJSON_GetObjectItem(json, "r1");
    const cJSON *r2 = cJSON_GetObjectItem(json, "r2");
    const cJSON *c = cJSON_GetObjectItem(json, "c");
    const cJSON *res = cJSON_GetObjectItem(json, "resistencia");

    if (modo && cJSON_IsString(modo)) {
        if (strcmp(modo->valuestring, "astable") == 0 && r1 && r2 && c) {
         
            ESP_LOGI(TAG, "Modo: %s, R1: %f, R2: %f, C: %f", modo->valuestring, r1->valuedouble, r2->valuedouble, c->valuedouble);
  			snprintf(r1_string, sizeof(r1_string), "%.2f", r1->valuedouble);
	        snprintf(r2_string, sizeof(r2_string), "%.2f", r2->valuedouble);
	        snprintf(c_string, sizeof(c_string), "%.2f", c->valuedouble);
    		ESP_LOGI(TAG, "Modo: %s, R1: %s, R2: %s, C: %s", modo_string, r1_string, r2_string, c_string);
    		ESTADO_EJECUTADO = ESTADO_ASTABLE;
    		
        } else if (strcmp(modo->valuestring, "monostable") == 0 && r1 && c) {
            
            ESP_LOGI(TAG, "Modo: %s, R1: %f, C: %f", modo->valuestring, r1->valuedouble, c->valuedouble);
            snprintf(r1_string, sizeof(r1_string), "%.2f", r1->valuedouble);
	        snprintf(c_string, sizeof(c_string), "%.2f", c->valuedouble);
    		ESP_LOGI(TAG, "Modo: %s, R1: %s, C: %s", modo_string, r1_string, c_string);
    		ESTADO_EJECUTADO = ESTADO_MONOSTABLE;
        } else if (strcmp(modo->valuestring, "pwm") == 0 && res) {
			
            ESP_LOGI(TAG, "Modo: %s, R: %f", modo->valuestring, res->valuedouble);
        	ESTADO_EJECUTADO = ESTADO_PWM;
        } else {
            ESP_LOGW(TAG, "Parámetros inválidos o incompletos para el modo: %s", modo->valuestring);
        }
    }

    cJSON_Delete(json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");

    return ESP_OK;
}

/* Handler para /stop */
esp_err_t stop_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Simulación detenida.");
    ESTADO_EJECUTADO = ESTADO_DETENER;
    httpd_resp_send(req, "<h2>Simulación detenida. <a href='/'>Volver</a></h2>", HTTPD_RESP_USE_STRLEN);
    
    ESTADO_EJECUTADO = ESTADO_DETENER;
    return ESP_OK;
    
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t set_uri = {
		    .uri       = "/set",
		    .method    = HTTP_POST,
		    .handler   = set_post_handler,
		    .user_ctx  = NULL
		};
		httpd_register_uri_handler(server, &set_uri);

        httpd_uri_t stop_uri = {
            .uri       = "/stop",
            .method    = HTTP_GET,
            .handler   = stop_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stop_uri);
        
        httpd_uri_t uri_get_pwm = {
            .uri = "/get_pwm",
            .method = HTTP_GET,
            .handler = get_pwm_handler
        };
        httpd_register_uri_handler(server, &uri_get_pwm);
    }

    return server;
}


void IRAM_ATTR isr_monostable_trigger(void* arg) {
	
	  static uint32_t last_interrupt_time = 0;
    uint32_t current_time = xTaskGetTickCountFromISR();
    
 
    if (VALIDAR_MONOSTABLE && (current_time - last_interrupt_time > pdMS_TO_TICKS(200))) {
        last_interrupt_time = current_time;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(PUNTERO_MONOSTABLE_TASK, 0, eNoAction, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

void TAREA_MONOSTABLE(void *pvParameters) {
    while (1) {
        // Espera hasta que reciba notificación de la ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Calcular el tiempo alto basado en R y C
        double r = atof(r1_string); // Ohmios
        double c = atof(c_string);  // microfaradios
        double tiempo_alto_s = 1.1 * r * (c * 1e-6);
        int tiempo_ms = (int)(tiempo_alto_s * 1000);

        ESP_LOGI("MONOSTABLE", "Trigger! Encendiendo salida por %d ms", tiempo_ms);

        // Activa la salida (ej. LED, PWM, etc.)
        gpio_set_level(PWM_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(tiempo_ms));
        gpio_set_level(PWM_PIN, 0);
    }
}

void app_main(void) {
	
    nvs_flash_init();
    
    
    while (true) {
		
		if (ESTADO_SIGUIENTE == ESTADO_INIT)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_INIT();
            gpio_intr_disable(BT_PIN);
            start_webserver();
        }
        if (ESTADO_SIGUIENTE == ESTADO_ESPERA)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_ESPERA();
        }
        if (ESTADO_SIGUIENTE == ESTADO_ASTABLE)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_ASTABLE();
        }
        if (ESTADO_SIGUIENTE == ESTADO_MONOSTABLE)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_MONOSTABLE();
        }
        if (ESTADO_SIGUIENTE == ESTADO_PWM)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_PWM();
        }
        if (ESTADO_SIGUIENTE == ESTADO_ERROR)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_ERROR();
        }
        if (ESTADO_SIGUIENTE == ESTADO_DETENER)
        {
            ESTADO_SIGUIENTE = Func_ESTADO_DETENER();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
	}
    
}

int Func_ESTADO_INIT(){
	
	
	/*ESP_LOGI("ESTADO INIT","INICIALIZANDO LEDC");
	ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configurar el canal del LEDC
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 0,
        .gpio_num   = PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_1
    };
    ledc_channel_config(&ledc_channel);*/
	
	gpio_config_t IO_CONFIG;
	
		IO_CONFIG.mode = GPIO_MODE_OUTPUT;
        IO_CONFIG.pin_bit_mask = (1ULL << PWM_PIN);  
        IO_CONFIG.pull_down_en = GPIO_PULLDOWN_DISABLE;
        IO_CONFIG.pull_up_en = GPIO_PULLUP_DISABLE;
        IO_CONFIG.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&IO_CONFIG);
        
        IO_CONFIG.mode = GPIO_MODE_INPUT,
        IO_CONFIG.pin_bit_mask = (1ULL << BT_PIN);
        IO_CONFIG.pull_down_en = GPIO_PULLDOWN_DISABLE;
        IO_CONFIG.pull_up_en = GPIO_PULLUP_ENABLE;
	    IO_CONFIG.intr_type = GPIO_INTR_NEGEDGE;
		gpio_config(&IO_CONFIG);
		
		gpio_install_isr_service(0);
		gpio_isr_handler_add(BT_PIN, isr_monostable_trigger, NULL);
		 
		xTaskCreate(TAREA_MONOSTABLE,
		    "TAREA_MONOSTABLE",
		    2048,
		    NULL,
		    5,
		    &PUNTERO_MONOSTABLE_TASK);
		    
		adc1_config_width(ADC_WIDTH_BIT_12);
	    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
	
	    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
	                             ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
/*	
	ESP_LOGI("ESP32_INIT", "INICIALIZANDO CONFIGURACION DEL TIMER...");

        XTIMER_ASIGNACION_PINES = xTimerCreate("Timer_INTRR_200MS", 
                            (pdMS_TO_TICKS(TIMER_INTERVALO)),
                            pdTRUE, 
                            ( void * )TIMER_ID, 
                            ASIGNACION_GPIO_50MS
        );
            if (XTIMER_ASIGNACION_PINES == NULL)
            {
                ESP_LOGE("ESP32_INIT", "EL TIMER NO FUE CREADO");
            }else 
            {
                if (xTimerStart(XTIMER_ASIGNACION_PINES, 0) != pdPASS)
                {
                    ESP_LOGE("ESP32_INIT", "EL TIMER PODRIA NO ESTAR SETEADO EN EL ESTADO ACTIVO");
                }
                else
                {
                    ESP_LOGI("ESP32_INIT", "EL TIMER FUE CREADO CON EXITO");
                } 
            }
         */   
     ESP_LOGI("ESP32_INIT", "INICIALIZANDO CONFIGURACION DEL WIFI AP...");
     
     esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32_Emulador555",
            .ssid_len = strlen("ESP32_Emulador555"),
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen("12345678") == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Punto de acceso iniciado. SSID: ESP32_Emulador555");
     
     
	
	return ESTADO_ESPERA;
}
int Func_ESTADO_ESPERA(){
	
	ESP_LOGI("STATE MACHINE"," ESTADO ESPERA");
	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_ESPERA;
	ESTADO_EJECUTADO = -1;
	
	
	while (true) {
		
		if (ESTADO_EJECUTADO == ESTADO_ASTABLE) {
			
			return ESTADO_ASTABLE;
		}
		if (ESTADO_EJECUTADO == ESTADO_MONOSTABLE) {
			
			return ESTADO_MONOSTABLE;
		}
		if (ESTADO_EJECUTADO == ESTADO_PWM) {
			
			return ESTADO_PWM;
		}
		if (ESTADO_ACTUAL == ESTADO_ERROR) {
			
			return ESTADO_ERROR;
		}
		if (ESTADO_EJECUTADO == ESTADO_DETENER) {
			
			return ESTADO_DETENER;
		}
		
		vTaskDelay(pdMS_TO_TICKS(100));

	}
	
}
int Func_ESTADO_ASTABLE(){
	
	ESP_LOGI("STATE MACHINE"," ESTADO ASTABLE");
	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_ASTABLE;
	gpio_set_direction(2, GPIO_MODE_OUTPUT);
	
	double r1_n = atof(r1_string);  // Ohm
    double r2_n = atof(r2_string);
    double c_n  = atof(c_string);   // microFaradios

    // Convertir a segundos (1uF = 1e-6F)
    double c_farads = c_n * 1e-6;

    double t_alto = 0.693 * (r1_n + r2_n) * c_farads;
    double t_bajo = 0.693 * r2_n * c_farads;
    double periodo = t_alto + t_bajo;
    double frecuencia = 1.0 / periodo;
    double duty = (t_alto / periodo) * 100;
    
	t_alto_ms = (int)(t_alto * 1000.0);
	t_bajo_ms = (int)(t_bajo * 1000.0);
	
	
	//////////////////////////////
    ESP_LOGI(TAG, "== MODO ASTABLE ==");
    ESP_LOGI(TAG, "R1 = %.2f Ohm, R2 = %.2f Ohm, C = %.6f F", r1_n, r2_n, c_farads);
    ESP_LOGI(TAG, "T_alto = %.6f s", t_alto);
    ESP_LOGI(TAG, "T_bajo = %.6f s", t_bajo);
    ESP_LOGI(TAG, "Periodo = %.6f s", periodo);
    ESP_LOGI(TAG, "Frecuencia = %.2f Hz", frecuencia);
    ESP_LOGI(TAG, "Duty cycle = %.2f %%", duty);
    
    CREAR_TIMER();
	   
    
  //  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_duty_int);
//	ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
	
	while (true) {
		
		
		if (ESTADO_EJECUTADO == ESTADO_ASTABLE) {
			
			return ESTADO_ESPERA;
		}
		if (ESTADO_EJECUTADO == ESTADO_MONOSTABLE) {
			
			return ESTADO_MONOSTABLE;
		}
		if (ESTADO_EJECUTADO == ESTADO_PWM) {
			
			return ESTADO_PWM;
		}
		if (ESTADO_EJECUTADO == ESTADO_DETENER) {
			ESTADO_ANTERIOR = ESTADO_ASTABLE;
			return ESTADO_DETENER;
		}
/*		if (ESTADO_ACTUAL == ESTADO_ERROR) {
			
			return ESTADO_ERROR;
		}*/


		vTaskDelay(pdMS_TO_TICKS(100));
	}
	
}

int Func_ESTADO_MONOSTABLE(){
	
	ESP_LOGI("STATE MACHINE"," ESTADO MONOSTABLE");
	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_MONOSTABLE;
	
	VALIDAR_MONOSTABLE = true;
	//vTaskResume(PUNTERO_MONOSTABLE_TASK);
	gpio_intr_enable(BT_PIN); 
	
	return ESTADO_ESPERA;
	
}
int Func_ESTADO_PWM(){
	
	ESP_LOGI("STATE MACHINE"," ESTADO PWM");
	ESTADO_ANTERIOR = ESTADO_ACTUAL;
	ESTADO_ACTUAL = ESTADO_PWM;
	
	ledc_timer_config_t timer = {
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = 1000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
ledc_timer_config(&timer);

ledc_channel_config_t channel = {
    .channel    = LEDC_CHANNEL_0,
    .duty       = 0,
    .gpio_num   = 2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_sel  = LEDC_TIMER_0
};
ledc_channel_config(&channel);
	
    
    while (true) {
		
		uint32_t adc_reading = 0;
	    for (int i = 0; i < NO_OF_SAMPLES; i++) {
	        adc_reading += adc1_get_raw(ADC_CHANNEL);
	    }
	    adc_reading /= NO_OF_SAMPLES;
	   	adc_reading = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
	   	float voltaje = (adc_reading / 1000.0);
		resistencia_estim = ((voltaje / 3.3)) * R_MAX_OHMS;
		
		printf("Voltaje: %.2f V | Porcentaje: %.1f%% | Resistencia estimada: %.2f ohm\n",
               voltaje, (voltaje / 3.3) * 100.0, resistencia_estim);
        int pwm_val = (int)((resistencia_estim / R_MAX_OHMS) * MAX_PWM);
        
		pwm_val = fmin(MAX_PWM, fmax(0, pwm_val));
		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_val);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            if (ESTADO_EJECUTADO == ESTADO_DETENER) {
				
				ESTADO_ANTERIOR = ESTADO_PWM;
				return ESTADO_DETENER;
			}
			
			vTaskDelay(pdMS_TO_TICKS(100));
	}
	
}
int Func_ESTADO_ERROR(){
	
	ESP_LOGI("STATE MACHINE"," ESTADO ERROR");
	return ESTADO_ESPERA;
}

int Func_ESTADO_DETENER(){
	
	
	
	ESP_LOGI("STATE MACHINE"," ESTADO DETENER");
	ESTADO_ACTUAL = ESTADO_DETENER;
	
	while (true) {
		
		if (ESTADO_ANTERIOR == ESTADO_MONOSTABLE) {
			VALIDAR_MONOSTABLE = false;
			vTaskSuspend(PUNTERO_MONOSTABLE_TASK);
			//gpio_intr_disable(BT_PIN); 
			ESP_LOGI("STATE MACHINE","MONOSTABLE DETENIDO");
			return ESTADO_ESPERA;
		}
		if (ESTADO_ANTERIOR == ESTADO_ASTABLE) {
			
			gpio_set_level(PWM_PIN, 0);
			xTimerStop(ASTABLE_TIMER, 0);
			ESP_LOGI("STATE MACHINE","ASTABLE DETENIDO");
			return ESTADO_ESPERA;
		}
		if (ESTADO_ANTERIOR == ESTADO_PWM) {
			
			ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
			ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
			ESP_LOGI("STATE MACHINE","PWM DETENIDO");
			return ESTADO_ESPERA;
		}
		
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	
	return ESTADO_ESPERA;
}


void CREAR_TIMER(){
	
	 if (ASTABLE_TIMER == NULL) {
		    ASTABLE_TIMER = xTimerCreate("AstableTimer", pdMS_TO_TICKS((int)t_alto_ms), pdFALSE, NULL, ASTABLE_TIMER_callback);
		    xTimerStart(ASTABLE_TIMER, 0);
		    estado_alto = true;
		} else {
		    xTimerStop(ASTABLE_TIMER, 0);
		    xTimerChangePeriod(ASTABLE_TIMER, pdMS_TO_TICKS((int)t_alto_ms), 0);
		    xTimerStart(ASTABLE_TIMER, 0);
		    estado_alto = true;
		}
}

static void ASTABLE_TIMER_callback(TimerHandle_t xTimer) {
    if (estado_alto) {
        gpio_set_level(PWM_PIN, 0);  // Apagar salida
        xTimerChangePeriod(ASTABLE_TIMER, pdMS_TO_TICKS((int)t_bajo_ms), 0);
        estado_alto = false;
    } else {
        gpio_set_level(PWM_PIN, 1);  // Encender salida
        xTimerChangePeriod(ASTABLE_TIMER, pdMS_TO_TICKS((int)t_alto_ms), 0);
        estado_alto = true;
    }
}
//vTaskResume(task_estado_puerta_handle);
//vTaskSuspend(task_estado_puerta_handle);