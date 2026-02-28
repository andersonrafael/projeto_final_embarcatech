// =============================================================================
// laser_distance.c
// BitDogLab — Monitoramento com FreeRTOS + MQTT HiveMQ
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "ws2812.pio.h"
#include "mpu6050.h"
#include "display.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

// =============================================================================
// CONFIGURAÇÕES
// =============================================================================

#define WIFI_SSID               "SEU_WIFI_AQUI"
#define WIFI_PASSWORD           "SUA_SENHA_AQUI"

#define MQTT_BROKER             "broker.hivemq.com"
#define MQTT_BROKER_PORT        1883
#define MQTT_CLIENT_ID          "bitdoglab_pico_001"

#define TOPIC_DISTANCIA         "bitdoglab/distancia"
#define TOPIC_INCLINACAO        "bitdoglab/inclinacao"
#define TOPIC_ALERTA            "bitdoglab/alerta"
#define TOPIC_LEDS              "bitdoglab/leds"
#define TOPIC_STATUS            "bitdoglab/status"

#define I2C0_PORT               i2c0
#define BUZZER_PIN              21
#define LED_PIN                 7
#define BUTTON_A                5
#define NUM_PIXELS              25

#define VL53L0X_ADDR            0x29
#define REG_IDENT_ID            0xC0
#define REG_START               0x00
#define REG_DIST_MM             0x1E

#define MQTT_PUBLISH_INTERVAL_MS 5000

// Tempo mínimo entre acionamentos do botão (debounce por software)
#define BUTTON_DEBOUNCE_MS      200

// =============================================================================
// ESTRUTURA DE DADOS
// =============================================================================

typedef struct {
    int     distancia_mm;
    float   inclinacao_deg;
    bool    alarme_colisao;
    bool    alarme_inclinacao;
    bool    leds_ativos;
} SensorData_t;

// =============================================================================
// VARIÁVEIS GLOBAIS
// =============================================================================

static QueueHandle_t    xSensorQueue    = NULL;
static mqtt_client_t   *mqtt_client     = NULL;
static volatile bool    mqtt_conectado  = false;

// CORREÇÃO DO BOTÃO:
// leds_enabled é escrita na ISR e lida nas tasks — precisa ser volatile
// para o compilador não fazer cache em registrador.
// O acesso é atômico em variável bool no RP2040 (single-core aqui), seguro.
static volatile bool    leds_enabled    = true;

// Timestamp do último acionamento do botão (debounce)
static volatile uint32_t ultimo_btn_ms  = 0;

// =============================================================================
// HOOKS DE DIAGNÓSTICO
// =============================================================================

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("ERRO FATAL: Stack overflow na task '%s'!\n", pcTaskName);
    while (1) { tight_loop_contents(); }
}

void vApplicationMallocFailedHook(void)
{
    printf("ERRO FATAL: Heap esgotado!\n");
    while (1) { tight_loop_contents(); }
}

// =============================================================================
// INTERRUPÇÃO DO BOTÃO A
//
// CORREÇÃO: o botão era lido por polling na vSensorTask (a cada 100ms),
// perdendo presses rápidos. Agora usa GPIO interrupt — resposta imediata
// independente do estado das tasks.
//
// Debounce por software: ignora presses com menos de 200ms de intervalo.
// =============================================================================

static void gpio_irq_handler(uint gpio, uint32_t events)
{
    if (gpio != BUTTON_A) return;
    if (!(events & GPIO_IRQ_EDGE_FALL)) return; // só borda de descida (press)

    uint32_t agora = to_ms_since_boot(get_absolute_time());
    if ((agora - ultimo_btn_ms) < BUTTON_DEBOUNCE_MS) return; // debounce
    ultimo_btn_ms = agora;

    leds_enabled = !leds_enabled;

    // printf dentro de ISR não é ideal, mas funciona no Pico com USB serial
    printf("[BTN] LEDs: %s\n", leds_enabled ? "ATIVADOS" : "SILENCIADOS");
}

// =============================================================================
// FUNÇÕES AUXILIARES — NeoPixel
// =============================================================================

static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static void set_leds_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = leds_enabled
        ? (((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b)
        : 0;
    for (int i = 0; i < NUM_PIXELS; i++) put_pixel(color);
}

// =============================================================================
// FUNÇÕES AUXILIARES — VL53L0X
// =============================================================================

static bool vl53l0x_init(void)
{
    uint8_t id, reg = REG_IDENT_ID;
    if (i2c_write_blocking(I2C0_PORT, VL53L0X_ADDR, &reg, 1, true) != 1) return false;
    i2c_read_blocking(I2C0_PORT, VL53L0X_ADDR, &id, 1, false);
    return (id == 0xEE);
}

static int get_dist(void)
{
    uint8_t cmd[2] = {REG_START, 0x01};
    i2c_write_blocking(I2C0_PORT, VL53L0X_ADDR, cmd, 2, false);
    uint8_t reg = REG_DIST_MM, buf[2];
    i2c_write_blocking(I2C0_PORT, VL53L0X_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C0_PORT, VL53L0X_ADDR, buf, 2, false);
    return (buf[0] << 8) | buf[1];
}

// =============================================================================
// MQTT — Callbacks e conexão
// =============================================================================

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    (void)client; (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao HiveMQ!\n");
        mqtt_conectado = true;
    } else {
        printf("[MQTT] Desconectado! Status: %d\n", status);
        mqtt_conectado = false;
    }
}

static void mqtt_publish_cb(void *arg, err_t result)
{
    (void)arg;
    if (result != ERR_OK) printf("[MQTT] Erro no publish: %d\n", result);
}

static ip_addr_t         broker_ip_resolvido;
static volatile bool     dns_resolvido = false;

static void dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *arg)
{
    (void)name; (void)arg;
    if (ipaddr) {
        broker_ip_resolvido = *ipaddr;
        dns_resolvido = true;
        printf("[DNS] Resolvido: %s\n", ip4addr_ntoa(ipaddr));
    } else {
        printf("[DNS] Falha ao resolver '%s'\n", name);
    }
}

static void mqtt_connect(void)
{
    dns_resolvido = false;

    cyw43_arch_lwip_begin();
    err_t dns_err = dns_gethostbyname(MQTT_BROKER, &broker_ip_resolvido, dns_found_cb, NULL);
    cyw43_arch_lwip_end();

    if (dns_err == ERR_OK) {
        dns_resolvido = true;
        printf("[DNS] Cache: %s\n", ip4addr_ntoa(&broker_ip_resolvido));
    } else if (dns_err == ERR_INPROGRESS) {
        printf("[DNS] Aguardando '%s'...\n", MQTT_BROKER);
        for (int i = 0; i < 50 && !dns_resolvido; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (!dns_resolvido) {
        printf("[MQTT] DNS falhou.\n");
        return;
    }

    if (!mqtt_client) mqtt_client = mqtt_client_new();
    if (!mqtt_client) {
        printf("[MQTT] Falha ao criar cliente.\n");
        return;
    }

    struct mqtt_connect_client_info_t ci = {
        .client_id   = MQTT_CLIENT_ID,
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive  = 60,
        .will_topic  = TOPIC_STATUS,
        .will_msg    = "offline",
        .will_qos    = 0,
        .will_retain = 0,
    };

    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(mqtt_client, &broker_ip_resolvido, MQTT_BROKER_PORT,
                                    mqtt_connection_cb, NULL, &ci);
    cyw43_arch_lwip_end();

    if (err != ERR_OK)
        printf("[MQTT] Erro ao conectar: %d\n", err);
    else
        printf("[MQTT] Conectando a %s:%d...\n", MQTT_BROKER, MQTT_BROKER_PORT);
}

static void mqtt_publicar(const char *topico, const char *payload)
{
    if (!mqtt_client || !mqtt_conectado) return;
    cyw43_arch_lwip_begin();
    mqtt_publish(mqtt_client, topico, payload, strlen(payload),
                 0, 0, mqtt_publish_cb, NULL);
    cyw43_arch_lwip_end();
}

// =============================================================================
// TASK: vSensorTask
// =============================================================================

void vSensorTask(void *pvParameters)
{
    bool laser_ok = (bool)(uintptr_t)pvParameters;
    printf("[Sensor] Task iniciada. Laser: %s\n", laser_ok ? "OK" : "FALHOU");

    while (1)
    {
        SensorData_t data;

        data.distancia_mm  = laser_ok ? get_dist() : -1;
        data.leds_ativos   = leds_enabled; // lê o volatile — sempre atual

        int16_t ax, ay, az;
        mpu6050_read_raw(&ax, &ay, &az);
        data.inclinacao_deg = mpu6050_get_inclination(ax, ay, az);

        data.alarme_colisao    = (data.distancia_mm > 0 && data.distancia_mm < 100);
        data.alarme_inclinacao = (data.inclinacao_deg > 70.0f || data.inclinacao_deg < -70.0f);

        xQueueOverwrite(xSensorQueue, &data);

        printf("[Sensor] D:%d mm | I:%.2f | Col:%d | Inc:%d | LEDs:%s\n",
               data.distancia_mm, data.inclinacao_deg,
               data.alarme_colisao, data.alarme_inclinacao,
               data.leds_ativos ? "ON" : "OFF");

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================================
// TASK: vDisplayTask
// =============================================================================

void vDisplayTask(void *pvParameters)
{
    (void)pvParameters;
    SensorData_t data;
    char str_info[24];

    printf("[Display] Task iniciada.\n");

    while (1)
    {
        if (xQueuePeek(xSensorQueue, &data, pdMS_TO_TICKS(200)) != pdTRUE) continue;

        clear_display();
        display_text("STATUS DO SISTEMA", 10, 0, 1);

        sprintf(str_info, "D: %d mm", data.distancia_mm);
        display_text_no_clear(str_info, 0, 15, 1);

        sprintf(str_info, "I: %.1f deg", data.inclinacao_deg);
        display_text_no_clear(str_info, 70, 15, 1);

        int bar = (data.distancia_mm > 500) ? 500 : (data.distancia_mm < 0 ? 0 : data.distancia_mm);
        bar = (bar * 120) / 500;
        ssd1306_draw_empty_square(&display, 4, 28, 120, 8);
        ssd1306_draw_square(&display, 4, 28, bar, 8);

        display_text_no_clear(mqtt_conectado ? "MQTT:OK" : "MQTT:--", 0, 40, 1);

        if (data.alarme_colisao) {
            display_text_no_clear("! COLISAO !", 25, 50, 1);
            pwm_set_gpio_level(BUZZER_PIN, 2000);
            set_leds_color(255, 0, 0);
        } else if (data.alarme_inclinacao) {
            display_text_no_clear("! INCLINACAO !", 10, 50, 1);
            pwm_set_gpio_level(BUZZER_PIN, 1000);
            set_leds_color(255, 100, 0);
        } else {
            pwm_set_gpio_level(BUZZER_PIN, 0);
            set_leds_color(0, 255, 0);
            if (!data.leds_ativos) {
                display_text_no_clear("MODO MUTE", 35, 50, 1);
            }
        }

        show_display();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================================
// TASK: vMqttTask
// =============================================================================

void vMqttTask(void *pvParameters)
{
    (void)pvParameters;
    SensorData_t data;
    char payload[64];
    TickType_t ultimo_publish = 0;

    printf("[MQTT] Task iniciada.\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    mqtt_connect();
    vTaskDelay(pdMS_TO_TICKS(3000));

    while (1)
    {
        if (!mqtt_conectado) {
            printf("[MQTT] Reconectando em 10s...\n");
            vTaskDelay(pdMS_TO_TICKS(10000));
            mqtt_connect();
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        TickType_t agora = xTaskGetTickCount();
        if ((agora - ultimo_publish) >= pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL_MS))
        {
            if (xQueuePeek(xSensorQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                snprintf(payload, sizeof(payload), "%d", data.distancia_mm);
                mqtt_publicar(TOPIC_DISTANCIA, payload);

                snprintf(payload, sizeof(payload), "%.1f", data.inclinacao_deg);
                mqtt_publicar(TOPIC_INCLINACAO, payload);

                snprintf(payload, sizeof(payload),
                         "{\"colisao\":%d,\"inclinacao\":%d}",
                         data.alarme_colisao ? 1 : 0,
                         data.alarme_inclinacao ? 1 : 0);
                mqtt_publicar(TOPIC_ALERTA, payload);

                mqtt_publicar(TOPIC_LEDS,   data.leds_ativos ? "ativo" : "mute");
                mqtt_publicar(TOPIC_STATUS, "online");

                printf("[MQTT] Publicado: D=%d | I=%.1f | Col=%d | Inc=%d | LEDs=%s\n",
                       data.distancia_mm, data.inclinacao_deg,
                       data.alarme_colisao, data.alarme_inclinacao,
                       data.leds_ativos ? "ativo" : "mute");

                ultimo_publish = agora;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =============================================================================
// TASK: vInitTask
// =============================================================================

void vInitTask(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500));
    printf("=== BitDogLab Inicializando ===\n");

    // CYW43
    if (cyw43_arch_init()) {
        printf("ERRO: cyw43_arch_init() falhou!\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("CYW43 OK\n");

    // Wi-Fi
    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi '%s'...\n", WIFI_SSID);

    int tentativas = 0;
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                               CYW43_AUTH_WPA2_AES_PSK, 10000) != 0) {
        tentativas++;
        printf("[Wi-Fi] Tentativa %d falhou.\n", tentativas);
        if (tentativas >= 5) { printf("[Wi-Fi] Sem Wi-Fi.\n"); break; }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    if (tentativas < 5)
        printf("[Wi-Fi] Conectado! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));

    // I2C
    i2c_init(I2C0_PORT, 400 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Display
    display_init();

    // Botão A com interrupção
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    // Registra a ISR — borda de descida = botão pressionado (pull-up ativo)
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Buzzer
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice, true);

    // NeoPixel
    uint offset = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, 0, offset, LED_PIN, 800000, false);

    // VL53L0X
    bool laser_ok = vl53l0x_init();
    printf("VL53L0X: %s\n", laser_ok ? "OK" : "Nao encontrado");

    // MPU6050
    mpu6050_init();
    printf("MPU6050: OK\n");

    printf("Heap livre: %u bytes\n", (unsigned)xPortGetFreeHeapSize());

    // Queue
    xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));
    if (!xSensorQueue) {
        printf("ERRO: Falha ao criar xSensorQueue!\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Tasks
    xTaskCreate(vSensorTask,  "Sensor",  1024, (void*)(uintptr_t)laser_ok, 3, NULL);
    xTaskCreate(vDisplayTask, "Display", 1024, NULL, 2, NULL);
    xTaskCreate(vMqttTask,    "MQTT",    2048, NULL, 1, NULL);

    printf("=== Init concluida ===\n");
    vTaskDelete(NULL);
}

// =============================================================================
// MAIN
// =============================================================================

int main(void)
{
    stdio_init_all();
    sleep_ms(1000);
    printf("Booting BitDogLab...\n");

    xTaskCreate(vInitTask, "Init_Task", 4096, NULL, 2, NULL);

    vTaskStartScheduler();

    printf("ERRO FATAL: vTaskStartScheduler() retornou!\n");
    while (1) { tight_loop_contents(); }
}