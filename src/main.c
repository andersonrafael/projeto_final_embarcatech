#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Importa o programa PIO compilado (gerado pelo CMake)
#include "ws2812.pio.h"

#include "mpu6050/mpu6050.h"
#include "servo/servo.h"
#include "display/display.h"

#define ANGLE_ALERT_THRESHOLD 60.0f
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define BUZZER_PIN 21

// --- CONFIGURAÇÃO NEOPIXEL BITDOGLAB ---
#define NEOPIXEL_PIN 7 // Pino de dados dos LEDs (BitDogLab padrão)
#define NUM_PIXELS 25  // Matriz 5x5
#define IS_RGBW false  // WS2812 padrão é RGB apenas

// --- BOTÕES BITDOGLAB ---
#define BUTTON_A_PIN 5 // Botão A na BitDogLab
#define BUTTON_B_PIN 6 // Botão B na BitDogLab

// --- Estado dos LEDs ---
static bool leds_enabled = true; // Começa com LEDs ligados

// --- Funções do Buzzer ---
void pwm_init_buzzer(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 12500);
    pwm_config_set_clkdiv(&config, 10);
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

void beep(uint pin, uint duration_ms)
{
    pwm_set_gpio_level(pin, 6250);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(pin, 0);
}

// --- Funções para NeoPixel (PIO) ---
static inline void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b)
{
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

void set_neopixel_color(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t color = urgb_u32(r, g, b);
    // Define a cor para TODOS os LEDs da matriz
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        put_pixel(color);
    }
}

void clear_neopixel(void)
{
    // Apaga todos os LEDs
    set_neopixel_color(0, 0, 0);
}

void leds_init_neopixel(void)
{
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, NEOPIXEL_PIN, 800000, IS_RGBW);
}

// --- Funções para os botões ---
void buttons_init(void)
{
    // Configura botões como entrada com pull-up
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);

    gpio_init(BUTTON_B_PIN);
    gpio_set_dir(BUTTON_B_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_B_PIN);
}

bool is_button_a_pressed(void)
{
    // Botão pressionado = nível baixo (pull-up)
    return !gpio_get(BUTTON_A_PIN);
}

bool is_button_b_pressed(void)
{
    // Botão pressionado = nível baixo (pull-up)
    return !gpio_get(BUTTON_B_PIN);
}

// --- Detecção de borda dos botões ---
typedef struct
{
    uint64_t last_press_time;
    bool last_state;
    bool debounced_state;
} button_debounce_t;

static button_debounce_t button_a_state = {0, false, false};
static button_debounce_t button_b_state = {0, false, false};

bool check_button_press(button_debounce_t *state, bool current_state)
{
    uint64_t current_time = to_ms_since_boot(get_absolute_time());

    // Debounce de 50ms
    if (current_state != state->last_state)
    {
        state->last_state = current_state;
        state->last_press_time = current_time;
    }

    // Detecta borda de descida (botão pressionado) com debounce
    if (current_state && !state->debounced_state &&
        (current_time - state->last_press_time > 50))
    {
        state->debounced_state = true;
        return true;
    }

    // Atualiza estado debounced
    if (!current_state && state->debounced_state)
    {
        state->debounced_state = false;
    }

    return false;
}

// --- Controle dos LEDs baseado no ângulo e alarme ---
void update_leds_by_angle(float angle, bool alarm_triggered)
{
    if (!leds_enabled)
    {
        clear_neopixel();
        return;
    }

    if (alarm_triggered)
    {
        // Vermelho para alarme
        set_neopixel_color(255, 0, 0);
    }
    else if (fabs(angle) > 30.0f)
    {
        // Amarelo para ângulo moderado
        set_neopixel_color(255, 150, 0);
    }
    else
    {
        // Verde para ângulo normal
        set_neopixel_color(0, 255, 0);
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== Sistema BitDogLab - Controle NeoPixel com Botões ===\n");
    printf("Botão A: Desliga LEDs\n");
    printf("Botão B: Liga LEDs\n\n");

    // Inicializações
    mpu6050_init();
    servo_init();
    display_init();
    leds_init_neopixel();
    buttons_init();
    pwm_init_buzzer(BUZZER_PIN);

    // Variáveis para leitura do MPU
    int16_t ax, ay, az;
    float angle;
    uint angle_servo;

    // Variáveis para controle de tempo
    uint64_t last_button_check = 0;
    uint64_t last_display_update = 0;

    // Estado inicial dos LEDs (ligados)
    update_leds_by_angle(0, false);

    while (true)
    {
        uint64_t current_time = to_ms_since_boot(get_absolute_time());

        // Leitura do MPU6050
        mpu6050_read_raw(&ax, &ay, &az);
        angle = mpu6050_get_inclination(ax, ay, az);

        // Controle do servo
        angle_servo = (uint)((angle + 90.0f) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 180.0f);
        if (angle_servo > SERVO_MAX_ANGLE)
            angle_servo = SERVO_MAX_ANGLE;
        servo_set_angle(angle_servo);

        // Verificação dos botões (a cada 20ms)
        if (current_time - last_button_check > 20)
        {
            // Verifica botão A (Desliga LEDs)
            if (check_button_press(&button_a_state, is_button_a_pressed()))
            {
                printf("Botão A pressionado - Desligando LEDs\n");
                leds_enabled = false;
                clear_neopixel();
                display_text_no_clear("LEDs: OFF  ", 0, 0, 1);
            }

            // Verifica botão B (Liga LEDs)
            if (check_button_press(&button_b_state, is_button_b_pressed()))
            {
                printf("Botão B pressionado - Ligando LEDs\n");
                leds_enabled = true;
                update_leds_by_angle(angle, fabs(angle) > ANGLE_ALERT_THRESHOLD);
                display_text_no_clear("LEDs:ON", 0, 0, 1);
            }

            last_button_check = current_time;
        }

        // Atualização dos LEDs baseada no ângulo (a cada 100ms)
        if (leds_enabled)
        {
            bool alarm_triggered = (fabs(angle) > ANGLE_ALERT_THRESHOLD);
            update_leds_by_angle(angle, alarm_triggered);
        }

        // Atualização do display (a cada 500ms)
        if (current_time - last_display_update > 500)
        {
            // Limpa área principal do display
            display_text_no_clear("                ", 0, 20, 2);

            // Mostra ângulo
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "Angulo: %.1f", angle);
            display_text_no_clear(buffer, 0, 24, 2);

            // Mostra estado dos LEDs
            if (leds_enabled)
            {
                display_text_no_clear("LEDs: ON", 0, 0, 1);
            }
            else
            {
                display_text_no_clear("LEDs: OFF  ", 0, 0, 1);
            }

            // Mostra alarme se necessário
            if (fabs(angle) > ANGLE_ALERT_THRESHOLD)
            {
                display_text_no_clear("ALERTA ", 0, 40, 1);
                beep(BUZZER_PIN, 200);
            }
            else
            {
                display_text_no_clear("Sistema OK  ", 0, 40, 1);
            }

            show_display();
            last_display_update = current_time;

            // Log no terminal
            printf("A-X: %d | A-Y: %d | A-Z: %d | Angulo: %.2f° | LEDs: %s\n",
                   ax, ay, az, angle, leds_enabled ? "LIGADO" : "DESLIGADO");
        }

        // Pequena pausa para não sobrecarregar
        sleep_ms(10);
    }

    return 0;
}