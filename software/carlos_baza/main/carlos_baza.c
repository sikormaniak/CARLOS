/*
 * Sterowanie platformą mecanum (4x N20, 4x DRV8833) z dwoma
 * niezależnymi źródłami komend prędkości:
 *
 *   1) micro-ROS  -> subskrybent /cmd_vel (geometry_msgs/msg/Twist)
 *   2) ELRS/CRSF  -> odbiornik RC podłączony przez UART (kanał zapasowy /
 *                    ręczne sterowanie / failsafe, NIEZALEŻNY od WiFi)
 *
 * Priorytet: gdy z odbiornika RC napływają świeże ramki ORAZ przełącznik
 * (kanał AUX, domyślnie CH5) jest w pozycji "override", pilot ma
 * pierwszeństwo nad komendami z ROS 2 -- to standardowe zabezpieczenie
 * (failsafe) w systemach zdalnie sterowanych. W przeciwnym razie robot
 * jedzie zgodnie z ostatnią komendą /cmd_vel; jeśli obie komendy są
 * nieaktualne (timeout), robot się zatrzymuje.
 *
 * UWAGA -- WERSJA BEZ ENKODERÓW:
 * Ze względu na niedobór dostępnych pinów GPIO enkodery kwadraturowe
 * (PCNT) oraz oparta na nich pętla regulacji PID zostały usunięte.
 * Sterowanie silnikami jest teraz w układzie otwartym (feedforward) --
 * zadana prędkość koła (rad/s) jest bezpośrednio, liniowo przeliczana
 * na wypełnienie PWM względem zakładanej maksymalnej prędkości obrotowej
 * silnika (MAX_WHEEL_SPEED_RAD_S), bez żadnej korekcji na podstawie
 * rzeczywiście zmierzonej prędkości. Konsekwencje praktyczne:
 *   - brak kompensacji różnic między silnikami (tolerancje produkcyjne,
 *     różne obciążenie każdego koła) -- robot może ciągnąć w bok,
 *   - brak kompensacji spadku napięcia baterii w czasie,
 *   - brak możliwości precyzyjnej odometrii (nie ma zliczania impulsów).
 *
 * WSZYSTKIE WARTOŚCI OZNACZONE "-- DOPASOWAĆ" WYMAGAJĄ WERYFIKACJI
 * WZGLĘDEM RZECZYWISTEGO SPRZĘTU PRZED URUCHOMIENIEM.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/uart.h"
#include "bdc_motor.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

static const char *TAG = "robot";

/* ==========================================================================
 *  Makra pomocnicze micro-ROS
 * ========================================================================== */
#define RCCHECK(fn) do { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGE(TAG, "Blad RCL w linii %d: %d. Restart.", __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
} while (0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGW(TAG, "Blad RCL (niekrytyczny) w linii %d: %d", __LINE__, (int)temp_rc); \
    } \
} while (0)

/* ==========================================================================
 *  Parametry PWM / MCPWM (sterowanie DRV8833 przez wejścia IN1/IN2)
 * ========================================================================== */
#define BDC_MCPWM_TIMER_RESOLUTION_HZ  10000000    // 10 MHz -> 1 tick = 0.1 us
#define BDC_MCPWM_FREQ_HZ              25000       // 25 kHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX        (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ)

/* ==========================================================================
 *  Sterowanie otwarte (feedforward) -- bez enkoderów / PID
 * ========================================================================== */
#define CONTROL_LOOP_PERIOD_MS         20          // 50 Hz

// Maksymalna prędkość obrotowa koła (rad/s) przy pełnym wypełnieniu PWM,
// odpowiadająca prędkości obrotowej wyjścia przekładni N20 bez obciążenia.
// -- DOPASOWAĆ na podstawie danych katalogowych silnika N20 (RPM przy
// zasilaniu nominalnym) przeliczonych na rad/s: rad/s = RPM * 2*pi/60.
#define MAX_WHEEL_SPEED_RAD_S          44.0f       // 420 RPM -> 43.98 rad/s, ale w praktyce przy obciążeniu i spadku napięcia

/* ==========================================================================
 *  Geometria platformy mecanum (wzory z rozdziału "Ruch")
 * ========================================================================== */
#define WHEEL_RADIUS_M                 0.029f      // 58 mm średnicy koła -> 29 mm promienia
#define WHEEL_LX_M                     0.040f      // 40 mm odległości osi koła od środka platformy wzdłuż osi X (przód-tył)
#define WHEEL_LY_M                     0.0725f      // 72.5 mm odległości osi koła od środka platformy wzdłuż osi Y (lewo-prawo)

/* ==========================================================================
 *  Limity prędkości platformy (skalowanie wychyleń drążków RC)
 * ========================================================================== */
#define MAX_LINEAR_SPEED_MPS           2.0f        // -- DOPASOWAĆ
#define MAX_ANGULAR_SPEED_RADPS        8.0f        // -- DOPASOWAĆ (teoretyczny sufit z geometrii ~11.3 rad/s;
                                                    // 8.0 daje solidny margines duty na pokonanie oporu obrotu
                                                    // w miejscu, ktory jest wiekszy niz przy jezdzie na wprost)

/* ==========================================================================
 *  Bezpieczeństwo -- timeouty utraty łączności (failsafe)
 * ========================================================================== */
#define CRSF_LINK_TIMEOUT_MS           500         // po tym czasie RC uznawane za "martwe"
#define CMD_VEL_TIMEOUT_MS             1000        // po tym czasie /cmd_vel uznawane za "martwe"

typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR,
    WHEEL_BL,
    WHEEL_BR,
    WHEEL_COUNT,
} wheel_id_t;

static const char *wheel_name[WHEEL_COUNT] = { "FL", "FR", "BL", "BR" };

/* Piny DRV8833 -- DOPASOWAĆ do rzeczywistego PCB */
typedef struct {
    int pwm_in1_gpio;
    int pwm_in2_gpio;
    int mcpwm_group_id;
} wheel_pin_config_t;

static const wheel_pin_config_t wheel_pins[WHEEL_COUNT] = {
    [WHEEL_FL] = { .pwm_in1_gpio = 5,  .pwm_in2_gpio = 20, .mcpwm_group_id = 0 },
    [WHEEL_FR] = { .pwm_in1_gpio = 32, .pwm_in2_gpio = 33, .mcpwm_group_id = 0 },
    [WHEEL_BL] = { .pwm_in1_gpio = 3,  .pwm_in2_gpio = 4,  .mcpwm_group_id = 1 },
    [WHEEL_BR] = { .pwm_in1_gpio = 1,  .pwm_in2_gpio = 2,  .mcpwm_group_id = 1 },
};

typedef struct {
    bdc_motor_handle_t motor;
    float target_speed_rad_s;
} wheel_ctx_t;

static wheel_ctx_t wheels[WHEEL_COUNT];

/* ==========================================================================
 *  Wspólny stan komend prędkości (chroniony mutexem) -- dwa niezależne
 *  źródła: micro-ROS (/cmd_vel) i CRSF (RC), plus znaczniki czasu ostatniej
 *  aktualizacji, wykorzystywane do wykrywania utraty łączności.
 * ========================================================================== */
typedef struct {
    float vx;
    float vy;
    float omega;
    TickType_t last_update_tick;
} velocity_command_t;

static SemaphoreHandle_t cmd_mutex;
static velocity_command_t cmd_vel_from_ros = { 0 };
static velocity_command_t cmd_vel_from_rc  = { 0 };
static volatile bool      rc_override_switch_on = false;
static volatile bool      full_power_test_on = false;

/* ==========================================================================
 *  Biezacy aktywny tryb sterowania -- do podgladu po UART i przez ROS
 * ========================================================================== */
typedef enum {
    CONTROL_MODE_STOP = 0,   // brak swiezych komend z RC i ROS -- zatrzymanie awaryjne
    CONTROL_MODE_ROS,        // jedziemy wg /cmd_vel
    CONTROL_MODE_RC,         // pilot ma priorytet (override)
    CONTROL_MODE_FULL_POWER, // test sprzetowy -- wszystkie kola 100% do przodu, najwyzszy priorytet
} control_mode_t;

// Odczyt/zapis z dwoch roznych taskow (control_loop_task pisze, microros_task
// czyta do publikacji) -- pojedynczy int, wystarczajaco atomowy dla tego
// diagnostycznego celu, bez potrzeby dodatkowego mutexa.
static volatile control_mode_t current_control_mode = CONTROL_MODE_STOP;

static const char *control_mode_name(control_mode_t mode)
{
    switch (mode) {
        case CONTROL_MODE_RC:          return "RC";
        case CONTROL_MODE_ROS:         return "ROS";
        case CONTROL_MODE_FULL_POWER:  return "FULL_POWER";
        default:                       return "STOP";
    }
}

/* ==========================================================================
 *  Kinematyka odwrotna mecanum: (vx, vy, omega) -> omega_FL/FR/BL/BR
 * ========================================================================== */
static void mecanum_inverse_kinematics(float vx, float vy, float omega,
                                        float out_wheel_speed_rad_s[WHEEL_COUNT])
{
    const float l = WHEEL_LX_M + WHEEL_LY_M;
    out_wheel_speed_rad_s[WHEEL_FL] = (vx - vy - l * omega) / WHEEL_RADIUS_M;
    out_wheel_speed_rad_s[WHEEL_FR] = (vx + vy + l * omega) / WHEEL_RADIUS_M;
    out_wheel_speed_rad_s[WHEEL_BL] = (vx + vy - l * omega) / WHEEL_RADIUS_M;
    out_wheel_speed_rad_s[WHEEL_BR] = (vx - vy + l * omega) / WHEEL_RADIUS_M;
}

static void mecanum_set_body_velocity(float vx, float vy, float omega)
{
    float wheel_speeds[WHEEL_COUNT];
    mecanum_inverse_kinematics(vx, vy, omega, wheel_speeds);
    for (wheel_id_t id = 0; id < WHEEL_COUNT; id++) {
        wheels[id].target_speed_rad_s = wheel_speeds[id];
    }
}

/* ==========================================================================
 *  Inicjalizacja silnika dla jednego koła
 * ========================================================================== */
static void wheel_motor_init(wheel_id_t id)
{
    const wheel_pin_config_t *p = &wheel_pins[id];
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz   = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = p->pwm_in1_gpio,
        .pwmb_gpio_num = p->pwm_in2_gpio,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id      = p->mcpwm_group_id,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &wheels[id].motor));
    ESP_ERROR_CHECK(bdc_motor_enable(wheels[id].motor));
    ESP_LOGI(TAG, "Silnik %s zainicjalizowany (grupa MCPWM %d)", wheel_name[id], p->mcpwm_group_id);
}

/* ==========================================================================
 *  Sterowanie otwarte: zadana prędkość koła (rad/s) -> wypełnienie PWM
 * ========================================================================== */
// Minimalne wypelnienie PWM (jako udzial <0,1> BDC_MCPWM_DUTY_TICK_MAX)
// potrzebne, zeby przelamac tarcie statyczne silnika N20 + przekladni +
// rolek mecanum. Bez tego liniowe przeliczenie daje przy niskich/srednich
// wychyleniach drazka duty ktore "buczy" ale nie rusza kola -- ruch byl
// widoczny tylko w skrajnych rogach drazka (duty > ~40%).
// -- DOPASOWAC pomiarowo: podnosic recznie duty od 0% w gore (np. przez
// tymczasowy hardcode w tej funkcji) i zanotowac przy jakiej wartosci kolo
// (pod obciazeniem, zamontowane na platformie) faktycznie zaczyna sie krecic.
#define MIN_DUTY_RATIO   0.35f

static void wheel_apply_target_speed(wheel_id_t id)
{
    float target = wheels[id].target_speed_rad_s;
    float duty_ratio;

    if (fabsf(target) < 0.001f) {
        // brak komendy -- pelny STOP, bez podnoszenia duty
        duty_ratio = 0.0f;
    } else {
        // Liniowe przeliczenie <0, MAX_WHEEL_SPEED_RAD_S> na <0, 1>,
        // a nastepnie przeskalowanie tak, aby najmniejsza niezerowa
        // komenda dawala juz MIN_DUTY_RATIO, a maksymalna -- 100%.
        float raw_ratio = fabsf(target) / MAX_WHEEL_SPEED_RAD_S;
        if (raw_ratio > 1.0f) {
            raw_ratio = 1.0f;
        }
        duty_ratio = MIN_DUTY_RATIO + raw_ratio * (1.0f - MIN_DUTY_RATIO);
    }

    uint32_t duty = (uint32_t)(duty_ratio * (float)BDC_MCPWM_DUTY_TICK_MAX);

    if (target >= 0.0f) {
        ESP_ERROR_CHECK(bdc_motor_forward(wheels[id].motor));
    } else {
        ESP_ERROR_CHECK(bdc_motor_reverse(wheels[id].motor));
    }
    ESP_ERROR_CHECK(bdc_motor_set_speed(wheels[id].motor, duty));
}

/* ==========================================================================
 *  Pętla sterowania: arbitraż źródła komendy (RC > ROS > STOP) +
 *  bezpośrednie zastosowanie zadanej prędkości na każdym kole (open-loop)
 * ========================================================================== */
static void control_loop_task(void *arg)
{
    while (1) {
        TickType_t now = xTaskGetTickCount();

        xSemaphoreTake(cmd_mutex, portMAX_DELAY);
        velocity_command_t rc  = cmd_vel_from_rc;
        velocity_command_t ros = cmd_vel_from_ros;
        bool rc_switch = rc_override_switch_on;
        bool full_power = full_power_test_on;
        xSemaphoreGive(cmd_mutex);

        bool rc_fresh  = (now - rc.last_update_tick)  <= pdMS_TO_TICKS(CRSF_LINK_TIMEOUT_MS);
        bool ros_fresh = (now - ros.last_update_tick) <= pdMS_TO_TICKS(CMD_VEL_TIMEOUT_MS);

        float vx = 0.0f, vy = 0.0f, omega = 0.0f;
        control_mode_t mode;

        if (rc_fresh && full_power) {
            // Test sprzetowy -- najwyzszy priorytet, pomija kinematyke mecanum
            mode = CONTROL_MODE_FULL_POWER;
        } else if (rc_fresh && rc_switch) {
            // Pilot ma priorytet -- tryb recznego sterowania / failsafe
            vx = rc.vx; vy = rc.vy; omega = rc.omega;
            mode = CONTROL_MODE_RC;
        } else if (ros_fresh) {
            // Brak nadpisania z RC -- jedziemy wg ostatniej komendy /cmd_vel
            vx = ros.vx; vy = ros.vy; omega = ros.omega;
            mode = CONTROL_MODE_ROS;
        } else {
            // Obie komendy nieaktualne -- zatrzymanie awaryjne
            vx = 0.0f; vy = 0.0f; omega = 0.0f;
            mode = CONTROL_MODE_STOP;
        }

        if (mode != current_control_mode) {
            ESP_LOGI(TAG, "Zmiana trybu sterowania: %s -> %s",
                     control_mode_name(current_control_mode), control_mode_name(mode));
            current_control_mode = mode;
        }

        // -- DEBUG: okresowy zrzut biezacych komend, do usuniecia po diagnozie
        static TickType_t last_debug_tick = 0;
        if ((now - last_debug_tick) >= pdMS_TO_TICKS(500)) {
            last_debug_tick = now;
            ESP_LOGI(TAG, "DEBUG mode=%s vx=%.3f vy=%.3f omega=%.3f | rc(fresh=%d sw=%d fp=%d vx=%.3f vy=%.3f om=%.3f) ros(fresh=%d)",
                     control_mode_name(mode), vx, vy, omega,
                     rc_fresh, rc_switch, full_power, rc.vx, rc.vy, rc.omega,
                     ros_fresh);
        }

        if (mode == CONTROL_MODE_FULL_POWER) {
            // Wszystkie kola na 100% duty, do przodu -- test sprzetowy
            // (silniki/sterowniki/okablowanie), bez udzialu kinematyki mecanum
            for (wheel_id_t id = 0; id < WHEEL_COUNT; id++) {
                wheels[id].target_speed_rad_s = MAX_WHEEL_SPEED_RAD_S;
            }
        } else {
            mecanum_set_body_velocity(vx, vy, omega);
        }

        for (wheel_id_t id = 0; id < WHEEL_COUNT; id++) {
            wheel_apply_target_speed(id);
            ESP_LOGD(TAG, "  kolo %s: target=%.3f rad/s", wheel_name[id], wheels[id].target_speed_rad_s);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}

/* ==========================================================================
 *  Odbiór i dekodowanie protokolu CRSF z odbiornika ELRS (UART)
 * ========================================================================== */
#define ELRS_UART_NUM       UART_NUM_1
#define ELRS_TXD_PIN        46
#define ELRS_RXD_PIN        47
#define ELRS_BAUD_RATE      420000
#define BUF_SIZE            1024

#define CRSF_SYNC_BYTE                      0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED   0x16
#define CRSF_CHANNEL_COUNT                  16

// Numeracja kanałów RC (AETR) -- DOPASOWAĆ do konfiguracji nadajnika
#define CRSF_CH_ROLL_VY     3   // kanał 1 -- ruch boczny (strafe)
#define CRSF_CH_PITCH_VX    1   // kanał 2 -- jazda przod/tyl
#define CRSF_CH_YAW_OMEGA   0   // kanał 4 -- obrot
#define CRSF_CH_AUX_OVERRIDE 4  // kanał 5 -- przelacznik trybu recznego
#define CRSF_CH_FULL_POWER_TEST 5  // kanał 6 -- test sprzetowy, wszystkie kola 100% do przodu

#define CRSF_RAW_MIN        172
#define CRSF_RAW_MAX        1811
#define CRSF_RAW_MID        992
#define CRSF_SWITCH_THRESHOLD 1500

typedef struct {
    uint8_t sync;
    uint8_t length;
    uint8_t type;
    uint8_t payload[64];
} crsf_frame_t;

typedef struct {
    uint16_t channels[CRSF_CHANNEL_COUNT];
} crsf_channels_t;

static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void decode_rc_channels(const uint8_t *payload, crsf_channels_t *channels)
{
    channels->channels[0]  = (uint16_t)((payload[0]     | payload[1]  << 8)  & 0x07FF);
    channels->channels[1]  = (uint16_t)((payload[1]>>3  | payload[2]  << 5)  & 0x07FF);
    channels->channels[2]  = (uint16_t)((payload[2]>>6  | payload[3]  << 2  | payload[4] << 10) & 0x07FF);
    channels->channels[3]  = (uint16_t)((payload[4]>>1  | payload[5]  << 7)  & 0x07FF);
    channels->channels[4]  = (uint16_t)((payload[5]>>4  | payload[6]  << 4)  & 0x07FF);
    channels->channels[5]  = (uint16_t)((payload[6]>>7  | payload[7]  << 1  | payload[8] << 9)  & 0x07FF);
    channels->channels[6]  = (uint16_t)((payload[8]>>2  | payload[9]  << 6)  & 0x07FF);
    channels->channels[7]  = (uint16_t)((payload[9]>>5  | payload[10] << 3)  & 0x07FF);
    channels->channels[8]  = (uint16_t)((payload[11]    | payload[12] << 8)  & 0x07FF);
    channels->channels[9]  = (uint16_t)((payload[12]>>3 | payload[13] << 5)  & 0x07FF);
    channels->channels[10] = (uint16_t)((payload[13]>>6 | payload[14] << 2  | payload[15] << 10) & 0x07FF);
    channels->channels[11] = (uint16_t)((payload[15]>>1 | payload[16] << 7)  & 0x07FF);
    channels->channels[12] = (uint16_t)((payload[16]>>4 | payload[17] << 4)  & 0x07FF);
    channels->channels[13] = (uint16_t)((payload[17]>>7 | payload[18] << 1  | payload[19] << 9)  & 0x07FF);
    channels->channels[14] = (uint16_t)((payload[19]>>2 | payload[20] << 6)  & 0x07FF);
    channels->channels[15] = (uint16_t)((payload[20]>>5 | payload[21] << 3)  & 0x07FF);
}

// Przeliczenie surowej wartosci kanalu CRSF (172..1811, srodek 992) na
// znormalizowany wspolczynnik <-1.0, 1.0>
static float crsf_channel_to_normalized(uint16_t raw)
{
    float normalized = ((float)raw - CRSF_RAW_MID) / ((CRSF_RAW_MAX - CRSF_RAW_MIN) / 2.0f);
    if (normalized > 1.0f)  normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;
    return normalized;
}

static void handle_rc_channels_frame(const uint8_t *payload)
{
    crsf_channels_t channels;
    decode_rc_channels(payload, &channels);

    // -- DEBUG: okresowy zrzut surowych wartosci wszystkich 16 kanalow CRSF,
    // do usuniecia po ustaleniu poprawnego mapowania kanalow
    static TickType_t last_raw_log_tick = 0;
    TickType_t now_raw = xTaskGetTickCount();
    if ((now_raw - last_raw_log_tick) >= pdMS_TO_TICKS(500)) {
        last_raw_log_tick = now_raw;
        ESP_LOGI(TAG, "RAW CH: 0=%4d 1=%4d 2=%4d 3=%4d 4=%4d 5=%4d 6=%4d 7=%4d",
                 channels.channels[0], channels.channels[1], channels.channels[2], channels.channels[3],
                 channels.channels[4], channels.channels[5], channels.channels[6], channels.channels[7]);
    }

    float vx    = crsf_channel_to_normalized(channels.channels[CRSF_CH_PITCH_VX])  * MAX_LINEAR_SPEED_MPS;
    float vy    = crsf_channel_to_normalized(channels.channels[CRSF_CH_ROLL_VY])   * MAX_LINEAR_SPEED_MPS;
    float omega = crsf_channel_to_normalized(channels.channels[CRSF_CH_YAW_OMEGA]) * MAX_ANGULAR_SPEED_RADPS;
    bool  override_on   = channels.channels[CRSF_CH_AUX_OVERRIDE]     > CRSF_SWITCH_THRESHOLD;
    bool  full_power_on  = channels.channels[CRSF_CH_FULL_POWER_TEST] > CRSF_SWITCH_THRESHOLD;

    xSemaphoreTake(cmd_mutex, portMAX_DELAY);
    cmd_vel_from_rc.vx = vx;
    cmd_vel_from_rc.vy = vy;
    cmd_vel_from_rc.omega = omega;
    cmd_vel_from_rc.last_update_tick = xTaskGetTickCount();
    rc_override_switch_on = override_on;
    full_power_test_on = full_power_on;
    xSemaphoreGive(cmd_mutex);
}

static void process_crsf_frame(const crsf_frame_t *frame)
{
    uint8_t crc_calc     = crsf_crc8(&frame->type, frame->length - 1);
    uint8_t crc_received = frame->payload[frame->length - 2];
    if (crc_calc != crc_received) {
        ESP_LOGW(TAG, "CRSF: bledny CRC (obliczony 0x%02X, odebrany 0x%02X)", crc_calc, crc_received);
        return;
    }

    if (frame->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        handle_rc_channels_frame(frame->payload);
    }
    // Pozostale typy ramek (telemetria) pominiete -- do rozbudowy w razie potrzeby
}

static void uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = ELRS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(ELRS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ELRS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ELRS_UART_NUM, ELRS_TXD_PIN, ELRS_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART CRSF zainicjalizowany (RX=GPIO%d, TX=GPIO%d, %d bd)",
             ELRS_RXD_PIN, ELRS_TXD_PIN, ELRS_BAUD_RATE);
}

static void elrs_read_task(void *arg)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    crsf_frame_t frame;
    uint8_t frame_position = 0;
    bool frame_started = false;

    while (1) {
        int len = uart_read_bytes(ELRS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(20));

        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];

            if (!frame_started && byte == CRSF_SYNC_BYTE) {
                frame.sync = byte;
                frame_position = 0;
                frame_started = true;
                continue;
            }
            if (!frame_started) continue;

            if (frame_position == 0) {
                frame.length = byte;
                if (frame.length > 64 || frame.length < 2) {
                    frame_started = false;
                    continue;
                }
                frame_position++;
                continue;
            }
            if (frame_position == 1) {
                frame.type = byte;
                frame_position++;
                continue;
            }
            if (frame_position >= 2 && frame_position < frame.length + 1) {
                frame.payload[frame_position - 2] = byte;
                frame_position++;
                if (frame_position == frame.length + 1) {
                    process_crsf_frame(&frame);
                    frame_started = false;
                    frame_position = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    free(data);
}

/* ==========================================================================
 *  micro-ROS -- subskrybent /cmd_vel (geometry_msgs/msg/Twist)
 * ========================================================================== */
static void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    xSemaphoreTake(cmd_mutex, portMAX_DELAY);
    cmd_vel_from_ros.vx    = (float)msg->linear.x;
    cmd_vel_from_ros.vy    = (float)msg->linear.y;
    cmd_vel_from_ros.omega = (float)msg->angular.z;
    cmd_vel_from_ros.last_update_tick = xTaskGetTickCount();
    xSemaphoreGive(cmd_mutex);
}

/* ==========================================================================
 *  micro-ROS -- publisher /control_mode (std_msgs/msg/String)
 *  Publikuje biezacy aktywny tryb sterowania ("RC" / "ROS" / "STOP"),
 *  do podgladu: ros2 topic echo /control_mode
 * ========================================================================== */
static rcl_publisher_t   control_mode_publisher;
static std_msgs__msg__String control_mode_msg;
static char control_mode_msg_buffer[16];

static void control_mode_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;

    const char *name = control_mode_name(current_control_mode);
    strncpy(control_mode_msg_buffer, name, sizeof(control_mode_msg_buffer) - 1);
    control_mode_msg_buffer[sizeof(control_mode_msg_buffer) - 1] = '\0';
    control_mode_msg.data.data      = control_mode_msg_buffer;
    control_mode_msg.data.size      = strlen(control_mode_msg_buffer);
    control_mode_msg.data.capacity  = sizeof(control_mode_msg_buffer);

    RCSOFTCHECK(rcl_publish(&control_mode_publisher, &control_mode_msg, NULL));
}

static void microros_task(void *arg)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_mecanum_node", "", &support));

    rcl_subscription_t cmd_vel_subscriber;
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    static geometry_msgs__msg__Twist cmd_vel_msg;

    // Publisher /control_mode -- diagnostyczny, jaki tryb sterowania jest
    // aktywny w danej chwili (RC / ROS / STOP)
    RCCHECK(rclc_publisher_init_default(
        &control_mode_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/control_mode"));

    rcl_timer_t control_mode_timer;
    RCCHECK(rclc_timer_init_default(
        &control_mode_timer,
        &support,
        RCL_MS_TO_NS(200),   // co 200 ms
        control_mode_timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_mode_timer));

    ESP_LOGI(TAG, "micro-ROS: wezel uruchomiony, subskrypcja /cmd_vel + publisher /control_mode aktywne");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&control_mode_publisher, &node));
    RCCHECK(rcl_timer_fini(&control_mode_timer));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ==========================================================================
 *  app_main -- inicjalizacja wszystkich podsystemow
 * ========================================================================== */
void app_main(void)
{
    ESP_LOGI(TAG, "Inicjalizacja robota: mecanum (open-loop) + CRSF + micro-ROS");

    cmd_mutex = xSemaphoreCreateMutex();

    for (wheel_id_t id = 0; id < WHEEL_COUNT; id++) {
        wheel_motor_init(id);
        wheels[id].target_speed_rad_s = 0.0f;
    }

    uart_init();

    xTaskCreate(elrs_read_task,    "elrs_read",    8192, NULL, 10, NULL);
    xTaskCreate(control_loop_task, "control_loop", 4096, NULL, 6,  NULL);
    xTaskCreate(microros_task,     "microros",     16384, NULL, 5,  NULL);
}
