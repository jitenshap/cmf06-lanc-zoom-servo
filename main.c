#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "lanc_master_rx.pio.h"

#define LANC_PIN 2
#define LANC_PIO pio0
#define LANC_SM 0
#define LANC_BAUD 9600
#define LANC_FRAME_BYTES 8

#define FOLLOW_FOCUS_UART uart0
#define FOLLOW_FOCUS_TX_PIN 0
#define FOLLOW_FOCUS_RX_PIN 1
#define FOLLOW_FOCUS_BAUD 115200
#define FOLLOW_FOCUS_PACKET_BYTES 14
#define FOLLOW_FOCUS_PACKET_INTERVAL_US 18800
#define FOLLOW_FOCUS_CMD_FOCUS 0xB8
#define FOLLOW_FOCUS_CMD_ZOOM 0xC8
#define FOLLOW_FOCUS_REVERSE_ZOOM true
#define FOLLOW_FOCUS_WRAP_ZOOM false
#define FOLLOW_FOCUS_END_DECEL_COUNTS 512
#define ZOOM_RAMP_COMMANDS 10

static volatile int32_t g_zoom_step = 0;
static volatile uint16_t g_focus_position = 0;
static volatile uint16_t g_zoom_position = 0;

static const uint16_t zoom_speed_steps[] = {
    0,
    50,
    100,
    200,
    400,
    750,
    1250,
    2000,
    3000,
};

static int32_t abs_i32(int32_t value) {
    return value < 0 ? -value : value;
}

static int32_t max_i32(int32_t a, int32_t b) {
    return a > b ? a : b;
}

static void ramp_zoom_step_towards(int32_t target_step) {
    int32_t current_step = g_zoom_step;
    int32_t ramp_basis = max_i32(abs_i32(current_step), abs_i32(target_step));
    int32_t ramp_delta = (ramp_basis + ZOOM_RAMP_COMMANDS - 1) / ZOOM_RAMP_COMMANDS;

    if (ramp_delta < 1) {
        ramp_delta = 1;
    }

    if (current_step < target_step) {
        current_step += ramp_delta;
        if (current_step > target_step) {
            current_step = target_step;
        }
    } else if (current_step > target_step) {
        current_step -= ramp_delta;
        if (current_step < target_step) {
            current_step = target_step;
        }
    }

    g_zoom_step = current_step;
}

static int32_t zoom_step_for_direction(uint16_t base_step, bool tele) {
    int32_t step = tele ? (int32_t)base_step : -(int32_t)base_step;

    return FOLLOW_FOCUS_REVERSE_ZOOM ? -step : step;
}

static uint16_t next_zoom_position(uint16_t position, int32_t step) {
    if (FOLLOW_FOCUS_WRAP_ZOOM) {
        return (uint16_t)(position + step);
    }

    if (step > 0) {
        uint32_t distance_to_end = 0xFFFFu - position;
        uint32_t step_abs = (uint32_t)step;

        if (distance_to_end == 0) {
            return 0xFFFF;
        }

        if (distance_to_end > FOLLOW_FOCUS_END_DECEL_COUNTS) {
            uint32_t step_to_decel_zone = distance_to_end - FOLLOW_FOCUS_END_DECEL_COUNTS;

            if (step_abs > step_to_decel_zone) {
                step_abs = step_to_decel_zone;
            }
        } else {
            uint32_t decel_step =
                (distance_to_end * distance_to_end + FOLLOW_FOCUS_END_DECEL_COUNTS - 1) /
                FOLLOW_FOCUS_END_DECEL_COUNTS;

            if (decel_step < 1) {
                decel_step = 1;
            }

            if (step_abs > decel_step) {
                step_abs = decel_step;
            }
        }

        if (step_abs == 0) {
            step_abs = 1;
        }

        if (step_abs > distance_to_end) {
            return 0xFFFF;
        }

        return (uint16_t)(position + step_abs);
    }

    if (step < 0) {
        uint32_t distance_to_end = position;
        uint32_t step_abs = (uint32_t)(-step);

        if (distance_to_end == 0) {
            return 0;
        }

        if (distance_to_end > FOLLOW_FOCUS_END_DECEL_COUNTS) {
            uint32_t step_to_decel_zone = distance_to_end - FOLLOW_FOCUS_END_DECEL_COUNTS;

            if (step_abs > step_to_decel_zone) {
                step_abs = step_to_decel_zone;
            }
        } else {
            uint32_t decel_step =
                (distance_to_end * distance_to_end + FOLLOW_FOCUS_END_DECEL_COUNTS - 1) /
                FOLLOW_FOCUS_END_DECEL_COUNTS;

            if (decel_step < 1) {
                decel_step = 1;
            }

            if (step_abs > decel_step) {
                step_abs = decel_step;
            }
        }

        if (step_abs == 0) {
            step_abs = 1;
        }

        if (step_abs > distance_to_end) {
            return 0;
        }

        return (uint16_t)(position - step_abs);
    }

    int32_t next_position = (int32_t)position + step;

    if (next_position < 0) {
        return 0;
    }

    if (next_position > 0xFFFF) {
        return 0xFFFF;
    }

    return (uint16_t)next_position;
}

static uint16_t crc16_xmodem(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static void make_follow_focus_packet(uint8_t command,
                                     uint16_t value,
                                     uint8_t packet[FOLLOW_FOCUS_PACKET_BYTES]) {
    packet[0] = 0x24;
    packet[1] = 0x3C;
    packet[2] = 0x08;
    packet[3] = 0x00;
    packet[4] = command;
    packet[5] = 0x13;
    packet[6] = 0x00;
    packet[7] = 0x00;
    packet[8] = 0x00;
    packet[9] = 0x00;
    packet[10] = (uint8_t)(value & 0xFFu);
    packet[11] = (uint8_t)(value >> 8);

    uint16_t crc = crc16_xmodem(&packet[4], 8);
    packet[12] = (uint8_t)(crc & 0xFFu);
    packet[13] = (uint8_t)(crc >> 8);
}

static void follow_focus_send_packet(uint8_t command, uint16_t value) {
    uint8_t packet[FOLLOW_FOCUS_PACKET_BYTES];

    make_follow_focus_packet(command, value, packet);
    uart_write_blocking(FOLLOW_FOCUS_UART, packet, FOLLOW_FOCUS_PACKET_BYTES);
}

static void follow_focus_sender_core(void) {
    while (true) {
        follow_focus_send_packet(FOLLOW_FOCUS_CMD_FOCUS, g_focus_position);
        sleep_us(FOLLOW_FOCUS_PACKET_INTERVAL_US);

        g_zoom_position = next_zoom_position(g_zoom_position, g_zoom_step);
        follow_focus_send_packet(FOLLOW_FOCUS_CMD_ZOOM, g_zoom_position);
        sleep_us(FOLLOW_FOCUS_PACKET_INTERVAL_US);
    }
}

static void follow_focus_uart_init(void) {
    uart_init(FOLLOW_FOCUS_UART, FOLLOW_FOCUS_BAUD);
    gpio_set_function(FOLLOW_FOCUS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FOLLOW_FOCUS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(FOLLOW_FOCUS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(FOLLOW_FOCUS_UART, true);
}

static bool frame_is_all_zero(const uint8_t frame[LANC_FRAME_BYTES]) {
    for (uint i = 0; i < LANC_FRAME_BYTES; i++) {
        if (frame[i] != 0) {
            return false;
        }
    }

    return true;
}

static void print_frame(const uint8_t frame[LANC_FRAME_BYTES]) {
    for (uint i = 0; i < LANC_FRAME_BYTES; i++) {
        printf("%s0x%02X", i == 0 ? "" : ", ", frame[i]);
    }
}

static int zoom_tele_speed(uint8_t command) {
    if ((command <= 0x0Eu) && ((command & 0x01u) == 0u)) {
        return (int)(command / 2u) + 1;
    }

    return 0;
}

static int zoom_wide_speed(uint8_t command) {
    if ((command >= 0x10u) && (command <= 0x1Eu) && ((command & 0x01u) == 0u)) {
        return (int)((command - 0x10u) / 2u) + 1;
    }

    return 0;
}

static void handle_zoom_tele(uint8_t speed) {
    printf("zoom tele speed: %u\r\n", speed);
}

static void handle_zoom_wide(uint8_t speed) {
    printf("zoom wide speed: %u\r\n", speed);
}

static void handle_unhandled_command(const uint8_t frame[LANC_FRAME_BYTES]) {
    printf("unhandled command ");
    print_frame(frame);
    printf("\r\n");
}

static void handle_lanc_frame(const uint8_t frame[LANC_FRAME_BYTES]) {
    static int32_t last_reported_zoom_step = 0;

    if (frame_is_all_zero(frame)) {
        ramp_zoom_step_towards(0);
        last_reported_zoom_step = 0;
        return;
    }

    if (frame[0] == 0x28u) {
        int tele_speed = zoom_tele_speed(frame[1]);
        int wide_speed = zoom_wide_speed(frame[1]);

        if (tele_speed != 0) {
            int32_t next_step = zoom_step_for_direction(zoom_speed_steps[tele_speed], true);

            if (next_step != last_reported_zoom_step) {
                handle_zoom_tele((uint8_t)tele_speed);
                last_reported_zoom_step = next_step;
            }
            ramp_zoom_step_towards(next_step);
            return;
        }

        if (wide_speed != 0) {
            int32_t next_step = zoom_step_for_direction(zoom_speed_steps[wide_speed], false);

            if (next_step != last_reported_zoom_step) {
                handle_zoom_wide((uint8_t)wide_speed);
                last_reported_zoom_step = next_step;
            }
            ramp_zoom_step_towards(next_step);
            return;
        }
    }

    ramp_zoom_step_towards(0);
    last_reported_zoom_step = 0;
    handle_unhandled_command(frame);
}

static void lanc_master_rx_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = lanc_master_rx_program_get_default_config(offset);

    sm_config_set_in_pins(&c, pin);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_in_shift(&c, true, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (LANC_BAUD * 8.0f));

    pio_gpio_init(pio, pin);
    gpio_put(pin, 0);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("LANC 1-wire open-drain master RX start on GP%d\r\n", LANC_PIN);
    printf("Follow focus UART TX GP%d, RX GP%d, %d 8N1\r\n",
           FOLLOW_FOCUS_TX_PIN,
           FOLLOW_FOCUS_RX_PIN,
           FOLLOW_FOCUS_BAUD);
    follow_focus_uart_init();
    multicore_launch_core1(follow_focus_sender_core);

    PIO pio = LANC_PIO;
    uint sm = LANC_SM;
    uint offset = pio_add_program(pio, &lanc_master_rx_program);

    lanc_master_rx_init(pio, sm, offset, LANC_PIN);

    while (true) {
        uint8_t frame[LANC_FRAME_BYTES];

        for (uint i = 0; i < LANC_FRAME_BYTES; i++) {
            uint8_t raw = (uint8_t)(pio_sm_get_blocking(pio, sm) >> 24);
            frame[i] = (uint8_t)(raw ^ 0xFFu);
        }

        handle_lanc_frame(frame);
    }
}
