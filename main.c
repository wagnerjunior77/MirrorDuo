// main.c — Pergunta A/B, leitura opcional de cor e depois oxímetro (ativo no I2C0 via extensor)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "src/ssd1306.h"
#include "src/ssd1306_i2c.h"
#include "src/ssd1306_font.h"

#include "src/cor.h"         // cor_init, cor_read_rgb_norm, cor_classify, cor_class_to_str
#include "src/oximetro.h"    // oxi_init, oxi_start, oxi_poll, oxi_get_state, ...

// ==== OLED em I2C1 (BitDog) ====
#define OLED_I2C   i2c1
#define OLED_SDA   14
#define OLED_SCL   15
#define OLED_ADDR  0x3C

// ==== SENSORES no I2C0 (EXTENSOR) ====
#define COL_I2C    i2c0   // TCS34725
#define COL_SDA    0
#define COL_SCL    1

#define OXI_I2C    i2c0   // MAX3010x
#define OXI_SDA    0
#define OXI_SCL    1

// Botões BitDog
#define BUTTON_A   5
#define BUTTON_B   6

static ssd1306_t oled;
static bool oled_ok = false;

// contagem por classe de cor
static uint32_t g_cor_counts[COR_CLASS_COUNT] = {0};

// ---------- I2C / OLED helpers ----------
static void i2c_setup(i2c_inst_t *i2c, uint sda, uint scl, uint hz) {
    i2c_init(i2c, hz);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
}

static void oled_lines(const char *l1, const char *l2, const char *l3, const char *l4) {
    if (!oled_ok) return;
    ssd1306_clear(&oled);
    if (l1) ssd1306_draw_string(&oled, 0,  0, 1, l1);
    if (l2) ssd1306_draw_string(&oled, 0, 16, 1, l2);
    if (l3) ssd1306_draw_string(&oled, 0, 32, 1, l3);
    if (l4) ssd1306_draw_string(&oled, 0, 48, 1, l4);
    ssd1306_show(&oled);
}

// ---------- Botões (edge + holdoff) ----------
static bool edge_press(bool now, bool *prev) {
    bool fired = (now && !*prev);
    *prev = now;
    return fired;
}

// ---------- Estados ----------
typedef enum {
    ST_ASK = 0,        // Pergunta inicial
    ST_COLOR_PREP,     // Mensagem de instrução para cor
    ST_COLOR_LOOP,     // Loop de leitura de cor (A captura)
    ST_OXI_PREP,       // Inicializa/ativa oxímetro
    ST_OXI_RUN         // Oxímetro rodando
} state_t;

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    // OLED em I2C1 (14/15)
    i2c_setup(OLED_I2C, OLED_SDA, OLED_SCL, 400000);
    oled.external_vcc = false;
    oled_ok = ssd1306_init(&oled, 128, 64, OLED_ADDR, OLED_I2C);

    // Botões
    gpio_init(BUTTON_A); gpio_set_dir(BUTTON_A, GPIO_IN); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_set_dir(BUTTON_B, GPIO_IN); gpio_pull_up(BUTTON_B);
    bool a_prev = false, b_prev = false;

    // Flags de init de sensores (os dois no I2C0 via extensor)
    bool cor_ready = false;
    bool oxi_inited = false;

    state_t st = ST_ASK;
    state_t last_st = -1;
    uint32_t t_last = 0;

    // holdoff para ignorar cliques enquanto o dedo ainda está em cima do botão
    uint32_t holdoff_until_ms = 0;

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        // aplica holdoff de clique
        bool a_now_raw = !gpio_get(BUTTON_A); // ativo baixo
        bool b_now_raw = !gpio_get(BUTTON_B);
        bool a_now = (now_ms >= holdoff_until_ms) ? a_now_raw : false;
        bool b_now = (now_ms >= holdoff_until_ms) ? b_now_raw : false;

        bool a_edge = edge_press(a_now, &a_prev);
        bool b_edge = edge_press(b_now, &b_prev);

        // desenha a tela só quando muda de estado
        if (st != last_st) {
            switch (st) {
                case ST_ASK:
                    oled_lines("Fazer leitura de cor?", "(A) Sim   (B) Nao", "", "");
                    break;
                case ST_COLOR_PREP:
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", "Posicione o cartao", "");
                    break;
                case ST_COLOR_LOOP:
                    // tela dinamica atualizada no loop
                    break;
                case ST_OXI_PREP:
                    oled_lines("Oximetro ativando...", "Posicione o dedo", "", "");
                    break;
                case ST_OXI_RUN:
                    // será atualizado dinamicamente
                    break;
            }
            last_st = st;
        }

        switch (st) {
        case ST_ASK:
            if (a_edge) {
                // Inicia etapa de cor
                if (!cor_ready) {
                    i2c_setup(COL_I2C, COL_SDA, COL_SCL, 100000);          // I2C0 via extensor
                    cor_ready = cor_init(COL_I2C, COL_SDA, COL_SCL);
                }
                if (!cor_ready) {
                    oled_lines("TCS34725 nao encontrado", "Pulando etapa de cor", "", "");
                    holdoff_until_ms = now_ms + 300;
                    sleep_ms(900);
                    st = ST_OXI_PREP;
                } else {
                    holdoff_until_ms = now_ms + 300;
                    st = ST_COLOR_PREP;
                }
            } else if (b_edge) {
                // Pula direto para oxímetro
                holdoff_until_ms = now_ms + 300;
                st = ST_OXI_PREP;
            }
            break;

        case ST_COLOR_PREP:
            // Pausa visual curta e segue para o loop de cor
            if (now_ms - t_last > 250) {
                t_last = now_ms;
                st = ST_COLOR_LOOP;
            }
            break;

        case ST_COLOR_LOOP: {
            // Atualiza leitura ~5 Hz
            if (now_ms - t_last > 200) {
                t_last = now_ms;

                float rf, gf, bf, cf;
                if (cor_read_rgb_norm(&rf, &gf, &bf, &cf)) {
                    cor_class_t cls = cor_classify(rf, gf, bf, cf);
                    const char *nome = cor_class_to_str(cls);

                    char l3[24], l4[24];
                    int R = (int)(rf + 0.5f);
                    int G = (int)(gf + 0.5f);
                    int B = (int)(bf + 0.5f);

                    snprintf(l3, sizeof l3, "Cor: %s", nome);
                    snprintf(l4, sizeof l4, "R%3d G%3d B%3d", R, G, B);
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", l3, l4);
                } else {
                    oled_lines("Lendo Cor...", "Aperte A p/ capturar", "Sem leitura", "");
                }
            }

            if (a_edge) {
                // Captura a cor atual e avança
                float rf, gf, bf, cf;
                if (cor_read_rgb_norm(&rf, &gf, &bf, &cf)) {
                    cor_class_t cls = cor_classify(rf, gf, bf, cf);
                    g_cor_counts[cls]++;
                    const char *nome = cor_class_to_str(cls);

                    char msg[26];
                    snprintf(msg, sizeof msg, "Cor %s captada!", nome);
                    oled_lines(msg, "", "", "");
                    holdoff_until_ms = now_ms + 350;
                    sleep_ms(900);

                    st = ST_OXI_PREP;   // segue para oxímetro
                } else {
                    oled_lines("Falha na leitura", "Tente novamente", "", "");
                    holdoff_until_ms = now_ms + 300;
                    sleep_ms(800);
                }
            }
            break;
        }

        case ST_OXI_PREP: {
            // Inicializa oxímetro (I2C0 via extensor) e inicia
            if (!oxi_inited) {
                i2c_setup(OXI_I2C, OXI_SDA, OXI_SCL, 100000);              // I2C0 via extensor
                oxi_inited = oxi_init(OXI_I2C, OXI_SDA, OXI_SCL);
            }
            if (!oxi_inited) {
                oled_lines("MAX3010x nao encontrado", "Verifique cabos", "Voltando ao menu", "");
                holdoff_until_ms = now_ms + 400;
                sleep_ms(1200);
                st = ST_ASK;
                break;
            }
            oxi_start();                                                   // liga LED e zera filtros
            holdoff_until_ms = now_ms + 300;
            t_last = now_ms;
            st = ST_OXI_RUN;
            break;
        }

        case ST_OXI_RUN: {
            // permite abortar com B
            if (b_edge) {
                oled_lines("Oxímetro cancelado", "Voltando ao menu...", "", "");
                holdoff_until_ms = now_ms + 300;
                sleep_ms(700);
                st = ST_ASK;
                break;
            }

            oxi_poll(now_ms);

            // atualiza a tela a cada ~200 ms
            if (now_ms - t_last > 200) {
                t_last = now_ms;

                oxi_state_t s = oxi_get_state();
                if (s == OXI_WAIT_FINGER) {
                    oled_lines("Oxímetro ativo", "Posicione o dedo", "Aguardando...", "(B) Voltar");
                } else if (s == OXI_SETTLE) {
                    oled_lines("Oxímetro ativo", "Calibrando...", "Mantenha o dedo", "(B) Voltar");
                } else if (s == OXI_RUN) {
                    int n, tgt; oxi_get_progress(&n, &tgt);
                    float live = oxi_get_bpm_live();
                    char l2[22], l3[22];
                    snprintf(l2, sizeof l2, "BPM~ %.1f", live);
                    snprintf(l3, sizeof l3, "Validas: %d/%d", n, tgt);
                    oled_lines("Medindo...", l2, l3, "(B) Voltar");
                } else if (s == OXI_DONE) {
                    float final_bpm = oxi_get_bpm_final();
                    char l2[22]; snprintf(l2, sizeof l2, "BPM FINAL: %.1f", final_bpm);
                    oled_lines("Concluido!", l2, "", "");
                    sleep_ms(1500);
                    st = ST_ASK;   // volta ao menu por enquanto
                } else if (s == OXI_ERROR) {
                    oled_lines("ERRO no oxímetro", "Cheque conexoes", "", "");
                    sleep_ms(1500);
                    st = ST_ASK;
                }
            }
            break;
        }
        }

        sleep_ms(10);
    }
}
