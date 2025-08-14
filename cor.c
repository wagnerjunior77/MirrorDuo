// TCS34725 - leitura e classificação de cor com rótulo "semáforo"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

// ==== I2C (TCS na I2C0, conector direito da BitDog – aqui usando GP0/GP1) ====
#define I2C_PORT   i2c0
#define SDA_PIN    0
#define SCL_PIN    1
#define I2C_HZ     100000

// ==== TCS34725 regs/const ====
#define TCS_ADDR       0x29
#define TCS_CMD        0x80
#define TCS_AUTO_INC   0x20

#define TCS_ENABLE     0x00
#define TCS_ATIME      0x01
#define TCS_CONTROL    0x0F
#define TCS_ID         0x12
#define TCS_STATUS     0x13
#define TCS_CDATA      0x14  // +CL / CH
#define TCS_RDATA      0x16  // +RL / RH
#define TCS_GDATA      0x18
#define TCS_BDATA      0x1A

// ENABLE bits
#define TCS_PON        0x01
#define TCS_AEN        0x02

// Ganho: 0=1x, 1=4x, 2=16x, 3=60x
#define TCS_GAIN_1X    0x00
#define TCS_GAIN_4X    0x01
#define TCS_GAIN_16X   0x02
#define TCS_GAIN_60X   0x03

// ATIME: tempo = 2.4ms * (256 - ATIME)
// 0xC0 -> ~153.6 ms (igual ao seu micropython)
#define TCS_ATIME_153MS 0xC0

// helpers I2C
static inline bool tcs_w8(uint8_t reg, uint8_t v) {
    uint8_t b[2] = { (uint8_t)(TCS_CMD | reg), v };
    return i2c_write_timeout_us(I2C_PORT, TCS_ADDR, b, 2, false, 2000) == 2;
}
static inline bool tcs_rn(uint8_t reg, uint8_t *dst, size_t n) {
    uint8_t cmd = (uint8_t)(TCS_CMD | TCS_AUTO_INC | reg);
    int w = i2c_write_timeout_us(I2C_PORT, TCS_ADDR, &cmd, 1, true, 2000);
    if (w < 0) return false;
    return i2c_read_timeout_us(I2C_PORT, TCS_ADDR, dst, n, false, 2000) == (int)n;
}

static bool tcs_init(void) {
    // liga I2C
    i2c_init(I2C_PORT, I2C_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // confere ID
    uint8_t id = 0;
    if (!tcs_rn(TCS_ID, &id, 1)) return false;
    // TCS34725 costuma retornar 0x44
    if (id != 0x44 && id != 0x10 && id != 0x14) {
        printf("ID inesperado: 0x%02X (esperado 0x44)\n", id);
    }

    // Alimenta + ADC
    tcs_w8(TCS_ENABLE, TCS_PON);
    sleep_ms(3);
    tcs_w8(TCS_ENABLE, TCS_PON | TCS_AEN);

    // ATIME e ganho
    tcs_w8(TCS_ATIME,  TCS_ATIME_153MS);
    tcs_w8(TCS_CONTROL, TCS_GAIN_16X);

    return true;
}

static bool tcs_read_raw(uint16_t *c, uint16_t *r, uint16_t *g, uint16_t *b) {
    uint8_t d[8];
    if (!tcs_rn(TCS_CDATA, d, 8)) return false;
    *c = (uint16_t)(d[1] << 8 | d[0]);
    *r = (uint16_t)(d[3] << 8 | d[2]);
    *g = (uint16_t)(d[5] << 8 | d[4]);
    *b = (uint16_t)(d[7] << 8 | d[6]);
    return true;
}

// ------------ utilitários cor --------------
static void rgb_to_hsv(float r, float g, float b, float *H, float *S, float *V) {
    float maxv = fmaxf(r, fmaxf(g, b));
    float minv = fminf(r, fminf(g, b));
    float d = maxv - minv;
    *V = maxv;
    *S = (maxv <= 1e-6f) ? 0.f : d / maxv;

    float h;
    if (d <= 1e-6f) h = 0.f;
    else if (maxv == r) h = 60.f * fmodf(((g - b) / d), 6.f);
    else if (maxv == g) h = 60.f * (((b - r) / d) + 2.f);
    else                h = 60.f * (((r - g) / d) + 4.f);
    if (h < 0.f) h += 360.f;
    *H = h;
}

// rótulo de cor “fino” + rótulo de semáforo
static void color_labels(float H, float S, float V,
                         const char **fine, const char **traffic)
{
    // baixa luz / preto
    if (V < 0.07f) { *fine = "preto"; *traffic = "VERMELHO"; return; }

    // quase sem saturação => branco/cinza
    if (S < 0.18f) {
        if (V > 0.85f) { *fine = "branco"; *traffic = "AMARELO"; }
        else           { *fine = "cinza";  *traffic = "AMARELO"; }
        return;
    }

    // buckets de matiz (em graus)
    if (H < 15 || H >= 345) { *fine = "vermelho"; *traffic = "VERMELHO"; return; }
    if (H < 45)   { *fine = "laranja"; *traffic = "AMARELO"; return; }
    if (H < 75)   { *fine = "amarelo"; *traffic = "AMARELO"; return; }
    if (H < 165)  { *fine = "verde";   *traffic = "VERDE";   return; }
    if (H < 195)  { *fine = "ciano";   *traffic = "VERDE";   return; }
    if (H < 255)  { *fine = "azul";    *traffic = "VERMELHO";return; }
    if (H < 300)  { *fine = "roxo";    *traffic = "VERMELHO";return; }
    /* 300–345 */ { *fine = "magenta"; *traffic = "VERMELHO";return; }
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(10);

    printf("\nTCS34725 — classificação de cor (ganho 16x, ATIME~153ms)\n");

    if (!tcs_init()) {
        printf("Falha ao iniciar TCS34725.\n");
        while (1) tight_loop_contents();
    }

    while (true) {
        uint16_t c, r, g, b;
        if (!tcs_read_raw(&c, &r, &g, &b)) {
            printf("Erro de leitura I2C\n");
            sleep_ms(50);
            continue;
        }

        // Normaliza por 'clear' (evita overflow e escala para 0..1)
        float rf=0, gf=0, bf=0;
        if (c > 0) {
            rf = (float)r / (float)c;
            gf = (float)g / (float)c;
            bf = (float)b / (float)c;
        }
        // clamp
        if (rf<0) rf=0; if (rf>1) rf=1;
        if (gf<0) gf=0; if (gf>1) gf=1;
        if (bf<0) bf=0; if (bf>1) bf=1;

        float H,S,V;
        rgb_to_hsv(rf, gf, bf, &H, &S, &V);

        const char *fine="?", *traf="?";
        color_labels(H, S, V, &fine, &traf);

        
        // (>>8 para aproximar 0–255, como no seu MicroPython)
        printf("Clr=%4u  R=%3u  G=%3u  B=%3u  |  cor=%-8s  [semaforo=%s]\n",
               (unsigned)(c>>8), (unsigned)(r>>8), (unsigned)(g>>8), (unsigned)(b>>8),
               fine, traf);

        sleep_ms(100); // ~10 Hz — suficiente dado ATIME≈153 ms
    }
}
