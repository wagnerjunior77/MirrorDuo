#include "oximetro.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <string.h>

// ======== Barramento & endereço ========
#define OXI_I2C      i2c1
#define OXI_SDA_PIN  2
#define OXI_SCL_PIN  3
#define OXI_ADDR     0x57
#define OXI_HZ       100000

// ======== Parâmetros de detecção/filtragem ========
#define FINGER_IR_MIN     6000.0f   // limiar p/ detectar dedo (ajuste 3k–12k)
#define THR_FACTOR        0.45f     // limiar relativo de pico (0.40–0.55)
#define RMS_BETA          0.03f     // reação do RMS (0.02–0.05)
#define SETTLE_SAMPLES    200       // ~2 s a 100 Hz
#define TARGET_VALID      20        // nº leituras válidas p/ fechar
#define ACCEPT_DEV_FRAC   0.18f     // ±18% da mediana para aceitar
#define TRIM_EACH_SIDE    2         // aparar 2 de cada lado (10% de 20)

// ======== I2C helpers ========
static inline bool w8(uint8_t reg, uint8_t v){
    uint8_t b[2] = {reg, v};
    return i2c_write_timeout_us(OXI_I2C, OXI_ADDR, b, 2, false, 2000) == 2;
}
static inline bool rn(uint8_t reg, uint8_t *dst, size_t n){
    if (i2c_write_timeout_us(OXI_I2C, OXI_ADDR, &reg, 1, true, 2000) < 0) return false;
    return i2c_read_timeout_us (OXI_I2C, OXI_ADDR, dst, n, false, 2000) == (int)n;
}

// ======== MAX30100 ========
static bool max30100_init_ll(void){
    w8(0x06,0x40); sleep_ms(10);            // reset
    w8(0x02,0x00); w8(0x03,0x00); w8(0x04,0x00);  // FIFO ptrs
    w8(0x07,(1u<<6)|(0b0011<<2)|0b11);      // SPO2: 100Hz, 16-bit
    w8(0x09,0x35); w8(0x0A,0x35);           // LED ~19mA
    w8(0x06,0x03);                          // mode SPO2
    w8(0x01,0x10);                          // interrupt (opcional)
    return true;
}
static bool max30100_read(uint16_t *ir, uint16_t *red){
    uint8_t d[4]; if(!rn(0x05,d,4)) return false;
    *ir = (d[0]<<8)|d[1]; *red = (d[2]<<8)|d[3]; return true;
}

// ======== MAX30102 ========
static bool max30102_init_ll(void){
    w8(0x09,0x40); sleep_ms(10);                          // reset
    w8(0x08,(0b010<<5)|(1<<4)|0x00);                      // FIFO: avg=4, rollover=1
    w8(0x0A,(0b10<<5)|(0b011<<2)|0b11);                   // SPO2: 100Hz, 18-bit
    w8(0x0C,0x35); w8(0x0D,0x35);                         // LED ~19mA
    w8(0x11,(0x01)|(0x02<<4)); w8(0x12,0x00);             // slots: RED, IR
    w8(0x04,0x00); w8(0x05,0x00); w8(0x06,0x00);          // clear FIFO
    w8(0x09,0x03);                                        // mode SPO2
    return true;
}
static bool max30102_read(uint32_t *ir, uint32_t *red){
    uint8_t d[6]; if(!rn(0x07,d,6)) return false;
    *red = (((uint32_t)d[0]<<16)|((uint32_t)d[1]<<8)|d[2]) & 0x3FFFF;
    *ir  = (((uint32_t)d[3]<<16)|((uint32_t)d[4]<<8)|d[5]) & 0x3FFFF;
    return true;
}

// ======== Estado global do driver ========
static bool       g_inited   = false;
static bool       g_is30102  = false;
static oxi_state_t g_state   = OXI_WAIT;

// Filtros e buffers
#define RR_MAX 8
static float    ema_dc=0.0f, rms=1.0f, prev_ac=0.0f;
static uint32_t rr[RR_MAX]; static int rr_n=0;
static uint32_t last_beat_ms=0;
static float    bpm_s=0.0f;            // exibido
#define BUF_MAX 32
static float    buf[BUF_MAX]; static int buf_n=0;
static float    bpm_final_hold=0.0f;   // guarda final quando congela

// ======== helpers ========
static void hr_reset(void){
    ema_dc = 0.0f; rms = 1.0f; prev_ac = 0.0f;
    rr_n = 0; last_beat_ms = 0; bpm_s = 0.0f;
    buf_n = 0; bpm_final_hold = 0.0f;
}

static float median_u32(const uint32_t* v,int n){
    uint32_t a[RR_MAX];
    for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ uint32_t x=a[i], j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? (float)a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
static float median_f(const float* v,int n){
    float a[BUF_MAX]; for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ float x=a[i]; int j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
static float trimmed_mean(float* a,int n,int trim){
    for(int i=1;i<n;i++){ float x=a[i]; int j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    int start = trim, end = n - trim; if(end<=start){ start=0; end=n; }
    double s=0; for(int i=start;i<end;i++) s+=a[i];
    return (float)(s / (end-start));
}

static float hr_update(float sample, uint32_t now_ms){
    const float a_dc = 0.01f;
    ema_dc += a_dc * (sample - ema_dc);
    float ac = sample - ema_dc;

    rms = sqrtf((1.f - RMS_BETA)*rms*rms + RMS_BETA*ac*ac);
    float thr = THR_FACTOR * rms;

    float bpm = 0.f;
    if (prev_ac <= thr && ac > thr) {
        if (last_beat_ms == 0 || (now_ms - last_beat_ms) > 280) {
            if (last_beat_ms != 0) {
                uint32_t dt = now_ms - last_beat_ms;
                if (rr_n < RR_MAX) rr[rr_n++] = dt;
                else { memmove(rr, rr+1, (RR_MAX-1)*sizeof(uint32_t)); rr[RR_MAX-1] = dt; }
                bpm = 60000.f / (float)dt;
            }
            last_beat_ms = now_ms;
        }
    }
    prev_ac = ac;
    return bpm;
}

// ======== API ========
bool oxi_init(void){
    // Configura I2C1 nos pinos GP2/GP3 (100 kHz)
    i2c_init(OXI_I2C, OXI_HZ);
    gpio_set_function(OXI_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OXI_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OXI_SDA_PIN);
    gpio_pull_up(OXI_SCL_PIN);

    // Detecta Part ID
    uint8_t part=0;
    if (!rn(0xFF, &part, 1)) return false;
    g_is30102 = (part == 0x15); // 0x11 = 30100, 0x15 = 30102

    // Configura sensor
    if (g_is30102) max30102_init_ll(); else max30100_init_ll();

    g_inited = true;
    g_state  = OXI_WAIT;
    hr_reset();
    return true;
}

void oxi_start(void){
    if (!g_inited) return;
    g_state = OXI_WAIT;
    hr_reset();
}

oxi_state_t oxi_get_state(void){
    return g_state;
}

bool oxi_poll(uint32_t now_ms, float *bpm_display, int *valid_count, bool *done, float *bpm_final){
    if (!g_inited) return false;
    if (bpm_display) *bpm_display = bpm_s;
    if (valid_count) *valid_count = buf_n;
    if (done)        *done        = (g_state == OXI_FROZEN);
    if (bpm_final)   *bpm_final   = bpm_final_hold;

    // lê amostra
    float ir = 0, red = 0;
    if (g_is30102) {
        uint32_t ir32=0, rd32=0;
        if (!max30102_read(&ir32,&rd32)) return false;
        ir = (float)ir32; red = (float)rd32;
    } else {
        uint16_t ir16=0, rd16=0;
        if (!max30100_read(&ir16,&rd16)) return false;
        ir = (float)ir16; red = (float)rd16;
    }

    bool finger = (ir > FINGER_IR_MIN);

    static int      settle_cnt = 0;
    static double   sum=0.0, sum2=0.0;

    switch (g_state) {
    case OXI_WAIT:
        if (finger) {
            settle_cnt = 0; sum = 0.0; sum2 = 0.0;
            hr_reset();
            g_state = OXI_SETTLE;
        }
        break;

    case OXI_SETTLE:
        if (!finger) { g_state = OXI_WAIT; break; }
        sum  += ir;
        sum2 += (double)ir*(double)ir;
        settle_cnt++;
        if (settle_cnt >= SETTLE_SAMPLES) {
            double mean = sum / settle_cnt;
            double var  = fmax(1.0, (sum2/settle_cnt) - mean*mean);
            // injeta baseline nos filtros
            ema_dc = (float)mean;
            rms    = (float)sqrt(var);
            prev_ac = 0.0f; rr_n=0; last_beat_ms=0; bpm_s=0.0f; buf_n=0;
            g_state = OXI_RUN;
        }
        break;

    case OXI_RUN: {
        if (!finger) { g_state = OXI_WAIT; break; }
        float bpm_inst = hr_update(ir, now_ms);
        if (bpm_inst > 35.f && bpm_inst < 180.f && rr_n >= 3) {
            float rr_med  = median_u32(rr, rr_n);
            float bpm_med = 60000.f / rr_med;
            const float a = 0.18f;     // suavização da exibição
            bpm_s = (1.f - a)*bpm_s + a*bpm_med;

            // critério de estabilidade: aceita amostra dentro de ±18% da mediana acumulada
            bool accept = true;
            if (buf_n >= 5) {
                float med = median_f(buf, buf_n);
                float dev = fabsf(bpm_s - med) / fmaxf(1.f, med);
                if (dev > ACCEPT_DEV_FRAC) accept = false;
            }
            if (accept && buf_n < BUF_MAX) buf[buf_n++] = bpm_s;

            if (buf_n >= TARGET_VALID) {
                float tmp[BUF_MAX];
                for (int i=0;i<buf_n;i++) tmp[i] = buf[i];
                bpm_final_hold = trimmed_mean(tmp, buf_n, TRIM_EACH_SIDE);
                g_state = OXI_FROZEN;
            }
        }
        break;
    }

    case OXI_FROZEN:
        // permanece travado até oxi_start()
        break;
    }

    if (bpm_display) *bpm_display = bpm_s;
    if (valid_count) *valid_count = buf_n;
    if (done)        *done        = (g_state == OXI_FROZEN);
    if (bpm_final)   *bpm_final   = bpm_final_hold;
    return true;
}
