// MAX3010x BPM — polling com histerese de contato e aceitação adaptativa
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

// ===== I2C / endereço =====
#define I2C_PORT  i2c1
#define SDA_PIN   2
#define SCL_PIN   3
#define I2C_HZ    100000
#define ADDR      0x57

static inline bool w8(uint8_t r, uint8_t v){ uint8_t b[2]={r,v}; return i2c_write_timeout_us(I2C_PORT,ADDR,b,2,false,2000)==2; }
static inline bool rn(uint8_t r, uint8_t *d, size_t n){
    if(i2c_write_timeout_us(I2C_PORT,ADDR,&r,1,true,2000)<0) return false;
    return i2c_read_timeout_us(I2C_PORT,ADDR,d,n,false,2000)==(int)n;
}

// ===== MAX30100 =====
static bool max30100_init(void){
    w8(0x06,0x40); sleep_ms(10);
    w8(0x02,0x00); w8(0x03,0x00); w8(0x04,0x00);
    w8(0x07,(1u<<6)|(0b0011<<2)|0b11);     // 100 Hz, 16-bit, hi-res
    w8(0x09,0x35);                         // ~19 mA (RED|IR no 30100 é 0x09)
    w8(0x06,0x03);                         // SPO2
    return true;
}
static bool max30100_read(uint16_t *ir, uint16_t *red){
    uint8_t d[4]; if(!rn(0x05,d,4)) return false;
    *ir=(d[0]<<8)|d[1]; *red=(d[2]<<8)|d[3]; return true;
}

// ===== MAX30102 (caso a placa seja 30102) =====
static bool max30102_init(void){
    w8(0x09,0x40); sleep_ms(10);
    w8(0x08,(0b010<<5)|(1<<4));            // AVG=4, rollover
    w8(0x0A,(0b10<<5)|(0b011<<2)|0b11);    // 100 Hz, 18-bit
    w8(0x0C,0x35); w8(0x0D,0x35);          // RED/IR
    w8(0x11,(0x01)|(0x02<<4)); w8(0x12,0x00);
    w8(0x04,0x00); w8(0x05,0x00); w8(0x06,0x00);
    w8(0x09,0x03);
    return true;
}
static bool max30102_read(uint32_t *ir, uint32_t *red){
    uint8_t d[6]; if(!rn(0x07,d,6)) return false;
    *red=(((uint32_t)d[0]<<16)|((uint32_t)d[1]<<8)|d[2])&0x3FFFF;
    *ir =(((uint32_t)d[3]<<16)|((uint32_t)d[4]<<8)|d[5])&0x3FFFF;
    return true;
}

// ===== HR pipeline =====
#define RMS_BETA          0.03f
#define THR_FACTOR        0.45f
#define PRINT_MS          1000
#define RR_MAX            8

static float ema_dc=0, rms=1, prev_ac=0;
static uint32_t rr[RR_MAX]; static int rr_n=0;
static uint32_t last_beat_ms=0; static float bpm_s=0;

static void hr_reset(void){ ema_dc=0; rms=1; prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0; }

static float median_u32(const uint32_t* v,int n){
    uint32_t a[RR_MAX]; for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ uint32_t x=a[i], j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? (float)a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}

static float hr_update(float sample, uint32_t now_ms){
    ema_dc += 0.01f*(sample-ema_dc);
    float ac = sample - ema_dc;
    rms = sqrtf((1.f-RMS_BETA)*rms*rms + RMS_BETA*ac*ac);
    float thr = THR_FACTOR * rms;

    float bpm = 0.f;
    if (prev_ac <= thr && ac > thr) {
        if (last_beat_ms == 0 || (now_ms - last_beat_ms) > 280) {
            if (last_beat_ms) {
                uint32_t dt = now_ms - last_beat_ms;  // <- aqui estava "now"
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

// ===== Coleta com aceitação adaptativa =====
#define TARGET_VALID      20
#define TRIM_EACH_SIDE    2
#define BUF_MAX           32

static float buf[BUF_MAX]; static int buf_n=0; static bool frozen=false;
static uint32_t last_accept_ms=0;

static float median_float(const float* v,int n){
    float a[BUF_MAX]; for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ float x=a[i]; int j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
static float trimmed_mean(float* a,int n,int trim){
    for(int i=1;i<n;i++){ float x=a[i]; int j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    int start = trim, end = n - trim; if(end<=start) { start=0; end=n; }
    double s=0; for(int i=start;i<end;i++) s+=a[i];
    return (float)(s / (end-start));
}
static void reset_collection(void){ buf_n=0; frozen=false; last_accept_ms=to_ms_since_boot(get_absolute_time()); }

// ===== Histerese + debounce de contato =====
#define FINGER_ON_MIN     5000.0f     // entra acima disso (ajuste se precisar)
#define FINGER_OFF_MIN    3500.0f     // sai só abaixo disso (histerese)
#define ON_CONFIRM_SAMP   8           // ~80 ms
#define OFF_CONFIRM_SAMP  30          // ~300 ms

static bool finger=false; static int on_cnt=0, off_cnt=0;
static inline void update_finger(float ir){
    if(!finger){
        if(ir > FINGER_ON_MIN){ if(++on_cnt >= ON_CONFIRM_SAMP){ finger=true; on_cnt=off_cnt=0; } }
        else on_cnt=0;
    }else{
        if(ir < FINGER_OFF_MIN){ if(++off_cnt >= OFF_CONFIRM_SAMP){ finger=false; on_cnt=off_cnt=0; } }
        else off_cnt=0;
    }
}

// ===== Main =====
int main(void){
    stdio_init_all(); while(!stdio_usb_connected()) sleep_ms(10);
    printf("\nMAX3010x — BPM final após 20 leituras válidas (com histerese)\n");

    i2c_init(I2C_PORT, I2C_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); gpio_pull_up(SCL_PIN);

    uint8_t part=0; bool ok=rn(0xFF,&part,1);
    bool is30102 = ok && (part==0x15);
    if(is30102){ printf("Sensor: MAX30102\n"); max30102_init(); }
    else       { printf("Sensor: MAX30100/compatível\n"); max30100_init(); }

    enum { WAIT, SETTLE, RUN } st = WAIT;
    uint32_t last_print=0;
    int settle_cnt=0; double sum=0, sum2=0;

    while(true){
        uint32_t now = to_ms_since_boot(get_absolute_time());
        float ir=0, red=0;

        if(is30102){ uint32_t ir32, rd32; if(max30102_read(&ir32,&rd32)){ ir=ir32; red=rd32; } }
        else        { uint16_t ir16, rd16; if(max30100_read(&ir16,&rd16)){ ir=ir16; red=rd16; } }

        update_finger(ir);

        switch(st){
        case WAIT:
            if(finger){
                settle_cnt=0; sum=0; sum2=0;
                reset_collection(); hr_reset();
                st = SETTLE;
                printf("Detectei dedo. Calibrando...\n");
            } else {
                static uint32_t t=0; if(now-t>700){ printf("Posicione o dedo no sensor...\n"); t=now; }
            }
            break;

        case SETTLE:
            if(!finger){ st = WAIT; break; }
            sum  += ir; sum2 += (double)ir*(double)ir; settle_cnt++;
            if(settle_cnt >= 200){                 // ~2 s
                double mean = sum / settle_cnt;
                double var  = fmax(1.0, (sum2/settle_cnt) - mean*mean);
                ema_dc = (float)mean;
                rms    = (float)sqrt(var);
                prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0;
                st = RUN;
                printf("OK!  Medindo...\n");
                last_accept_ms = now;
            }
            break;

        case RUN:
            if(!finger){ st = WAIT; printf("Dedo removido — pronto para nova medição.\n"); break; }
            {
                float bpm_inst = hr_update(ir, now);
                if(bpm_inst>35 && bpm_inst<180 && rr_n>=3){
                    float rr_med = median_u32(rr, rr_n);
                    float bpm_med = 60000.f/rr_med;
                    const float a=0.18f;
                    bpm_s = (1.f-a)*bpm_s + a*bpm_med;

                    if(now-last_print > PRINT_MS){
                        last_print = now;

                        if(!frozen){
                            // banda de aceitação adaptativa
                            float band = (buf_n<6)?0.40f : (buf_n<12?0.25f:0.18f);
                            bool accept=true;
                            if(buf_n>=5){
                                float med = median_float(buf, buf_n);
                                float dev = fabsf(bpm_s - med) / fmaxf(1.f, med);
                                if(dev > band) accept=false;
                            }
                            if(accept && buf_n<BUF_MAX){
                                buf[buf_n++] = bpm_s;
                                last_accept_ms = now;
                            }

                            printf("BPM: %.1f  (amostras válidas: %d/%d)\n", bpm_s, buf_n, TARGET_VALID);

                            // fecha
                            if(buf_n >= TARGET_VALID){
                                float tmp[BUF_MAX]; for(int i=0;i<buf_n;i++) tmp[i]=buf[i];
                                float bpm_final = trimmed_mean(tmp, buf_n, TRIM_EACH_SIDE);
                                printf("\n==== BPM FINAL: %.1f  (com %d leituras) ====\n\n", bpm_final, buf_n);
                                frozen = true;
                            }

                            // timeout de progresso: se 8 s sem aceitar, reinicia calibração
                            if(!frozen && (now - last_accept_ms) > 8000){
                                printf("Sinal instável — recalibrando...\n");
                                st = SETTLE; settle_cnt=0; sum=0; sum2=0;
                                reset_collection(); hr_reset();
                            }
                        }
                    }
                }
            }
            break;
        }
        sleep_ms(10); // ~100 Hz
    }
}
