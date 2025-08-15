#include "oximetro.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

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
static bool max30100_init_(void){
    w8(0x06,0x40); sleep_ms(10);               // reset
    w8(0x02,0x00); w8(0x03,0x00); w8(0x04,0x00);
    w8(0x07,(1u<<6)|(0b0011<<2)|0b11);         // 100 Hz, 16-bit
    w8(0x09,0x35); w8(0x0A,0x35);              // ~19 mA
    w8(0x06,0x03);                              // SPO2
    return true;
}
static bool max30100_read_(uint16_t *ir, uint16_t *red){
    uint8_t d[4]; if(!rn(0x05,d,4)) return false;
    *ir=(d[0]<<8)|d[1]; *red=(d[2]<<8)|d[3]; return true;
}

// ===== MAX30102 =====
static bool max30102_init_(void){
    w8(0x09,0x40); sleep_ms(10);
    w8(0x08,(0b010<<5)|(1<<4)|0x00);
    w8(0x0A,(0b10<<5)|(0b011<<2)|0b11);        // 100 Hz
    w8(0x0C,0x35); w8(0x0D,0x35);
    w8(0x11,(0x01)|(0x02<<4)); w8(0x12,0x00);
    w8(0x04,0x00); w8(0x05,0x00); w8(0x06,0x00);
    w8(0x09,0x03);
    return true;
}
static bool max30102_read_(uint32_t *ir, uint32_t *red){
    uint8_t d[6]; if(!rn(0x07,d,6)) return false;
    *red=(((uint32_t)d[0]<<16)|((uint32_t)d[1]<<8)|d[2])&0x3FFFF;
    *ir =(((uint32_t)d[3]<<16)|((uint32_t)d[4]<<8)|d[5])&0x3FFFF;
    return true;
}

// ===== Algoritmo =====
#define FINGER_IR_MIN   6000.0f
#define RMS_BETA        0.03f
#define THR_FACTOR      0.45f
#define SETTLE_SAMPLES  200

#define BUF_MAX  32
#define RR_MAX    8

static bool is30102=false;

static float ema_dc=0, rms=1, prev_ac=0;
static uint32_t rr[RR_MAX]; static int rr_n=0;
static uint32_t last_beat_ms=0;
static float bpm_s=0;

static float buf[BUF_MAX]; static int buf_n=0;
static bool frozen=false;
static int collected=0;

static float median_u32(const uint32_t* v,int n){
    uint32_t a[RR_MAX]; for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ uint32_t x=a[i], j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? (float)a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
static float median_float(const float* v,int n){
    float a[BUF_MAX]; for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ float x=a[i]; int j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
static float trimmed_mean(float* a,int n,int trim){
    for(int i=1;i<n;i++){ float x=a[i]; int j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    int s=trim, e=n-trim; if(e<=s){ s=0; e=n; }
    double sum=0; for(int i=s;i<e;i++) sum+=a[i];
    return (float)(sum/(e-s));
}

static float hr_update(float sample, uint32_t now_ms){
    const float a_dc=0.01f;
    ema_dc += a_dc*(sample-ema_dc);
    float ac = sample - ema_dc;
    rms = sqrtf((1.f-RMS_BETA)*rms*rms + RMS_BETA*ac*ac);
    float thr = THR_FACTOR * rms;

    float bpm=0.f;
    if(prev_ac <= thr && ac > thr){
        if(last_beat_ms==0 || (now_ms-last_beat_ms)>280){
            if(last_beat_ms){
                uint32_t dt = now_ms - last_beat_ms;
                if(rr_n<RR_MAX) rr[rr_n++]=dt;
                else { memmove(rr,rr+1,(RR_MAX-1)*sizeof(uint32_t)); rr[RR_MAX-1]=dt; }
                bpm = 60000.f/(float)dt;
            }
            last_beat_ms = now_ms;
        }
    }
    prev_ac = ac;
    return bpm;
}

bool oxi_init(void){
    // i2c1 já pode estar iniciado pelo OLED — não tem problema reiniciar
    i2c_init(I2C_PORT, I2C_HZ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN); gpio_pull_up(SCL_PIN);

    uint8_t part=0; bool ok = rn(0xFF, &part, 1);
    is30102 = ok && (part==0x15);
    if(is30102) max30102_init_();
    else        max30100_init_();
    oxi_reset_session();
    return ok;
}

void oxi_reset_session(void){
    ema_dc=0; rms=1; prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0;
    buf_n=0; frozen=false; collected=0;
}

bool oxi_step(int target_valid, float *bpm_progress, int *valid_count, bool *done, float *bpm_final){
    *done=false; *bpm_final=0.f; *bpm_progress=0.f; *valid_count=collected;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    float ir=0;

    if(is30102){
        uint32_t ir32, rd32;
        if(!max30102_read_(&ir32,&rd32)) return false;
        ir = (float)ir32;
    }else{
        uint16_t ir16, rd16;
        if(!max30100_read_(&ir16,&rd16)) return false;
        ir = (float)ir16;
    }

    static enum { WAIT, SETTLE, RUN } st = WAIT;
    static int settle_cnt=0; static double sum=0, sum2=0;

    bool finger = (ir > FINGER_IR_MIN);
    if(st==WAIT){
        if(finger){ settle_cnt=0; sum=0; sum2=0; ema_dc=0; rms=1; prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0; st=SETTLE; }
    }else if(st==SETTLE){
        if(!finger){ st=WAIT; }
        else {
            sum += ir; sum2 += (double)ir*(double)ir; settle_cnt++;
            if(settle_cnt >= SETTLE_SAMPLES){
                double mean = sum/settle_cnt;
                double var  = fmax(1.0, (sum2/settle_cnt)-mean*mean);
                ema_dc=(float)mean; rms=(float)sqrt(var); prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0;
                st=RUN;
            }
        }
    }else{ // RUN
        if(!finger){ st=WAIT; }
        else{
            float bpm_inst = hr_update(ir, now);
            if(bpm_inst>35 && bpm_inst<180 && rr_n>=3){
                float rr_med = median_u32(rr, rr_n);
                float bpm_med = 60000.f/rr_med;
                const float a=0.18f;
                bpm_s = (1.f-a)*bpm_s + a*bpm_med;

                // aceita leitura estável
                static float hist[BUF_MAX];
                bool accept=true;
                if(buf_n>=5){
                    float med = median_float(hist, buf_n);
                    float dev = fabsf(bpm_s - med) / fmaxf(1.f, med);
                    if(dev > 0.18f) accept=false;
                }
                if(!frozen && accept){
                    if(buf_n<BUF_MAX) { hist[buf_n]=bpm_s; buf[buf_n++]=bpm_s; }
                    collected = buf_n;
                    *bpm_progress = bpm_s;
                    *valid_count = collected;

                    if(collected >= target_valid){
                        float tmp[BUF_MAX];
                        for(int i=0;i<buf_n;i++) tmp[i]=buf[i];
                        float final = trimmed_mean(tmp, buf_n, 2);
                        *bpm_final = final;
                        *done = true;
                        frozen=true;
                    }
                } else {
                    *bpm_progress = bpm_s;
                    *valid_count = collected;
                }
            }
        }
    }
    return true;
}
