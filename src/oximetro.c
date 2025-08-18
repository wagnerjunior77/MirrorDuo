#include "oximetro.h"
#include <string.h>
#include <math.h>
#include "hardware/i2c.h"

// ======= Ajustes do algoritmo =======
#define I2C_ADDR        0x57

#define FINGER_IR_MIN   6000.0f
#define THR_FACTOR      0.45f
#define RMS_BETA        0.03f
#define SETTLE_SAMPLES  200

#define TARGET_VALID    20
#define ACCEPT_DEV_FRAC 0.18f
#define TRIM_EACH_SIDE  2

#define PRINT_PERIOD_MS 200
#define SAMPLE_PERIOD_MS 10

// ======= I2C helpers =======
static i2c_inst_t *g_i2c = NULL;
static uint g_sda=0, g_scl=0;

static inline bool w8(uint8_t r, uint8_t v){
    uint8_t b[2]={r,v};
    int w = i2c_write_timeout_us(g_i2c, I2C_ADDR, b, 2, false, 2000);
    return (w == 2);
}
static inline bool rn(uint8_t r, uint8_t *d, size_t n){
    int w = i2c_write_timeout_us(g_i2c, I2C_ADDR, &r, 1, true, 2000);
    if(w < 0) return false;
    int rr = i2c_read_timeout_us(g_i2c, I2C_ADDR, d, n, false, 2000);
    return (rr == (int)n);
}

// ======= MAX30100 =======
static bool max30100_init(void){
    bool ok=true;
    ok &= w8(0x06,0x40); sleep_ms(10);                 // reset
    ok &= w8(0x02,0x00); ok &= w8(0x03,0x00); ok &= w8(0x04,0x00);
    ok &= w8(0x07,(1u<<6)|(0b0011<<2)|0b11);           // SPO2 cfg: 100 Hz, 16-bit
    ok &= w8(0x09,0x35); ok &= w8(0x0A,0x35);          // LED ~19 mA
    ok &= w8(0x06,0x03);                               // mode = SPO2
    uint8_t mode=0; ok &= rn(0x06,&mode,1);            // confirma modo
    if((mode & 0x07) != 0x03) ok=false;
    return ok;
}
static bool max30100_read(uint16_t *ir, uint16_t *red){
    uint8_t d[4];
    if(!rn(0x05,d,4)) return false;
    *ir=(d[0]<<8)|d[1]; *red=(d[2]<<8)|d[3];
    return true;
}

// ======= MAX30102 =======
static bool max30102_init(void){
    bool ok=true;
    ok &= w8(0x09,0x40); sleep_ms(10);                 // reset
    ok &= w8(0x08,(0b010<<5)|(1<<4)|0x00);             // FIFO: avg=4, rollover
    ok &= w8(0x0A,(0b10<<5)|(0b011<<2)|0b11);          // SPO2: 100 Hz
    ok &= w8(0x0C,0x35); ok &= w8(0x0D,0x35);          // LED currents
    ok &= w8(0x11,(0x01)|(0x02<<4)); ok &= w8(0x12,0x00); // slots
    ok &= w8(0x04,0x00); ok &= w8(0x05,0x00); ok &= w8(0x06,0x00); // FIFO ptrs
    ok &= w8(0x09,0x03);                               // mode = SPO2
    uint8_t mode=0; ok &= rn(0x09,&mode,1);
    if((mode & 0x07) != 0x03) ok=false;
    return ok;
}
static bool max30102_read(uint32_t *ir, uint32_t *red){
    uint8_t d[6];
    if(!rn(0x07,d,6)) return false;
    *red=(((uint32_t)d[0]<<16)|((uint32_t)d[1]<<8)|d[2])&0x3FFFF;
    *ir =(((uint32_t)d[3]<<16)|((uint32_t)d[4]<<8)|d[5])&0x3FFFF;
    return true;
}

// ======= Pipeline HR =======
#define RR_MAX 8
static float ema_dc=0, rms=1, prev_ac=0;
static uint32_t rr[RR_MAX]; static int rr_n=0;
static uint32_t last_beat_ms=0; static float bpm_s=0;

static void hr_reset(void){
    ema_dc=0; rms=1; prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0;
}
static float median_u32(const uint32_t* v,int n){
    uint32_t a[RR_MAX];
    for(int i=0;i<n;i++) a[i]=v[i];
    for(int i=1;i<n;i++){ uint32_t x=a[i], j=i; while(j>0 && a[j-1]>x){a[j]=a[j-1]; j--; } a[j]=x; }
    return (n&1)? (float)a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
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

// ======= Coleta para BPM final =======
#define BUF_MAX 32
static float buf[BUF_MAX]; static int buf_n=0; static bool frozen=false;

static float median_float(const float* v,int n){
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
static void reset_collection(void){ buf_n=0; frozen=false; }

// ======= Estado geral =======
static oxi_state_t g_state = OXI_IDLE;
static bool g_is30102 = false;
static bool g_inited  = false;

static uint32_t sample_last_ms = 0;
static uint32_t print_last_ms  = 0;

static int      settle_cnt=0; 
static double   sum=0, sum2=0;

static float    bpm_final = NAN;

// ======= API =======
bool oxi_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin){
    g_i2c = i2c; g_sda=sda_pin; g_scl=scl_pin;

    i2c_init(g_i2c, 100000);
    gpio_set_function(g_sda, GPIO_FUNC_I2C);
    gpio_set_function(g_scl, GPIO_FUNC_I2C);
    gpio_pull_up(g_sda); gpio_pull_up(g_scl);

    // Prova de presença no 0x57 (qualquer reg simples)
    uint8_t tmp=0;
    if(!rn(0x00,&tmp,1) && !rn(0x01,&tmp,1)) { // se ambos falham, não está no barramento
        g_inited=false; g_state=OXI_ERROR;
        return false;
    }

    // Tenta identificar MAX30102 via PART ID (0xFF==0x15). Se falhar, assume 30100.
    uint8_t part=0;
    bool ok_part = rn(0xFF,&part,1);
    g_is30102 = ok_part && (part==0x15);

    bool init_ok = g_is30102 ? max30102_init() : max30100_init();
    if(!init_ok){
        g_inited=false; g_state=OXI_ERROR;
        return false;
    }

    g_inited = true;
    g_state  = OXI_IDLE;
    return true;
}

void oxi_start(void){
    if(!g_inited){ g_state = OXI_ERROR; return; }
    if(g_is30102) max30102_init(); else max30100_init();

    hr_reset(); reset_collection();
    settle_cnt=0; sum=0; sum2=0;
    sample_last_ms = 0; print_last_ms = 0;
    bpm_final = NAN;

    g_state = OXI_WAIT_FINGER;
}

void oxi_abort(void){
    g_state = OXI_IDLE;
}

static bool read_sample(float *ir_f, float *red_f){
    if(!g_inited) return false;
    if(g_is30102){
        uint32_t ir32, rd32; if(!max30102_read(&ir32, &rd32)) return false;
        *ir_f = (float)ir32; *red_f = (float)rd32;
    } else {
        uint16_t ir16, rd16; if(!max30100_read(&ir16, &rd16)) return false;
        *ir_f = (float)ir16; *red_f = (float)rd16;
    }
    return true;
}

void oxi_poll(uint32_t now_ms){
    if(g_state==OXI_IDLE || g_state==OXI_ERROR || g_state==OXI_DONE) return;
    if(now_ms - sample_last_ms < SAMPLE_PERIOD_MS) return;
    sample_last_ms = now_ms;

    float ir=0, red=0;
    if(!read_sample(&ir, &red)) return;

    bool finger = (ir > FINGER_IR_MIN);

    switch(g_state){
    case OXI_WAIT_FINGER:
        if(finger){
            settle_cnt=0; sum=0; sum2=0;
            hr_reset(); reset_collection();
            g_state = OXI_SETTLE;
        }
        break;

    case OXI_SETTLE:
        if(!finger){ g_state = OXI_WAIT_FINGER; break; }
        sum  += ir; sum2 += (double)ir*(double)ir; settle_cnt++;
        if(settle_cnt >= SETTLE_SAMPLES){
            double mean = sum / settle_cnt;
            double var  = fmax(1.0, (sum2/settle_cnt) - mean*mean);
            ema_dc = (float)mean;
            rms    = (float)sqrt(var);
            prev_ac=0; rr_n=0; last_beat_ms=0; bpm_s=0;
            g_state = OXI_RUN;
        }
        break;

    case OXI_RUN: {
        if(!finger){ g_state = OXI_WAIT_FINGER; break; }
        float bpm_inst = hr_update(ir, now_ms);
        if(bpm_inst>35 && bpm_inst<180 && rr_n>=3){
            float rr_med = median_u32(rr, rr_n);
            float bpm_med = 60000.f/rr_med;
            const float a=0.18f;
            bpm_s = (1.f-a)*bpm_s + a*bpm_med;

            if(now_ms - print_last_ms > PRINT_PERIOD_MS){
                print_last_ms = now_ms;
                if(!frozen){
                    bool accept=true;
                    if(buf_n>=5){
                        float med = median_float(buf, buf_n);
                        float dev = fabsf(bpm_s - med) / fmaxf(1.f, med);
                        if(dev > ACCEPT_DEV_FRAC) accept=false;
                    }
                    if(accept && buf_n<BUF_MAX) buf[buf_n++] = bpm_s;

                    if(buf_n >= TARGET_VALID){
                        float tmp[BUF_MAX]; for(int i=0;i<buf_n;i++) tmp[i]=buf[i];
                        bpm_final = trimmed_mean(tmp, buf_n, TRIM_EACH_SIDE);
                        frozen = true; g_state = OXI_DONE;
                    }
                }
            }
        }
        break;
    }

    default: break;
    }
}

oxi_state_t oxi_get_state(void){ return g_state; }

void oxi_get_progress(int *valid_count, int *target_valid){
    if(valid_count)  *valid_count  = buf_n;
    if(target_valid) *target_valid = TARGET_VALID;
}
float oxi_get_bpm_live(void){ return bpm_s; }
float oxi_get_bpm_final(void){ return bpm_final; }
