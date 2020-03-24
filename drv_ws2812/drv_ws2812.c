
#include <nrfx.h>
#include <string.h>
#include <nrfx_pwm.h>
#include <hal/nrf_gpio.h>
#include "drv_ws2812.h"
#include "nrf_delay.h"

#define WS2812_T1H                  14 | 0x8000
#define WS2812_T0H                  6 | 0x8000

//liming
#define LED_MATRIX_WIDTH        30
#define LED_MATRIX_HEIGHT       1

#if defined(NEOPIXEL_RING)
#define LED_MATRIX_TOTAL_BYTE_WIDTH	LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT * 4
#define LED_MATRIX_TOTAL_BIT_WIDTH	LED_MATRIX_TOTAL_BYTE_WIDTH * 8
#else
#define LED_MATRIX_TOTAL_BYTE_WIDTH	LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT * 3
#define LED_MATRIX_TOTAL_BIT_WIDTH	LED_MATRIX_TOTAL_BYTE_WIDTH * 8
#endif


static rgb_color_t led_matrix_buffer[LED_MATRIX_WIDTH][LED_MATRIX_HEIGHT];

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t pwm_duty_cycle_values[LED_MATRIX_TOTAL_BIT_WIDTH];
volatile bool pwm_sequencue_finished = true;

void pwm_handler(nrfx_pwm_evt_type_t event_type)
{
    switch(event_type)
    {
	case NRFX_PWM_EVT_FINISHED:
	    pwm_sequencue_finished = true;
	    break;
	default:
	    break;
    }
}

static nrf_pwm_sequence_t pwm_sequence =
{
    .values.p_individual = pwm_duty_cycle_values,
    .length          = (sizeof(pwm_duty_cycle_values) / sizeof(uint16_t)),
    .repeats         = 0,
    .end_delay       = 0
};

static uint32_t pwm_init(void)
{
    nrfx_pwm_config_t pwm_config = NRFX_PWM_DEFAULT_CONFIG;
    pwm_config.output_pins[0] = NRFX_PWM_PIN_NOT_USED; 
    pwm_config.output_pins[1] = WS2812_PIN; 
    pwm_config.output_pins[2] = NRFX_PWM_PIN_NOT_USED;
    pwm_config.output_pins[3] = NRFX_PWM_PIN_NOT_USED;
    pwm_config.load_mode    = NRF_PWM_LOAD_INDIVIDUAL;
    // WS2812 protocol requires a 800 kHz PWM frequency. PWM Top value = 20 and Base Clock = 16 MHz achieves this
    pwm_config.top_value    = 20; 
    pwm_config.base_clock   = NRF_PWM_CLK_16MHz;
    
    return nrfx_pwm_init(&m_pwm0, &pwm_config, pwm_handler);
}



static void convert_rgb_to_pwm_sequence(void)
{
    uint8_t * ptr = (uint8_t *)led_matrix_buffer;
    uint32_t i = 0;
    for(int led = 0; led < LED_MATRIX_TOTAL_BYTE_WIDTH; led++)
    {
        for(int bit = 7; bit >= 0; bit--)
        {
            uint8_t b = (*ptr >> bit) & 0x01;
            uint16_t pwm = 0;
            if(b == 1)
            {
                pwm = WS2812_T1H;
            }
            else
            {
                pwm = WS2812_T0H;
            }
            pwm_duty_cycle_values[i++].channel_1 = pwm;
        }
        ptr++;
    }
}

uint32_t drv_ws2812_init(void)
{   
    volatile uint32_t size = sizeof(led_matrix_buffer);
    memset(led_matrix_buffer, 0x00, sizeof(led_matrix_buffer));   
    return pwm_init();
}

uint32_t drv_ws2812_display(void)
{
    if(!pwm_sequencue_finished) 
    {
        return NRF_ERROR_BUSY;
    }
    convert_rgb_to_pwm_sequence();
    pwm_sequencue_finished = false;
    uint32_t err_code = nrfx_pwm_simple_playback(&m_pwm0, &pwm_sequence, 1, NRFX_PWM_FLAG_STOP);
    return err_code;
}

uint32_t drv_ws2812_pixel_draw(uint16_t x, uint16_t y, uint32_t color)
{
    uint32_t err_code = NRF_SUCCESS;
    if(x > LED_MATRIX_WIDTH - 1)
    {
        err_code = NRF_ERROR_INVALID_PARAM;
    }
    if(y > LED_MATRIX_HEIGHT - 1)
    {
	err_code = NRF_ERROR_INVALID_PARAM;
    }
 
#if defined(NEOPIXEL_RING)
    led_matrix_buffer[x][y].w = (color & 0xFF000000) >> 24;
#endif
    led_matrix_buffer[x][y].r = (color & 0x00FF0000) >> 16;
    led_matrix_buffer[x][y].g = (color & 0x0000FF00) >> 8;
    led_matrix_buffer[x][y].b = (color & 0x000000FF);
    
    return err_code;
}

uint32_t drv_ws2812_rectangle_draw(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint32_t color)
{
    uint32_t err_code;
    for(int h = y; h < (y + height); h++)
    {
        for(int w = x; w < (x + width); w++)
        {
            err_code = drv_ws2812_pixel_draw(w, h, color);
            if(err_code) return err_code;
        }
    }
    return NRF_SUCCESS;
}


static float min(float a, float b, float c)
{
  float m;
  
  m = a < b ? a : b;
  return (m < c ? m : c); 
}

static float max(float a, float b, float c)
{
  float m;
  
  m = a > b ? a : b;
  return (m > c ? m : c); 
}
  
void rgb2hsv(uint8_t r, uint8_t g, uint8_t b, float *h, float *s, float *v)
{
  float red, green ,blue;
  float cmax, cmin, delta;
  
  red = (float)r / 255;
  green = (float)g / 255;
  blue = (float)b / 255;
  
  cmax = max(red, green, blue);
  cmin = min(red, green, blue);
  delta = cmax - cmin;

  /* H */
  if(delta == 0)
  {
    *h = 0;
  }
  else
  {
    if(cmax == red)
    {
      if(green >= blue)
      {
        *h = 60 * ((green - blue) / delta);
      }
      else
      {
        *h = 60 * ((green - blue) / delta) + 360;
      }
    }
    else if(cmax == green)
    {
      *h = 60 * ((blue - red) / delta + 2);
    }
    else if(cmax == blue) 
    {
      *h = 60 * ((red - green) / delta + 4);
    }
  }
  
  /* S */
  if(cmax == 0)
  {
    *s = 0;
  }
  else
  {
    *s = delta / cmax;
  }
  
  /* V */
  *v = cmax;
}
  
void hsv2rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    int hi = ((int)h / 60) % 6;
    float f = h * 1.0 / 60 - hi;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1- (1 - f) * s);
    switch (hi){
        case 0:
            *r = 255 * v;
            *g = 255 * t;
            *b = 255 * p;
            break;
        case 1:
            *r = 255 * q;
            *g = 255 * v;
            *b = 255 * p;
            break;
        case 2:
            *r = 255 * p;
            *g = 255 * v;
            *b = 255 * t;
            break;
        case 3:
            *r = 255 * p;
            *g = 255 * q;
            *b = 255 * v;
            break;
        case 4:
            *r = 255 * t;
            *g = 255 * p;
            *b = 255 * v;
            break;
        case 5:
            *r = 255 * v;
            *g = 255 * p;
            *b = 255 * q;
            break;
    }
}


uint32_t changeL(uint32_t rgb, float k) 
{
	uint8_t r, g, b;
	float h, s, v;
	uint8_t cmax, cmin, cdes;
 
	r = (uint8_t) (rgb >> 16);
	g = (uint8_t) (rgb >> 8);
	b = (uint8_t) (rgb);
 
	//os_printf("1:  0x%x \r\n", rgb);
	//os_printf("1:  %d %d %d \r\n", r, g, b);
 
	cmax = r > g ? r : g;
	if (b > cmax)
		cmax = b;
	cmin = r < g ? r : g;
	if (b < cmin)
		cmin = b;
	cdes = cmax - cmin;
 
	v = cmax / 255.0f;
	s = cmax == 0 ? 0 : cdes / (float) cmax;
	h = 0;
 
	if (cmax == r && g >= b)
		h = ((g - b) * 60.0f / cdes) + 0;
	else if (cmax == r && g < b)
		h = ((g - b) * 60.0f / cdes) + 360;
	else if (cmax == g)
		h = ((b - r) * 60.0f / cdes) + 120;
	else
		h = ((r - g) * 60.0f / cdes) + 240;
 
	//////
 
	v *= k;
 
	float f, p, q, t;
	float rf, gf, bf;
	int i = ((int) (h / 60) % 6);
	f = (h / 60) - i;
	p = v * (1 - s);
	q = v * (1 - f * s);
	t = v * (1 - (1 - f) * s);
	switch (i) {
	case 0:
		rf = v;
		gf = t;
		bf = p;
		break;
	case 1:
		rf = q;
		gf = v;
		bf = p;
		break;
	case 2:
		rf = p;
		gf = v;
		bf = t;
		break;
	case 3:
		rf = p;
		gf = q;
		bf = v;
		break;
	case 4:
		rf = t;
		gf = p;
		bf = v;
		break;
	case 5:
		rf = v;
		gf = p;
		bf = q;
		break;
	default:
		break;
	}
 
	r = (uint8_t) (rf * 255.0);
	g = (uint8_t) (gf * 255.0);
	b = (uint8_t) (bf * 255.0);
 
	uint32_t returnColor = ((uint32_t) r << 16) | ((uint32_t) g << 8) | b;
	//os_printf("2:  %d %d %d \r\n", (int) r, (int) g, (int) b);
	//os_printf("2:  0x%x \r\n\r\n", returnColor);
	return returnColor;
}
