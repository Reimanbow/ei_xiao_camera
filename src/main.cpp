#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include <SPI.h>
#include <SD.h>
#include "FS.h"

#include <esp_camera.h>
#include <U8x8lib.h>

#include "pins.h"

#include <xiao_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

bool ei_image_init(void);
bool ei_image_capture();

const int buttonPin = D1;
int buttonState = 0;

uint16_t imageCount = 0;
char filename[32];
bool isPressed = false;

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,       // QQVGA-UXGA Do not use sizes above QVGA when not JPEG
    .jpeg_quality = 10,

    // 0-63 lower number means higher quality
    .fb_count = 1, // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/SCL, /* data=*/SDA, /* reset=*/U8X8_PIN_NONE);

bool writeImageFile(fs::FS &fs, const char *path, uint8_t *data, size_t len)
{
  u8x8.printf("Writing file: %s\n", path);
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    u8x8.println("Failed to open file for writing");
    Serial.println("Failed to open file for writing");
    return false;
  }
  if (file.write(data, len) == len)
  {
    u8x8.println("File written");
    Serial.println("File written");
  }
  else
  {
    u8x8.println("Write failed");
    Serial.println("Write failed");
    return false;
  }
  file.close();

  imageCount++;
  sprintf(filename, "/Image_%04d.jpg", imageCount);
  return true;
}

bool ei_image_init(void)
{
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
  {
    u8x8.printf("Camera init failed with error 0x%x\n", err);
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  return true;
}

bool ei_image_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
  u8x8.clear();
  u8x8.setCursor(0, 0);

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb)
  {
    u8x8.println("Camera capture failed\n");
    Serial.println("Camera capture failed\n");
    return false;
  }

  writeImageFile(SD, filename, fb->buf, fb->len);

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted){
    u8x8.println("Conversion failed");
    Serial.println("Conversion failed");
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)){
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height
    );
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr){
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0){
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }

  return 0;
}

void refreshIndex(void)
{
  while (true)
  {
    sprintf(filename, "/Image_%04d.jpg", imageCount);
    if (SD.exists(filename))
    {
      imageCount++;
    }
    else
    {
      break;
    }
  }
}

void setup()
{
  u8x8.setI2CAddress(0x78);
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  Wire.begin();

  ei_image_init();

  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(9600);
  pinMode(D2, OUTPUT);
  
  u8x8.clear();
  u8x8.setCursor(0, 0);
  
  if (!SD.begin(D2))
  {
    u8x8.println("initialization \n failed!");
    Serial.println("initialization \n failed!");
    return;
  }
  refreshIndex();
  u8x8.println("initialization \n done.");
  Serial.println("initialization \n done.");
  delay(2000);
}

void loop()
{
  buttonState = digitalRead(buttonPin);

  if(buttonState == LOW){
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr){
      u8x8.println("ERR: Failed to allocate snapshot buffer!\n");
      return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_image_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false){
      u8x8.println("Failed to capture image");
      free(snapshot_buf);
      return;
    }

    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK){
      u8x8.printf("ERR: Failed to run classifier (%d)\n", err);
      return;
    }

    Serial.printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);
    
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("  %s: ", ei_classifier_inferencing_categories[i]);
        Serial.printf("%.5f\r\n", result.classification[i].value);
        u8x8.printf("  %s: ", ei_classifier_inferencing_categories[i]);
        u8x8.printf("%.5f\r\n", result.classification[i].value);
    }
    free(snapshot_buf);
  }
}
