#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2S mic
#define I2S_WS   5
#define I2S_SD   6
#define I2S_SCK  4

#define SAMPLES 64
#define SAMPLING_FREQ 16000

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

void setup() {
  Serial.begin(115200);

  Wire.begin(11, 12);  // SDA, SCL

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
    while (1);
  }

  display.clearDisplay();

  // I2S setup
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = SAMPLES,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void loop() {
  int32_t buffer[SAMPLES];
  size_t bytesRead;

  i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);

  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = buffer[i] >> 16;
    vImag[i] = 0;
  }

  // FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  display.clearDisplay();

  int barWidth = 4;
  int spacing = 2;

  //Frequency bars
  for (int i = 2; i < 22; i++) {
    int height = constrain(vReal[i] / 100, 0, 60);

    int x = (i - 2) * (barWidth + spacing);
    int y = SCREEN_HEIGHT - height;

    display.fillRect(x, y, barWidth, height, SSD1306_WHITE);
  }

  display.display();
}
