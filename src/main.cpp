#include <Arduino.h>
#include <driver/i2s.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define I2S_WS 17
#define I2S_SD 3
#define I2S_SCK 18
#define I2S_PORT_0 I2S_NUM_0
#define I2S_PORT_1 I2S_NUM_1
#define MAX98357_LRC 40
#define MAX98357_BCLK 41
#define MAX98357_DIN 42
#define SAMPLE_RATE 16000
#define SPEAK_BTN 19

WiFiClient wifi;
PubSubClient client(wifi);

int16_t audioData[100];

void i2sInit()
{
  const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = i2s_bits_per_sample_t(16),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false};
  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT_0, &i2s_config, 0, NULL));
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_SD};

  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT_0, &pin_config));
  ESP_ERROR_CHECK(i2s_start(I2S_PORT_0));

  i2s_config_t i2sOut_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = i2s_bits_per_sample_t(16),
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 64};
  ESP_ERROR_CHECK(i2s_driver_install(I2S_PORT_1, &i2sOut_config, 0, NULL));

  const i2s_pin_config_t i2sOut_pin_config = {
      .bck_io_num = MAX98357_BCLK,
      .ws_io_num = MAX98357_LRC,
      .data_out_num = MAX98357_DIN,
      .data_in_num = -1};
  ESP_ERROR_CHECK(i2s_set_pin(I2S_PORT_1, &i2sOut_pin_config));
  Serial.println("i2s init success");
}

void wifiInit()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin("wink", "Mishiweilai123");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wifi connected");
}

void mqttInit()
{
  client.setServer("10.0.0.216", 1883);
  client.connect("esp32");
  client.setKeepAlive(20);
  while (!client.connected())
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("mqtt connected");
}
void setup()
{
  Serial.begin(115200);
  pinMode(SPEAK_BTN, INPUT_PULLUP);
  i2sInit();
  wifiInit();
  mqttInit();
}

void loop()
{
  if (!digitalRead(SPEAK_BTN))
  {
    delay(20);
    if (!digitalRead(SPEAK_BTN))
    {
      Serial.println("Recording...");
      size_t bytesRead = 0,
             bytes_written = 0;
      client.publish("data", "start");
      while (!digitalRead(SPEAK_BTN))
      {
        bytesRead = 0;
        bytes_written = 0;
        esp_err_t result = i2s_read(I2S_PORT_0, audioData, sizeof(audioData), &bytesRead, portMAX_DELAY);
        i2s_write(I2S_PORT_1, audioData, sizeof(audioData), &bytes_written, portMAX_DELAY);
        bool res = client.publish("data", (uint8_t *)audioData, bytesRead);
      }
      i2s_zero_dma_buffer(I2S_PORT_1); // 清空I2S DMA缓冲区
      client.publish("data", "completed");
      Serial.println("Recording completed.");
    }
  }
  if (!client.connected())
  {
    client.connect("esp32");
    sleep(1000);
  }
  client.loop();
}
