// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK 0
#define DISABLE_CODE_FOR_RECEIVER

/* Includes ---------------------------------------------------------------- */
#include <Arduino.h>
#include <PDM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define IR_SEND_PIN 4
#include <NguyenHua_94.3per_inferencing.h>

#include <IRremote.hpp>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define FRAME_WIDTH (16)
#define FRAME_HEIGHT (16)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const unsigned char snowflake[] PROGMEM = {
    0x01, 0x00, 0x03, 0x80, 0x03, 0x80, 0x73, 0x9c, 0x77, 0xdc, 0x7c, 0x7c, 0x18, 0x30, 0x08, 0x20,
    0x18, 0x30, 0x7c, 0x7c, 0x77, 0xdc, 0x73, 0x9c, 0x03, 0x80, 0x03, 0x80, 0x01, 0x00};

const unsigned char conditioner[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0xc0, 0x03, 0xc0, 0x03,
    0xcf, 0xf3, 0x5f, 0xfa, 0x7f, 0xfe, 0x00, 0x00, 0x08, 0x10, 0x12, 0x48, 0x02, 0x40, 0x00, 0x00};

static unsigned int temp = 25;                                                                // Set the default temperatur is 25 degree
static unsigned int current_state = 0, pre_state = 0;                                                                // Set the state ON/OFF of remote
static uint32_t irCommand[] = {0x4FC, 0x6FC, 0xEFC, 0xAF, 0x2FC, 0x3FC, 0xBFC, 0x9FC, 0x1FC}; // Hex IR data
static unsigned int time1 = 0, time2 = 0;

/** Audio buffers, pointers and selectors */
typedef struct
{
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// static void sendIR(uint16_t rawData, int sizeRaw, int frePro);
// static void controlLed(uint8_t R, uint8_t G, uint8_t B);
// static void inferLed();

#define RED 5
#define GREEN 6
#define BLUE 7

/**
  @brief      Control RGB LED
*/
void controlLed(uint8_t R, uint8_t G, uint8_t B)
{
  digitalWrite(RED, R);
  digitalWrite(GREEN, G);
  digitalWrite(BLUE, B);
}

/**
  @brief      LED for inferencing
*/
void inferLed()
{
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
  delay(250);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, LOW);
  delay(250);
  digitalWrite(GREEN, HIGH);
  digitalWrite(RED, HIGH);
  digitalWrite(BLUE, HIGH);
  delay(250);
  digitalWrite(GREEN, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(BLUE, LOW);
  delay(250);
}

/**
   @brief      Send the raw IR signal
*/
void sendIR(int current_state, int pre_state, int temp)
{
  if ((current_state == 1) && (pre_state == 0)) // Turn ON at 25 degree
  {
    if (temp >= 28)
    {
      display.clearDisplay();
      display.setTextSize(5);
      display.setCursor(20, 0);
      display.println("MAX");
      display.display();
      delay(400);

      acDisplay(temp);
    }
    else if (temp <= 20) 
    {
      display.clearDisplay();
      display.setTextSize(5);
      display.setCursor(20, 0);
      display.println("MIN");
      display.display();
      delay(400);

      acDisplay(temp);
    }
    else {
      acDisplay(temp);
    }
    IrSender.sendSamsung48(0x24D, irCommand[temp - 20], 2); // The first index of array is the temperatur at 20 degree
    delay(2000);
  }
  else if ((current_state == 1) && (pre_state == 1))  // state = 0, Turn OFF screen
  {
    return;
  } 
  else if ((current_state == 0) && (pre_state == 1)) {
    IrSender.sendSamsung48(0x24D, 0x7DE, 2);
    delay(2000);
  }
  else {
    return;
  }
}

/**
   @brief      Arduino setup function
*/
void setup()
{

  // put your setup code here, to run once:
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  display.display();

  Serial.begin(115200);
  IrSender.begin();

  Serial.println("Edge Impulse Inferencing Demo");

  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false)
  {
    ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return;
  }
}

/**
   @brief      Arduino main function. Runs the inferencing loop.
*/
void loop()
{
  ei_printf("Starting inferencing in 1 seconds...\n");
  inferLed();

  ei_printf("Recording...\n");

  bool m = microphone_inference_record();
  if (!m)
  {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }

  ei_printf("Recording done\n");

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK)
  {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  /**
   * @brief Test send raw IR
   *
   */
  // Turn ON the air conditioner
  if (result.classification[2].value > 0.85)
  {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 0);
    display.println("DSOFT");
    display.display();
    delay(400);

    acDisplay(temp);
    current_state = 1;
    controlLed(1, 0, 1); // Green - ON
    Serial.print(temp);
    sendIR(current_state, pre_state,  temp);
    pre_state = 1;
  }

  // Turn OFF the air conditioner
  if (result.classification[3].value > 0.90)
  {
    current_state = 0;
    display.clearDisplay();
    display.setTextSize(5);
    display.setCursor(20, 0);
    display.println("OFF");
    display.display();
    time2 = millis();

    controlLed(0, 1, 1); // Red - stop
    sendIR(current_state, pre_state, temp);
    Serial.println("Turn OFF air conditioner");
    pre_state = 0;
  }

  // Noise or Unknown
  if ((result.classification[1].value > 0.90) || (result.classification[4].value > 0.90))
  {
    controlLed(1, 1, 1);
  }

  // Turn down the temperature
  if (result.classification[0].value > 0.90)
  {

    controlLed(0, 1, 0); // Violet - down
    if (current_state == 1)
    {
      temp--;
      if (temp <= 20)
        temp = 20;
    }
    sendIR(current_state, 0, temp);
    Serial.println("Current temp is: ");
    Serial.print(temp);
  }

  // Turn up the temperature
  if (result.classification[5].value > 0.90)
  {
    // Blue - up
    controlLed(1, 1, 0);
    if (current_state == 1)
    {
      temp++;
      if (temp >= 28)
        temp = 28;
    }
    sendIR(current_state, 0, temp);
    Serial.println("Current temp is: ");
    Serial.print(temp);
  }

  if ((time2 - time1) >= 2000)
  {
    Serial.print(time2 - time1);
    display.clearDisplay();
    display.display();
    time2 = 0;
  }
  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
  }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

/**
 * @brief   snowDisplay function
 *
 * @param temp
 */
void snowDisplay(int temp)
{
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(20, 0);
  display.println(temp);
  display.drawCircle(81, 4, 4.5, SSD1306_WHITE); // Vẽ biểu tượng độ C

  display.setTextSize(5);
  display.setCursor(87, 0);
  display.println("C");

  display.setTextSize(1.5);
  display.setCursor(0, 45);
  display.println("Fan : High");
  display.setCursor(0, 55);
  display.println("Mode: Cool");
  display.drawBitmap(107, 45, snowflake, 15, 15, 1);
  display.display();
}

/**
 * @brief acDisplay function
 *
 * @param temp
 */
void acDisplay(int temp)
{
  display.clearDisplay();
  display.setTextSize(5);
  display.setCursor(20, 0);
  display.println(temp);
  display.drawCircle(81, 4, 4.5, SSD1306_WHITE); // Vẽ biểu tượng độ C

  display.setTextSize(5);
  display.setCursor(87, 0);
  display.println("C");

  display.setTextSize(1.5);
  display.setCursor(0, 45);
  display.println("Fan : High");
  display.setCursor(0, 55);
  display.println("Mode: Cool");
  display.drawBitmap(107, 45, conditioner, 15, 15, 1);
  display.display();
}

/**
   @brief      PDM buffer full callback
               Get data and call audio thread callback
*/
static void pdm_data_ready_inference_callback(void)
{
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

  if (inference.buf_ready == 0)
  {
    for (int i = 0; i < bytesRead >> 1; i++)
    {
      inference.buffer[inference.buf_count++] = sampleBuffer[i];

      if (inference.buf_count >= inference.n_samples)
      {
        inference.buf_count = 0;
        inference.buf_ready = 1;
        break;
      }
    }
  }
}

/**
   @brief      Init inferencing struct and setup/start PDM

   @param[in]  n_samples  The n samples

   @return     { description_of_the_return_value }
*/
static bool microphone_inference_start(uint32_t n_samples)
{
  inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

  if (inference.buffer == NULL)
  {
    return false;
  }

  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  // configure the data receive callback
  PDM.onReceive(&pdm_data_ready_inference_callback);

  PDM.setBufferSize(4096);

  // initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate
  if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY))
  {
    ei_printf("Failed to start PDM!");
    microphone_inference_end();

    return false;
  }

  // set the gain, defaults to 20
  PDM.setGain(127);

  return true;
}

/**
   @brief      Wait on new data

   @return     True when finished
*/
static bool microphone_inference_record(void)
{
  inference.buf_ready = 0;
  inference.buf_count = 0;

  while (inference.buf_ready == 0)
  {
    delay(10);
  }

  return true;
}

/**
   Get raw audio signal data
*/
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

  return 0;
}

/**
   @brief      Stop PDM and release buffers
*/
static void microphone_inference_end(void)
{
  PDM.end();
  free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
