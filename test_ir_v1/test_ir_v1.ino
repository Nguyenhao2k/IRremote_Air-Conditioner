// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK 0
#define DISABLE_CODE_FOR_RECEIVER

/* Includes ---------------------------------------------------------------- */
#include <Arduino.h>
#include <PDM.h>

#include "PinDefinitionsAndMore.h"
#include "IRremote.hpp"

#include <NguyenHua_model_v7_inferencing.h>

//static IRsend irsend(IR_SEND_PIN);

const uint16_t rawSignal[] = {
  9000, 4500, // Start bit
  // Data bits (16 bits for NEC protocol)
  560, 560, 560, 560, 560, 560, 560, 560, // "1"
  560, 560, 560, 560, 560, 1690, 560, 1690, // "0"
  560, 560, 560, 560, 560, 560, 560, 560, // "1"
  560, 560, 560, 560, 560, 1690, 560, 1690, // "0"
  // ...
  560, // Stop bit
  0 // Marks the end of the array
};

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

/*static void sendIR();
  static void controlLed(uint8_t R, uint8_t G, uint8_t B);
  static void inferLed();
  static bool microphone_inference_start(uint32_t n_samples);
  static bool microphone_inference_record(void);
  static void microphone_inference_end(void);
  static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
*/


#define RED D2
#define GREEN D3
#define BLUE D5
// #define IR_SEND_PIN 4

/**
  @brief      Control RGB LED
*/
void controlLed(uint8_t R, uint8_t G, uint8_t B)
{
  digitalWrite(RED, abs(1 - R));
  digitalWrite(GREEN, abs(1 - G));
  digitalWrite(BLUE, abs(1 - B));
}

/**
  @brief      LED for inferencing
*/
void inferLed()
{
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDR, HIGH);
  delay(250);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);
  delay(250);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDR, HIGH);
  delay(250);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDR, LOW);
  delay(250);
}

/**
   @brief      Send the raw IR signal
*/
static void sendIR()
{

  IrSender.sendRaw(rawSignal, sizeof(rawSignal) / sizeof(rawSignal[0]), 38);
  Serial.println("Raw IR signal sent!");
  delay(3000); // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
}

/**
   @brief      Arduino setup function
*/
void setup()
{

  // put your setup code here, to run once:
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  Serial.begin(115200);
  IrSender.begin();

  // comment out the below line to cancel the wait for USB connection (needed for native USB)

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
  //  controlLed(0, 0, 0);
  ei_printf("Recording...\n");
  digitalWrite(LEDB, HIGH);

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


  if (result.classification[0].value > 0.85) {
    controlLed(1, 0, 1); // Violet - down
    sendIR();
  }

  // else if (result.classification[1].value > 0.90)
  //   controlLed(1, 1, 1); // White - noise
  // else if (result.classification[2].value > 0.90)
  //   controlLed(0, 1, 0); // Green - on
  // else if (result.classification[3].value > 0.90)
  //   controlLed(1, 0, 0); // Red - stop
  // else if (result.classification[4].value > 0.90)
  //   controlLed(1, 1, 1); // Yellow - unknown
  // else if (result.classification[5].value > 0.90)
  //   controlLed(0, 0, 1); // Blue - up
  else {
    controlLed(0, 0, 0);
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
