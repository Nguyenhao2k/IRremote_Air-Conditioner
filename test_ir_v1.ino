

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK 0
#define DISABLE_CODE_FOR_RECEIVER

/* Includes ---------------------------------------------------------------- */
#include <Arduino.h>
#include <PDM.h>

#include <NguyenHua_model_v7_inferencing.h>

#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>


static IRsend irsend(IR_SEND_PIN);

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

/**
   @brief      Arduino setup function
*/

#define RED D2
#define GREEN D3
#define BLUE D4
#define IR_SEND_PIN D4

void controlLed(uint8_t R, uint8_t G, uint8_t B)
{
  digitalWrite(RED, abs(1 - R));
  digitalWrite(GREEN, abs(1 - G));
  digitalWrite(BLUE, abs(1 - B));
}

void inferLed()
{
  digitalWrite(LEDG, HIGH);
  delay(250);
  digitalWrite(LEDG, LOW);
  delay(250);
  digitalWrite(LEDG, HIGH);
  delay(250);
  digitalWrite(LEDG, LOW);
  delay(250);
}

void setup()
{
  // IR test send signal
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
  delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

#if defined(IR_SEND_PIN)
  IrSender.begin(); // Start with IR_SEND_PIN as send pin and enable feedback LED at default feedback LED pin
  Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));
#else
  IrSender.begin(3, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN); // Specify send pin and enable feedback LED at default feedback LED pin
  Serial.println(F("Send IR signals at pin 3"));
#endif


  // put your setup code here, to run once:
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  Serial.begin(115200);
  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  while (!Serial)
    ;

  Serial.println("Edge Impulse Inferencing Demo");
  pinMode(LED_BUILTIN, OUTPUT);

#if defined(IR_SEND_PIN)
  irSender.begin(); // Start with IR_SEND_PIN as send pin and enable feedback LED at default feedback LED pin
  Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));
#else
  irSender.begin(3, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN); // Specify send pin and enable feedback LED at default feedback LED pin
  Serial.println(F("Send IR signals at pin 3"));
#endif

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

const uint16_t rawDataP[]
#if defined(__AVR__)
PROGMEM
#endif
= { 180, 90 /*Start bit*/, 11, 11, 11, 11, 11, 34, 11, 34/*0011 0xC of 16 bit address LSB first*/, 11, 11, 11, 11, 11, 11, 11,
    11/*0000*/, 11, 34, 11, 34, 11, 11, 11, 34/*1101 0xB*/, 11, 34, 11, 34, 11, 34, 11, 34/*1111*/, 11, 11, 11, 11, 11, 11, 11,
    34/*0001 0x08 of command LSB first*/, 11, 34, 11, 11, 11, 11, 11, 11/*1000 0x01*/, 11, 34, 11, 34, 11, 34, 11,
    11/*1110 Inverted 8 of command*/, 11, 11, 11, 34, 11, 34, 11, 34/*0111 inverted 1 of command*/, 11 /*stop bit*/
  };


//static void sendIR()
//{
//  Serial.println(F("Send NEC 16 bit address 0xFB0C and data 0x18 with (50 us) tick resolution timing (8 bit array format) "));
//  Serial.flush();
//  irsend.sendRaw(rawDataP, sizeof(rawDataP) / sizeof(rawDataP[0]), NEC_KHZ);
//
//  delay(3000); // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal
//}
/**
   @brief      Arduino main function. Runs the inferencing loop.
*/
void loop()
{

  Serial.println(F("Send NEC 16 bit address 0xFB0C and data 0x18 with (50 us) tick resolution timing (8 bit array format) "));
  Serial.flush();
  IrSender.sendRaw_P(rawDataP, sizeof(rawDataP) / sizeof(rawDataP[0]), NEC_KHZ);

  delay(1000); // delay must be greater than 5 ms (RECORD_GAP_MICROS), otherwise the receiver sees it as one long signal

  Serial.println(F("Send NEC 16 bit address 0x0102, 8 bit data 0x34 with generated timing"));
  Serial.flush();
  IrSender.sendNEC(0x0102, 0x34, 0);

  delay(3000);

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

  // code control detect > 85%
  //[0,1,2,3,4,5]: [down, noise, on, stop, unknown, up]: [W-W-Y-R-B-G]
  if (result.classification[0].value > 0.80)
  {
    controlLed(1, 0, 1); // Violet - down
    //    sendIR();
  }
  // else if (result.classification[1].value > 0.85)
  //   controlLed(1, 1, 1); // White - noise
  // else if (result.classification[2].value > 0.85)
  //   controlLed(0, 1, 0); // Green - on
  // else if (result.classification[3].value > 0.85)
  //   controlLed(1, 0, 0); // Red - stop
  // else if (result.classification[4].value > 0.85)
  //   controlLed(1, 1, 0); // Yellow - unknown
  // else if (result.classification[5].value > 0.85)
  //   controlLed(0, 0, 1); // Blue - up
  else
  {
    controlLed(0, 0, 0);
  } // off

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
