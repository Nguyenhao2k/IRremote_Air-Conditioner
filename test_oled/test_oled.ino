#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// #define FRAME_DELAY (42)
#define FRAME_WIDTH (16)
#define FRAME_HEIGHT (16)
// #define FRAME_COUNT (sizeof(frames) / sizeof(frames[0]))
const unsigned char snowflake [] PROGMEM = {
  0x03, 0x00, 0x07, 0x80, 0x07, 0x80, 0x23, 0x10, 0xe3, 0x1c, 0x73, 0x38, 0xdf, 0xec, 0x07, 0x80,
  0x07, 0x80, 0xff, 0xec, 0x73, 0x38, 0xe3, 0x1c, 0x23, 0x10, 0x07, 0x80, 0x07, 0x80, 0x03, 0x00
};

const unsigned char conditioner [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0xc0, 0x03, 0xc0, 0x03, 
  0xcf, 0xf3, 0x5f, 0xfa, 0x7f, 0xfe, 0x00, 0x00, 0x08, 0x10, 0x12, 0x48, 0x02, 0x40, 0x00, 0x00
};


int frame = 0;
void setup() {
  // Khởi tạo màn hình OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();  // Hiển thị màn hình ban đầu

  // Vẽ các điều khiển của remote điều hòa
  // display.setTextSize(1);
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);

  //  display.setCursor(0, 0);
  //  display.println("MAY DIEU HOA");

  display.setTextSize(5);
  display.setCursor(20, 0);
  display.println("22");
  display.drawCircle(81, 4, 4.5, SSD1306_WHITE); // Vẽ biểu tượng độ C

  display.setTextSize(5);
  display.setCursor(87, 0);
  display.println("C");

  display.setTextSize(1.5);
  display.setCursor(0, 45);
  display.println("Fan : High");
  display.setCursor(0, 55);
  display.println("Mode: Cool");


  // display.drawBitmap(75, 8, snowflake, FRAME_WIDTH, FRAME_HEIGHT, 1);
  //display.drawBitmap(107, 45, snowflake, FRAME_WIDTH, FRAME_HEIGHT, 1);
  display.drawBitmap(107, 45, conditioner, FRAME_WIDTH, FRAME_HEIGHT, 1);
  display.display();

}

void loop() 
{

}