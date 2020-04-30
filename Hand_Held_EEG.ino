/*
	FFT analysis of EEG electrical data

  This program takes voltage readings from a BITalino EEG sensor on analog pin 5 
  and runs an FFT analysis which transfers the code from a voltage time domain
  to a frequency magnitude domain. This allows for the data to be analyzed 
  to monitor brain activity. 
  
  The program requires an arduino uno (or mega). An arduino mega is reccomended 
  due to the pin useage of the TFT touchshield. 
  
	Circuit:
	2.8" 320x240 Adafruit TFT Touchshield - ILI9341 (https://www.adafruit.com/product/1770)
	This is used for both touchscreen inputs and the graphical outputs.
	Inputs:
	3 Pre-gelled electrods (https://www.3m.com/3M/en_US/company-us/all-3m-products/~/3M-Red-Dot-Monitoring-Electrode-with-Foam-Tape-and-Sticky-Gel/?N=5002385+3293316191&rt=rud)
	Bitalino EEG microprocessor (https://bitalino.com/en/plugged-kit-bt) 
	
	Libraries:
	Arduino FFT: https://github.com/kosme/arduinoFFT
	SPI: https://github.com/PaulStoffregen/SPI
	Wire: https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h
	Adafruit ILI9341: https://github.com/adafruit/Adafruit_ILI9341
	Adafruit STMPE610: https://github.com/adafruit/Adafruit_STMPE610
	Adafruit GFX: https://github.com/adafruit/Adafruit-GFX-Library/archive/master.zip
	
	

  Graph Function: https://github.com/KrisKasprzak/GraphingFunction
  
	Date created: 4/24/20
	By: Thomas Pond & Ivan Oon 
	Modified: 4/29/20
	By: Thomas Pond 

*/
#include <arduinoFFT.h>                 //FFT library
#include <Adafruit_GFX.h>               //Core graphics library
#include <SPI.h>
#include <Wire.h>                       //Library isn't used, however the code doesn't work without it.
#include <Adafruit_ILI9341.h>           //Library for the graphic interface on the TFT touchshield 
#include <Adafruit_STMPE610.h>          //Library used for the hardware interface with the TFT touchshield

#define BLUE          0x001F
#define GREEN         0x07E0
#define CYAN          0x07FF
#define RED           0xF800
#define YELLOW        0xFFE0
#define ORANGE        0xFD20
#define PINK          0xF81F
#define WHITE         0xFFFF
#define BLACK         0x0000
#define DKBLUE        0x000D

// Calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000
double ox, oy ;                  //Previous spot placeholder for the graphic program
boolean display1 = true;         //This boolean statement determines if the graph is visible or not
int x = 0, y = 0;                //This initializes the x and y coordinante system
int EEG_val;                     //This initializes the analog data that will be collected

//Below is the data used to initialize the FFT function used below

#define SAMPLES 128              //Used for the FFT, must be a power of 2
#define SAMPLING_FREQUENCY 120   //unit is Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];


// The STMPE610 is initialized on pin #8
#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);

// This will initialize the pins for the display on pins #9 and #10
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);


//Button state
boolean read_state = true; //This initializes the switch state as on which is used later for the button press indication
int EEG_pin = A5; //Analog input of the EEG


void setup() {
  // Initializes start of TFT
  tft.begin();
  Serial.begin(115200);
  // Start our touchscreen
  if (!ts.begin()) {
    while (1);
  }
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  // Initialize EEG pin as an input
  pinMode(EEG_pin, INPUT);

  tft.fillScreen(ILI9341_BLACK);

}


void loop() {





  // Initial check to see if there is any touch indicaiton.
  if (ts.bufferEmpty()) {
    return;
  }

  // Retrieve a point based on touch.
  TS_Point p = ts.getPoint();

  // This maps the touch screen scale based on the callibration.
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());


  //This if statement looks for a change in pressure anywhere on the screen.
  if ((p.x > 0 && p.x < 230) && (p.y > 0 && p.y < 320)) {
    read_state = ! read_state; //This uses a change of boolean logic to indicate an activation of the switch
    delay(300);                //This delay is used to adjust the sensitivity of the touchscreen.
  }
  if (read_state) {
    display1 = true;
    tft.fillScreen(ILI9341_BLACK);
    for (int i = 0; i < SAMPLES; i++)
    {
      microseconds = micros();             //Used to protect from overflow.
      EEG_val = analogRead(EEG_pin);       //This is the analog read of the EEG data from the analog pin.
      vReal[i] = EEG_val;
      vImag[i] = 0;

      while (micros() < (microseconds + sampling_period_us)) {
      }
    }
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    for (int i = 0; i < (SAMPLES / 2); i++)
    {

      //Uncomment out these two serial print commands in order to export the data to excel.
      //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
      //Serial.print(" ");
      x = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES; //Frequency read from FFT
      y = vReal[i];                                 //Magnitude read from FFT
      Serial.println(vReal[i], 1);    //This line is used to show the FFT chart in the serial plotter.

      //The graph function below is provided by Kris Kasprzak. His Github tutorial is linked in the title.
      Graph(tft, x, y, 40, 200, 135, 135, 0, 64, 10, 0, 3000, 300, "EEG FFT output", "", "Magnitude", DKBLUE, RED, YELLOW, WHITE, BLACK, display1);

      delay(100);  //repeats the process every 100 milliseconds.
    }



  }

  else {
    display1 = false;    //This stops the display of data on the graph.
    //Below is used to set the screen to white and indicate a command for the user.
    tft.fillScreen(ILI9341_WHITE);
    tft.setCursor(50, 120);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(1);
    tft.print("Tap screen to start EEG");

  }

}

//This graph function below is provided by Kris Kasprzak.
//There were minor changes based on the scaling of the FFT analysis.
//Link to the github for more information: https://github.com/KrisKasprzak/GraphingFunction



void Graph(Adafruit_ILI9341 &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, String title, String xlabel, String ylabel, unsigned int gcolor, unsigned int acolor, unsigned int pcolor, unsigned int tcolor, unsigned int bcolor, boolean &redraw) {

  double ydiv, xdiv;
  // initialize old x and old y in order to draw the first point of the graph
  // but save the transformed value
  // note my transform funcition is the same as the map function, except the map uses long and we need doubles
  //static double ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
  //static double oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  double i;
  double temp;
  int rot, newrot;

  if (redraw == true) {

    redraw = false;
    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    // draw y scale
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;

      if (i == 0) {
        d.drawLine(gx, temp, gx + w, temp, acolor);
      }
      else {
        d.drawLine(gx, temp, gx + w, temp, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(gx - 40, temp);
    }
    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {

      // compute the transform

      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        d.drawLine(temp, gy, temp, gy - h, acolor);
      }
      else {
        d.drawLine(temp, gy, temp, gy - h, gcolor);
      }

      d.setTextSize(1);
      d.setTextColor(tcolor, bcolor);
      d.setCursor(temp, gy + 10);
    }

    //Draws the labels
    d.setTextSize(2);
    d.setTextColor(tcolor, bcolor);
    d.setCursor(gx, gy - h - 30);
    d.println(title);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx, gy + 20);
    d.println(xlabel);

    d.setTextSize(1);
    d.setTextColor(acolor, bcolor);
    d.setCursor(gx - 30, gy - h - 10);
    d.println(ylabel);


  }

  //graph drawn now plot the data
  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox, oy, x, y, pcolor);
  d.drawLine(ox, oy + 1, x, y + 1, pcolor);
  d.drawLine(ox, oy - 1, x, y - 1, pcolor);
  ox = x;
  oy = y;

}

