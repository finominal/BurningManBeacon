//#include <SPI.h>
//#include <Adafruit_WS2801.h>

#include <Adafruit_NeoPixel.h>


#include <BeaconEntity.h>

#define DATAPIN 6
#define p(x) Serial.print(x) 
#define pl(x) Serial.println(x) 

//control
const int totalPixels = 111;
const int numPixels = 111;

//Adafruit_WS2801 strip = Adafruit_WS2801(numPixels, 6, 7);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numPixels, 6, NEO_GRB + NEO_KHZ800);

volatile uint32_t pixelBuffer[numPixels];

//leds
volatile uint16_t brightness = 255;
volatile uint32_t red = Color(brightness,0,0);
volatile uint32_t green = Color(0,brightness,0);
volatile uint32_t blue = Color(0,0,brightness);
volatile uint32_t black = Color(0,0,0);

//physics
volatile float gravity = -9.8;
volatile float stickHeight = 2500;
volatile int bottom = 0;

//objects
struct ball {
  volatile uint32_t color;
  volatile float loc; //location in mm
  volatile float lastLoc; 
  volatile int lastLocCounter;
  volatile int pos;
  volatile float velocity;
  volatile bool bouncing; //bouncing, or moving up?
  volatile int wait;
  volatile int waiting;//current pause, count frames
  volatile float friction;
  
  //millis returns an unsigned long 
  volatile unsigned long lastFrameTime;
  volatile unsigned long frameTime;
};

 ball balls[3];
 Entity ENTITIES[3];
//InitializeEntities();


//blobs
const int countEntities = 3;//dont change this
const int entitySize = 16;
const int entitiesDelay = 45; //time in MS for a half step

//animation types
enum program {bouncingBalls,threeEntities, rainbowCycle};
program CURRENT_PROGRAM = bouncingBalls;


//Rotary Encoder
// usually the rotary encoders three pins have the ground pin in the middle

volatile const int  encoderPinA = 2;   // right
volatile const int  encoderPinB = 3;   // left
volatile const int  encoderButton = 4;    // depress encoder 

static boolean rotating=false;      //rotary debounce management

// interrupt service routine vars
volatile boolean A_set = false;              
volatile boolean B_set = false;


void setup()
{
  Serial.begin(115200);
  Serial.print("Start");
  strip.begin(); 
  strip.show();
  
  InitializeRotaryEncoder();
}

void loop()
{
  switch(CURRENT_PROGRAM)
  {
    case bouncingBalls:
      BouncingBalls();
      break;
    case threeEntities:
      ThreeEntities();
      break;
    case rainbowCycle:
      RainbowCycle(10);
      break;
  }
}

void CheckForButtonPress()
{
  if(digitalRead(encoderButton) == LOW) 
  {
    delay(1);
    
    if(digitalRead(encoderButton) == LOW) //if low, we have debounced
    {
      switch(CURRENT_PROGRAM)
      {
        case bouncingBalls:
          CURRENT_PROGRAM = threeEntities;
          break;
        case rainbowCycle:
          CURRENT_PROGRAM = bouncingBalls;
          
          break;
        case threeEntities:
          //CURRENT_PROGRAM = rainbowCycle;
          CURRENT_PROGRAM = bouncingBalls;
          break;

      }
      
      while(digitalRead(encoderButton) == LOW)//wait for button release
      {
        pl("wait"); 
        delay(25);
        digitalWrite(13,HIGH);
        delay(25);
        digitalWrite(13,LOW);
      }//wait for release
      pl("BtnRelease");
      
      ClearBuffer();
      WriteBufferToStrip();
    }
  }
}

void ClearBuffer()
{
  for(int i = 0; i<numPixels; i++) pixelBuffer[i] = 0;
}

void WriteBufferToStrip()
{
  for(int i = 0; i<numPixels; i++)
  {
    strip.setPixelColor(i,pixelBuffer[i]);
  }
  
  /*
  //mirror on the back leds, in reverse order 0=75, 1=76
  for(int i = numPixels; i<totalPixels; i++) 
  {
    strip.setPixelColor(i,pixelBuffer[totalPixels-1-i]);
  }
  */
  strip.show();
  /*
  //disable interrupts while pushing to strip. cant do it for balls as it messes up the timing.
  if(CURRENT_PROGRAM == bouncingBalls) {strip.show();}
  else
  {
    cli();
    strip.show();
    sei();
  }
  */
}


void BouncingBalls()
{
  InitializeBalls();
  pl("BouncingBalls");
  
  while(CURRENT_PROGRAM == bouncingBalls)
  { 
    if(rotating == false)
    {
      rotating = true;
      SetColorBrightness();
    }

    
    for(int i = 0; i<3; i++)//for each ball
    { 
      if(balls[i].bouncing)
      {
        if(balls[i].waiting > 0) 
        {
          balls[i].waiting--;
          pixelBuffer[balls[i].pos] |=  balls[i].color;
          
//DEV show positions  
          //p("b");p(i);p(" P="); pl(balls[i].pos);
        
          balls[i].lastFrameTime = millis();//reset compare last so we can start a new bounce  
        }
        else
        {
          balls[i].frameTime = millis();

          //Drop science, yo.
          balls[i].velocity += ((gravity * (balls[i].frameTime - balls[i].lastFrameTime))/1000);//apply gravity to the objects velocity
          balls[i].loc = balls[i].loc + balls[i].velocity * (numPixels/(stickHeight/1000)); //move location based on current velocity
   
          //check if at/passed the bottom, and reverse direction
          if(balls[i].loc < 0)
          {
            balls[i].loc = 0;//hit the bottom
            balls[i].velocity *= balls[i].friction;//impact friction
          }
          balls[i].pos = map(balls[i].loc,1,stickHeight,bottom,numPixels-1);//normalize the position
          
 //DEV show positions  
          //p("b");p(i);p(" P="); pl(balls[i].pos);
          
          //update strip by combining colors
          pixelBuffer[balls[i].pos] |=  balls[i].color;
          //check if at the bottom itterator
          
          //track if ball is at rest
          if(balls[i].loc == balls[i].lastLoc) {balls[i].lastLocCounter++; } else {balls[i].lastLocCounter = 0;}
          
          //update this location for use in the next frame
          balls[i].lastLoc = balls[i].loc;
          
          //if ball has been at the bottom for 100 frames, change the bouncing state to start rise. 
          if(balls[i].lastLocCounter>10) 
          { 
            balls[i].bouncing = false; //reset 
            balls[i].waiting = 80;//wait 80 frames, about 1 second.
          }
          
          //record the frame time, used in next itteration to determin velocity. 
          balls[i].lastFrameTime = balls[i].frameTime;
        }
      }
      else //not bouncing anymore, move up and pause 
      {
        if(balls[i].waiting > 0) 
        {
          balls[i].waiting--;
          //set the ball to be at the bottom position while waiting
          pixelBuffer[balls[i].pos] |=  balls[i].color;
        }
        else
        {
           if(balls[i].pos >= numPixels-1) //are we at the top yet?
           {
             balls[i].pos = numPixels-1; //in case we go one over
             balls[i].bouncing = true;
             balls[i].waiting = random(200, 1900);//how long (frames) to wait at the top of the stick for
             balls[i].friction = random(83, 98);
             balls[i].friction /= -100; //random returns positive int, we require negative float
             //Serial.print(i);Serial.print("f="); Serial.println(balls[i].friction);
             balls[i].loc = stickHeight;//reset location as we forced up via transalted position (pos)
             balls[i].velocity = 0;
           }
           else//not at the top, move ball up
           {
             balls[i].pos++;
             pixelBuffer[balls[i].pos] |=  balls[i].color;
             balls[i].waiting = balls[i].wait;
           }
        }
      }
    }
  
    //show what has been written to the strip.
    WriteBufferToStrip();
  
    //after pushing colors to the strip, clear all the balls from the strip ready to draw the next frame
    for(int i = 0; i<3; i++) pixelBuffer[balls[i].pos] = black;
      
    CheckForButtonPress();
  
  }//while(CURRENT_PROGRAM = bouncingBalls)
}

void InitializeBalls()
{
  pl("InitializeBalls");
  ClearLedBuffer();
  
  //delay(50);//helps with frame time calculations
  
  ball redBall =   {red,stickHeight,0,numPixels-1,numPixels-1,0,true,6,0  ,-0.93,millis()-20,millis()};
  //delay(50);  
  ball greenBall = {green,stickHeight,0,numPixels-1,numPixels-1,0,true,6,1500,-0.94,millis()-20,millis()};
  //delay(50); 
  ball blueBall =  {blue,stickHeight,0,numPixels-1,numPixels-1,0,true,6,2000,-0.95,millis()-20,millis()};
  //delay(50);
  
  balls[0] = redBall;
  balls[1] = greenBall;
  balls[2] = blueBall;
//  delay(50);
  PrintBallValues();

}

void SetColorBrightness()
{
  red = Color(brightness,0,0);
  green = Color(0,brightness,0);
  blue = Color(0,0,brightness);
  
  balls[0].color = red;
  balls[1].color = green;
  balls[2].color = blue;

}

void PrintBallValues()
{
  pl("BallValues:");
  for(int i = 0; i<3;i++)
  {
    p("B");pl(i);
    p("Color="); pl(balls[i].color);
    p("loc="); pl(balls[i].loc);
    p("lastLoc="); pl(balls[i].lastLoc);
    p("lastLocCounter="); pl(balls[i].lastLocCounter);
    p("pos="); pl(balls[i].pos);
    p("velocity="); pl(balls[i].velocity);
    p("bouncing="); pl(balls[i].bouncing);
    p("wait="); pl(balls[i].wait);
    p("waiting="); pl(balls[i].waiting);
    p("friction="); pl(balls[i].friction);
    p("lastFrameTime="); pl(balls[i].lastFrameTime);
    p("frameTime="); pl(balls[i].frameTime);
    pl();
  }
}


void InitializeEntities()
{  
  pl("Initialize Entities");
  ClearLedBuffer();
  delay(50);
  SetBlobBrightness();
  delay(50);
  pl("i");
  ENTITIES[0].direction = up;  
  ENTITIES[0].position = 0;
  
  ENTITIES[1].direction = up;  
  ENTITIES[1].position = 0 - (numPixels ) - entitySize;
  
  ENTITIES[2].direction = up;  
  ENTITIES[2].position = 0 - (numPixels * 2) - entitySize -10;  
  
  pl("iOK");
}

void SetBlobBrightness()
{

  p("SetBlobBrightness=");pl(brightness);
  ENTITIES[0].color = Color(brightness,0,0); //red
  ENTITIES[0].colorHalf = Color(brightness/3,0,0); //red
  ENTITIES[2].color = Color(0,0,brightness); //blue
  ENTITIES[2].colorHalf = Color(0,0,brightness/3); //blue
  ENTITIES[1].color = Color(0,brightness,0); //green
  ENTITIES[1].colorHalf = Color(0,brightness/3,0); //green
    //pl("bOK");
    delay(10);

}

void ThreeEntities()
{
  pl("ThreeEntities");
  InitializeEntities();
  while(CURRENT_PROGRAM == threeEntities)
  { 
    if(rotating == false)
    {
      rotating = true;
      SetBlobBrightness();
    }
    DisplayEntities();
    MoveEntities();
    delay(entitiesDelay);
    
    CheckForButtonPress();
  }
}


void DisplayEntities()
{
  //pl("DE");
  ClearLedBuffer();
  RenderEntitiesToLEDBuffer();
  WriteBufferToStrip();
  //ShowEntityPositions();
}

void ClearLedBuffer()
{
  for(int i = 0;i<numPixels;i++)
  {
    pixelBuffer[i] = 0;
  }
}

void RenderEntitiesToLEDBuffer()
{

  for(int i = 0; i < countEntities; i++)//for each entity
  {
    if(ENTITIES[i].position>=0)//check if position is in frame yet
    {
      if(ENTITIES[i].halfStep)
      {
        if(ENTITIES[i].direction == up)
        {
          pixelBuffer[ENTITIES[i].position-1] |= ENTITIES[i].colorHalf; //half fade the first pixel
          //if(DEV) pl(ENTITIES[i].color);
          //if(DEV) pl(pixels[ENTITIES[i].position-1]);
          for(int j = 0; j < entitySize-1; j++)//full brightness for the middle pixels
          {
            pixelBuffer[ENTITIES[i].position+j] |= ENTITIES[i].color;
          }
          pixelBuffer[ENTITIES[i].position+entitySize-1] |= ENTITIES[i].colorHalf; //half fade the last pixel
          ENTITIES[i].halfStep = false;//next step will NOT a half step
        }
        else //if(ENTITIES[i].direction == down)
        {
          pixelBuffer[ENTITIES[i].position] |= ENTITIES[i].colorHalf; //half fade the first pixel
          for(int j = 1; j < entitySize; j++)//full brightness for the middle pixels
          {
            pixelBuffer[ENTITIES[i].position+j] |= ENTITIES[i].color;
          }
          pixelBuffer[ENTITIES[i].position+entitySize] |= ENTITIES[i].colorHalf; //half fade the last pixel
          ENTITIES[i].halfStep = false;//next step will NOT be a half step
        }
      }
      else //not half step, display full unit
      {
        for(int j = 0; j < entitySize; j++)//for entity size (each pixel)
        {
          pixelBuffer[ENTITIES[i].position+j] |= ENTITIES[i].color;
        }
        ENTITIES[i].halfStep = true;//next step WILL be a half step        
      }
    }
  }
  //if(DEV) showPixels();
}


void MoveEntities()
{
  //pl("  moveEntities");
  for(int i = 0;i<countEntities;i++)
  {    
   if(ENTITIES[i].position <0){ENTITIES[i].position++;}//startup, enter from off screen

   else if(ENTITIES[i].halfStep == true) //only move entities that have completed a half step(false = complete)
    {
     if( ENTITIES[i].direction == up)
      {
         if(ENTITIES[i].position == numPixels - entitySize)
         {
           ENTITIES[i].direction = down;
           ENTITIES[i].position--;
         }
         else
         {
           ENTITIES[i].position++;
         }
      }
      else //diretion is down
      {
         if(ENTITIES[i].position == 0)
         {
           ENTITIES[i].direction = up;
           ENTITIES[i].position++;
         }
         else
         {
           ENTITIES[i].position--;
         }
       }
    }//end if(ENTITIES[i].halfStep == true)
   }//end for each entity
   //if(DEV)showEntityPositions();
}


void InitializeRotaryEncoder()
{
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  pinMode(encoderButton, INPUT);
 // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(encoderButton, HIGH);

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
}

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}




// Interrupt on A changing state
void doEncoderA(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;
    rotating = false;  // no more debouncing until loop() hits again
    // adjust counter + if A leads B
    if ( A_set && !B_set )
    {
      if(brightness >1 ) 
      {
        if(brightness < 12) {brightness--;}
        else {brightness /= 1.15;}
      }
    }
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) 
  delay (1);
  if( digitalRead(encoderPinB) != B_set ) {
  rotating = false;
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
     {
      if(brightness< 255)
      {
        if(brightness < 12) {brightness++;}
        else {brightness *= 1.15;}
        
        if(brightness > 255) brightness = 255;
      } 
    }
  }
}

// Slightly different, this one makes the rainbow wheel equally distributed 
// along the chain
void RainbowCycle(uint8_t wait) {
  int i, j;
  
  for (j=0; j < 256 && CURRENT_PROGRAM == rainbowCycle; j++) {     // 5 cycles of all 25 colors in the wheel
    for (i=0; i < numPixels; i++) {
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      pixelBuffer[i] =  Wheel( ((i * 256 / numPixels) + j) % 256);
    }  
    WriteBufferToStrip();   // write all the pixels out
    delay(wait);
    CheckForButtonPress();
  }
}



//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
