#include <Adafruit_NeoPixel.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SerialFlash.h>

/*********** PIN DEFINITION ***********/
//-----------Button----------
#define PINpotIncP        4
#define PINpotIncN        3

//-----------IHM----------
#define PINrgbLED        10
#define PINbuzzer        15

//-----------Supply----------
#define PINsupplyLevel   A6
#define PINswitchEnable  5
#define PINlipoStat      16
#define PINcommandSupply 21

//-----------SGT15000--------
#define SGT15000_MOSI     7
#define SGT15000_SCK     14


/***********      DEF      ***********/
//-----------supply----------------
boolean lipo_Charged = false;
boolean lipo_InCharge = false;
int supplyLevel = 0;
int supplyMax = 1023;

//-----------mode--------
int mode = 0;  //  0= init, 1= VOL,2=FREQ lowpass, 3=FREQ bandpass
long buttonTimer = 0;
long longPressTime = 100000;  //depend of the processor

//-----------volume--------
int volumeLevel = 10; // 1 - 50
#define volumeLevelMax 50
#define volumeLevelMin 1

//-----------frequency--------
int frequencyLevelLowPass = 100; //passe bas 1 - 100
int frequencyLevelBandPass = 50; //passe bande 1 - 100
#define frequencyLevelMax 100
#define frequencyLevelMin 1

//-----------LEDS----------------
const int RGB_NUMLEDS = 8;
Adafruit_NeoPixel RGBLEDS = Adafruit_NeoPixel(RGB_NUMLEDS, PINrgbLED, NEO_GRB + NEO_KHZ800);


//-----------AUDIO----------------
AudioInputI2S            i2s2;           //xy=278.2833251953125,317.99999237060547
AudioFilterStateVariable filter1;        //xy=452.2833251953125,339.99999237060547
AudioRecordQueue         queue1;         //xy=452.2833251953125,402
AudioMixer4              mixer1;         //xy=612.2833251953125,328.99999237060547
AudioOutputI2S           i2s1;           //xy=751.2833251953125,326.99999237060547
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioConnection          patchCord2(i2s2, 0, filter1, 0);
AudioConnection          patchCord3(i2s2, 0, mixer1, 0);
AudioConnection          patchCord4(filter1, 0, mixer1, 1);
AudioConnection          patchCord5(filter1, 1, mixer1, 2);
AudioConnection          patchCord6(filter1, 2, mixer1, 3);
AudioConnection          patchCord7(mixer1, 0, i2s1, 0);
AudioConnection          patchCord8(mixer1, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=278.2833251953125,381




/***********      FONCTIONS     ***********/
//-----------     ARDUINO    -------------
void Arduino_Init() {
  Serial.begin(115200);

  pinMode(PINbuzzer, OUTPUT);
  pinMode(PINcommandSupply, OUTPUT);
  pinMode(PINbuzzer, OUTPUT);
  pinMode(PINlipoStat, INPUT);
  pinMode(PINsupplyLevel, INPUT);
  pinMode(PINpotIncP, INPUT_PULLUP);
  pinMode(PINpotIncN, INPUT_PULLUP);

  digitalWrite(PINcommandSupply, HIGH); // device on
  delay(1000);  // time to release switch

  RGBLEDS.begin(); //start rgb leds
  attachInterrupt(digitalPinToInterrupt(PINpotIncN), pot_detectionACTIVATE, CHANGE); // encoder pin on interrupt PINpotIncN
  attachInterrupt(digitalPinToInterrupt(PINswitchEnable), switch_detectionACTIVATE, RISING);  // encoder switch pin
}

void switch_detectionACTIVATE() { //switch activate

  while (digitalRead(PINswitchEnable)) { // TODO can be improve
    buttonTimer ++;
    Serial.println(buttonTimer);
    if (buttonTimer > longPressTime) {
      Serial.println("SLEEP");
      digitalWrite(PINcommandSupply, LOW);
      while (1);
    }
  }
  buttonTimer = 0;

  if (mode < 3) mode ++;
  else mode = 1;

  if (mode == 1)  Serial.println("MODE VOLUME");
  if (mode == 2)  Serial.println("MODE FREQUENCY LOWPASS");
  if (mode == 3)  Serial.println("MODE FREQUENCY BANDPASS");

  RGB_colorLED(mode);
}

void pot_detectionACTIVATE() { //pot activate
  switch (mode) {
    case 1:
      if ((digitalRead(PINpotIncP) == digitalRead(PINpotIncN)) && volumeLevel < volumeLevelMax) volumeLevel++;
      if ((digitalRead(PINpotIncP) != digitalRead(PINpotIncN)) && volumeLevel > volumeLevelMin) volumeLevel--;
      SetVolume(volumeLevel);
      RGB_colorLED(mode);
      break;
    case 2:
      if ((digitalRead(PINpotIncP) == digitalRead(PINpotIncN)) && frequencyLevelLowPass < frequencyLevelMax) frequencyLevelLowPass++;
      if ((digitalRead(PINpotIncP) != digitalRead(PINpotIncN)) && frequencyLevelLowPass > frequencyLevelMin) frequencyLevelLowPass--;
      SetFilter(mode);
      RGB_colorLED(mode);
      break;
    case 3:
      if ((digitalRead(PINpotIncP) == digitalRead(PINpotIncN)) && frequencyLevelBandPass < frequencyLevelMax) frequencyLevelBandPass++;
      if ((digitalRead(PINpotIncP) != digitalRead(PINpotIncN)) && frequencyLevelBandPass > frequencyLevelMin) frequencyLevelBandPass--;
      SetFilter(mode);
      RGB_colorLED(mode);
      break;
  }
}

//-----------  check power   -------------
void checkPowerLevel() {
  supplyLevel =  map(analogRead(PINsupplyLevel), 0, supplyMax, 0, 5);

  if (supplyLevel > 4) lipo_InCharge = true; //> 3.6v -> batt in charge
  else lipo_InCharge = false;

  if (!digitalRead(PINlipoStat)) lipo_Charged = false;
  else lipo_Charged = true;
}

//-----------  BUZZER   -------------
void BUZZER_pulse(int NbPulse, int Time) {
  //TODO change VOLUME
  for (int i = 0; i < NbPulse; i++) {
    digitalWrite(PINbuzzer, HIGH);
    delay(Time);
    digitalWrite(PINbuzzer, LOW);
  }
  //TODO REAPPLY VOLUME
}




//-----------     SGT15000   -------------
void SGT15000_Init() {
  AudioMemory(60); // Mic configuration

  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);

  SetVolume(0.6);
  sgtl5000_1.micGain(36);

  SPI.setMOSI(SGT15000_MOSI);
  SPI.setSCK(SGT15000_SCK);
}


//-----------  Setvolume -------------
void SetVolume(int SetDB) {
  float volume = map(SetDB,  volumeLevelMin,  volumeLevelMax, 0, 100);
  volume /= 100;
  Serial.print ("VOLUME: "); Serial.println (volume);
  sgtl5000_1.volume(volume);
}

//-----------  SetFilter -------------
void SetFilter(int FILTERmode) {
  int frequency = 0;

  switch (FILTERmode) {
    case 2: //low pass filter
      frequency = map(frequencyLevelLowPass, frequencyLevelMin, frequencyLevelMax, 1, 1000); //from 0 - 1000Hz
      Serial.print ("FREQUENCY lowpass: "); Serial.println (frequency);

      mixer1.gain(0, 0);  // no filter
      mixer1.gain(1, 1);  // passe bas
      mixer1.gain(2, 0);  // passe bande
      mixer1.gain(3, 0);  // passe haut

      filter1.frequency(frequency); //passe bas chamberlin 12db/octave roll-off
      filter1.resonance(0.707);
      break;
    case 3: //band pass filter
      frequency = map(frequencyLevelBandPass, frequencyLevelMin, frequencyLevelMax, 1, 1000); //from 0 - 1000Hz
      Serial.print ("FREQUENCY bandpass: "); Serial.println (frequency);
      mixer1.gain(0, 0);  // no filter
      mixer1.gain(1, 0);  // passe bas
      mixer1.gain(2, 1);  // passe bande
      mixer1.gain(3, 0);  // passe haut
      filter1.frequency(frequency); //passe bas chamberlin 12db/octave roll-off
      filter1.resonance(0.707);
      break;
  }
}


//-----------  RGB LEDS   -------------
void RGB_colorLED(int LEDmode) {
  for (int i = 0; i < RGB_NUMLEDS; i++) RGBLEDS.setPixelColor(i, RGBLEDS.Color(0, 0, 0)); //clear leds
  int nbLEDs = 0;
  switch (LEDmode) {
    case 0: //init
      for (int i = 0; i < RGB_NUMLEDS; i++) RGBLEDS.setPixelColor(i, RGBLEDS.Color(100, 100, 100));
      break;
    case 1: //VOLUME
      nbLEDs = map(volumeLevel, volumeLevelMin, volumeLevelMax, 1, RGB_NUMLEDS);
      for (int i = 0; i < nbLEDs; i++) RGBLEDS.setPixelColor(i, RGBLEDS.Color(0, 0, 100));
      break;
    case 2: //FREQUENCY lowpass
      nbLEDs = map(frequencyLevelLowPass, frequencyLevelMin, frequencyLevelMax, 1, RGB_NUMLEDS);
      for (int i = 0; i < nbLEDs; i++) RGBLEDS.setPixelColor(i, RGBLEDS.Color(100, 0, 0));
      break;
    case 3: //FREQUENCY bandpass
      nbLEDs = map(frequencyLevelBandPass, frequencyLevelMin, frequencyLevelMax, 0, RGB_NUMLEDS);
      RGBLEDS.setPixelColor(nbLEDs, RGBLEDS.Color(100, 100, 0)); //just one LED
      break;

  }
  RGBLEDS.show();
}

/***********     SETUP     ***********/
void setup() {
  Arduino_Init();
  SGT15000_Init();
}


/***********     LOOP      ***********/
void loop() {
  // checkPowerLevel();
  delay(1000);

  //TODO
}







