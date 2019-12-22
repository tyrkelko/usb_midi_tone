#include <SN76489.h>
#include <usbh_midi.h>
#include <SPI.h>

USB Usb;
USBH_MIDI  Midi(&Usb);
#define PIN_NotWE 3

// Define FREQUENCY according to the frequency in your hardware setup
#define FREQUENCY 4000000.0
#define PIN_SER   7
#define PIN_LATCH 6
#define PIN_CLK   5

SN76489 mySN76489 = SN76489(PIN_NotWE, PIN_SER, PIN_LATCH, PIN_CLK, FREQUENCY);

void MIDI_poll();
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime);
void waitForUsbInit();

//int tonePin = 8;
boolean bFirst;
uint16_t pid, vid;
uint8_t notesCounter = 0;
uint8_t tonePlayed[3];
uint8_t noisePlayed = 0;
void setup()
{
  bFirst = true;
  vid = pid = 0;
  Serial.begin(115200);
  Serial.println("Starting...");

//  tone(tonePin, midiToFreq(69));
  delay( 100 );
 // noTone(tonePin);
  Serial.println("Started...");
  mySN76489.setAttenuation(0, 0xF);
  mySN76489.setAttenuation(1, 0xF);
  mySN76489.setAttenuation(2, 0xF);
  mySN76489.setAttenuation(3, 0xF);
  for (int i = 0; i < sizeof(tonePlayed); i++) tonePlayed[i] = 0;
  waitForUsbInit();
}

void loop()
{
  Usb.Task();
  uint8_t usbTaskState = Usb.getUsbTaskState();
  if ( usbTaskState == USB_STATE_RUNNING )
  {
    MIDI_poll();
  } else {
    char stateStringBuffer[50];
    Serial.print(getUsbTaskStateString(usbTaskState));
    sprintf(stateStringBuffer, " (%d)", usbTaskState);
    Serial.println(stateStringBuffer);
    delay(200);
  }
}

// Poll USB MIDI Controler and send to serial MIDI
void MIDI_poll()
{
  char buf[20];
  uint8_t bufMidi[64];
  uint16_t  rcvd;

  if (Midi.vid != vid || Midi.pid != pid) {
    sprintf(buf, "VID:%04X, PID:%04X", Midi.vid, Midi.pid);
    Serial.println(buf);
    vid = Midi.vid;
    pid = Midi.pid;
  }
  if (Midi.RecvData( &rcvd,  bufMidi) == 0 ) {
    if ((bufMidi[1]==0x80) || (bufMidi[1]==0x90)) {
      if ((bufMidi[1] == 0x90) && (bufMidi[3] > 0x00)) playNote(bufMidi[2], bufMidi[3]);
      if ((bufMidi[1] == 0x80) || ((bufMidi[1] == 0x90) && (bufMidi[3] == 0x00))) stopNote(bufMidi[2], bufMidi[3]);
      for (int i = 1; i < 4; i++) {
        sprintf(buf, " %02X", bufMidi[i]);
        Serial.print(buf);
      }
      Serial.println("");
    }
  }
}

// Delay time (max 16383 us)
void doDelay(uint32_t t1, uint32_t t2, uint32_t delayTime)
{
  uint32_t t3;

  if ( t1 > t2 ) {
    t3 = (0xFFFFFFFF - t1 + t2);
  } else {
    t3 = t2 - t1;
  }

  if ( t3 < delayTime ) {
    delayMicroseconds(delayTime - t3);
  }
}

void playNote(uint8_t midiNote, uint8_t midiVelocity) {
  Serial.print("Play note ");
  Serial.println(midiToFreq(midiNote));
  if ((midiNote >= 0x28) && (midiNote <= 0x2F)) { // 76489 noise generator
    mySN76489.setAttenuation(3, 0x0);
    mySN76489.setNoise(midiNote % 0x4, midiNote / 0x8);
    noisePlayed = midiNote;
  } else { // midi notes
    for (uint8_t toneGenerator = 0; toneGenerator < sizeof(tonePlayed); toneGenerator++) {
      if (tonePlayed[toneGenerator] == 0) {
        mySN76489.setAttenuation(toneGenerator, 0x0);  
        mySN76489.setFrequency(toneGenerator, midiToFreq(midiNote));
        tonePlayed[toneGenerator] = midiNote;
        toneGenerator=sizeof(tonePlayed);
      }
    }
    //tone(tonePin, midiToFreq(midiNote));
    notesCounter++;
  }
}
void stopNote(uint8_t midiNote, uint8_t midiVelocity) {
  Serial.print("Stop:");
  if ((midiNote >= 0x28 ) && (midiNote <= 0x2F) && (midiNote == noisePlayed)) {
    mySN76489.setAttenuation(3, 0xF);
  } else {
    for (uint8_t toneGenerator = 0; toneGenerator < sizeof(tonePlayed); toneGenerator++) {
      if (tonePlayed[toneGenerator] == midiNote) {
        mySN76489.setAttenuation(toneGenerator, 0xF);
        tonePlayed[toneGenerator] = 0;
        Serial.print("Tone #");
        Serial.print(toneGenerator);
        Serial.print("Note #");
        Serial.println(midiNote);      
      }
    }
  }
}

uint16_t midiToFreq(uint8_t midiNote) {
  Serial.println((uint16_t)440.0*pow(2.0,((float)midiNote-69.0)/12.0));
  if (midiNote < 0x28)return (uint16_t)440.0*pow(2.0,((float)midiNote-9.0)/24.0);
  return (uint16_t)440.0*pow(2.0,((float)midiNote-69.0)/12.0);
}

void waitForUsbInit() {
    while (Usb.Init() == -1) {
    Serial.println("Waiting for USB INIT");
    delay(500); //wait
  }
  Serial.println("USB Initialized");
}

String getUsbTaskStateString(uint8_t usbTaskState) {
  switch (usbTaskState) {
    case USB_STATE_MASK : return "USB State Mask";
    case USB_DETACHED_SUBSTATE_INITIALIZE : return "USB_DETACHED_SUBSTATE_INITIALIZE";
    case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE : return "USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE";
    case USB_DETACHED_SUBSTATE_ILLEGAL : return "USB_DETACHED_SUBSTATE_ILLEGAL";
    case USB_ATTACHED_SUBSTATE_SETTLE : return "USB_ATTACHED_SUBSTATE_SETTLE";
    case USB_ATTACHED_SUBSTATE_RESET_DEVICE : return "USB_ATTACHED_SUBSTATE_RESET_DEVICE";
    case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE : return "USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE";
    case USB_ATTACHED_SUBSTATE_WAIT_SOF : return "USB_ATTACHED_SUBSTATE_WAIT_SOF";
    case USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE : return "USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE";
    case USB_STATE_ADDRESSING : return "USB_STATE_ADDRESSING";
    case USB_STATE_CONFIGURING : return "USB_STATE_CONFIGURING";
    case USB_STATE_RUNNING : return "USB_STATE_RUNNING";
    case USB_STATE_ERROR : return "USB_STATE_ERROR";
    default : return "Not USB_STATE_RUNNING";
  }
}
