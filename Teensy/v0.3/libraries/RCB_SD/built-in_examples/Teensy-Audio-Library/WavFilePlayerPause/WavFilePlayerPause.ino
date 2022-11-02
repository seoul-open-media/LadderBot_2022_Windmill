#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

AudioPlaySdWav playWav1;
AudioOutputI2S audioOutput;

AudioConnection patchCord1(playWav1, 0, audioOutput, 0);
AudioConnection patchCord2(playWav1, 1, audioOutput, 1);
AudioControlSGTL5000 sgtl5000_1;

#define SDCARD_CS_PIN 10
#define SDCARD_MOSI_PIN 11
#define SDCARD_SCK_PIN 13

void setup()
{
    Serial.begin(9600);

    // Audio connections require memory to work.  For more
    // detailed information, see the MemoryAndCpuUsage example
    AudioMemory(8);

    // Comment these out if not using the audio adaptor board.
    // This may wait forever if the SDA & SCL pins lack
    // pullup resistors
    sgtl5000_1.enable();
    sgtl5000_1.volume(0.5);

    SPI.setMOSI(SDCARD_MOSI_PIN);
    SPI.setSCK(SDCARD_SCK_PIN);
    if (!(SD.begin(SDCARD_CS_PIN)))
    {
        // stop here, but print a message repetitively
        while (1)
        {
            Serial.println("Unable to access the SD card");
            delay(500);
        }
    }
}

void playFile(const char *filename)
{
    Serial.print("Playing file: ");
    Serial.println(filename);

    // Start playing the file.  This sketch continues to
    // run while the file plays.
    playWav1.play(filename);

    // A brief delay for the library read WAV info
    delay(5);

    // Simply wait for the file to finish playing.
    while (!playWav1.isStopped())
    {
        if (playWav1.isPlaying())
        {
            // play for 10 seconds
            delay(10000);
        }
        else if (playWav1.isPaused())
        {
            // pause for 2 seconds
            delay(2000);
        }

        // toggle play/pause state
        playWav1.togglePlayPause();
        Serial.println(filename);
        Serial.print("positionMillis() = ");
        Serial.println(playWav1.positionMillis());
        Serial.print("lengthMillis()   = ");
        Serial.println(playWav1.lengthMillis());
        Serial.print("isPlaying() = ");
        Serial.println(playWav1.isPlaying());
        Serial.print("isPaused()  = ");
        Serial.println(playWav1.isPaused());
        Serial.print("isStopped() = ");
        Serial.println(playWav1.isStopped());
        Serial.println();
    }
}

void loop()
{
    playFile("SDTEST1.WAV"); // filenames are always uppercase 8.3 format
    delay(500);
    playFile("SDTEST2.WAV");
    delay(500);
    playFile("SDTEST3.WAV");
    delay(500);
    playFile("SDTEST4.WAV");
    delay(1500);
}
