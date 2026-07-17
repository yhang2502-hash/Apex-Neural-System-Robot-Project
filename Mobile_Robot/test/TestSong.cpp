#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

SoftwareSerial mp3Serial(2, 3);   // RX, TX

DFRobotDFPlayerMini player;

void setup()
{
    Serial.begin(9600);

    mp3Serial.begin(9600);

    Serial.println("Starting DFPlayer...");

    if (!player.begin(mp3Serial))
    {
        Serial.println("DFPlayer NOT Found!");

        while (1);
    }

    Serial.println("DFPlayer Ready!");

    player.volume(20);      // 0~30

    delay(1000);

    player.playMp3Folder(1);    // Play /mp3/0001.mp3
}

void loop()
{

}