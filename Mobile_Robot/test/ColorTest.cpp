#include <Arduino.h>

// =============================
// TCS3200 CONNECTION
// =============================
#define COLOR_OUT 6
#define COLOR_S2  7
#define COLOR_S3  8

// =============================
// Read Pulse
// =============================
unsigned long readColorPulse(bool s2, bool s3)
{
    digitalWrite(COLOR_S2, s2);
    digitalWrite(COLOR_S3, s3);
    delay(50);

    unsigned long pulse = pulseIn(COLOR_OUT, LOW, 100000);

    if (pulse == 0)
        pulse = 99999;

    return pulse;
}

// =============================
// Detect Color
// =============================
String detectColorName()
{
    unsigned long red   = readColorPulse(LOW, LOW);
    unsigned long blue  = readColorPulse(LOW, HIGH);
    unsigned long green = readColorPulse(HIGH, HIGH);

    Serial.print("R=");
    Serial.print(red);
    Serial.print("  G=");
    Serial.print(green);
    Serial.print("  B=");
    Serial.println(blue);

    // -----------------------------
    // No object
    // Adjust this value if needed
    // -----------------------------
   // No object / table background
if (red > 60 && green > 60 && blue > 60) {
  return "NO OBJECT";
}

    // -----------------------------
    // BLUE
    // -----------------------------
    if (blue < red && blue < green)
        return "BLUE";

    // -----------------------------
    // YELLOW
    // Red and Green are both strong
    // -----------------------------
    if (red < blue &&
        green < blue &&
        abs((int)red - (int)green) < 25)
        return "YELLOW";

    // -----------------------------
    // RED
    // -----------------------------
    if (red < green && red < blue)
        return "RED";

    return "UNKNOWN";
}

// =============================
// Setup
// =============================
void setup()
{
    Serial.begin(9600);

    pinMode(COLOR_S2, OUTPUT);
    pinMode(COLOR_S3, OUTPUT);
    pinMode(COLOR_OUT, INPUT);

    Serial.println("====================");
    Serial.println("TCS3200 TEST");
    Serial.println("====================");
}

// =============================
// Loop
// =============================
void loop()
{
    String color = detectColorName();

    Serial.print("Detected: ");
    Serial.println(color);

    Serial.println();

    delay(500);
}