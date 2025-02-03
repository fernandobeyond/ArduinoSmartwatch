#include <Wire.h>
#include <Adafruit_MPU6050.h> // Libreria sensor acelerometro y giroscopio
#include <LiquidCrystal_I2C.h> // Libreria pantalla I2C
#include <MAX30105.h> // Libreria sensor pulsometro y oxigeno
#include "heartRate.h" // Libreria de conteo de pulsos

LiquidCrystal_I2C lcd(0x27, 16, 2);
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

// Salida de Led que avisa la caida
const int ledPin = 7;

// Parametros para la deteccion pulso y conteo
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Inicializacion del LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  
  // Verifica que el sensor pulsometro este funcionando
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("Error al iniciar el sensor MAX30102");
    lcd.setCursor(0, 1);
    lcd.print("MAX30102 Error");
    while (1);
  }
  particleSensor.setup();

  // Verifica que el sensor acelerometro este funcionando
  if (!mpu.begin()) {
    Serial.println("Error al iniciar el sensor MPU6050");
    lcd.setCursor(0, 1);
    lcd.print("MPU6050 Error");
    while (1);
  }
  
  // Establece la salida del LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Limpia toda configuracion previa del LCD
  lcd.clear();
}

void loop() {
  // Parametros de lectural del sensor infrarojo y rojo
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // Lectura en el serial monitor para evaluar parametros
  Serial.print("IR: ");
  Serial.print(irValue);
  Serial.print("\t Red: ");
  Serial.print(redValue);

  float spo2 = 0.0;

  // Algoritmo para conteo de los pulsos
  if (checkForBeat(irValue) == false) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    // Lectura en el serial monitor para evaluar parametros
    Serial.print("\t bPM1: ");
    Serial.print(beatsPerMinute);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;

      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // Algoritmo para conteo del oxigeno en la sangre
  if (irValue > 20000 && redValue > 20000) {
    float ratio = (float)redValue / irValue;
    spo2 = 110.0 - 25.0 * ratio;
    if (spo2 < 0) spo2 = 0;
    if (spo2 > 100) spo2 = 100;
  }

  // Lectura en el serial monitor para evaluar parametros
  Serial.print("\t bPM2: ");
  Serial.println(beatsPerMinute);

  lcd.setCursor(0, 0);
  lcd.print("BPM: ");
  if (spo2 > 0) {
    lcd.print(beatsPerMinute, 0); // Imprimir en el I2C el BPM si esta disponible
  } else {
    lcd.print("---");
  }

  lcd.setCursor(0, 1);
  lcd.print("SpO2: ");
  if (spo2 > 0) {
    lcd.print(spo2, 0); // Imprimir en el I2C el oxigeno (SpO2) si esta disponible
    lcd.print("%");
  } else {
    lcd.print("---");
  }

  // Leer datos del MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calcular magnitud de la aceleración
  float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x + 
                               a.acceleration.y * a.acceleration.y + 
                               a.acceleration.z * a.acceleration.z);
  
  // Algoritmo para detectar caída
  if (accelMagnitude > 15.0) { // Establece un umbral de caída de 15 m/s^2
    // Si detecta caida, el led enciente y muestra mensaje en pantalla I2C
    digitalWrite(ledPin, HIGH);
    lcd.clear();
    lcd.print("Caida detectada!");
    delay(2000);
    lcd.clear();
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(500);
}
