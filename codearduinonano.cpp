#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7789.h> 
#include <SPI.h>

/* --- CORES --- */
#define ST77XX_GRAY 0x8410 
#define ST77XX_DARKGREY 0x4208
#define ST77XX_ORANGE 0xFA60 

/* --- PINOS (Arduino Nano) --- */
#define TFT_CS    10  
#define TFT_RST   9   
#define TFT_DC    8   
#define BUTTON_PIN 2  

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

/* --- CONFIGURAÇÃO DE PRECISÃO --- */
#define SAMPLES 64              
#define SAMPLING_FREQUENCY 120  
#define ZONA_MORTA 0.06          
#define FATOR_SENSIBILIDADE 1.30 

Adafruit_MPU6050 mpu;
ArduinoFFT<double> FFT = ArduinoFFT<double>(); 

unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];

/* --- VARIÁVEIS --- */
float offsetRoll = 0.0;
float offsetPitch = 0.0;
double amplitudeAtual = 0.0;
double limiteAmplitude = 0.5; 
double frequenciaCorte = 0.0; 
bool calibrado = false;

// Variáveis visuais
double frequenciaExibida = 0.0;
double amplitudeVisual = 0.0; 

void desenharInterfaceFixa() {
  tft.fillScreen(ST77XX_BLACK); 
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 10);
  tft.println(F("MONITORAMENTO DC")); 
  tft.drawFastHLine(0, 35, 320, ST77XX_CYAN); 

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GRAY); 
  
  tft.setCursor(10, 50); tft.print(F("FREQ / RPM:"));
  tft.setCursor(160, 50); tft.print(F("TEMP / AMP:")); 
  tft.setCursor(10, 130); tft.print(F("INCLINACAO:"));
  
  tft.drawRect(10, 95, 140, 10, ST77XX_DARKGREY);
}

// --- FUNÇÃO AUXILIAR DE AMOSTRAGEM ---
double coletarAmostraRapida() {
    double minVal = 100000, maxVal = -100000;
    for (int k = 0; k < SAMPLES; k++) {
      microseconds = micros();
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      if (a.acceleration.x > maxVal) maxVal = a.acceleration.x;
      if (a.acceleration.x < minVal) minVal = a.acceleration.x;
      
      while (micros() - microseconds < sampling_period_us) {}
    }
    return (maxVal - minVal);
}

void animacaoCalibracao() {
  // ESTÁGIO 1: LEITURA DO AMBIENTE (20 SEG)
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(40, 30); tft.print(F("ETAPA 1/2"));
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 60); tft.print(F("LENDO AMBIENTE..."));
  tft.setTextSize(1);
  tft.setCursor(30, 90); tft.print(F("(MOTOR DESLIGADO - 20s)"));

  unsigned long tempoInicio = millis();
  double ruidoAmbienteMax = 0.0;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  offsetRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  offsetPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  while (millis() - tempoInicio < 20000) {
      int prog = map(millis() - tempoInicio, 0, 20000, 0, 320);
      tft.fillRect(0, 150, prog, 5, ST77XX_CYAN);
      double amp = coletarAmostraRapida();
      if (amp > ruidoAmbienteMax && amp < 2.0) ruidoAmbienteMax = amp; 
  }

  // CONTAGEM PARA LIGAR O MOTOR
  tft.fillScreen(ST77XX_BLACK);
  for(int i = 5; i > 0; i--) {
      tft.fillRect(0, 0, 320, 170, ST77XX_BLACK);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setTextSize(3);
      tft.setCursor(30, 40);
      tft.print(F("LIGUE O MOTOR"));
      tft.setTextColor(ST77XX_WHITE);
      tft.setTextSize(6);
      tft.setCursor(140, 90);
      tft.print(i);
      delay(1000);
  }

  // ESTÁGIO 2: LEITURA OPERACIONAL (20 SEG)
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.setCursor(40, 30); tft.print(F("ETAPA 2/2"));
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 60); tft.print(F("ANALISANDO MOTOR..."));
  tft.setTextSize(1);
  tft.setCursor(60, 90); tft.print(F("(AGUARDE 20s)"));
  
  double somaAmplitudes = 0.0;
  double maxPico = 0.0;
  long qtdLeituras = 0;
  
  tempoInicio = millis();
  
  while (millis() - tempoInicio < 20000) {
      int progresso = map(millis() - tempoInicio, 0, 20000, 0, 320);
      tft.fillRect(0, 150, progresso, 10, ST77XX_GREEN);

      double minVal = 100000, maxVal = -100000;
      for (int k = 0; k < SAMPLES; k++) {
        microseconds = micros();
        mpu.getEvent(&a, &g, &temp);
        if (a.acceleration.x > maxVal) maxVal = a.acceleration.x;
        if (a.acceleration.x < minVal) minVal = a.acceleration.x;
        while (micros() - microseconds < sampling_period_us) {}
      }
      
      double ampInstantanea = maxVal - minVal;
      if(ampInstantanea > 5.0) continue; 

      if (ampInstantanea > maxPico) maxPico = ampInstantanea;
      if(ampInstantanea > ruidoAmbienteMax) {
          somaAmplitudes += ampInstantanea;
          qtdLeituras++;
      }
  }

  double mediaVibracao = 0.0;
  if(qtdLeituras > 0) mediaVibracao = somaAmplitudes / qtdLeituras;
  if (mediaVibracao < (ruidoAmbienteMax + 0.05)) {
      mediaVibracao = 0.20; 
  }

  limiteAmplitude = mediaVibracao * FATOR_SENSIBILIDADE;
  if(limiteAmplitude < 0.15) limiteAmplitude = 0.15;

  calibrado = true;
  
  // Tela de Resumo
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 20); tft.print(F("CALIBRADO!"));
  tft.setTextSize(1);
  tft.setCursor(10, 60); tft.print(F("Ruido Amb: ")); tft.print(ruidoAmbienteMax);
  tft.setCursor(10, 80); tft.print(F("Media Motor: ")); tft.print(mediaVibracao);
  tft.setCursor(10, 110); 
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(F("LIMITE ALERTA: ")); tft.print(limiteAmplitude);
  
  delay(5000); 
  desenharInterfaceFixa();
}

void setup(void) {
  Serial.begin(115200); 
  pinMode(BUTTON_PIN, INPUT); 

  tft.init(170, 320); 
  tft.setRotation(1); 
  desenharInterfaceFixa(); 

  if (!mpu.begin()) {
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(10, 80);
    tft.print(F("ERRO: SENSOR OFF"));
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
      animacaoCalibracao();
  }
  
  double minVal = 100000;
  double maxVal = -100000;
  double somaAmostras = 0; 

  /* --- 1. LEITURA DE AMOSTRAS --- */
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    if (a.acceleration.x > maxVal) maxVal = a.acceleration.x;
    if (a.acceleration.x < minVal) minVal = a.acceleration.x;
    
    vReal[i] = a.acceleration.x; 
    vImag[i] = 0; 
    somaAmostras += vReal[i];

    while (micros() - microseconds < sampling_period_us) {}
  }
  
  amplitudeAtual = maxVal - minVal;
  amplitudeVisual = (amplitudeAtual * 0.85) + (amplitudeVisual * 0.15);

  /* --- 2. FFT e FREQUÊNCIA --- */
  double leituraInstantanea = 0.0;
  
  if (amplitudeAtual < ZONA_MORTA) {
      leituraInstantanea = 0.0;
      frequenciaExibida = 0.0; 
      amplitudeVisual = 0.0;   
  } else {
      double media = somaAmostras / SAMPLES;
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = vReal[i] - media; 
      }

      FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);
      leituraInstantanea = FFT.majorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  }

  if (leituraInstantanea > 0.1) {
      frequenciaExibida = (leituraInstantanea * 0.8) + (frequenciaExibida * 0.2);
  } else {
      frequenciaExibida = 0.0;
  }

  int rpm = (int)(frequenciaExibida * 60);

  /* --- 3. SENSORES E INCLINAÇÃO --- */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float roll = (atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI) - offsetRoll;
  float pitch = (atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI) - offsetPitch;

  /* --- 4. DETECÇÃO DE ALERTA --- */
  bool emAlerta = (calibrado && amplitudeAtual > limiteAmplitude);

  /* --- 5. DISPLAY (VISUAL) --- */
  tft.fillRect(10, 65, 140, 30, ST77XX_BLACK); 
  tft.setTextSize(3); 
  tft.setCursor(10, 65);
  if (emAlerta) tft.setTextColor(ST77XX_RED);
  else tft.setTextColor(ST77XX_MAGENTA);
  
  if(frequenciaExibida < 0.5) tft.print("0.0");
  else tft.print(frequenciaExibida, 1);

  tft.setTextSize(1);
  tft.setCursor(95, 65);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(F("RPM:"));
  tft.setCursor(95, 78);
  tft.setTextSize(2);
  tft.print(rpm);

  int larguraBarra;
  if (calibrado) {
      larguraBarra = map((long)(amplitudeVisual * 1000), 0, (long)(limiteAmplitude * 1000), 0, 138);
  } else {
      larguraBarra = map((long)(amplitudeVisual * 100), 0, 200, 0, 138); 
  }
  if (larguraBarra > 138) larguraBarra = 138; 
  
  tft.fillRect(11, 96, 138, 8, ST77XX_BLACK);
  uint16_t corBarra = emAlerta ? ST77XX_RED : ST77XX_GREEN;
  tft.fillRect(11, 96, larguraBarra, 8, corBarra);

  tft.fillRect(160, 65, 100, 25, ST77XX_BLACK); 
  tft.setTextSize(2);
  tft.setCursor(160, 65);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print(temp.temperature, 1);
  tft.setTextSize(1);
  tft.print(" C");

  tft.fillRect(160, 95, 120, 20, ST77XX_BLACK); 
  tft.setCursor(160, 95);
  tft.setTextSize(2);
  if (emAlerta) tft.setTextColor(ST77XX_RED); 
  else tft.setTextColor(ST77XX_ORANGE);
  tft.print(amplitudeVisual, 3);
  tft.setTextSize(1);
  tft.print(" g");

  tft.fillRect(10, 145, 300, 20, ST77XX_BLACK); 
  tft.setTextSize(2); 
  tft.setCursor(10, 145);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("P:"); tft.print(pitch, 0);
  tft.setCursor(160, 145);
  tft.print("R:"); tft.print(roll, 0);
  
  if(emAlerta) tft.fillRect(0, 165, 320, 5, ST77XX_RED); 
  else tft.fillRect(0, 165, 320, 5, ST77XX_BLACK); 

  /* --- 6. TRANSMISSÃO SERIAL (PARA ESP32) --- */
  // Formato: <frequencia,rpm,amplitude,temperatura,pitch,roll,alerta>
  
  Serial.print("<");
  Serial.print(frequenciaExibida, 1);
  Serial.print(",");
  Serial.print(rpm);
  Serial.print(",");
  Serial.print(amplitudeAtual, 3);
  Serial.print(",");
  Serial.print(temp.temperature, 1);
  Serial.print(",");
  Serial.print(pitch, 1);
  Serial.print(",");
  Serial.print(roll, 1);
  Serial.print(",");
  Serial.print(emAlerta ? 1 : 0); 
  Serial.println(">");
}
