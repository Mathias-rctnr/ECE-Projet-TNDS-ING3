/* #include <Arduino.h>
#include <math.h>
#include <ArduinoFFT.h>
#include "arduinoMFCC.h"
#include <typeinfo>
#include <string.h>

#define BUFFER_SIZE 32000
#define DOWNSAMPLE_FACTOR 4
#define OUTPUT_SIZE (BUFFER_SIZE / DOWNSAMPLE_FACTOR)
#define FRAME_SIZE 256
#define OVERLAP_SIZE 128 // 50% overlap

#define MFCC_SIZE 13
#define FREQ_ECH 8000
#define DCT_MFCC_SIZE 7

#define PinTimer 3
#define FILTER_TAP_NUM 31

volatile uint16_t buffer[BUFFER_SIZE];
volatile uint16_t downsampledBuffer[OUTPUT_SIZE];
volatile uint16_t bufferIndex = 0;
volatile bool bufferReady = false;
volatile int sampleIndex = 0;
bool recording = false;
bool applyFilter = false; // Variable pour activer/désactiver le filtre

const int LED1 = 50;  // LED connectée à la pin D50
const int LED2 = 51;  // LED connectée à la pin D51
const int bouton = 2; // Utilise D2 comme bouton

unsigned long lastButtonPress = 0; // Pour le debounce

// Coefficients du filtre IIR
float b[3], a[3];
float w[2] = {0.0, 0.0};  // État du filtre

// Déclarez un objet arduinoMFCC global
arduinoMFCC mymfcc(MFCC_SIZE,DCT_MFCC_SIZE, FRAME_SIZE, FREQ_ECH);

float mfcc[MFCC_SIZE];

void calculerMFCC(float* frame, float* mfcc_coeffs) {
    Serial.println("Début de calculerMFCC"); // Debug

    // Convertir les données de frame en float

    Serial.println("Conversion en float terminée"); // Debug

    // Utiliser la bibliothèque arduinoMFCC pour calculer les coefficients MFCC
    mymfcc.compute(frame, mfcc_coeffs);

    Serial.println("Calcul MFCC terminé"); // Debug
}

void traitementMFCC(float* frameTEMP, float* mfcc_coeffs){

  float frame[128]={
    6.00,1591.00,1568.00,1708.00,1669.00,1680.00,1670.00,1614.00,1690.00,1641.00,1623.00,1588.00,1659.00,
    1537.00,1382.00,1360.00,1304.00,1402.00,1221.00,1411.00,1447.00,1488.00,1506.00,1428.00,1467.00,1504.00
    ,1570.00,1659.00,1820.00,1891.00,1981.00,1933.00,1953.00,1846.00,1863.00,1778.00,1751.00,1758.00,1596.00
    ,1562.00,1628.00,1495.00,1409.00,1397.00,1412.00,1303.00,1479.00,1401.00,1608.00,1564.00,1614.00,1773.00,
    1732.00,1838.00,1917.00,1822.00,1721.00,1730.00,1701.00,1574.00,1693.00,1543.00,1540.00,1397.00,1383.00,
    1381.00,1373.00,1273.00,1474.00,1351.00,1487.00,1482.00,1517.00,1433.00,1637.00,1479.00,1524.00,1411.00,
    1393.00,1456.00,1428.00,1437.00,1453.00,1518.00,1460.00,1524.00,1398.00,1372.00,1341.00,1395.00,1494.00,
    1352.00,1366.00,1398.00,1526.00,1603.00,1632.00,1624.00,1678.00,1634.00,1766.00,1606.00,1673.00,1592.00,
    1624.00,1537.00,1319.00,1345.00,1511.00,1624.00,1576.00,1618.00,1648.00,1511.00,1539.00,3.42};
  // Vérification : Imprimer le contenu de la frame
        Serial.print("] = {");
        for (int j = 0; j < FRAME_SIZE; j++) {
            //Serial.print(j);
            //Serial.print(":");
            Serial.print(frame[j]);
            Serial.print(", ");
        }
        Serial.println("};");
        Serial.print("Sortie boucle");

        // Calculer les coefficients MFCC
        calculerMFCC(frame, mfcc);

        // Sortie des coefficients MFCC (par exemple, imprimer sur Serial)
        Serial.print("MFCC Frame XX");
        //Serial.print(i);
        Serial.print(": ");
        for (int j = 0; j < MFCC_SIZE; j++) {
            Serial.print(mfcc[j]);
            Serial.print(" ");
        }
        Serial.println();
}

void traiterFrames(uint16_t* buffer, int tailleBuffer, int tailleFrame, int tailleOverlap) {
    int tailleStep = tailleFrame - tailleOverlap;
    int nombreFrames = (tailleBuffer - tailleOverlap) / tailleStep;

    Serial.print("Nombre de frames: ");
    Serial.println(nombreFrames);

    for (int i = 0; i < nombreFrames; i++) {
        Serial.print("Traitement de la frame ");
        Serial.println(i);

        uint16_t frame[FRAME_SIZE];
        memcpy(frame, &buffer[i * tailleStep], tailleFrame * sizeof(uint16_t));

        Serial.println("Debut conversion Float");
        float floatFrame[FRAME_SIZE];
        for (int i = 0; i < FRAME_SIZE; i++) {
            floatFrame[i] = static_cast<float>(frame[i]);
        }
        Serial.println("Fin conversion Float");
        
        Serial.print("Frame ");
        Serial.print(i);
        Serial.println(":");
        Serial.print("float frame[");
        Serial.print(tailleFrame);

        traitementMFCC(floatFrame, mfcc);
    }

    Serial.println("Fin du traitement des frames");
}

void calculateIIRCoefficients(float cutoffFreq, float sampleRate) {
  float normFreq = 2 * cutoffFreq / sampleRate;
  float theta = PI * normFreq;
  //float d = 1.0 / cos(theta);
  float beta = 0.5 * ((1.0 - (0.5 * sin(theta))) / (1.0 + (0.5 * sin(theta))));
  float gamma = (0.5 + beta) * cos(theta);
  float alpha = (0.5 + beta - gamma) * 0.5;
  
  b[0] = alpha;
  b[1] = 2.0 * alpha;
  b[2] = alpha;
  a[0] = 1.0;
  a[1] = -2.0 * gamma;
  a[2] = 2.0 * beta;
}

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37; // Active le périphérique ADC
  ADC->ADC_MR = ADC_MR_PRESCAL(0) // Définit le diviseur de fréquence à 255
  | ADC_MR_STARTUP_SUT64
  | ADC_MR_TRACKTIM(15) // Définit le temps de suivi à 15 périodes d'ADC_CLK
  | ADC_MR_SETTLING_AST3;
  ADC->ADC_CHER = 0xC0; // Active le canal 7 (A0)
  
  // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency 
  PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Active le périphérique TC0 
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Définit la valeur RC pour une fréquence samplingFrequency Hz 
  TC0->TC_CHANNEL[0].TC_RC = 21; // 32kHz -> ( MCK / 128 / 32000 ) -> 20.5078125 -> 20
  // Active l'interruption de comparaison RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Active l'interruption TC0_IRQn dans le NVIC
  NVIC_EnableIRQ(TC0_IRQn);
  // activation du compteur
  // activation du déclenchement logiciel
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void setupDAC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  DACC->DACC_MR = DACC_MR_TRGEN_DIS
  | DACC_MR_USER_SEL_CHANNEL1
  | DACC_MR_WORD_HALF
  | DACC_MR_REFRESH(1)
  | DACC_MR_STARTUP_8
  | DACC_MR_MAXS;
  DACC->DACC_CHER = DACC_CHER_CH1;  // Activer le canal 1
  NVIC_EnableIRQ(DACC_IRQn);
}

void TC0_Handler() {
  // Lit le registre d'état pour effacer le drapeau d'interruption 
  TC0->TC_CHANNEL[0].TC_SR;
  
  // Démarre une nouvelle conversion ADC
  ADC->ADC_CR = ADC_CR_START;
  
  // Si buffer non plein, stocke les données ADC
  if (recording && sampleIndex < BUFFER_SIZE) {
    buffer[bufferIndex++] = ADC->ADC_CDR[7];
    sampleIndex++;
    if (bufferIndex >= BUFFER_SIZE) {
      bufferReady = true;
      bufferIndex = 0;
      recording = false;
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      Serial.println("Appel de transfertDATA");
    }
  }
}

float applyIIRFilter(float input) {
  float output = b[0] * input + w[0];
  w[0] = b[1] * input + w[1] - a[1] * output;
  w[1] = b[2] * input - a[2] * output;
  return output;
}

void downsampleAndFilter(uint16_t* inputBuffer, uint16_t* outputBuffer, int inputLength, int factor) {
  int outputIndex = 0;
  for (int i = 0; i < inputLength; i += factor) {
    float sample = inputBuffer[i];
    if (applyFilter) {
      sample = applyIIRFilter(sample);
    }
    outputBuffer[outputIndex++] = (uint16_t)sample;
  }
}

void startRecording() {
  //delay(250);
  unsigned long currentTime = millis();
  if (currentTime - lastButtonPress > 200) { // Debounce check
    lastButtonPress = currentTime;
    if (!recording && !bufferReady) {
      bufferIndex = 0;
      sampleIndex = 0;
      recording = true;
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      Serial.println("Début de l’enregistrement");
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PinTimer, OUTPUT);
  pinMode(bouton, INPUT_PULLUP); // Bouton pour démarrer l'enregistrement
  attachInterrupt(digitalPinToInterrupt(bouton), startRecording, FALLING);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  setupADC();
  setupDAC();

  // Calculer les coefficients du filtre IIR
  calculateIIRCoefficients(4000, 32000);

  Serial.println("Creation hamming window");
  mymfcc.create_hamming_window();
  Serial.println("Creation mel filter bank");
  mymfcc.create_mel_filter_bank();
  Serial.println("Creation dct matrix");
  mymfcc.create_dct_matrix();

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
}

/* void loop() {
  if (bufferReady) {
    downsampleAndFilter((uint16_t*)buffer, (uint16_t*)downsampledBuffer, BUFFER_SIZE, DOWNSAMPLE_FACTOR);

    for (int i = 0; i < OUTPUT_SIZE; i++) {
      Serial.println(downsampledBuffer[i]);
      // DACC->DACC_CDR = DACC_CDR_DATA(downsampledBuffer[i]); // Décommenter si DAC utilisé
    }
    bufferReady = false;
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    Serial.println("Enregistrement terminé");
  }
}

void loop() {
  if (bufferReady) {
    Serial.println("Début du traitement du buffer");
    downsampleAndFilter((uint16_t*)buffer, (uint16_t*)downsampledBuffer, BUFFER_SIZE, DOWNSAMPLE_FACTOR);
    traiterFrames((uint16_t*)downsampledBuffer, OUTPUT_SIZE, FRAME_SIZE, OVERLAP_SIZE);

    bufferReady = false;
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    Serial.println("Enregistrement terminé et traitement des données terminé");
  }
} */

#include <Arduino.h>
#include <math.h>
#include <ArduinoFFT.h>
#include "arduinoMFCC.h"

#define BUFFER_SIZE 32000
#define DOWNSAMPLE_FACTOR 4
#define OUTPUT_SIZE (BUFFER_SIZE / DOWNSAMPLE_FACTOR)
#define FRAME_SIZE 256
#define OVERLAP_SIZE 128 // 50% overlap

#define MFCC_SIZE 13
#define FREQ_ECH 8000
#define DCT_MFCC_SIZE 7

#define PinTimer 3
#define FILTER_TAP_NUM 31

volatile uint16_t* buffer = NULL;
uint16_t* downsampledBuffer = NULL;
volatile uint16_t bufferIndex = 0;
volatile bool bufferReady = false;
volatile int sampleIndex = 0;
bool recording = false;
bool applyFilter = false; // Variable pour activer/désactiver le filtre

const int LED1 = 50;  // LED connectée à la pin D50
const int LED2 = 51;  // LED connectée à la pin D51
const int bouton = 2; // Utilise D2 comme bouton

bool infoFrame = false;
bool infoMFCC = true;

unsigned long lastButtonPress = 0; // Pour le debounce

// Coefficients du filtre IIR
float b[3], a[3];
float w[2] = {0.0, 0.0};  // État du filtre

// Déclarez un objet arduinoMFCC global
arduinoMFCC mymfcc(MFCC_SIZE,DCT_MFCC_SIZE, FRAME_SIZE, FREQ_ECH);

float mfcc[MFCC_SIZE];

void calculerMFCC(float* frame, float* mfcc_coeffs) {
    //Serial.println("Début de calculerMFCC"); // Debug

    // Utiliser la bibliothèque arduinoMFCC pour calculer les coefficients MFCC
    mymfcc.compute(frame, mfcc_coeffs);

    if (infoMFCC)
    {
      Serial.println("Calcul MFCC terminé"); // Debug
      // Débogage : imprimer les coefficients MFCC calculés
      for (int i = 0; i < MFCC_SIZE; i++) {
          Serial.print("MFCC[");
          Serial.print(i);
          Serial.print("]: ");
          Serial.println(mfcc_coeffs[i]);
      }

      Serial.println("Fin de calculerMFCC"); // Debug
    }
}

void traiterFrames(uint16_t* buffer) {
    int numFrames = (OUTPUT_SIZE - FRAME_SIZE) / OVERLAP_SIZE + 1;
    for (int frameNum = 0; frameNum < numFrames; frameNum++) {
        int startIndex = frameNum * OVERLAP_SIZE;
        
        // Copie de la frame actuelle dans frameTestFloat
        float frameTestFloat[FRAME_SIZE];
        for (int i = 0; i < FRAME_SIZE; i++) {
          frameTestFloat[i] = buffer[startIndex + i];
        }

        if (infoFrame)
        {
          // Affichage de la frame actuelle
          Serial.print("Frame ");
          Serial.print(frameNum);
          Serial.println(":");
          for (int j = 0; j < FRAME_SIZE; j++) {
              Serial.print(j);
              Serial.print(": ");
              Serial.print(buffer[startIndex + j]);
              Serial.print(" ");
          }
          Serial.println();
        }

        // Calcul des coefficients MFCC
        calculerMFCC(frameTestFloat, mfcc);

        if (infoFrame){
          // Affichage des coefficients MFCC
          Serial.print("MFCC Frame ");
          Serial.print(frameNum);
          Serial.println(": ");
          for (int j = 0; j < MFCC_SIZE; j++) {
              Serial.print(mfcc[j]);
              Serial.print(" ");
          }
          Serial.println();

          Serial.println("Fin du traitement de la frame");
        }
    }
}

void calculateIIRCoefficients(float cutoffFreq, float sampleRate) {
  float normFreq = 2 * cutoffFreq / sampleRate;
  float theta = PI * normFreq;
  //float d = 1.0 / cos(theta);
  float beta = 0.5 * ((1.0 - (0.5 * sin(theta))) / (1.0 + (0.5 * sin(theta))));
  float gamma = (0.5 + beta) * cos(theta);
  float alpha = (0.5 + beta - gamma) * 0.5;
  
  b[0] = alpha;
  b[1] = 2.0 * alpha;
  b[2] = alpha;
  a[0] = 1.0;
  a[1] = -2.0 * gamma;
  a[2] = 2.0 * beta;
}

void setupADC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID37; // Active le périphérique ADC
  ADC->ADC_MR = ADC_MR_PRESCAL(0) // Définit le diviseur de fréquence à 255
  | ADC_MR_STARTUP_SUT64
  | ADC_MR_TRACKTIM(15) // Définit le temps de suivi à 15 périodes d'ADC_CLK
  | ADC_MR_SETTLING_AST3;
  ADC->ADC_CHER = 0xC0; // Active le canal 7 (A0)
  
  // Configure Timer Counter 0 Channel 0 (TC0) pour samplingFrequency 
  PMC->PMC_PCER0 |= PMC_PCER0_PID27; // Active le périphérique TC0 
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_CPCTRG;
  // Définit la source d'horloge à TCLK4 (MCK / 128, 84 MHz / 128 = 656.25 kHz)
  // Définit la valeur RC pour une fréquence samplingFrequency Hz 
  TC0->TC_CHANNEL[0].TC_RC = 21; // 32kHz -> ( MCK / 128 / 32000 ) -> 20.5078125 -> 20
  // Active l'interruption de comparaison RC
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
  // Active l'interruption TC0_IRQn dans le NVIC
  NVIC_EnableIRQ(TC0_IRQn);
  // activation du compteur
  // activation du déclenchement logiciel
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

void setupDAC() {
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  DACC->DACC_MR = DACC_MR_TRGEN_DIS
  | DACC_MR_USER_SEL_CHANNEL1
  | DACC_MR_WORD_HALF
  | DACC_MR_REFRESH(1)
  | DACC_MR_STARTUP_8
  | DACC_MR_MAXS;
  DACC->DACC_CHER = DACC_CHER_CH1;  // Activer le canal 1
  NVIC_EnableIRQ(DACC_IRQn);
}

void TC0_Handler() {
  // Lit le registre d'état pour effacer le drapeau d'interruption 
  TC0->TC_CHANNEL[0].TC_SR;
  
  // Démarre une nouvelle conversion ADC
  ADC->ADC_CR = ADC_CR_START;
  
  // Si buffer non plein, stocke les données ADC
  if (recording && sampleIndex < BUFFER_SIZE) {
    buffer[bufferIndex++] = ADC->ADC_CDR[7];
    sampleIndex++;
    if (bufferIndex >= BUFFER_SIZE) {
      bufferReady = true;
      bufferIndex = 0;
      recording = false;
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      Serial.println("Appel de transfertDATA");
    }
  }
}

float applyIIRFilter(float input) {
  float output = b[0] * input + w[0];
  w[0] = b[1] * input + w[1] - a[1] * output;
  w[1] = b[2] * input - a[2] * output;
  return output;
}

void downsampleAndFilter(uint16_t* inputBuffer, uint16_t* outputBuffer, int inputLength, int factor) {
  int outputIndex = 0;
  for (int i = 0; i < inputLength; i += factor) {
    float sample = inputBuffer[i];
    if (applyFilter) {
      sample = applyIIRFilter(sample);
    }
    outputBuffer[outputIndex++] = (uint16_t)sample;
  }
  free(inputBuffer);
}

void startRecording() {
  //delay(250);
  unsigned long currentTime = millis();
  if (currentTime - lastButtonPress > 200) { // Debounce check
    lastButtonPress = currentTime;
    if (!recording && !bufferReady) {
      bufferIndex = 0;
      sampleIndex = 0;
      recording = true;
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      Serial.println("Début de l’enregistrement");
    }
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PinTimer, OUTPUT);
  pinMode(bouton, INPUT_PULLUP); // Bouton pour démarrer l'enregistrement
  attachInterrupt(digitalPinToInterrupt(bouton), startRecording, FALLING);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  setupADC();
  setupDAC();

  buffer = (uint16_t*) malloc(BUFFER_SIZE * sizeof(uint16_t));
  if (buffer == NULL) {
      Serial.println("Échec de l'allocation de buffer");
      // Gestion de l'échec d'allocation ici
  }
  
  // Allocation pour le buffer de downsample
  downsampledBuffer = (uint16_t*) malloc(OUTPUT_SIZE * sizeof(uint16_t));
  if (downsampledBuffer == NULL) {
      Serial.println("Échec de l'allocation de downsampledBuffer");
      // Gestion de l'échec d'allocation ici
  }

  // Calculer les coefficients du filtre IIR
  calculateIIRCoefficients(4000, 32000);

  mymfcc.create_hamming_window();
  mymfcc.create_mel_filter_bank();
  mymfcc.create_dct_matrix();

  // Frame de test

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
}

/* void loop() {
  if (bufferReady) {
    downsampleAndFilter((uint16_t*)buffer, (uint16_t*)downsampledBuffer, BUFFER_SIZE, DOWNSAMPLE_FACTOR);

    for (int i = 0; i < OUTPUT_SIZE; i++) {
      Serial.println(downsampledBuffer[i]);
      // DACC->DACC_CDR = DACC_CDR_DATA(downsampledBuffer[i]); // Décommenter si DAC utilisé
    }
    bufferReady = false;
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    Serial.println("Enregistrement terminé");
  }
} */

void loop() {
  if (bufferReady) {
    Serial.println("Début du traitement du buffer");
    downsampleAndFilter((uint16_t*)buffer, (uint16_t*)downsampledBuffer, BUFFER_SIZE, DOWNSAMPLE_FACTOR);
    traiterFrames(downsampledBuffer);

    bufferReady = false;
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    Serial.println("Enregistrement terminé et traitement des données terminé");
  }
}