#include <Arduino.h>
#include <math.h>
#include <ArduinoFFT.h>
#include "arduinoMFCC.h"
#include "NeuralNetwork.h"
#include "poids.h"

#define NumberOf(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0]))) // calculates the number of layers (in this case 4)
#define _1_OPTIMIZE B01010010
#define BUFFER_SIZE 32000
#define DOWNSAMPLE_FACTOR 4
#define OUTPUT_SIZE (BUFFER_SIZE / DOWNSAMPLE_FACTOR)
#define FRAME_SIZE 256
#define OVERLAP_SIZE 163 // 50% overlap

#define MFCC_SIZE 13
#define FREQ_ECH 8000
#define DCT_MFCC_SIZE 7

#define ROWS 48
#define COLS 13

#define PinTimer 3
#define FILTER_TAP_NUM 31

//!\ Réseau

extern const float biases[];
extern const float weights[];

//float biases[] = {};
//float weights[] = {};

const unsigned int layers[] = {780, 100, 50, 2};
float *output; // 4th layer's output(s)

NeuralNetwork NN(layers, const_cast<float*>(weights), const_cast<float*>(biases), NumberOf(layers));

//!\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

volatile uint16_t* buffer = NULL;
float (*tab_MFCC)[COLS] = NULL;
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
bool infoMFCC = false;

unsigned long lastButtonPress = 0; // Pour le debounce

// Coefficients du filtre IIR
float b[3], a[3];
float w[2] = {0.0, 0.0};  // État du filtre

// Déclarez un objet arduinoMFCC global
arduinoMFCC mymfcc(MFCC_SIZE,DCT_MFCC_SIZE, FRAME_SIZE, FREQ_ECH);

float mfcc[MFCC_SIZE];

void Choix_Reseau(NeuralNetwork NN, float tabMFCC[ROWS][COLS]) {
    for (int i = 0; i < 48; i++)  // Supposition que vous voulez traiter les 48 premières lignes
    {
        const float* input = tabMFCC[i];  // Pointeur vers le début de la ligne i du tableau
        output = NN.FeedForward(input); // FeedForwards the input[i]-array through the NN  |  returns the predicted output(s)
        Serial.println(output[0]);  // Affiche les résultats avec 7 chiffres après la virgule
    }
    //NN.print();
}

void calculerMFCC(float* frame, float* mfcc_coeffs, int frameNum) {
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
          Serial.println(mfcc_coeffs[i]/10);
          tab_MFCC[frameNum][i] = mfcc_coeffs[i]/10;
      }

      Serial.println("Fin de calculerMFCC"); // Debug
    }
    else{
      Serial.print("Numero de frame ");
      Serial.println(frameNum);
      for (int i = 0; i < MFCC_SIZE; i++) {
        tab_MFCC[frameNum][i] = mfcc_coeffs[i]/10;
      }
    }
}

void traiterFrames(uint16_t* buffer) {
    int numFrames = (OUTPUT_SIZE - FRAME_SIZE) / OVERLAP_SIZE + 1;

    tab_MFCC = (float (*)[COLS]) malloc(ROWS * COLS * sizeof(float));
    if (tab_MFCC == NULL) {
        Serial.println("Échec de l'allocation de tab_MFCC");
    } else {
        Serial.println("Allocation réussie de tab_MFCC");
    }
    Serial.print("NUM DE FRAME:");
    Serial.println(numFrames);
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
          /* for (int j = 0; j < FRAME_SIZE; j++) {
              Serial.print(j);
              Serial.print(": ");
              Serial.print(buffer[startIndex + j]);
              Serial.print(" ");
          } */
          Serial.println();
        }
        free(buffer);

        // Calcul des coefficients MFCC
        calculerMFCC(frameTestFloat, mfcc, frameNum);

        if (infoFrame){
          // Affichage des coefficients MFCC
          Serial.print("MFCC Frame ");
          Serial.print(frameNum);
          Serial.println(": ");
          for (int j = 0; j < MFCC_SIZE; j++) {
              Serial.print(mfcc[j]/10);
              Serial.print(" ");
          }
          Serial.println();

          Serial.println("Fin du traitement de la frame");
        }
    }

    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            Serial.print(tab_MFCC[i][j]);  // Affiche chaque valeur avec 6 chiffres après la virgule
            Serial.print(" ");               // Espace entre les valeurs pour la lisibilité
        }
        Serial.println();  // Nouvelle ligne après chaque ligne de 13 valeurs
    }
    Choix_Reseau(NN, tab_MFCC);
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
  //Serial.begin(460800);
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