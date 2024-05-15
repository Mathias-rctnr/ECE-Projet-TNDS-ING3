/* #include <Arduino.h>

#define PinTimer 3
#define ADC_MR_PRESCAL_VALUE 0x01 // Valeur de prescaler pour ADC_MR_PRESCAL
#define BUFFER_SIZE 256          // Taille du buffer circulaire

uint16_t bufferCirculaire[BUFFER_SIZE]; // Buffer circulaire pour les échantillons
int head = 0;  // Pointeur de tête pour l'ajout des données
int tail = 0;  // Pointeur de queue pour le traitement des données

const int LED1 = 50;  // LED connectée à la pin D50
const int LED2 = 51;  // LED connectée à la pin D51
const int bouton = 2; // Utilise D2 comme bouton

const int numSamples = 32000; // Calcul du nombre total d'échantillons sampleRate * duration
int sampleIndex = 0;          // Index pour l'enregistrement
bool isRecording = false;     // Flag pour contrôler l'enregistrement

uint16_t echantillonsReechantillonnes[numSamples / 4]; // Échantillons rééchantillonnés après filtrage

unsigned long lastButtonPress = 0; // Pour le debouncing

const int FACTEUR_REECHANTILLONNAGE = 4; // Facteur de rééchantillonnage

void ajouterAuBuffer(uint16_t nouvelEchantillon) {
    bufferCirculaire[head] = nouvelEchantillon;
    head = (head + 1) % BUFFER_SIZE;
    if (head == tail) {  // Gestion du dépassement
        tail = (tail + 1) % BUFFER_SIZE;  // Déplace la queue si nécessaire
    }
    Serial.print("Buffer head: "); Serial.print((int)head); Serial.print(", tail: "); Serial.println((int)tail);
}

int appliquerFiltreRIF() {
    static const int coef[5] = {1, 2, 3, 2, 1};  // Coefficients du filtre RIF
    int somme = 0;
    for (int i = 0; i < 5; i++) {
        int index = (tail + i) % BUFFER_SIZE;
        somme += bufferCirculaire[index] * coef[i];
    }
    return somme / 9;  // Normalisation du filtre
}

void downsample(uint16_t *input, int inputLength, int factor) {
    int outputIndex = 0;
    for (int i = 0; i < inputLength; i += factor) {
        input[outputIndex++] = input[i];
    }
}

void transfertDATA() {
    // Afficher les données rééchantillonnées
    Serial.println("AVANT SERIAL TRANSFERTDATA");
    Serial.println("FILTREE DOWN SAMPLEE A 8KHz");
    for (int i = 0; i < 10; i++) { // Imprimer seulement les 10 premiers échantillons
        Serial.print("echantillonsReechantillonnes["); Serial.print(i); Serial.print("]: "); Serial.println((int)echantillonsReechantillonnes[i]);
        delay(10); // Ajouter une petite pause pour vider le buffer série
    }
    Serial.println("APRES SERIAL TRANSFERTDATA");
}

void setupTimer() {
    PMC->PMC_PCER0 |= PMC_PCER0_PID27;
    TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
    TC0->TC_CHANNEL[0].TC_RC = 5250 / 8; // Fréquence d’horloge de 1 MHz
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR;
    TC0->TC_CHANNEL[0].TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG;
    NVIC_EnableIRQ(TC0_IRQn);
}

void setupADC() {
    PMC->PMC_PCER0 |= PMC_PCER0_PID29;
    ADC->ADC_MR = ADC_MR_TRGEN_DIS
    | ADC_MR_LOWRES_BITS_12
    | ADC_MR_PRESCAL(3)
    | ADC_MR_STARTUP_SUT64
    | (0x2 << 24);
    ADC->ADC_CHER = ADC_CHER_CH0;
    ADC->ADC_IER = ADC_IER_EOC0;
}

void setupDAC() {
    PMC->PMC_PCER1 |= PMC_PCER1_PID38;
    DACC->DACC_MR = DACC_MR_TRGEN_DIS
    | DACC_MR_USER_SEL_CHANNEL1
    | DACC_MR_WORD_HALF
    | DACC_MR_REFRESH(1)
    | DACC_MR_STARTUP_8
    | DACC_MR_MAXS;
    DACC->DACC_CHER = DACC_CHER_CH1;
    DACC->DACC_IER |= DACC_IER_EOC;
    NVIC_EnableIRQ(DACC_IRQn);
}

void TC0_Handler() {
    TC0->TC_CHANNEL[0].TC_SR;  // Clear interrupt flag
    if (isRecording && sampleIndex < numSamples) {
        uint16_t rawValue = analogRead(A0);
        Serial.print("rawValue: "); Serial.println((int)rawValue);
        ajouterAuBuffer(rawValue);
        if (sampleIndex % FACTEUR_REECHANTILLONNAGE == 0) {
            uint16_t filteredSample = appliquerFiltreRIF();
            int index = sampleIndex / FACTEUR_REECHANTILLONNAGE;
            if (index < (numSamples / 4)) {
                echantillonsReechantillonnes[index] = filteredSample;
            }
            Serial.print("filteredSample: "); Serial.println((int)filteredSample);
        }

        sampleIndex++;
        if (sampleIndex >= numSamples) {
            isRecording = false;
            digitalWrite(LED1, HIGH);
            digitalWrite(LED2, LOW);
            Serial.println("Appel de transfertDATA");
            transfertDATA();  // Process or save the downsampled data
            Serial.println("Enregistrement terminé");
        }
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Setup started");
    pinMode(PinTimer, OUTPUT);
    pinMode(bouton, INPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    setupTimer();
    setupADC();
    setupDAC();

    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    Serial.println("Setup finished");
}

void loop() {
    unsigned long currentTime = millis();
    if (digitalRead(bouton) == HIGH && !isRecording && (currentTime - lastButtonPress > 200)) {
        lastButtonPress = currentTime;
        isRecording = true;
        sampleIndex = 0;
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);
        Serial.println("Début de l’enregistrement");
    }
    
    if (!isRecording && sampleIndex >= numSamples) {
        Serial.println("Enregistrement terminé");
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
    }
} */



#include <Arduino.h>

#define PinTimer 3
#define BUFFER_SIZE 32000
#define DOWNSAMPLE_FACTOR 4
#define OUTPUT_SIZE (BUFFER_SIZE / DOWNSAMPLE_FACTOR)
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

const float firCoefficients[FILTER_TAP_NUM] = {
  // Coefficients du filtre passe-bas (par exemple, conçu avec un outil de conception de filtre FIR)
  -0.0012, -0.0022, -0.0043, -0.0076, -0.0116, -0.0156, -0.0187, -0.0195, 
  -0.0162, -0.0074, 0.0090, 0.0334, 0.0652, 0.1025, 0.1424, 0.1811, 
  0.2147, 0.2391, 0.2512, 0.2496, 0.2343, 0.2065, 0.1694, 0.1270, 
  0.0841, 0.0454, 0.0153, -0.0040, -0.0124, -0.0105, -0.0009
};

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
  TC0->TC_CHANNEL[0].TC_RC = 20; // 32kHz -> ( MCK / 128 / 32000 ) -> 20.5078125 -> 20
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

void applyFIRFilter(uint16_t* inputBuffer, uint16_t* outputBuffer, int inputLength, int outputLength) {
  for (int i = 0; i < outputLength; i++) {
    float filteredSample = 0.0;
    for (int j = 0; j < FILTER_TAP_NUM; j++) {
      int bufferIndex = (i * DOWNSAMPLE_FACTOR + j) % inputLength;
      filteredSample += firCoefficients[j] * inputBuffer[bufferIndex];
    }
    outputBuffer[i] = (uint16_t)filteredSample;
  }
}

void downsample(uint16_t* inputBuffer, uint16_t* outputBuffer, int inputLength, int factor) {
  int outputIndex = 0;
  for (int i = 0; i < inputLength; i += factor) {
    outputBuffer[outputIndex++] = inputBuffer[i];
  }
}

void startRecording() {
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
  Serial.begin(460800);
  pinMode(PinTimer, OUTPUT);
  pinMode(bouton, INPUT_PULLUP); // Bouton pour démarrer l'enregistrement
  attachInterrupt(digitalPinToInterrupt(bouton), startRecording, FALLING);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  setupADC();
  setupDAC();

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);
}

void loop() {
  if (bufferReady) {
    if (applyFilter) {
      applyFIRFilter((uint16_t*)buffer, (uint16_t*)downsampledBuffer, BUFFER_SIZE, OUTPUT_SIZE);
    } else {
      downsample((uint16_t*)buffer, (uint16_t*)downsampledBuffer, BUFFER_SIZE, DOWNSAMPLE_FACTOR);
    }

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









