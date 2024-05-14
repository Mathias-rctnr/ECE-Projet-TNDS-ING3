#include <Arduino.h>

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
}