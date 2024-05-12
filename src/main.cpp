#include <Arduino.h>
#include <cmath>

#define PinTimer 3
#define ADC_MR_PRESCAL_VALUE 0x01 // valeur de prescaler pour ADC_MR_PRESCAL
#define RC 0 //Todo: Définir la valeur trouver pour le registre de comparaison

const int LED1 = 50;  // LED connectée à la pin D50
const int LED2 = 51;  // LED connectée à la pin D51

const int bouton = 2; // Utilise D2 comme bouton

const int sampleRate = 32000;  // Fréquence d'échantillonnage en Hz
const int duration = 1;       // Durée en secondes
const int numSamples = sampleRate * duration;  // Calcul du nombre total d'échantillons
int enregistrement[numSamples];      // Tableau pour stocker les échantillons
int enregistrementFiltre[numSamples]; // Tableau pour stocker les échantillons filtrés
int sampleIndex = 0; // Index pour l'enregistrement
volatile bool isRecording = false; // Flag pour contrôler l'enregistrement

unsigned long lastButtonPress = 0; // Pour le debouncing

const int TAILLE_BUFFER = 100; // Exemple de taille de tampon
int tampon[TAILLE_BUFFER];     // Tampon pour le traitement
int indexTampon = 0;           // Index actuel dans le tampon
int sortieFiltree[numSamples]; // Sortie après le filtrage
int echantillonsReechantillonnes[numSamples]; // Échantillons rééchantillonnés
int compteurReechantillonnage = 0; // Compteur pour le rééchantillonnage
const int FACTEUR_REECHANTILLONNAGE = 4; // Facteur de rééchantillonnage
int coefficients[5] = {1, 2, 3, 2, 1}; // Exemple simple de coefficients de filtre
const int NB_COEFFS = 5; // Nombre de coefficients

// Filtre passe-bas
const float fc = 3970;
const float fs = 8000;
const float pi = 3.14159265358979323846;
const float omega_c = tan(pi * fc / fs);
const float alpha = omega_c / (1.0 + omega_c);
float previousOutput = 0.0;

int downsampledSamples[numSamples / 4];

//****************** BUFFER

void initialiserTampon() {
    for (int i = 0; i < TAILLE_BUFFER; i++) {
        tampon[i] = 0;
    }
    indexTampon = 0;
}

void ajouterAuTampon(int nouvelEchantillon) {
    tampon[indexTampon] = nouvelEchantillon;
    indexTampon = (indexTampon + 1) % TAILLE_BUFFER;
}

int appliquerFiltre() {
    long somme = 0;
    for (int i = 0; i < NB_COEFFS; ++i) {
        int index = (indexTampon - i + TAILLE_BUFFER) % TAILLE_BUFFER;
        somme += tampon[index] * coefficients[i];
    }
    return somme >> 8;  // Ajuster le décalage selon la précision des coefficients
}

// Fonction pour gérer le rééchantillonnage
void appliquerReechantillonnage(int valeurFiltree, int* indexReechantillonnage) {
    if (++compteurReechantillonnage >= FACTEUR_REECHANTILLONNAGE) {
        compteurReechantillonnage = 0;
        echantillonsReechantillonnes[(*indexReechantillonnage)++] = valeurFiltree;
    }
}

//****************** FIN BUFFER

void downsample(int* input, int* output, int inputLength, int factor) {
    int outputIndex = 0;
    for (int i = 0; i < inputLength; i += factor) {
        output[outputIndex++] = input[i];
    }
}

/* float filtrePasseBas(float input) {
    float output = alpha * input + (1 - alpha) * previousOutput;
    previousOutput = output;
    return output;
} */

void setupTimer() {
    PMC->PMC_PCER0 |= PMC_PCER0_PID27;
    TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC;
    TC0->TC_CHANNEL[0].TC_RC = 5250/8; // Fréquence d'horloge de 1 MHz      //! DIVISE PAR 2 POUR 8KHz
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
    TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR;
    TC0->TC_CHANNEL[0].TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG;
    NVIC_EnableIRQ(TC0_IRQn);
}

/* void TC0_Handler() {
    TC0->TC_CHANNEL[0].TC_SR; // Clear interrupt flag
    if (isRecording && sampleIndex < numSamples) {
        int rawValue = analogRead(A0);  // Lire la valeur du microphone
        int filteredValue = filtrePasseBas(rawValue);  // Filtrer la valeur
        enregistrement[sampleIndex] = rawValue;
        enregistrementFiltre[sampleIndex] = filteredValue;
        DACC->DACC_CDR = filteredValue >> 2;  // Envoyer la valeur filtrée au DAC, ajuster pour la résolution du DAC si nécessaire
        sampleIndex++;
    }
    digitalWrite(PinTimer, !digitalRead(PinTimer));
} */

void TC0_Handler() {
    TC0->TC_CHANNEL[0].TC_SR;  // Clear interrupt flag
    if (isRecording && sampleIndex < (TAILLE_BUFFER / FACTEUR_REECHANTILLONNAGE)) {
        int valeurBrute = analogRead(A0);
        ajouterAuTampon(valeurBrute);
        int valeurFiltree = appliquerFiltre();
        appliquerReechantillonnage(valeurFiltree, &sampleIndex);
        sortieFiltree[indexTampon] = valeurFiltree;

        digitalWrite(PinTimer, !digitalRead(PinTimer));
    }
}

void setupADC(){
    PMC->PMC_PCER0 |= PMC_PCER0_PID29;
    ADC->ADC_MR = ADC_MR_TRGEN_DIS             // Désactiver le déclenchement externe
                | ADC_MR_LOWRES_BITS_12        // Résolution de 12 bits
                | ADC_MR_PRESCAL(3)            // Prescaler à 3
                | ADC_MR_STARTUP_SUT64         // Temps de démarrage à 64 périodes d'ADC_CLK
                | (0x2 << 24);                 // Délai de stabilisation de 16 périodes d'ADC_CLK
    ADC->ADC_CHER = ADC_CHER_CH0; // Sélectionner le canal 0
    ADC->ADC_IER = ADC_IER_EOC0; // Activer l'interruption pour la fin de conversion du canal 0
}

void setupDAC(){
    PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Active le périphérique DAC
    DACC->DACC_MR = DACC_MR_TRGEN_DIS // Désactive le déclencheur externe
                    | DACC_MR_USER_SEL_CHANNEL1 // select canal 1 // DACC_MR_USER_SEL_CHANNEL0
                    | DACC_MR_WORD_HALF // Largeur de mot de 16 bits (0 - 4095)
                    | DACC_MR_REFRESH(1) // Temps de rafraîchissement (dans les cycles de l'horloge du périphérique)
                    | DACC_MR_STARTUP_8 // Temps de démarrage (8 * 6 cycles)
                    | DACC_MR_MAXS; // Utilise le contrôleur DMA pour les transferts DAC
    // Active le canal 1 du DAC
    DACC->DACC_CHER = DACC_CHER_CH1;// DACC_CHER_CH0
    DACC->DACC_IER |= DACC_IER_EOC;
    NVIC_EnableIRQ(DACC_IRQn);
}

void DACC_Handler() {
    DACC->DACC_ISR; // Clear DAC status register
}

void transfertDATA() {
    // Appliquez le downsampling aux échantillons filtrés
    downsample(enregistrementFiltre, downsampledSamples, numSamples, 4);

    // Afficher les données filtrées downsamplées
    Serial.println("FILTREE DOWN SAMPLEE A 8KHz");
    for (int i = 0; i < numSamples / 4; i++) {
        Serial.println(downsampledSamples[i]);
    }

    // Optionnellement, vous pouvez également imprimer les données brutes
    Serial.println("NON FILTREE");
    for (int i = 0; i < numSamples; i++) {
        Serial.println(enregistrement[i]);
    }
}

int calculCoef(){
  return 0;
}

void setup() {
    Serial.begin(460800);
    pinMode(PinTimer, OUTPUT);
    pinMode(bouton, INPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    setupTimer();
    setupADC();
    setupDAC();

    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
}

void loop() {
    unsigned long currentTime = millis();
    if (digitalRead(bouton) == HIGH && !isRecording && (currentTime - lastButtonPress > 200)) {
        delay(300);
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);
        lastButtonPress = currentTime;
        isRecording = true;
        sampleIndex = 0;
    }

    if (isRecording && sampleIndex >= numSamples) {
        isRecording = false;
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
        transfertDATA();  // Afficher les données après l'enregistrement
    }   //TEST GIT
}
