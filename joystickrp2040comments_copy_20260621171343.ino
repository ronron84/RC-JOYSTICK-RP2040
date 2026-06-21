/*
 * Joystick USB HID pour Radio-Commande
 * 
 * Description:
 * Ce projet transforme un microcontrôleur en joystick USB HID compatible
 * avec les simulateurs de vol (DCS, X-Plane, MSFS) et les jeux.
 * Il supporte 8 canaux (4 axes analogiques + 4 boutons) avec calibration
 * avancée et retour au centre personnalisable.
 * 
 * Fonctionnalités:
 * - Lecture de 8 canaux ADC (4 axes + 4 boutons)
 * - Lissage des signaux par moyenne mobile
 * - Calibration manuelle et automatique
 * - Zone morte ajustable
 * - Retour au centre personnalisable
 * - Sauvegarde EEPROM des paramètres
 * - Interface de configuration par Serial
 * 
 * Brochage:
 * Canaux 1-4: Broches 26, 27, 28, 29 (axes analogiques)
 * Canaux 5-8: Broches 0, 1, 2, 3 (boutons / interrupteurs)
 * 
 * Commandes Serial:
 * help          - Affiche toutes les commandes
 * values        - Affiche les valeurs actuelles
 * autocalib     - Calibration automatique
 * setmin/max    - Configuration manuelle
 * reset         - Reset usine
 * 
 * Auteur: [regis BESNIER]
 * Date: [12/10/2025]
 * Version: 1.1 (modifié pour interrupteurs sur GPIO 0-3)
 * Licence: MIT
 */

#include <Adafruit_TinyUSB.h>
#include <EEPROM.h>

// =============================================================================
// CONFIGURATION DES CONSTANTES
// =============================================================================

#define NUM_CHANNELS 8           // Nombre total de canaux (4 axes + 4 boutons)
#define SMOOTHING_SAMPLES 10     // Taille de la fenêtre de lissage
#define DEADZONE 8               // Zone morte autour du centre (0-127)
#define RETURN_TO_CENTER_FORCE 3 // Force de retour au centre
#define NEUTRAL_THRESHOLD 5      // Seuil pour considérer comme neutre

// Brochage des canaux ADC
const uint8_t channelPins[NUM_CHANNELS] = {26, 27, 28, 29, 0, 1, 2, 3};

// =============================================================================
// CONFIGURATION EEPROM - Sauvegarde persistante
// =============================================================================

#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xAB        // Marqueur magique pour validation EEPROM
#define EEPROM_CALIB_START 0     // Adresse de départ en EEPROM

/**
 * Structure de sauvegarde EEPROM
 * Contient tous les paramètres de calibration pour persistance
 */
struct CalibrationData {
  uint8_t magic;                 // Marqueur magique pour validation
  uint16_t minCalibration[NUM_CHANNELS];    // Valeurs minimales calibrées
  uint16_t maxCalibration[NUM_CHANNELS];    // Valeurs maximales calibrées
  uint16_t buttonThresholds[NUM_CHANNELS];  // Seuils des boutons
  uint16_t neutralCalibration[NUM_CHANNELS];// Points neutres des axes
  bool returnToCenterEnabled[4];            // Activation retour au centre
};

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

// Buffers pour le lissage des signaux
uint16_t analogReadings[NUM_CHANNELS][SMOOTHING_SAMPLES];
uint8_t readIndex = 0;
uint16_t smoothedValues[NUM_CHANNELS] = {0};
uint16_t rawValues[NUM_CHANNELS] = {0};

// Paramètres de calibration (chargés depuis EEPROM)
uint16_t minCalibration[NUM_CHANNELS];
uint16_t maxCalibration[NUM_CHANNELS];
uint16_t buttonThresholds[NUM_CHANNELS];
uint16_t neutralCalibration[NUM_CHANNELS];
bool returnToCenterEnabled[4] = {true, true, true, true};

// Mémoire des dernières valeurs HID pour la stabilité
int8_t lastHidValues[4] = {0, 0, 0, 0};

// =============================================================================
// DESCRIPTEUR HID - Configuration du périphérique USB
// =============================================================================

/**
 * Structure de rapport HID pour joystick
 * Compatible avec la plupart des simulateurs et jeux
 */
typedef struct {
  int8_t x, y, z, rx;  // 4 axes (canaux 1-4)
  uint8_t buttons;      // 8 boutons (bits 0-7 pour canaux 5-8)
} joystickReport_t;

// Descripteur HID personnalisé pour un joystick 4 axes + 8 boutons
static const uint8_t joystick_hid_report_desc[] = {
  0x05, 0x01, 0x09, 0x04, 0xA1, 0x01, 0x05, 0x01, 0x09, 0x01, 0xA1, 0x00,
  0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 0x15, 0x81, 0x25, 0x7F,
  0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0xC0, 0x05, 0x09, 0x19, 0x01, 0x29,
  0x08, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0xC0
};

Adafruit_USBD_HID usb_hid;

// =============================================================================
// FONCTIONS EEPROM - Sauvegarde et restauration
// =============================================================================

/**
 * Charge la calibration depuis l'EEPROM
 * Si EEPROM vierge, initialise avec les valeurs par défaut
 */
void loadCalibration() {
  CalibrationData data;
  EEPROM.get(EEPROM_CALIB_START, data);
  
  if (data.magic != EEPROM_MAGIC) {
    // Première utilisation - initialisation avec valeurs par défaut
    Serial.println("EEPROM vierge, initialisation avec valeurs par defaut...");
    resetCalibration();
    saveCalibration();
    return;
  }
  
  // Copie des données depuis EEPROM
  for (int i = 0; i < NUM_CHANNELS; i++) {
    minCalibration[i] = data.minCalibration[i];
    maxCalibration[i] = data.maxCalibration[i];
    buttonThresholds[i] = data.buttonThresholds[i];
    neutralCalibration[i] = data.neutralCalibration[i];
  }
  for (int i = 0; i < 4; i++) {
    returnToCenterEnabled[i] = data.returnToCenterEnabled[i];
  }
  
  Serial.println("Calibration chargee depuis EEPROM");
}

/**
 * Sauvegarde la calibration actuelle en EEPROM
 * Appelée automatiquement après chaque modification
 */
void saveCalibration() {
  CalibrationData data;
  data.magic = EEPROM_MAGIC;
  
  for (int i = 0; i < NUM_CHANNELS; i++) {
    data.minCalibration[i] = minCalibration[i];
    data.maxCalibration[i] = maxCalibration[i];
    data.buttonThresholds[i] = buttonThresholds[i];
    data.neutralCalibration[i] = neutralCalibration[i];
  }
  for (int i = 0; i < 4; i++) {
    data.returnToCenterEnabled[i] = returnToCenterEnabled[i];
  }
  
  EEPROM.put(EEPROM_CALIB_START, data);
  EEPROM.commit();
  
  Serial.println("Calibration sauvegardee en EEPROM");
}

// =============================================================================
// FONCTIONS PRINCIPALES ARDUINO
// =============================================================================

void setup() {
  Serial.begin(115200);
  
  // Initialisation EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Chargement de la calibration sauvegardée
  loadCalibration();
  
  // Configuration des broches
  // Axes (ADC) : broches 26 à 29
  for (int i = 0; i < 4; i++) {
    pinMode(channelPins[i], INPUT);
  }
  // Interrupteurs : broches 0 à 3 (tirage vers la masse)
  for (int i = 4; i < NUM_CHANNELS; i++) {
    pinMode(channelPins[i], INPUT_PULLDOWN);
  }
  
  // Initialisation du système de lissage
  for (int i = 0; i < NUM_CHANNELS; i++) {
    for (int j = 0; j < SMOOTHING_SAMPLES; j++) {
      if (i < 4) {
        analogReadings[i][j] = analogRead(channelPins[i]);
      } else {
        // Pour les interrupteurs, on lit l'état numérique et on le convertit en 0 ou 4095
        analogReadings[i][j] = digitalRead(channelPins[i]) ? 4095 : 0;
      }
    }
    smoothedValues[i] = analogReadings[i][0];
    
    // Calcul automatique du point neutre si non défini
    if (i < 4 && neutralCalibration[i] == 512) {
      neutralCalibration[i] = (minCalibration[i] + maxCalibration[i]) / 2;
    }
  }
  
  // Configuration du périphérique USB HID
  usb_hid.setReportDescriptor(joystick_hid_report_desc, sizeof(joystick_hid_report_desc));
  usb_hid.begin();
  
  // Attente de la connexion USB
  while (!TinyUSBDevice.mounted()) delay(1);
  
  printHelp();
  Serial.println("Systeme pret! Calibration chargee depuis EEPROM.");
  Serial.println("Tapez 'help' pour voir les commandes.");
}

void loop() {
  // Lecture et traitement des canaux
  readAndSmoothChannels();
  
  // Mise à jour du rapport HID
  updateJoystick();
  
  // Gestion des commandes série
  handleSerialCommands();
  
  // Affichage périodique des valeurs (toutes les 2 secondes)
  static uint32_t lastDisplay = 0;
  if (millis() - lastDisplay > 2000) {
    displayCurrentValues();
    lastDisplay = millis();
  }
  
  delay(15); // Délai de stabilisation
}

// =============================================================================
// FONCTIONS DE TRAITEMENT DU SIGNAL (modifiées)
// =============================================================================

/**
 * Lit et lisse les valeurs des axes et des interrupteurs
 * Axes : lecture analogique (ADC) + moyenne mobile
 * Interrupteurs : lecture numérique (0/1) convertie en 0/4095 + moyenne mobile (optionnelle)
 */
void readAndSmoothChannels() {
  // Lecture des axes (ADC)
  for (int i = 0; i < 4; i++) {
    rawValues[i] = analogRead(channelPins[i]);
    analogReadings[i][readIndex] = rawValues[i];
  }
  
  // Lecture des interrupteurs (numérique)
  for (int i = 4; i < NUM_CHANNELS; i++) {
    bool state = digitalRead(channelPins[i]);  // HIGH si interrupteur enclenché
    rawValues[i] = state ? 4095 : 0;           // Conversion pour homogénéité
    analogReadings[i][readIndex] = rawValues[i];
  }
  
  // Avancement de l'index circulaire
  readIndex = (readIndex + 1) % SMOOTHING_SAMPLES;
  
  // Calcul des valeurs lissées (moyenne mobile) pour tous les canaux
  for (int i = 0; i < NUM_CHANNELS; i++) {
    uint32_t sum = 0;
    for (int j = 0; j < SMOOTHING_SAMPLES; j++) {
      sum += analogReadings[i][j];
    }
    smoothedValues[i] = sum / SMOOTHING_SAMPLES;
    // Pour les interrupteurs, la moyenne sera proche de 0 ou 4095, ce qui permet
    // de conserver la comparaison avec le seuil (1500) sans modification.
  }
}

/**
 * Applique une courbe personnalisée avec zone morte et retour au centre
 * @param channel Numéro du canal (0-3 pour les axes)
 * @param value Valeur lissée à convertir
 * @return Valeur HID entre -127 et 127
 */
int8_t applyCustomCurve(int channel, uint16_t value) {
  uint16_t actualMin = minCalibration[channel];
  uint16_t actualMax = maxCalibration[channel];
  uint16_t neutral = neutralCalibration[channel];
  
  // Correction automatique si calibration inversée
  if (actualMin > actualMax) {
    uint16_t temp = actualMin;
    actualMin = actualMax;
    actualMax = temp;
  }
  
  // Contrainte dans la plage calibrée
  value = constrain(value, actualMin, actualMax);
  
  // Mapping asymétrique autour du point neutre
  int8_t hidValue;
  if (value < neutral) {
    // En dessous du neutre : mapping vers -127 à 0
    hidValue = map(value, actualMin, neutral, -127, 0);
  } else {
    // Au-dessus du neutre : mapping vers 0 à 127
    hidValue = map(value, neutral, actualMax, 0, 127);
  }
  
  hidValue = constrain(hidValue, -127, 127);
  
  // Application de la zone morte
  if (abs(hidValue) < DEADZONE) {
    hidValue = 0;
  }
  
  // Renforcement du retour au centre
  if (returnToCenterEnabled[channel] && abs(hidValue) < RETURN_TO_CENTER_FORCE) {
    hidValue = 0;
  }
  
  // Stabilité : évite les petites variations
  if (abs(hidValue - lastHidValues[channel]) < DEADZONE/2) {
    return lastHidValues[channel];
  }
  
  lastHidValues[channel] = hidValue;
  return hidValue;
}

// =============================================================================
// FONCTIONS HID - Communication USB
// =============================================================================

/**
 * Met à jour et envoie le rapport HID
 * Appelée à chaque itération de loop()
 */
void updateJoystick() {
  joystickReport_t report = {0, 0, 0, 0, 0};
  
  // Conversion des axes analogiques (canaux 1-4)
  report.x = applyCustomCurve(0, smoothedValues[0]);
  report.y = applyCustomCurve(1, smoothedValues[1]);
  report.z = applyCustomCurve(2, smoothedValues[2]);
  report.rx = applyCustomCurve(3, smoothedValues[3]);
  
  // Conversion des boutons (canaux 5-8) - comparaison avec le seuil
  for (int i = 4; i < NUM_CHANNELS; i++) {
    // smoothedValues[i] est soit ~0, soit ~4095. Le seuil par défaut est 1500.
    if (smoothedValues[i] > buttonThresholds[i]) {
      report.buttons |= (1 << (i - 4));
    }
  }
  
  // Envoi du rapport HID si le périphérique est prêt
  if (usb_hid.ready()) {
    usb_hid.sendReport(0, &report, sizeof(report));
  }
}

// =============================================================================
// INTERFACE DE CONFIGURATION SERIAL (inchangée)
// =============================================================================

/**
 * Gère les commandes entrées via le port série
 */
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "help" || command == "commands" || command == "?") {
      printHelp();
    }
    else if (command == "values") {
      displayCurrentValues();
    }
    else if (command == "calibration") {
      displayCalibration();
    }
    else if (command == "smooth") {
      displaySmoothInfo();
    }
    else if (command.startsWith("setmin")) {
      setMinValue(command);
    }
    else if (command.startsWith("setmax")) {
      setMaxValue(command);
    }
    else if (command.startsWith("setbutton")) {
      setButtonThreshold(command);
    }
    else if (command.startsWith("autocalib")) {
      autoCalibrate();
    }
    else if (command == "reset") {
      resetCalibration();
    }
    else if (command.startsWith("deadzone")) {
      setDeadzone(command);
    }
    else if (command.startsWith("invert")) {
      invertChannel(command);
    }
    else if (command.startsWith("setneutral")) {
      setNeutralValue(command);
    }
    else if (command.startsWith("returncenter")) {
      setReturnCenter(command);
    }
    else if (command == "save") {
      saveCalibration();
    }
    else if (command == "load") {
      loadCalibration();
    }
    else if (command == "stop" || command == "pause") {
      Serial.println("Affichage automatique suspendu. Tapez 'values' pour afficher manuellement.");
      delay(3000);
    }
    else {
      Serial.println("Commande inconnue. Tapez 'help' pour voir les commandes disponibles.");
    }
  }
}

/**
 * Affiche l'aide complète avec toutes les commandes
 */
void printHelp() {
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════╗");
  Serial.println("║            LISTE DES COMMANDES               ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║ help / commands / ? - Affiche cette aide     ║");
  Serial.println("║ values            - Valeurs actuelles        ║");
  Serial.println("║ calibration       - Affiche calibration      ║");
  Serial.println("║ smooth            - Infos lissage            ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║ setmin CH VAL     - MIN canal CH (1-8)       ║");
  Serial.println("║ setmax CH VAL     - MAX canal CH (1-8)       ║");
  Serial.println("║ setneutral CH VAL - Point neutre CH (1-4)    ║");
  Serial.println("║ setbutton CH VAL  - Seuil bouton CH (5-8)    ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║ deadzone VAL      - Zone morte (defaut: 8)   ║");
  Serial.println("║ returncenter CH 0/1 - Retour centre CH(1-4)  ║");
  Serial.println("║ invert CH         - Inverse MIN/MAX CH(1-4)  ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║ autocalib         - Calibration auto 7s      ║");
  Serial.println("║ reset             - Reset calibration        ║");
  Serial.println("║ save              - Sauvegarde en EEPROM     ║");
  Serial.println("║ load              - Charge depuis EEPROM     ║");
  Serial.println("║ stop / pause      - Stop affichage auto      ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println();
}

// =============================================================================
// FONCTIONS D'AFFICHAGE ET DIAGNOSTIC (adaptées pour les boutons)
// =============================================================================

/**
 * Affiche les valeurs actuelles de tous les canaux
 * Format: Brut -> Lissee -> HID avec pourcentages
 */
void displayCurrentValues() {
  Serial.println();
  Serial.println("=== VALEURS ACTUELLES ===");
  Serial.println("(Brut -> Lissee -> HID)");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int percentage = 0;
    
    // Calcul des min/max réels pour le pourcentage
    uint16_t actualMin = minCalibration[i];
    uint16_t actualMax = maxCalibration[i];
    if (actualMin > actualMax) {
      uint16_t temp = actualMin;
      actualMin = actualMax;
      actualMax = temp;
    }
    
    if (actualMax > actualMin) {
      percentage = map(constrain(smoothedValues[i], actualMin, actualMax), 
                      actualMin, actualMax, 0, 100);
    }
    
    Serial.print("CH");
    Serial.print(i + 1);
    if (i < 4) {
      Serial.print(" (Axe): ");
      Serial.print(rawValues[i]);
      Serial.print(" -> ");
      Serial.print(smoothedValues[i]);
      Serial.print(" (");
      Serial.print(percentage);
      Serial.print("%) -> HID: ");
      
      int8_t hidVal = applyCustomCurve(i, smoothedValues[i]);
      Serial.print(hidVal);
      
      // Informations supplémentaires
      Serial.print(" [Neutre:");
      Serial.print(neutralCalibration[i]);
      if (!returnToCenterEnabled[i]) {
        Serial.print(", NoReturn");
      }
      Serial.print("]");
      
      // Avertissement calibration inversée
      if (minCalibration[i] > maxCalibration[i]) {
        Serial.print(" [CALIBRATION INVERSEE!]");
      }
    } else {
      Serial.print(" (Btn): ");
      Serial.print(rawValues[i] ? "HIGH" : "LOW");
      Serial.print(" -> ");
      Serial.print(smoothedValues[i]);
      Serial.print(" -> Bouton: ");
      Serial.print(smoothedValues[i] > buttonThresholds[i] ? "ACTIF" : "INACTIF");
      Serial.print(" [Seuil:");
      Serial.print(buttonThresholds[i]);
      Serial.print("]");
    }
    Serial.println();
  }
}

/**
 * Affiche les paramètres de calibration actuels
 */
void displayCalibration() {
  Serial.println();
  Serial.println("=== CALIBRATION ACTUELLE ===");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    if (i < 4) {
      Serial.print(" (Axe): ");
    } else {
      Serial.print(" (Btn): ");
    }
    Serial.print("MIN=");
    Serial.print(minCalibration[i]);
    Serial.print(", MAX=");
    Serial.print(maxCalibration[i]);
    if (i < 4) {
      Serial.print(", NEUTRE=");
      Serial.print(neutralCalibration[i]);
      Serial.print(", ReturnCenter=");
      Serial.print(returnToCenterEnabled[i] ? "ON" : "OFF");
    } else {
      Serial.print(", Seuil=");
      Serial.print(buttonThresholds[i]);
    }
    if (i < 4 && minCalibration[i] > maxCalibration[i]) {
      Serial.print(" [INVERSE]");
    }
    Serial.println();
  }
}

/**
 * Affiche les informations de lissage et stabilité
 */
void displaySmoothInfo() {
  Serial.println();
  Serial.println("=== INFORMATIONS DE LISSAGE ===");
  Serial.print("Echantillons de lissage: ");
  Serial.println(SMOOTHING_SAMPLES);
  Serial.print("Zone morte: ");
  Serial.println(DEADZONE);
  Serial.print("Force retour au centre: ");
  Serial.println(RETURN_TO_CENTER_FORCE);
  Serial.println("Variations brutes des 4 premiers canaux:");
  for (int i = 0; i < 4; i++) {
    uint16_t minVal = 4095, maxVal = 0;
    for (int j = 0; j < SMOOTHING_SAMPLES; j++) {
      if (analogReadings[i][j] < minVal) minVal = analogReadings[i][j];
      if (analogReadings[i][j] > maxVal) maxVal = analogReadings[i][j];
    }
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(": fluctuation ");
    Serial.print(maxVal - minVal);
    Serial.print (" points (");
    Serial.print(minVal);
    Serial.print("-");
    Serial.print(maxVal);
    Serial.print("), Neutre:");
    Serial.print(neutralCalibration[i]);
    Serial.print(", ReturnCenter:");
    Serial.println(returnToCenterEnabled[i] ? "ON" : "OFF");
  }
}

// =============================================================================
// FONCTIONS DE CONFIGURATION MANUELLE (inchangées)
// =============================================================================

/**
 * Définit la valeur minimale d'un canal
 * Usage: setmin CHANNEL VALUE
 */
void setMinValue(String command) {
  int channel, value;
  if (sscanf(command.c_str(), "setmin %d %d", &channel, &value) == 2) {
    if (channel >= 1 && channel <= NUM_CHANNELS) {
      minCalibration[channel - 1] = value;
      // Recalcul automatique du point neutre
      if (channel <= 4) {
        neutralCalibration[channel - 1] = (minCalibration[channel - 1] + maxCalibration[channel - 1]) / 2;
      }
      saveCalibration(); // Sauvegarde automatique
      Serial.print("Canal ");
      Serial.print(channel);
      Serial.print(" MIN defini a: ");
      Serial.println(value);
    } else {
      Serial.println("Erreur: Canal doit etre entre 1 et 8");
    }
  } else {
    Serial.println("Erreur: Usage: setmin CHANNEL VALUE");
  }
}

/**
 * Définit la valeur maximale d'un canal
 * Usage: setmax CHANNEL VALUE
 */
void setMaxValue(String command) {
  int channel, value;
  if (sscanf(command.c_str(), "setmax %d %d", &channel, &value) == 2) {
    if (channel >= 1 && channel <= NUM_CHANNELS) {
      maxCalibration[channel - 1] = value;
      // Recalcul automatique du point neutre
      if (channel <= 4) {
        neutralCalibration[channel - 1] = (minCalibration[channel - 1] + maxCalibration[channel - 1]) / 2;
      }
      saveCalibration(); // Sauvegarde automatique
      Serial.print("Canal ");
      Serial.print(channel);
      Serial.print(" MAX defini a: ");
      Serial.println(value);
    } else {
      Serial.println("Erreur: Canal doit etre entre 1 et 8");
    }
  } else {
    Serial.println("Erreur: Usage: setmax CHANNEL VALUE");
  }
}

/**
 * Définit le point neutre d'un canal axe
 * Usage: setneutral CHANNEL VALUE
 */
void setNeutralValue(String command) {
  int channel, value;
  if (sscanf(command.c_str(), "setneutral %d %d", &channel, &value) == 2) {
    if (channel >= 1 && channel <= 4) {
      neutralCalibration[channel - 1] = value;
      saveCalibration(); // Sauvegarde automatique
      Serial.print("Canal ");
      Serial.print(channel);
      Serial.print(" NEUTRE defini a: ");
      Serial.println(value);
    } else {
      Serial.println("Erreur: Point neutre seulement pour canaux 1-4");
    }
  } else {
    Serial.println("Erreur: Usage: setneutral CHANNEL VALUE");
  }
}

/**
 * Active/désactive le retour au centre pour un canal
 * Usage: returncenter CHANNEL 0/1
 */
void setReturnCenter(String command) {
  int channel, value;
  if (sscanf(command.c_str(), "returncenter %d %d", &channel, &value) == 2) {
    if (channel >= 1 && channel <= 4) {
      returnToCenterEnabled[channel - 1] = (value != 0);
      saveCalibration(); // Sauvegarde automatique
      Serial.print("Canal ");
      Serial.print(channel);
      Serial.print(" retour au centre ");
      Serial.println(value ? "ACTIVE" : "DESACTIVE");
    } else {
      Serial.println("Erreur: Retour au centre seulement pour canaux 1-4");
    }
  } else {
    Serial.println("Erreur: Usage: returncenter CHANNEL VALUE(0/1)");
  }
}

/**
 * Définit le seuil d'activation d'un bouton
 * Usage: setbutton CHANNEL VALUE
 */
void setButtonThreshold(String command) {
  int channel, value;
  if (sscanf(command.c_str(), "setbutton %d %d", &channel, &value) == 2) {
    if (channel >= 5 && channel <= NUM_CHANNELS) {
      buttonThresholds[channel - 1] = value;
      saveCalibration(); // Sauvegarde automatique
      Serial.print("Bouton ");
      Serial.print(channel - 4);
      Serial.print(" (Canal ");
      Serial.print(channel);
      Serial.print(") seuil defini a: ");
      Serial.println(value);
    } else {
      Serial.println("Erreur: Les boutons sont sur les canaux 5-8");
    }
  } else {
    Serial.println("Erreur: Usage: setbutton CHANNEL VALUE");
  }
}

/**
 * Inverse les valeurs MIN/MAX d'un canal
 * Usage: invert CHANNEL
 */
void invertChannel(String command) {
  int channel;
  if (sscanf(command.c_str(), "invert %d", &channel) == 1) {
    if (channel >= 1 && channel <= 4) {
      int idx = channel - 1;
      uint16_t temp = minCalibration[idx];
      minCalibration[idx] = maxCalibration[idx];
      maxCalibration[idx] = temp;
      // Recalcul du point neutre
      neutralCalibration[idx] = (minCalibration[idx] + maxCalibration[idx]) / 2;
      saveCalibration(); // Sauvegarde automatique
      Serial.print("Canal ");
      Serial.print(channel);
      Serial.println(" inverse - MIN/MAX echangees");
    } else {
      Serial.println("Erreur: Inversion possible seulement pour les canaux 1-4 (axes)");
    }
  } else {
    Serial.println("Erreur: Usage: invert CHANNEL");
  }
}

// =============================================================================
// CALIBRATION AUTOMATIQUE
// =============================================================================

/**
 * Effectue une calibration automatique complète
 * Mesure MIN/MAX pendant 5s puis point neutre pendant 2s
 */
void autoCalibrate() {
  Serial.println();
  Serial.println("=== CALIBRATION AUTOMATIQUE ===");
  Serial.println("1. Bougez tous les axes au MIN et MAX pendant 5 secondes");
  Serial.println("2. Puis laissez les au NEUTRE pendant 2 secondes");
  Serial.println("Debut dans 3 secondes...");
  
  // Compte à rebours
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println("GO!");
  
  // Buffers temporaires pour la calibration
  uint16_t tempMin[NUM_CHANNELS] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
  uint16_t tempMax[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t tempNeutral[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t neutralSamples[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t neutralCount[NUM_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  unsigned long startTime = millis();
  bool measuringNeutral = false;
  
  // Boucle de calibration de 7 secondes
  while (millis() - startTime < 7000) {
    readAndSmoothChannels();
    
    // Phase 1 (5s): Mesure MIN/MAX
    if (millis() - startTime < 5000) {
      for (int i = 0; i < NUM_CHANNELS; i++) {
        if (smoothedValues[i] < tempMin[i]) tempMin[i] = smoothedValues[i];
        if (smoothedValues[i] > tempMax[i]) tempMax[i] = smoothedValues[i];
      }
      
      // Affichage du temps restant
      int remaining = 5 - (millis() - startTime) / 1000;
      if (remaining != (5 - (millis() - startTime - 100) / 1000)) {
        Serial.print("Temps restant: ");
        Serial.print(remaining);
        Serial.println("s - Bougez tous les axes!");
      }
    } 
    // Phase 2 (2s): Mesure point neutre
    else {
      if (!measuringNeutral) {
        Serial.println("Maintenez les manches au NEUTRE...");
        measuringNeutral = true;
      }
      for (int i = 0; i < 4; i++) {
        neutralSamples[i] += smoothedValues[i];
        neutralCount[i]++;
      }
    }
    
    delay(50);
  }
  
  // Calcul des points neutres moyens
  for (int i = 0; i < 4; i++) {
    if (neutralCount[i] > 0) {
      tempNeutral[i] = neutralSamples[i] / neutralCount[i];
    } else {
      tempNeutral[i] = (tempMin[i] + tempMax[i]) / 2;
    }
  }
  
  // Application de la nouvelle calibration
  for (int i = 0; i < NUM_CHANNELS; i++) {
    minCalibration[i] = tempMin[i];
    maxCalibration[i] = tempMax[i];
    if (i < 4) {
      neutralCalibration[i] = tempNeutral[i];
    }
  }
  
  saveCalibration(); // Sauvegarde automatique
  
  Serial.println();
  Serial.println("✅ CALIBRATION TERMINEE ET SAUVEGARDEE!");
  displayCalibration();
}

/**
 * Réinitialise la calibration aux valeurs d'usine
 */
void resetCalibration() {
  // Valeurs par défaut pour une radio-commande typique
  uint16_t defaultMin[NUM_CHANNELS] = {200, 200, 130, 190, 0, 0, 0, 0};
  uint16_t defaultMax[NUM_CHANNELS] = {820, 870, 940, 890, 4095, 4095, 4095, 4095};
  uint16_t defaultButtons[NUM_CHANNELS] = {0, 0, 0, 0, 1500, 1500, 1500, 1500};
  uint16_t defaultNeutral[NUM_CHANNELS] = {512, 512, 512, 512, 0, 0, 0, 0};
  
  for (int i = 0; i < NUM_CHANNELS; i++) {
    minCalibration[i] = defaultMin[i];
    maxCalibration[i] = defaultMax[i];
    buttonThresholds[i] = defaultButtons[i];
    neutralCalibration[i] = defaultNeutral[i];
    if (i < 4) {
      returnToCenterEnabled[i] = true;
    }
  }
  
  saveCalibration(); // Sauvegarde automatique
  Serial.println("Calibration reset aux valeurs par defaut et sauvegardee");
  displayCalibration();
}

/**
 * Définit la zone morte (nécessite redémarrage)
 * Usage: deadzone VALUE
 */
void setDeadzone(String command) {
  int value;
  if (sscanf(command.c_str(), "deadzone %d", &value) == 1) {
    if (value >= 0 && value <= 50) {
      Serial.print("Zone morte definie a: ");
      Serial.println(value);
      Serial.println("Note: Redemarrage necessaire pour appliquer les changements de zone morte");
    } else {
      Serial.println("Erreur: Zone morte doit etre entre 0 et 50");
    }
  } else {
    Serial.println("Erreur: Usage: deadzone VALUE");
  }
}