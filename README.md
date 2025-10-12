# Joystick USB HID pour Radio-Commande avec un RP2040

Transformez votre radio-commande en joystick USB professionnel pour simulateurs de vol.
## ✨ Installation rapide sur le RP2040

1. Branchez votre RP2040 en préssant le bouton BOOT, votre RP2040 est reconnu comme un lecteur, une fenêtre de l'explorateur windows s'ouvre sur le lecteur.
2. Copier le fichier joystickrp2040comments.ino.uf2 préalablement téléchargé sur ce lecteur. Ca y est c'est fait!
   
## ✨ Fonctionnalités

- Support 8 canaux (4 axes + 4 boutons)
- Lissage avancé du signal
- Calibration manuelle et automatique  
- Sauvegarde EEPROM des paramètres
- Retour au centre personnalisable
- Interface de configuration série

## 🚀 Utilisation rapide

1. Branchez votre radio-commande
2. Ouvrez le moniteur série (115200 bauds)
3. Tapez `autocalib` pour calibration automatique
4. Utilisez `values` pour vérifier les valeurs

## 📖 Commandes principales

- `autocalib` - Calibration automatique 7 secondes
- `values` - Affiche les valeurs actuelles
- `setmin/setmax` - Configuration manuelle
- `reset` - Reset usine

## 🔧 Brochage

| Canal | Broche | Type      |
|-------|--------|-----------|
| 1     | 26     | Axe X     |
| 2     | 27     | Axe Y     |
| 3     | 28     | Axe Z     |
| 4     | 29     | Axe RX    |
| 5-8   | 0-3    | Boutons   |
