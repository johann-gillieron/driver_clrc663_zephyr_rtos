# Driver CLRC663 pour Zephyr RTOS (FR)

Ce dépôt contient un driver permettant d'utiliser le lecteur NFC CLRC663 en SPI avec Zephyr RTOS via une API NFC simple. Il est conçu pour la version `3.5.99` du noyau Zephyr.

Ce driver a été développé dans le cadre d'un travail de Bachelor pour lire et écrire des tags NFC DESFire à l'aide du CLRC663.

## Fonctionnalités de l'API NFC (nfc.h)

L'API fournie, `zephyr/drivers/nfc.h`, propose les fonctionnalités suivantes :

- **Activer le champ RF** : Le champ RF est désactivé après l'initialisation de la puce.
- **Désactiver le champ RF**.
- **Sélectionner et activer un tag NFC ISO14443A** : Retourne la longueur de l'UID, l'UID, le SAK, l'ATQA, et l'ATS si le PICC est conforme à ISO14443-4.
- **Communiquer directement avec le PICC** en utilisant le protocole ISO14443-4.
- **Calibrer le LCPD (Low Power Card Detection)**.
- **Activer la fonction LCPD** : Un interrupteur se déclenche sur la broche IRQ physique de la puce lorsqu'un PICC est détecté.

## Utilisation

Le reste du travail de bachelor peut être utilisé comme exemple ce trouve [ici](https://github.com/johann-gillieron/acces_securite_par_badge_rfid).

## Installation

Pour installer ce driver, veuillez suivre les étapes ci-dessous :

1. Clonez ce dépôt dans votre espace de travail Zephyr.
   ```sh
   git clone https://github.com/johann-gillieron/driver_clrc663_zephyr_rtos.git

2. Ajoutez le driver à la racine de Zephyr en modifiant les fichiers de configuration appropriés:
    - Ajoutez "add_subdirectory_ifdef(CONFIG_CLRC663 clrc663)" à la fin du fichier drivers/CMakeLists.txt
    - Ajoutez "source "drivers/clrc663/Kconfig"" à la fin du fichier drivers/Kconfig

3. Compilez et flashez votre projet sur le matériel cible.

# Driver CLRC663 for Zephyr RTOS (EN)

This repository contains a driver for using the CLRC663 NFC reader via SPI with Zephyr RTOS using a simple NFC API. It was designed for Zephyr kernel version `3.5.99`.

This driver was developed as part of a Bachelor's thesis to read and write DESFire NFC tags using the CLRC663.

## NFC API Features (nfc.h)

The provided API, `zephyr/drivers/nfc.h`, offers the following features:

- **Enable RF field**: The RF field is disabled after chip initialization
- **Disable RF field**.
- **Select and activate an ISO14443A NFC tag**: Returns the UID length, UID, SAK, ATQA, and ATS if the PICC is ISO14443-4 compliant.
- **Communicate directly with the PICC** using the ISO14443-4 protocol.
- **Calibrate LCPD (Low Power Card Detection)**.
- **Enable LCPD**: A switch is triggered on the chip's physical IRQ pin when a PICC is detected.

## Usage

The rest of the bachelor's project can be used as an example and can be found [here](https://github.com/johann-gillieron/acces_securite_par_badge_rfid).

## Installation

To install this driver, please follow the steps below:

1. Clone this repository into your Zephyr workspace.
   ```sh
   git clone https://github.com/johann-gillieron/driver_clrc663_zephyr_rtos.git

2. Add the driver to the Zephyr root directory by modifying the appropriate configuration files
   - Add "add_subdirectory_ifdef(CONFIG_CLRC663 clrc663)" to the end of the drivers/CMakeLists.txt file
   - Add "source "drivers/clrc663/Kconfig"" to the end of the drivers/Kconfig file

3. Compile and flash your project to the target hardware.
