# Driver CLRC663 pour Zephyr RTOS

Ce dépôt contient un driver permettant d'utiliser le lecteur NFC CLRC663 en SPI avec Zephyr RTOS via une API NFC simple. Il est conçu pour la version 3.5.99 du noyau Zephyr.

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

Un exemple d'utilisation sera bientôt disponible.

## Installation

Pour installer ce driver, veuillez suivre les étapes ci-dessous :

1. Clonez ce dépôt dans votre espace de travail Zephyr.
   ```sh
   git clone <url_du_dépôt>

2. Ajoutez le driver à la racine de Zephyr en modifiant les fichiers de configuration appropriés:
    - Ajoutez "add_subdirectory_ifdef(CONFIG_CLRC663 clrc663)" à la fin du fichier drivers/CMakeLists.txt
    - Ajoutez "source "drivers/clrc663/Kconfig"" à la fin du fichier drivers/Kconfig

4. Compilez et flashez votre projet sur le matériel cible.

## Contributions

Les contributions sont les bienvenues. Veuillez soumettre des pull requests ou ouvrir des issues pour signaler des bugs ou proposer des améliorations.

## Licence
Ce projet est sous licence MIT.
