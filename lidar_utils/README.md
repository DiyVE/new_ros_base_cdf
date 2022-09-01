lidar\_utils
============

Cette node contient le code de l'évitement au lidar.

Setup
-----

Le lidar est branché à la raspi, la raspi calcule la présence d'obstacle. Puis les gpios de la raspi sont utilisées 
pour simuler une detection ultrason au niveau de la carte. Il y a deux cables, un entre la rpi et le connecteur US
avant, et un entre la rpi et le connecteur US arrière.

avantages:
 - plus de réactivité qu'avec la communication standard
 - du coté pic, l'évitement repose toujours sur le même code.

inconvénients:
 - on utilise deux gpios de la rpi 