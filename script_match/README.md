script\_match
=============

Cette node se charge des scripts de match. Pour la configurer correctement il faut:
 1. regler les constantes dans `config/actions.py`, notamment la couleur par défaut.
 2. définir les symmétries des actions dans `config/symmetries.py`
 3. définir les actions dans `config/actions.py`. Voir les commentaires du code pour plus d'infos.
 4. definir le script de match à éxecuter dans la node `bringup/`