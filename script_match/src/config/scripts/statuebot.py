#!/usr/bin/env python
# coding=utf-8

from config.actions_2022 import *
from config.scripts.statuebot_init_recal import action_sequence as init_recal
from numpy import pi

action_sequence = init_recal + [
    {
        "description": "on attend que la laisse soit déclanchée",
        "function": wait_laisse,
        "params": {}
    },
    {
        "description": "On lance la boucle infinie d'attente",
        "function": boucle_fin_match,
        "params": {}
    }
]