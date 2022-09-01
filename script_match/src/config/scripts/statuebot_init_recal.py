#!/usr/bin/env python
# coding=utf-8

from config.actions_2022 import *
from numpy import pi

action_sequence = [
    {
        "description": "On free les moteurs",
        "function": motion_free,
        "params": {}
    },
    {
        "description": "on change la couleur en bleu",
        "function": ledrgb,
        "params": {
            "r": 0,
            "g": 0,
            "b": 255
        }
    },
        {
        "description": "On clear l'ecran",
        "function": print_lcd,
        "params": {
            "line": 2,
            "text": "                "
        }
    },
    {
        "description": "On clear l'ecran",
        "function": print_lcd,
        "params": {
            "line": 1,
            "text": "                "
        }
    },
    {
        "description": "On enable les sick",
        "function": sick_enable,
        "params": {
            "avant": False,
            "arriere": False
        }
    },
    {
        "description": "On init les AX",
        "function": init_ax,
        "params": {}
    },
    {
        "description": "on attend la demande d'initilisation",
        "function": wait_init,
        "params": {}
    },
    {
        "description": "on attend la demande de recalage",
        "function": wait_recal,
        "params": {}
    },
    {
        "description": "set origine x",
        "function": set_x,
        "params": {
            "x": 0
        }
    },
    {
        "description": "set origine y",
        "function": set_y,
        "params": {
            "y": 0
        }
    },
    {
        "description": "set origine t",
        "function": set_t,
        "params": {
            "t": 0
        }
    },
    {
        "description": "On se colle à l'axe des Y pour recalage en X",
        "function": move_speed,
        "params": {
            "v": -0.13,
            "time": 1.5
        }
    },
    {
        "description": "prise origine x",
        "function": set_x,
        "params": {
            "x": 0.089
        }
    },
    {
        "description": "angle de calage",
        "function": set_t,
        "params": {
            "t": 0
        }
    },
    {
        "description": "on se recule un peu",
        "function": moveseg_xy,
        "params": {
            "x": 0.225,
            "y": 0
        }
    },
    {
        "description": "On se tourne vers la bordure de l'axe des X",
        "function": angle,
        "params": {
            "theta": 1.51
        }
    },
    {
        "description": "On se colle à l'axe des X pour recalage en Y",
        "function": move_speed,
        "params": {
            "v": 0.13,
            "time": 6
        }
    },
    {
        "description": "prise origine y",
        "function": set_y,
        "params": {
            "y": -0.089 #A verif
        }
    },
    {
        "description": "On affiche le message de fin de recalage",
        "function": print_lcd,
        "params": {
            "line": 1,
            "text": "Recalage termine"
        }
    },
    {
        "description": "On affiche le message de fin de recalage",
        "function": print_lcd,
        "params": {
            "line": 2,
            "text": " Attente  match"
        }
    },
    {
        "description": "On signal que le robot est prêt",
        "function": ledrgb,
        "params": {
            "r": 0,
            "g": 255,
            "b": 0
        }
    },
        {
        "description": "on attend 2 secondes avant de mettre les leds à la couleur de l'équipe",
        "function": tempo,
        "params": {
            "time": 2
        }
    }
]