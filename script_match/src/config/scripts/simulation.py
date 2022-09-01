#!/usr/bin/env python
# coding=utf-8

from config.actions_2022 import *

action_sequence = [
    {
        "description": "on attend 5 secondes",
        "function": tempo,
        "params": {
            "time": 5
        }
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
        "description": "On va en x=2.5 et y=1",
        "function": chainedmove,
        "params": {
            "x": 2.5,
            "y": 1
        }
    },
    {
        "description": "On change la couleur en rouge",
        "function": ledrgb,
        "params": {
            "r": 255,
            "g": 0,
            "b": 0
        }
    },
    {
        "description": "On va en x=1 et y=1.5",
        "function": chainedmove,
        "params": {
            "x": 1,
            "y": 1.5
        }
    },
    {
        "description": "On va en x=1 et y=1.5",
        "function": chainedmove,
        "params": {
            "x": 2,
            "y": 1.75
        }
    },
    {
        "description": "On change la couleur en vert",
        "function": ledrgb,
        "params": {
            "r": 0,
            "g": 255,
            "b": 0
        }
    },
    {
        "description": "On rentre a la maison",
        "function": chainedmove,
        "params": {
            "x": 0,
            "y": 0
        }
    },
    {
        "description": "tempo tres longue",
        "function": tempo,
        "params": {
            "time": 100
        }
    }
]