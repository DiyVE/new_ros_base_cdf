#!/usr/bin/env python
# coding=utf-8

from tokenize import Pointfloat
from xmlrpc.client import Boolean
import rospy
from std_msgs.msg import String, Bool, Int16
from geometry_msgs.msg import Point
from config.symmetries import ax12_symmetry
from cdf_msgs.msg import Pic_Action
import numpy as np

import time

DEFAULT_TEAM = "Jaune"  # couleur par défaut, chaîne de caractères exacte envoyée par le PIC
DEFAULT_TEMPO = 0.2  # tempo entre deux actions
DEFAULT_TIMEOUT = 20  # delai pour dire que l'action a échouée (pas vraiment utilisé dans cette version du code )

"""
Dans la suite du code env correspond a un objet MatchEnvironment ( défini dans match_handler.py ), 
Cet objet contient les infos dont chaque fonction a besoin. Il dispose des attributs suivantes
env.script_name
env.state_team
env.TEAM_factor
env.state_status_match
env.state_motion_done
env.state_ax12_status
env.score_pub

et des fonctions suivantes:
env.update_team() => met a jour env.state_team et renvoie sa nouvelle valeur
env.update_team_factor() => met a jour env.TEAM_factor et renvoie sa nouvelle valeur


Les actions sont donc des fonctions qui prennent en paramètre:
- l'environment décrit précédemment ( nommé env et donné en premier paramètre )
- d'autres paramètres, qui seront donnés dans les scripts de matchs

Ce sont les actions qui permettent de temporiser les scripts de matchs. Il faut donc que les fonctions soient
bloquantes le temps que l'action se réalise. Pour se faire, il y a deux possibilités:
 1. un bon vieux sleep
 2. utiliser la fonction rospy.wait_for_message pour bloquer an attendant qu'un message arrive sur un topic
"""

#####################################################
# ACTIONS
#####################################################

def tempo(env, time):
    """
    Fonction bloquante pendant <time> secondes

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - time (float) : Le temps d'attente en secondes
    """
    rospy.loginfo("waiting %s secs" % time)
    rospy.sleep(time)

def score(env, plus_score):
    """
    Fonction qui change et affiche le score sur le lcd du PIC

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - s (int) : Le nombre de point à ajouter au score
    """
    newscore = env.score + plus_score
    rospy.loginfo("score: %s" % newscore)
    print_lcd(env, 1, "             ")
    print_lcd(env, 2, "Score: %d" % newscore)
    env.score = newscore

def wait_init(env):
    """
    Fonction permettant de gérer la sélection de l'équipe en phase d'initialisation

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
    """
    rospy.loginfo("waiting Init button press")
    buttons = rospy.wait_for_message(
        env.topic_dict["buttons"],
        String)
    print_lcd(env, 1, "Choix Equipe :")
    print_lcd(env, 2, "   Jaune   ")
    while buttons.data[3] != "1":
        buttons = rospy.wait_for_message(
            env.topic_dict["buttons"],
            String)
        env.state_team = "Jaune" if int(buttons.data[2]) == 0 else "Violet"
        rospy.loginfo("Team changed to %s" % env.state_team)
        if env.state_team == "Jaune":
            print_lcd(env, 2, "   Jaune   ")
            ledrgb(env, 255, 255, 0)
        elif env.state_team == "Violet":
            ledrgb(env, 127, 0, 255)
            print_lcd(env, 2, "   Violet   ")
    env.update_team_factor()
    rospy.loginfo("Init Buttons Pressed with team %s!" % env.state_team)
        

def wait_recal(env):
    """
    Fonction attendant l'ordre de démarrage du recalage

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
    """
    rospy.loginfo("waiting Recal button press")
    buttons = rospy.wait_for_message(
        env.topic_dict["buttons"],
        String)
    while buttons.data[4] != "1":
        buttons = rospy.wait_for_message(
            env.topic_dict["buttons"],
            String)
    ledrgb(env, 0, 0, 255)
    rospy.loginfo("Recal Buttons Pressed !")

def init_ax(env):
    """
    Fonction qui initialise les axes du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
    """
    rospy.loginfo("restart timer")
    msg = Pic_Action()

    msg.action_destination = 'motor'
    msg.action_msg = "restarttimer"
    env.action_pub.publish(msg)

    rospy.loginfo("init ax12")
    msg.action_msg = "INITAX"
    env.action_pub.publish(msg)
    rospy.sleep(4)
    for command in []: # Remplir la liste avec les actions a réaliser à l'initialisation
        succes = False
        rospy.loginfo("doing AX12Action: %s" % command)
        msg.action_msg = command
        env.action_pub.publish(msg)
    


def wait_laisse(env):
    """
    Attend que la laisse soit déclenchée et démarre les timers

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
    """
    ledrgb(env, 255, 255, 0) if env.state_team == DEFAULT_TEAM else ledrgb(env, 127, 0, 255)

    rospy.loginfo("waiting the start of the match")

    buttons = rospy.wait_for_message(
        env.topic_dict["buttons"],
        String)
    while buttons.data[1] != "0":
        buttons = rospy.wait_for_message(
            env.topic_dict["buttons"],
            String)
        if buttons.data[5] == "1":
            rospy.loginfo("reset button pressed !")
            raise rospy.ROSInterruptException
    ledrgb(env, 255, 0, 0)

    # On démarre les timers
    msg = Pic_Action()
    msg.action_destination = 'user'
    msg.action_msg = "starttimer"
    env.action_pub.publish(msg)
    msg.action_destination = 'motor'
    msg.action_msg = "starttimer"
    env.action_pub.publish(msg)

    # On mémorise le temps de départ
    env.start_timestamp = time.time()

    rospy.loginfo("Match started !")

def check_position(env, status, raise_on_evitement=False):
    """
    Vérification de la position du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - status (bool) : True si on veut vérifier que le robot est à la position attendue, False si on veut vérifier que le robot n'est pas à la position attendue
    """
    try:
        dt = time.time() - env.start_timestamp
    except:
        dt = -1.
    if (dt > 95) and (raise_on_evitement):
        rospy.logerr("90s with raise_on_evitement=True")
        raise RuntimeError("90s triggered")
    try:
        rospy.loginfo("checking position")
        action_done = rospy.wait_for_message(
            env.topic_dict["motion_done"],
            Bool, timeout=DEFAULT_TIMEOUT)
        while action_done.data != status:
            action_done = rospy.wait_for_message(
                env.topic_dict["motion_done"],
                Bool, timeout=DEFAULT_TIMEOUT)
            if env.robot_stopped == True:
                if raise_on_evitement:
                    rospy.logerr("evitement triggered wiht raise_on_evitement=True")
                    raise RuntimeError("evitement triggered")
                else:
                    while env.robot_stopped == True:
                        pass
                    return False
        return status
    except rospy.exceptions.ROSException as e:
        if rospy.is_shutdown():
            raise e
        rospy.loginfo("action timed out !")
        return False

def chainedmove(env, x, y, raise_on_evitement=False):
    success = False
    while not success:
        rospy.loginfo("chainedmove to %s, %s" % (x, y))
        env.nav_goal_pub.publish(Point(x, y, 0))
        success = check_position(env, False, raise_on_evitement)
        success = check_position(env, True, raise_on_evitement) and not success

def moveseg_xy(env, x, y, raise_on_evitement=False):
    '''
    Fonction qui envoi une commande de mouvement rectiligne à une position x, y

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - x (float) : La position en x
        - y (float) : La position en y
    '''
    success = False
    while not success:
        rospy.loginfo("moving to %.3f, %.3f" % (x, (y*env.TEAM_factor)))
        msg = Pic_Action()
        # todo: check format commande
        msg.action_destination = 'motor'
        msg.action_msg = "MOVESEG %.3f %.3f" % (x, y*env.TEAM_factor)
        env.action_pub.publish(msg)
        success = check_position(env, False, raise_on_evitement)
        success = check_position(env, True, raise_on_evitement) and not success

def move_arc(env, x, y):
    """
    Fonction qui envoi une commande de mouvement circulaire à une position x, y

    Paramètres
    ----------

        - env (<MatchEnvironment>) : L'environment du match
        - x (float) : La position en x
        - y (float) : La position en y
    """
    success = False
    while not success:
        rospy.loginfo("moving arc to %.3f, %.3f" % (x, (y*env.TEAM_factor)))
        msg = Pic_Action()
        # todo; check TEAM factor ( sur x ou y? )
        # todo: check format commande
        msg.action_destination = 'motor'
        msg.action_msg = "MOVE %.3f %.3f" % (x, y*env.TEAM_factor)
        env.action_pub.publish(msg)
        rospy.sleep(DEFAULT_TEMPO)
        success = check_position(env, False)
        success = check_position(env, True) and not success

def angle(env, theta):
    """
    Fonction qui envoi une commande d'angle au robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - theta (float) : L'angle à atteindre
    """
    success = False
    while not success:
        rospy.loginfo("changing angle to %.3f" % (theta * env.TEAM_factor))
        msg = Pic_Action()
        msg.action_destination = 'motor'
        msg.action_msg = "ANGLE %.3f" % (theta * env.TEAM_factor)
        env.action_pub.publish(msg)
        rospy.sleep(DEFAULT_TEMPO)
        success = check_position(env, False)
        success = check_position(env, True) and not success

def move_speed(env, v, time):
    """
    Fonction qui dépace le robot à une vitesse `v` pendant `time` secondes

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - v (float) : La vitesse de déplacement
        - time (float) : Le temps de déplacement
    """
    rospy.loginfo("move speed at v:%.3f pendant:%.3f" % (v, time))
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "SPEED %.3f 0" % v
    env.action_pub.publish(msg)
    rospy.sleep(time)
    motion_free(env)

def move_endstop(env, v):
    """
    Fonction qui dépace le robot à une vitesse `v` jusqu'à ce que les endstops soient activés

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - v (float) : La vitesse de déplacement
    """
    rospy.loginfo("move speed at v:%.3f " % v)
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "SPEED %.3f 0" % v
    env.action_pub.publish(msg)
    success = check_position(env, False)
    success = check_position(env, True) and not success
    motion_free(env)

def motion_free(env):
    """
    Fonction permettant de libérer les moteurs du robot

    Paramètres
    ----------

        - env (<MatchEnvironment>) : L'environment du match
    """
    rospy.loginfo("motion free")
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "FREE"
    env.action_pub.publish(msg)
    rospy.sleep(DEFAULT_TEMPO)


def set_t(env, t):
    """
    Fonction permettant de recalibrer l'angle du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - t (float) : L'angle à atteindre
    """
    rospy.loginfo("set_t %.3f" % (t *env.TEAM_factor))
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "SETT %.3f" % (t * env.TEAM_factor)
    env.action_pub.publish(msg)
    rospy.sleep(DEFAULT_TEMPO)


def set_y(env, y):
    """
    Fonction permettant de recalibrer la position en x du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - x (float) : La position en x à atteindre
    """
    rospy.loginfo("set_y %.3f" % (y * env.TEAM_factor))
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "SETY %.3f" % (y*env.TEAM_factor)
    env.action_pub.publish(msg)
    rospy.sleep(DEFAULT_TEMPO)


def set_x(env, x):
    """
    Fonction permettant de recalibrer la position en y du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - y (float) : La position en y à atteindre
    """
    rospy.loginfo("set_x %.3f" % x)
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "SETX %.3f" % x
    env.action_pub.publish(msg)
    rospy.sleep(DEFAULT_TEMPO)


def set_v(env, v):
    """
    Fonction permettant de régler la vitesse max de déplacement rectiligne du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - v (float) : La vitesse de déplacement
    """
    rospy.loginfo("set_v %.3f" % v)
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "VMAX %.3f" % v
    env.action_pub.publish(msg)
    rospy.sleep(DEFAULT_TEMPO)

def set_vt(env, vt):
    """
    Fonction permettant de régler la vitesse max de rotation du robot

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - v (float) : La vitesse de déplacement
    """
    rospy.loginfo("set_vt %.3f" % vt)
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "VTMAX %.3f" % vt
    env.action_pub.publish(msg)
    rospy.sleep(DEFAULT_TEMPO)

def do_AX12action(env, command, timeout=DEFAULT_TIMEOUT):
    """
    Execute une commande ax12, bloque tant que l'action n'est pas terminée, retente sur timeout

    Paramètres
    ----------
        - command (str) : commande a executer ( chaine de caracteres sans le `\n` )
        - time (float) : temps d'attente avant de passer a l'action suivante
    """
    succes = False
    while not succes:
        rospy.loginfo("doing AX12Action: %s" % command)
        msg = Pic_Action()
        msg.action_destination = 'user'
        msg.action_msg = command if env.state_team == DEFAULT_TEAM else ax12_symmetry[command]
        env.action_pub.publish(msg)
        succes = check_AX12action(env, command, timeout)

def check_AX12action(env, command, timeout=DEFAULT_TIMEOUT):
    """
    Retourne la valeur True si l'action AX12 s'est bien passée

    Paramètres
    ----------
        - env (<MatchEnvironment>) : L'environment du match
        - command (str) : commande ax a executer
        - time (float) : temps d'attente avant de passer a l'action suivante
    
    Return
    ------
        Retourne la valeur True si l'action AX12 s'est bien passée
    """
    try:
        command = command if env.state_team == DEFAULT_TEAM else ax12_symmetry[command]
        rospy.loginfo("checking AX12Action: %s" % command)
        action_done = rospy.wait_for_message(
            env.topic_dict["ax12_status"],
            Bool, timeout=timeout)
        return action_done

    except rospy.exceptions.ROSException as e:
        if rospy.is_shutdown():
            raise e
        rospy.loginfo("action timed out !")
        return False
        
def ledrgb(env, r, g, b, num=None):
    """
    Fonction qui permet de changer la couleur des LEDs

    Paramètres
    ----------
        - env (<MatchEnvironment>) : l'environnement du match
        - r (int): Valeur de rouge(0-255)
        - g (int) : Valeur de vert(0-255)
        - b (int) : Valeur de bleu(0-255)
        - num (int) : Numéro de la LED a modifier(1-5)
    """
    msg = Pic_Action()
    msg.action_destination = 'user'

    if num == None:
        msg.action_msg = "ledrgb %d %d %d" % (r, g, b)
    else:
        msg.action_msg = "ledrgb %d %d %d %d" % (r, g, b, num)
    
    env.action_pub.publish(msg)

def print_lcd(env, line, text):
    """
    Fonction qui permet d'afficher du texte sur le LCD

    Paramètres
    ----------
        - env (<MatchEnvironment>) : l'environnement du match
        - line (int) : Ligne sur laquelle afficher le texte (1 ou 2)
        - text (str) : Texte à afficher
    """
    msg = Pic_Action()
    msg.action_destination = 'user'
    msg.action_msg = 'screen ' + str(line) + ' "' + str(text) + '"' # Petite manip pour mettre les guillemets
    env.action_pub.publish(msg)

def sick_enable(env, avant, arriere):
    """
    Fonction qui active les sick avant et/ou arriere

    Paramètres
    ----------
        - env (<MatchEnvironment>) : l'environnement du match
        - avant (bool) : True si on active les sick avant
        - arriere (bool) : True si on active les sick arriere
    """
    msg = Pic_Action()
    msg.action_destination = 'motor'
    msg.action_msg = "ensick %d %d" % (int(avant), int(arriere))
    env.action_pub.publish(msg)

def boucle_fin_match(env):
    """
    Fonction qui permet de faire une boucle infinie pouvant être déclenchée à la fin du match

    Paramètres
    ----------
        - env (<MatchEnvironment>) : l'environnement du match
    """
    while not rospy.is_shutdown():
        pass

def danse(env):
    msg = Pic_Action()
    msg.action_destination = 'user'
    msg.action_msg = "danse"
    env.action_pub.publish(msg)