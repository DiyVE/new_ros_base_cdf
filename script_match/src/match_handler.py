#!/usr/bin/env python
# coding=utf-8

# import script_match as match
from std_msgs.msg import String, Bool
import rospy
import importlib
import config.scripts as scripts
import config.actions_2022 as action_functions
from cdf_msgs.msg import Pic_Action, Evitement
from config.actions_2022 import *


"""
Ce fichier contient les fonctions qui font appel au service (ros) de communication
avec le PIC
todo: ces fonctions pourraient devenir des services aussi ( a voir )
"""


#####################################################

class MatchEnvironment():

    def __init__(self, action_pub, nav_goal_pub, script_name, topic_dict):
        """
        Initialise l'environment de match.
        Il permet aux action de connaitre leur environment ( ex couleur, etat du match etc... )

        Parametres
        ----------
            - action_pub (Publisher) : publisher de la commande a envoyer au PIC
            - script_name (str) : nom du script a utiliser
            - topic_dict (dict) : dictionnaire de topics a utiliser
        """
        self.script_name = script_name
        self.topic_dict = topic_dict
        self.debug_mode = True
        self.score = 0
        self.state_status_match = 0
        self.action_pub = action_pub
        self.nav_goal_pub = nav_goal_pub
        self.robot_stopped = False
        self._last_motion_done = False
        self.evitement = True
        self.evit_mode = 'Normal' # Permet de savoir quelle stratégie d'évitement utiliser
        self.TEAM_factor = 1
        self.state_team = action_functions.DEFAULT_TEAM
        try:
            self.match_data = importlib.import_module("config.scripts.%s" % self.script_name)
        except ImportError as e:
            # on log le message d'erreur obtenu
            # e contient le message complet ( erreur + num de ligne + num de colone )
            rospy.logfatal("impossible de charger le script %s" % self.script_name)
            # on peut pas afficher de message aussi detaillé sur le lcd du coup on dit juste que le script est invalide

            # comme le script a planté, on leve ferme la node
            # raise e  # on pourrait lever une erreur (ce qui va tuer la node) mais ca afficherait un gros paté de texte
            rospy.signal_shutdown("script invalide")


    ######################################################
    # FONCTION UTILITAIRES
    ##########################    script_match = MatchEnvironment(action_pub, script_name, topic_dict, score_pub)
############################

    def get_match_sequence(self):
        """
        Renvoie la sequence d'actions a executer

        Return
        ------
            - action_sequence : la sequence d'action a executer ( liste de dictionnaires python )
        """
        return self.match_data.action_sequence

    def update_team_factor(self):
        """
        Met a jour la valeur du team factor ( et la valeur de la team au passage )

        Return
        ------
            - TEAM_factor (float) : valeur du tema factor
        """
        TEAM_factor = 1 if self.state_team == action_functions.DEFAULT_TEAM else -1
        self.TEAM_factor = TEAM_factor
        return TEAM_factor


def run_match(match_env):
    """
    Cette fonction execute la séquence d'action d'un match, connaissant le script_name et la couleur.
    Cette fonction considère que:
     - la couleur et le nom du script_name sont déja connus
     - le match a effectivement commencé ( cette fonction ne gère pas la laisse )
     - les fonctions appellées sont bloquantes

    Paramètres
    ----------
        - match_env (MatchEnvironment) : l'environnement de match
    """
    # on recupere la bonne sequence d'action suivant le script_name et la couleur
    action_sequence = match_env.get_match_sequence()
    # on parcourt la sequence d'actions et on les execute
    for action in action_sequence:
        rospy.loginfo(action["description"])
        # on recupère la fonction a appeler et ses paramètres
        try:
            # if script_match.evitement:
            #     moveseg_xy(script_match, 0.6, -1.6)
            #     moveseg_xy(script_match, 0.4, -1)
            #     break
            function_to_call = action["function"]
            function_params = action["params"]

            # # la gestion des variables a été désactivé a cause de conflits avec le changement de couleur
            # # on remplace les variables par leur valeurs
            # function_params = {k: script_match.get_variable(v) for k, v in function_params.items()}

            # on appele la fonction avec les bons parametres
            result = function_to_call(match_env, **function_params)
        except TypeError as te:
            # l'appel a la fonction a planté car on ne trouve pas action["function"] dans le fichier actions.py
            # on log en console
            rospy.logfatal(str(te))
            # on affiche sur le lcd
            # on attend 10 secondes ( si non la node se relance et du coup on voit pas le message s'afficher )
            rospy.sleep(10)
            raise te
        # on affiche le resultat de l'action
        # pour l'instant on utilise pas le resultat mais a terme on peut l'utiliser pour faire certaines choses
        # si l'action a echoué par exemple
        # ex: si un deplacement echoue, on retourne à la dernière position et on recommence
        if result is not None:
            rospy.loginfo("action gave result: %s" % result)

def urgenced_callback(msg):
    """
    callback appelé sur changement d'état de l'arrêt d'urgence. Tue la node s'il est enfoncé.
    :param msg: message avec l'info d'arret d'urgence
    :return: rien
    """
    if msg.data[0] == "0":
        ledrgb(script_match, 255, 0, 0)
        # print_lcd(script_match, 1, "  ATTENTION !!")
        print_lcd(script_match, 1, "Arret d'urgence")
        rospy.signal_shutdown("arret d'urgence activé")

if __name__ == '__main__':
    # init de la node, anonymous = False car on veut interdire que cette node soit lancée plusieurs fois
    rospy.init_node('match_handler', anonymous=False)

    # lecture des rosparams
    script_name = rospy.get_param('~script_name', 'simulation')
    score_topic = rospy.get_param('~score_topic', '/match_output/score')
    
    action_topic = rospy.get_param('~action_topic', '/robot_x/action')
    nav_goal_topic =  rospy.get_param('~nav_goal_pos_topic', '/robot_x/Pos_goal')
    # la perte de message implique un blocage du robot: queue_size=100 pour s'assurer de ne rien manquer
    action_pub = rospy.Publisher(action_topic, Pic_Action, queue_size=100, latch=True)
    nav_goal_pub = rospy.Publisher(nav_goal_topic, Point, queue_size=100, latch=True)
    # le topic dict permet de facilement ajouter/retirer des topic a l'environment
    topic_dict = {
        "motion_done": rospy.get_param('~motion_done_topic', '/robot_x/motion_done'),
        "ax12_status": rospy.get_param('~ax12_status_topic', '/robot_x/ax_done'),
        "buttons" : rospy.get_param('~buttons_topic', '/robot_x/buttons'),
        "urgence" : rospy.get_param('~urgence_topic', '/robot_x/urgence'),
    }

    # On déclare les Subscribers
    rospy.Subscriber(topic_dict['buttons'], String, urgenced_callback, queue_size=100)


    # creation de l'environment
    script_match = MatchEnvironment(action_pub, nav_goal_pub, script_name, topic_dict)

    while not rospy.is_shutdown():
        # on fait des matchs en boucle tant que la node n'est pas arretée
        # ( on est pas obligé de mettre cette boucle en réalité )
        rospy.loginfo("Attente du debut de match")
        try:
            run_match(script_match)
        except rospy.ROSInterruptException:
            rospy.loginfo("arret de la node")
            #ledrgb(script_match, 255, 0, 0)
        rospy.loginfo("match termine!")
