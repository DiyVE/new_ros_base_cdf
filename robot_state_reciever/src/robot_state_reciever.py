#!/usr/bin/env python
# coding=utf-8
# license removed for brevity
import rospy
from std_msgs.msg import String
from config.new_msg_format import msgs_dict
from os import walk


"""
    Cette Node a pour but de faire le pont entre les messages séries envoyés par le PIC et l'environnement ROS.
    Chaque trames envoyées par le PIC est constitué d'une première chaine de caractère servant à indentifier la trame qu'il va falloir interpréter.
    De cette manière la node est capable de connaitre la structure et le type de données envoyées par le PIC.
    Les données et structures attendues sont référencées dans le fichier config/new_msg_format.py
"""


def create_publisher(topic_rosparam, topic_default, type, *args, **kwargs):
    """
    Cree un publisher a partir d'un rosparam
    :param topic_rosparam: nom du rosparam
    :param topic_default: sa valeur par defaut
    :param type: le type de message
    :param args: non utilisé ( mais ne pas enlever pour autant ! )
    :param kwargs: non utilisé ( mais ne pas enlever pour autant ! )
    :return: le publisher
    """
    topic_name = rospy.get_param(topic_rosparam, topic_default)
    return rospy.Publisher(topic_name, type, queue_size=100)

def publish_value(msg, pub, publish_on_change=True, old_message=None):
    """
    Factorise le code d'envoie d'un message
    :param value: valeur a envoyer, doit etre du bon type!
    :param msg: message object, deja instancie (ex Int16() est un param valide
    :param pub: le publisher
    :param publish_on_change: si True, publie le message uniquement si la valeur a change depuis le dernier envoi
    :param old_message: ancienne valeur, utile si publish_on_change=True
    :return: la nouvelle valeur
    """
    rospy.logdebug("%s => %s" % (old_message, value))
    if (not publish_on_change) or (value != old_message):
        pub.publish(msg)
    return value

def parse_bytes(read_bytes):
    # On sépare la trame en morceaux séparés par une virgule
    parsed_values = read_bytes.split(',')
    try:
        # On récupère le type/nom de la trame recue
        selected_dict = msgs_dict[parsed_values.pop(0)]
        # On calcule le nombre d'éléments attendus dans la trame recue
        msg_lenght = 0
        for element in selected_dict.values():
            msg_lenght += len(element['structure'])

        # On vérifie que le nombre d'éléments présent dans la tramme est correct
        if (len(parsed_values) == msg_lenght):
            rospy.logdebug('succesfully parsed %s' % read_bytes)

            # On parcourt la structure de la trame recue pour interpréter correctement les valeurs
            for msglabel, value in selected_dict.items():
                # Contruction du message
                msg = value["type"]()
                for key, element in value["structure"].items():
                    path = element["path"]
                    element_value = element["value_expression"](parsed_values[key])
                    exec("msg.%s = %s" % (path, element_value))
                
                #On publie le message
                selected_dict[msglabel]["old_message"] = publish_value(  # on sauvegarde la valeur pour detecter les changement
                    msg=msg,  # on specifie le type de message
                    pub=value["pub"],  # on spécifie le publisher ( ajout lors du démarrage de la node )
                    publish_on_change=value["publish_on_change"],
                    # on définit si on publie a chaque trame ou sur changement de valeur
                    old_message=selected_dict[msglabel]["old_message"]  # on donne l'ancienne valeur
                )
        else:
            # Si le nombre d'éléments n'est pas correct, on loggue l'erreur
            rospy.logwarn("'%s' was not parsed because the frame takes %d values (%d given)" % (read_bytes, msg_lenght, len(parsed_values)))
    except KeyError as e:
        rospy.logwarn("The following key doesn't exist in the msgs_dict: %s" % e)

def select_usb_handler():
    """
        Parcourt l'ensemble des ports ttyUSB disponibles et les revois sous forme de liste
    """
    try:
        filenames = next(walk("/dev"), (None, None, []))[2]  # [] if no file
        valid_filenames = []
        for filename in filenames:
            if filename.startswith("ttyUSB"):
                valid_filenames.append(filename)
        return pi.serial_open("/dev/" + valid_filenames[0], baudrate, 0), pi.serial_open("/dev/" + valid_filenames[1], baudrate, 0)
    except IndexError:
        rospy.logerr("One of PIC cards are not reachable, check if the device is connected to the computer.\nRetrying in 1 seconds...")
        rospy.sleep(1)
        select_usb_handler()
        return None, None

if __name__ == '__main__':
    # declaration de la node
    # anonymous = False => erreur si on lance cette node plusieurs fois
    rospy.init_node('robot_state_reciever', anonymous=False)
    # params
    debug_mode = bool(rospy.get_param('~debug_mode', False))
    baudrate = rospy.get_param('~serial_baudrate', 115200)
    # frequence 9Hz alors que pic est a 10Hz => on est sur de rater des trames mais il est quasiment impossible de
    # rater plusieurs trames d'affilé
    freq = rospy.get_param('~pic_msg_freq', 9)
    resetPicAction_topic = rospy.get_param('~action_topic', '/robot_x/action')
    rawpicmessage_topic = rospy.get_param('~raw_pic_messages_topic', '/robot_x/raw_pic_messages')
    # la perte de message n'implique pas un blocage du robot: queue_size=1 pour libérer des ressourses
    parsedFails = 0

    # declaration des publishers
    # on prend msg_format et on ajoute a chaque element un champ pub et un champ old_message
    for msg_dict_key, msg_dict_value in msgs_dict.items():
        for msglabel, value in msg_dict_value.items():
            publisher = create_publisher(**value)
            msg_dict_value[msglabel]["pub"] = publisher
            old_message = None
            for key, element in value["structure"].items():
                msg_dict_value[msglabel]["old_message"] = old_message

    if debug_mode:
        # la perte de message n'implique pas un blocage du robot: queue_size=1 pour libérer des ressourses
        rospy.Subscriber(rawpicmessage_topic, String, lambda msg: parse_bytes(msg.data), queue_size=1)
        rospy.spin()
    else:
        # import serial
        import pigpio

        pi = pigpio.pi()
        Pic1, Pic2 = select_usb_handler()

        rate = rospy.Rate(freq)
        try:
            last_end_data_1 = ''
            last_end_data_2 = ''
            while not rospy.is_shutdown():
                # On recupere les trames des deux ports serie afin de pouvoir les parser et ensuite les publier si elles sont valides
                char_available = pi.serial_data_available(Pic1)
                if char_available != 0:
                    received_data = last_end_data_1 + str(pi.serial_read(Pic1)[1].decode('ASCII'))
                    try:
                        data_sequences = received_data.split('\n')
                    except:
                        rospy.loginfo("Error While Decoding Data : %s" % received_data[1])
                        data_sequences = []
                    last_end_data_1 = data_sequences.pop()
                    print(data_sequences)

                    for data_sequence in data_sequences:
                        parse_bytes(data_sequence)
                
                char_available = pi.serial_data_available(Pic2)
                if char_available != 0:
                    received_data = last_end_data_2 + str(pi.serial_read(Pic2)[1].decode('ASCII'))
                    try:
                        data_sequences = received_data.split('\n')
                    except:
                        rospy.loginfo("Error While Decoding Data : %s" % received_data[1])
                        data_sequences = []
                    
                    last_end_data_2 = data_sequences.pop()
                    for data_sequence in data_sequences:
                        parse_bytes(data_sequence)
                rate.sleep()

        except RuntimeError as e:
            print(e)
        finally:
            # si la node crache on ferme l'acces a l'uart
            pi.serial_close(Pic1)
            pi.serial_close(Pic2)
