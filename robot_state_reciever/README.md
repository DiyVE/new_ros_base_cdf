robot\_state\_reciever:
=======================

Cette Node a pour but de faire le pont entre les messages séries envoyés par le PIC et l'environnement ROS.
Cette node se comporte comme une sorte de publisheur qui distribue les informations dans différents topic qui peuvent contenir des messages de différents types.
Chaque trames envoyées par le PIC est constitué d'une première chaine de caractère servant à indentifier la trame qu'il va falloir interpréter.
De cette manière la node est capable de connaitre la structure et le type de données envoyées par le PIC.
Les données et structures attendues sont référencées dans le fichier `config/new_msg_format.py`

Remarque sur la construcution du fichier de configuration :
------------------------------------
Le fichier de configuration se trouve dans le dossier `src/config` du package.

Il est dans un premier temps important de déclarer l'ensembles des messages qui vont être utilisés dans les messages.

Il faut ensuite configurer le dictionnaire `msg_dict`.
Chaque clé du dictionnaire désigne la chaine de caractère qui devra être envoyée pour identifier une trame ayant une structure décrite dans la valeur associée a cette clé.

Une trame peut être constituée de valeurs qui devront être envoyées dans des messages pas forcément de même type ni de même taille.

Pour cela à chaque nom de trame on associe un dictionnaire qui désigne chaque type de messages. Les clefs de ce dictionnaire sont simplement une chaine de caractère qui permet d'identifier pour le développeur mais ne sert pas réellement.

Cependant la valeur associée a cette chaine contient toutes les informations concerant le message et sont décrite ci dessous :  
    - `topic_rosparam` : Définie le nom du paramètre pouvant servir à spécifier le topic sur lequel le message sera publié.  
    - `topic_default` : Définie le nom du topic par défaut sur lequel le message sera publié.  
    - `msg_type` : Définie le type de message qui sera publié.  
    - `publish_on_change` : Définit si le message sera publié quand il change ou non.  
    - `structure` : Définit la structure du message. Cette clé est elle aussi associée a une valeur qui est un dictionnaire (oui je sais ca fait beaucoup). Chaque clé de ce dictinnaire décrit la position dans la trame de la valeur que l'on souhaite traiter. A cette clé on associe deux paramètre que sont `value_expression` et `path`.
    Ils décrivent respectivement l'expression qui sera appliquée à la valeur de la trame ainsi que le chemin vers la variable qui contiendra cette valeur.


Par exemple si une trame contient un message de type `std_msgs/Float64` et un message de type `std_msgs/Int32`, on aura :  
<pre>
msg_dict = {
    'trame_1': {
        'Float msg': {
            'topic_rosparam': '~float_topic',
            'topic_default': 'float_topic',
            'msg_type': 'std_msgs/Float64',
            'publish_on_change': True,
            'structure': {
                'value_expression': x,
                'path': 'data'
            }
        },
        'Int msg': {
            'topic_rosparam': '~int_topic',
            'topic_default': 'int_topic',
            'msg_type': 'std_msgs/Int32',
            'publish_on_change': True,
            'structure': {
                'value_expression': x,
                'path': 'data'
            }
        }
    }
}
</pre>