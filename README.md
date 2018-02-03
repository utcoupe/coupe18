 Code source des robots d'UTCoupe 2018
=======

# Configuration

Avant toute chose, il faut cloner le répertoire sur votre ordinateur :
```
git clone git@github.com:utcoupe/coupe18.git
```

### Configurer l'environnement de développement

Un script d'installation automatique est disponible. Allez dans le dossier coupe18, et lancer simplement :
```
./scripts/install_utcoupe_setup.sh
```

Si c'est votre première installation, répondez "y" à toutes les questions.

### Compiler le système

Tout le système de cette année repose sur ROS (http://www.ros.org/), il faut donc compiler le système après l'avoir récupéré et configuré :
```
cd coupe18/ros_ws
catkin_make
```

# Lancement

//TODO

ou must then source the workspace with `source devel/setup.bash` or `source devel/setup.zsh` each
time you open a new terminal. Adding this line to your `~/.bashrc` or `~/.zshrc` (with the full
path to the setup file) will simply automate this step.

# Règles et Guidelines

Afin d'avoir un projet organisé et fonctionnel, voici quelques règles (par convention ou importantes pour le 
fonctionnement du projet) à suivre pour la création de branches git, paquets, noeuds ros, etc :

### Git

- Créer des branches sur git de la forme `namespace/package` si la branche correspond à un paquet ROS. (e.g. `ai/scheduler`, `memory/map`, etc)

### Paquets ROS

- Créer des paquets ROS nommés de la forme `namespace_package` (utile une fois qu'ils seront tous ensemble, ils seront ordonnés par
ordre alphabétique : plus visuel)

- Créer des serveurs de `topics`/`services`/`actions` nommés de la forme `/namespace/package/server_name` s'ils peuvent être accédés par des paquets 
extérieurs (ATTENTION : avec un `/` au début pour créer un nom absolu), `server_name` s'ils sont internes.

- Nommer les fichiers de définition `.msg`/`.srv`/`.action` en PascalCase (e.g. `GetValues.srv`) et les variables dedans en minuscules (format `var_name`).

### Python

- Afin de respecter le PEP8 : 4 espaces d'intentation (et non tabs).

### Données

- Unités de distance en mètres, transportées par des `float32`.

- Lors de la description d'une position d'une forme (cercle, ligne, rectangle, point...), donner la position par rapport au centre de la forme (sauf précision explicite et nécessaire). Par exemple, donner la position du centre d'un rectangle et non d'un coin.

# TODO à check + old stuff

Nécessité d'installer les dépendances JS avec le webclient v2 ?
```
npm install
```

### Lancer le projet 2017

Note : mode legacy, peut être non fonctionnel !

Pour lancer le serveur de communication par websocket :
```
npm run utcoupe
```

Pour lancer un serveur statique pour héberger le webclient :
```
npm run serve
```

Ensuite, aller sur l'adresse affiché par cette dernière commande et le webclient devrait être lancé.

/!\ Vérifiez que le webclient arrive à se connecter au serveur dans l'onglet Réseau /!\ Si ce n'est pas le cas, l'adresse du serveur est sans doute erronée : modifier le fichier `config.js` à la racine du projet.
