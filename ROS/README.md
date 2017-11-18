# Installation

Once the repo is cloned, run `catkin_make` inside this directory from a terminal to
generate the `build/` and `devel/` ROS folders.

You must then source the workspace with `source devel/setup.bash` or `source devel/setup.zsh` each
time you open a new terminal. Adding this line to your `~/.bashrc` or `~/.zshrc` (with the full
path to the setup file) will simply automate this step.

# Règles de bonne conduite

Afin d'avoir un projet organisé et fonctionnel, voici quelques règles (par convention ou importantes pour le 
fonctionnement du projet) à suivre pour la création de branches git, paquets, noeuds ros, etc :

### Git

- Créer des branches sur git de la forme `namespace/package` si la branche correspond à un paquet ROS. (e.g. `ai/scheduler`, `memory/map`, etc)

### Paquets ROS

- Créer des paquets ROS nommés de la forme `namespace_package` (utile une fois qu'ils seront tous ensemble, ils seront ordonnés par
ordre alphabétique : plus visuel)

- Créer des serveurs de `topics`/`services`/`actions` nommés de la forme `/namespace/package/server_name` s'ils peuvent être accédés par des paquets 
extérieurs (ATTENTION : avec un `/` au début pour créer un nom absolu), `server_name` s'ils sont internes.

### Python

- Afin de respecter le PEP8 : 4 espaces d'intentation (et non tabs).

### Données

- Unités de distance en mètres, transportées par des `float32`.

- Lors de la description d'une position d'une forme (cercle, ligne, rectangle, point...), donner la position par rapport au centre de la forme (sauf précision explicite et nécessaire). Par exemple, donner la position du centre d'un rectangle et non d'un coin.
