 Code source des robots d'UTCoupe 2017
=======

### Configurer l'environnement de développement

Un script d'installation automatique est disponible. Allez dans le dossier coupe17, et lancer simplement :
```
source ./scripts/install_utcoupe_setup.sh
```

Si c'est votre première installation, répondez "y" à toutes les questions.

Une fois terminé, pour installer les dépendances JavaScript :
```
npm install
```

### Lancer le projet

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

### Utiliser JSDoc

JSDoc s'installe via npm.
Pour l'installer en local sur la machine :
```
npm install
```

Pour générer la documentation du code javascript :
```
npm run doc
```

La documentation se trouve au format HTML/JS dans le dossier doc.

La documentation officielle de JSDoc est disponible [ici](http://usejsdoc.org).

### Configurer les Raspberry Pi

Une image fonctionnelle est disponible, il est préférable de partir de cette image.

Note : le script d'installation effectue ces opérations tout seul en détectant le système sur lequel il est lancé.

Sinon, il y a quelques subtilités pour installer tous les logiciels UTCoupe sur la Raspberry Pi.

Pour npm, la version dans les dépôts officiels est trop ancienne, il faut donc l'installer à la main.
```
sudo apt-get remove npm nodejs nodejs-legacy
curl -sL https://deb.nodesource.com/setup_4.x | sudo -E bash -
```

La version de nodejs sera correcte, mais ce n'est pas le cas de npm. Mettre à jour avec la version de travail (ici la 3.5.2) :
```
sudo npm install npm@3.5.2 -g
```

Ensuite concernant les drivers des clés WiFi 5 GHz, il faut installer des headers de linux (correspondant au noyau utilisé), afin de pouvoir compiler le driver :
```
sudo apt-get install raspberrypi-kernel-headers
```

Normalement un lien symbolique nommé build a été créé dans /lib/modules/<kernel-version>, pointant vers /usr/src/linux-headers-<kernel-version>.



# ROS README


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
