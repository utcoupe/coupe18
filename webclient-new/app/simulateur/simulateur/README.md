Simulateur UTCoupe
==================
## 1) Description des fichiers
Le simulateur utilise [Three.js](https://threejs.org/), qui est à minima supporté par la majorité des navigateurs en 2017. Cependant il est recommandé d'avoir un bon cpu car Three.js a tendance à être gourmand en ressources, particulièrement dans ce projet.

Les objets du simulateurs sont chargés à partir de scènes (contenant un objet 3D) exportées en Collada depuis Blender.

### Controlleur (controller.class.js)
Le controlleur est la classe qui est destinée à être appelée pour construire et mettre à jour le simulateur. Le constructeur prend en paramètre le chemin d'accès au fichier contenant la liste des objets à charger au format JSON, ainsi que le chemin d'accès des différentes ressources COLLADA à charger.

Le controlleur gère la création de la scène, les spots de lumières, la caméra et la création/modification des objets.

### Object3d (object3d.class.js)
L'objet 3D est une classe assez abstraite qui charge le collada associé à l'objet et stocke différents paramètres comme son nom, sa position, sa rotation.

### Robot (robot.class.js)
Classe héritant de Object3d, modifiée pour ajouter des paramètres aux robots.

### Position (position.class.js)
La postion est une classe qui en plus de stocker des coordonnées (cartésiennes ou angulaires), propose des fonctions simplifiant les conversions et les modifications.

### 3dobjects.json
Fichier JSON regroupant l'ensemble des paramètres des objets 3D (dont le nombre est assez exaustif) à charger dans le simulateur.

## 2) Fonctionnement
L'origine est placé dans le coin au fond à gauche du plateau pour les spectateurs. C'est le même repère qui es utilisé dans le règlement. Le simulateur travaille en mètres et en radians, mais le fichier contenant les objets utilise des angles en degrés pour plus de facilité. le plateau est décrit par les axes `(x, z)` avec `x` la longueur du plateau. L'axe de la hauteur est donc `y`. Le simulateur se contente d'afficher le retour des clients.

Chaque objet est indépendant des autres, il n'y a pas de détection de collision. Lorsque un des robots bouge, il appelle automatiquement la procédure de nettoyage des objets à sa portée et affiche le chemin qu'il souhaite suivre.