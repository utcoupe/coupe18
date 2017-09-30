IA des robots
=======

## Ordres de l'IA aux robots

### Ordres communs
- `send_message`
	- `name` : demande d'envoyer un message avec un certain nom à l'ia
	- `params` : paramètres à envoyer en pj du message
- `asserv.`... :
	- ...`set_pos` : définir la position actuelle
		- `x`
		- `y`
		- `a` : angle en radians [-pi, pi], le zéro étant l'axe des x, sens direct (attention, l'axe y est vers le public)
	- ...`goxy` : aller à ce point, en ligne droite, tourner si nécessaire au avant de commencer
		- `x`
		- `y`
		- `direction` (`forward`, `backward`, `whatever`) : si le sens de déplacement importe
	- ...`goa` : tourner le robot pour prendre cet angle
- `do_start_sequence` : demande de se placer, fermer et ouvrir tous les actionneurs avant le départ
- `collision` : stopper le robot, vider les files
- `pause`: stopper le robot, concerver les files et l'action en cours
- `resume` : reprendre l'ordre en cours au moment du pause
- `stop` : stopper le robot définitivement


### vers PR
- `take_module`
	- différence entre fusée et module seul ?
- `prepare_module` : engage un module dans les servos de drop
	- `color` (`yellow`, `blue`, `null`) : tourner le module avant de le poser (`null` : on ne le tourne pas)
	- `push_towards` (`left`, `right`, `don't`) : pousser le module une fois posé
- `drop_module` : laisse tomber un module une fois préparé (en tenant compte des paramètres de `prepare_module`)
	- `nb_modules_to_drop` (int) : nombre de dépôts à faire

### vers GR
- `funny_action` : lancer la fusée
- `swallow_balls` : démarrer l'aspirateur de balles
	- est-ce qu'on aspire pas les balles en permanence ?
- `throw_balls`
	- `duration` : temps en ms d'allumage des moteur
	- `speed` : vitesse du canon (unité à définir)
- `open_trunk` : ouvrir le coffre à balles


## Ancienne architecture
Structure interne de l'IA :
![alt tag](https://raw.githubusercontent.com/utcoupe/coupe15/master/ia/architecture_ia_utcoupe_2015.jpg)