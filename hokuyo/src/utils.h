#ifndef UTILS_H
#define UTILS_H

#include "fast_math.h"
#include "hokuyo_config.h"

typedef struct Cluster {
	Pt_t pts[MAX_DATA];
	int nb, size; // nb de pts dans le cluster, taille (entre les pts extrêmes du cluster)
	Pt_t center;
} Cluster_t;

typedef struct Cluster_simple {
	int size;
	Pt_t center;
} Cluster_simple_t;

/* Demande les points à l'Hokuyo
	IN: structure de l'Hokuyo à appeler
	OUT: liste de points (simple tableau de distances),
		nb de points retournés, -1 en cas d'échec
*/
int getPoints(Hok_t hok, Pt_t* pt_list);

/* Calcule et filtre les clusters à partir des points
	IN: liste de points, nb de points
	OUT: clusters, nb de robots (= de clusters)
*/
int getClustersFromPts(Pt_t *pt_list, int nb_pts, Cluster_t* clusters);

/* Range les robots (center et size uniquement) par ordre décroissant de taille. (attention, les points ne correspondent plus !)
	IN: nb de clusters, tableau des clusters, nb robots à chercher
	OUT: nb de clusters (= min entre n et nb_robots_to_find)
*/
int sortAndSelectRobots(int n, Cluster_t *robots, int nb_robots_to_find);

/* Trie les clusters (issus de getClustersFromPts) pour trouver le cône dans la zone attendue
	IN: nb de clusters, tableau des clusters, cone
	OUT: cluster du cône (en cas d'erreur, le centre est -1 -1)
*/
// Cluster_simple_t findCone(int n, Cluster_t *clusters, Pt_t coneCenter);

/* 
	IN: nombre de clusters et tab de clusters de chaque hok, nb_robots à trouver (>= n1 et >=n2)
	OUT: retourne le nb de robot trouvés final + result (leur position)
*/
// int mergeRobots(Cluster_t *r1, int n1, Cluster_t *r2, int n2, Cluster_t *result, int nb_robots_to_find);
int mergeRobots(Cluster_t *r1, int n1, Cluster_t *r2, int n2, Cluster_t *result);


/* Dit si 2 points sont dans la même zone (escalier / pas escalier)
	IN: 2 points
	OUT: boolean (true = 1, false = 0)
*/
int sameZone(Pt_t p1, Pt_t p2);

/* 
	IN: 
	OUT: 
*/
// int isIn(int e, int *tab, int tab_size);

#endif
