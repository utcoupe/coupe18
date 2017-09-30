#include "utils.h"
#include "fast_math.h"
#include <urg_ctrl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <jansson.h>

#define max(a, b) a>b?a:b
#define min(a, b) a<b?a:b

extern FILE* logfile;

int inZone(Pt_t pt, ScanZone_t zone) {
	return !(pt.x > zone.xmax || pt.x < zone.xmin || pt.y > zone.ymax || pt.y < zone.ymin);
}

int getPoints(Hok_t hok, Pt_t* pt_list) {
	long data[MAX_DATA];
	int n = urg_receiveData(hok.urg, data, MAX_DATA);
	if (n > 0) {
		int i, j=0;
		for (i=hok.imin; i<hok.imax; i++) {
			Pt_t pt = (Pt_t) { hok.pt.x + data[i]*fastCos(hok.fm, i), hok.pt.y + data[i]*fastSin(hok.fm, i) };
			if (inZone(pt, hok.zone)) {
				pt_list[j++] = pt;
			} else {
				pt_list[j++] = (Pt_t) { -1, -1 };
			}
		}
		return hok.imax-hok.imin;
	} else {
		fprintf(logfile, "%s%s : %s\n", PREFIX, hok.path, urg_error(hok.urg));
		return -1;
	}
}

int getClustersFromPts(Pt_t *pt_list, int nb_pts, Cluster_t* clusters) {
	int i, j, nbCluster = 0;
	if (nb_pts > 0) {
		int *clusters_index = malloc(nb_pts*sizeof(int));
		clusters_index[0] = 0; // numéro de cluster pour chaque point scanné
		for (i=0; i<nb_pts; i++) {
			if (pt_list[i].x == -1 && pt_list[i].y == -1) {
				clusters_index[i] = -1;
				continue;
			}

			/* si le pt actuel (i) est proche en distance avec un de ses précédents,
			 on le range dans le même cluster */
			int j = max(0, i - CLUSTER_POINTS_BACKWARDS);
			for (j=j; j<i; j++) {
				if (sameZone(pt_list[j], pt_list[i]) && dist_squared(pt_list[j], pt_list[i]) < MAX_DIST*MAX_DIST) {
					clusters_index[i] = clusters_index[j];
					break;
				}
			}
			/* si le point n'a pas été rangé ds un cluster, on en crée un */
			if (j == i) {
				clusters_index[i] = nbCluster;
				nbCluster++;

				if (nbCluster > MAX_CLUSTERS) {
					fprintf(logfile, "%sToo many clusters\n", PREFIX);
				}
			}
		}

		/* 
		Ça compte le nombre de points par clusters, ça met dans clusters[i].pts[] la liste des points associés à ce cluster
		et ça vire les points foirés (rangés au cluster -1)
		*/
		for (i=0; i<nbCluster; i++) {
			clusters[i].nb = 0;
		}
		for (i=0; i<nb_pts; i++) {
			int index = clusters_index[i];
			if (index >= 0) {
				clusters[index].pts[clusters[index].nb++] = pt_list[i];
			}
		}

		/*
		On supprime les clusters de taille inférieure à NB_PTS_MIN
		*/
		i = 0;
		while (i < nbCluster) {
			if (clusters[i].nb < NB_PTS_MIN) {
				for (j=i+1; j<nbCluster; j++) {
					clusters[j-1] = clusters[j];
				}
				nbCluster--;
			} else {
				i++;
			}
		}


		/*
		On calcule la taille (en profondeur) de chaque cluster et leur centre (moyenne des points)
		*/
		for (i=0; i<nbCluster; i++) {
			clusters[i].size = sqrt(dist_squared(clusters[i].pts[0], clusters[i].pts[clusters[i].nb-1]));
			long sumx = 0, sumy = 0;
			int j = 0;
			for (j=0; j<clusters[i].nb; j++) {
				sumx += clusters[i].pts[j].x;
				sumy += clusters[i].pts[j].y;
			}
			if (clusters[i].nb > 0) {
				clusters[i].center = (Pt_t) { sumx/clusters[i].nb, sumy/clusters[i].nb };
			}
		}
	}
	return nbCluster;
}

int sortAndSelectRobots(int n, Cluster_t *robots, int nb_robots_to_find){
	int i, nbr_robots = min(n, nb_robots_to_find);
	Cluster_t *r = malloc(n*sizeof(Cluster_t));
	memcpy(r, robots, n*sizeof(Cluster_t));// on copie nos clusters, pour pouvoir mettre les tailles des clusters rangés à 0
	for(i=0; i<nbr_robots; i++){ // on garde les nbr_robots plus grand clusters, qu'on range par ordre décroissant XXX -> source d'oublis de robots !! (en cas de main d'arbitre par ex)
		int maxSize = 0, maxId = 0;
		for(int j=0; j<n; j++){ // on cherche le cluster non rangé le plus grand
			if(r[j].size > maxSize){
				maxSize = r[j].size;
				maxId = j;
			}
		}
		robots[i].center = r[maxId].center; // on range le ième plus grand à la ième place !
		robots[i].size = r[maxId].size;
		r[maxId].size = 0;
	}
	return i;
}

int mergeRobots(Cluster_t *r1, int n1, Cluster_t *r2, int n2, Cluster_t *result) {
	Cluster_t all_bots[2*MAX_ROBOTS];
	int i, n_tot=0;

	// On copie les robots, n_tot va être égal à n1+n2
	for (i=0; i<n1; i++) {
		all_bots[n_tot++] = r1[i];
	}
	for (i=0; i<n2; i++) {
		all_bots[n_tot++] = r2[i];
	}

	// À faire uniquement si la seconde partie est commentée !
	memcpy(result, all_bots, n_tot*sizeof(Cluster_t));

	// La partie suivante n'est utile que pour la partie du milieu (dans le cas où chaque
		// hokuyo scanne la moitié du terrain) ou si on scanne toute la carte
		// Maintenant, osef parce que l'IA se charge de ça
	/*
	if (n_tot <= nb_robots_to_find) {
		memcpy(result, all_bots, n_tot*sizeof(Cluster_t));
		return n_tot;
	}

	struct corres { //permet d'établir des correspondances entre les index
		int r1;
		int r2;
	};
	while (n_tot > nb_robots_to_find) {
		//Brute force ici, on aura max 4 robots de chaque hokuyo, 4x4 = 16 cas, ca reste peu

		//Calcul de la distance entre chaque combinaison de coords
		int distR1R2[MAX_ROBOTS][MAX_ROBOTS], i, j;
		struct corres dist_indexes[MAX_ROBOTS*MAX_ROBOTS]; //Tableau d'index de distR1R2, sert a trier les distances
		//struct corres merged[MAX_ROBOTS]; //Correspondances finales et retenues
		int changed = 0;

		//Calcul des distaces
		for (i=0; i<n1; i++) {
			for (j=0; j<n2; j++) {
				distR1R2[i][j] = dist_squared(r1[i].center, r2[j].center);
			}
		}

		for (i=0; i<n1*n2; i++) {
			dist_indexes[i] = (struct corres){ i/n2, i%n2 }; // tableau des index, dans l'ordre. C'est magique ^^
		}

		//Tri ordre decroissant
		do {
			changed = 0;
			for (i=1; i<n1*n2; i++) {
				if (distR1R2[dist_indexes[i-1].r1][dist_indexes[i-1].r2] < distR1R2[dist_indexes[i].r1][dist_indexes[i].r2]) {
					struct corres temp = dist_indexes[i];
					dist_indexes[i] = dist_indexes[i-1];
					dist_indexes[i-1] = temp;
					changed = 1;
				}
			}
		} while (changed);

		for (i=n1+dist_indexes[0].r2; i<n_tot-1; i++) {
			all_bots[i] = all_bots[i+1];
		}
		n_tot--;
	}
	*/
	return n_tot;
}


int sameZone(Pt_t p1, Pt_t p2){
	int p1_zone = (p1.y>STAIRS_Y_MIN) && (p1.x > STAIRS_X_MIN) && (p1.x < STAIRS_X_MAX);
	int p2_zone = (p2.y>STAIRS_Y_MIN) && (p2.x > STAIRS_X_MIN) && (p2.x < STAIRS_X_MAX);
	// s'ils sont dans la même zone
	return p1_zone == p2_zone;
};

	
/*

int mergeRobots(Cluster_t *r1, int n1, Cluster_t *r2, int n2, Cluster_t *result, int nb_robots_to_find) {
	struct corres { //permet d'établir des correspondances entre les index
		int r1;
		int r2;
	};
	//Brute force ici, on aura max 4 robots de chaque hokuyo, 4x4 = 16 cas, ca reste peu
	//Calcul de la distance entre chaque combinaison de coords
	int distR1R2[MAX_ROBOTS][MAX_ROBOTS], i, j;
	struct corres dist_indexes[MAX_ROBOTS*MAX_ROBOTS]; //Tableau d'index de distR1R2, sert a trier les distances
	struct corres merged[MAX_ROBOTS]; //Correspondances finales et retenues
	int changed = 0;

	//Calcul des distaces
	for (i=0; i<n1; i++) {
		for (j=0; j<n2; j++) {
			distR1R2[i][j] = dist_squared(r1[i].center, r2[j].center);
		}
	}

	for (i=0; i<n1*n2; i++) {
		dist_indexes[i] = (struct corres){ i/n2, i%n2 };
	}

	do {
		changed = 0;
		for (i=1; i<n1*n2; i++) {
			if (distR1R2[dist_indexes[i-1].r1][dist_indexes[i-1].r2] > distR1R2[dist_indexes[i].r1][dist_indexes[i].r2]) {
				struct corres temp = dist_indexes[i];
				dist_indexes[i] = dist_indexes[i-1];
				dist_indexes[i-1] = temp;
				changed = 1;
			}
		}
	} while (changed);

	//Choix des correspondances en prenant la premiere qui vientdu plus petit au plus grand
	int used_R1_index[MAX_ROBOTS], used_R2_index[MAX_ROBOTS], nbr_found = 0;
	for (i=0; i<MAX_ROBOTS; i++) {
		used_R1_index[i] = -1;
		used_R2_index[i] = -1;
	}
	for (i=0; i<n1*n2; i++) {
		struct corres c = dist_indexes[i];

		if (distR1R2[c.r1][c.r2] > MAX_SIZE_TO_MERGE*MAX_SIZE_TO_MERGE) {
			break;
		}

		if (!isIn(c.r1, used_R1_index, n1) && //Aucun des deux robots n'est deja selectionné
			!isIn(c.r2, used_R2_index, n2)) { //On ajout la correspondace

			merged[nbr_found] = c;
			used_R1_index[nbr_found] = c.r1;
			used_R2_index[nbr_found] = c.r2;
			nbr_found++;
		}
	}

	for (i=0; i<nbr_found; i++) {
		struct corres c = merged[i];
		Cluster_t clu;
		clu.nb = 0;
		clu.center = (Pt_t) {(r1[c.r1].center.x + r2[c.r2].center.x) / 2, (r1[c.r1].center.y + r2[c.r2].center.y) / 2 };
		clu.size = (r1[c.r1].size + r2[c.r2].size) / 2;
		result[i] = clu;
	}

	int nbr_left = n1 + n2 - 2*nbr_found, clust_counter = 0;
	Cluster_t clust_left[MAX_ROBOTS*MAX_ROBOTS];
	for (i=0; i<n1; i++) {
		if (!isIn(i, used_R1_index, nbr_found)) {
			clust_left[clust_counter++] = r1[i];
		}
	}
	clust_counter = 0;
	for (i=0; i<n2; i++) {
		if (!isIn(i, used_R2_index, nbr_found)) {
			clust_left[clust_counter++] = r2[i];
		}
	}

	for (i=n1*n2-1; i>=0; i--) {
		struct corres c = dist_indexes[i];

		if (!isIn(c.r1, used_R1_index, nbr_found)) { 
			result[nbr_found] = r1[c.r1];
			used_R1_index[nbr_found] = c.r1;
			nbr_found++;
		}
		
		if (!isIn(c.r2, used_R2_index, nbr_found)) {
			result[nbr_found] = r2[c.r2];
			used_R2_index[nbr_found] = c.r2;
			nbr_found++;
		}
	}

	nbr_found = sortAndSelectRobots(nbr_found, result, nb_robots_to_find);
	return nbr_found;
}


int isIn(int e, int *tab, int tab_size) {
	int i, ret = 0;
	for (i=0; i<tab_size; i++) {
		if (tab[i] == e) {
			ret = 1; //Found
			break;
		}
	}
	return ret;
}
*/


// Cluster_simple_t findCone(int n, Cluster_t *clusters, Pt_t coneCenter){
// 	int i, found = -1;
// 	Cluster_simple_t result;

// 	for(i=0; i<n; i++){ // on cherche notre cône
// 		if ((dist_squared(clusters[i].center, coneCenter) < 6*3600) && (clusters[i].size < CONE_DIAM_MAX)){ // (3% * 2m)² = 3600 mm² + 600% "d'erreur" (à la louche)
// 			found = i;
// 		}
// 		// fprintf(logfile, "%d %d, size : %d\n", clusters[i].center.x, clusters[i].center.y, clusters[i].size);
// 	}

// 	if(found != -1){ // si cône trouvé
// 		result.size = clusters[found].size; // c'est bizarre, clusters[found].center vaut 0
// 		result.center = clusters[found].center;
// 	} else { // si cône pas trouvé
// 		result.center.x=-1;
// 		result.center.y=-1;
// 	}
// 	return result;
// }
