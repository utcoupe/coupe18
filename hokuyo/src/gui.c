#include "gui.h"
#include "stdio.h"
#include "fast_math.h"
#include "utils.h"
#include <SDL.h>

struct GUI_data{
	SDL_Surface *ecran, *terrain, *lidar, *robot, *point, *marker;
	SDL_Rect posTerrain, posMarker;
};
static struct GUI_data gui;

//Do NOT modify
#define GUI_WINDOW_REAL_SIZE_X (TABLE_X+2*BORDER_PADDING)
#define GUI_WINDOW_REAL_SIZE_Y (TABLE_Y+2*BORDER_PADDING)

#define GUI_TERRAIN_SIZE_X GUI_WINDOW_RESOLUTION_X*TABLE_X / GUI_WINDOW_REAL_SIZE_X
#define GUI_TERRAIN_SIZE_Y GUI_WINDOW_RESOLUTION_Y*TABLE_Y / GUI_WINDOW_REAL_SIZE_Y

#define GUI_MARKER_SIZE_X GUI_WINDOW_RESOLUTION_X*MARKER_SIZE / GUI_WINDOW_REAL_SIZE_X
#define GUI_MARKER_SIZE_Y GUI_WINDOW_RESOLUTION_Y*MARKER_SIZE / GUI_WINDOW_REAL_SIZE_Y

#define GUI_LIDAR_SIZE_X GUI_WINDOW_RESOLUTION_X*LIDAR_SIZE / GUI_WINDOW_REAL_SIZE_X
#define GUI_LIDAR_SIZE_Y GUI_WINDOW_RESOLUTION_Y*LIDAR_SIZE / GUI_WINDOW_REAL_SIZE_Y

SDL_Rect
getPixelCoord(int x, int y){
	SDL_Rect p;
	p.x = (BORDER_PADDING+(float)x) * GUI_WINDOW_RESOLUTION_X / GUI_WINDOW_REAL_SIZE_X;
	p.y = GUI_WINDOW_RESOLUTION_Y - ((BORDER_PADDING+(float)y) * GUI_WINDOW_RESOLUTION_Y / GUI_WINDOW_REAL_SIZE_Y);
	return p;
}

SDL_Rect
getPixelCoordWithSize(int x, int y, int sx, int sy){
	SDL_Rect p;
	p.x = (BORDER_PADDING+(float)(x)) * GUI_WINDOW_RESOLUTION_X / GUI_WINDOW_REAL_SIZE_X;
	p.y = GUI_WINDOW_RESOLUTION_Y - ((BORDER_PADDING+(float)(y+sy)) * GUI_WINDOW_RESOLUTION_Y / GUI_WINDOW_REAL_SIZE_Y);
	return p;
}


struct color newColor(int r, int g, int b){
	struct color c;
	c.r = r;
	c.g = g;
	c.b = b;
	return c;
}

void
initSDL(Pt_t positionLidar){

	gui.ecran = SDL_SetVideoMode(GUI_WINDOW_RESOLUTION_X, GUI_WINDOW_RESOLUTION_Y, 32, SDL_HWSURFACE);
	if(gui.ecran == NULL) exit(EXIT_FAILURE);

	gui.terrain = SDL_CreateRGBSurface(SDL_HWSURFACE, GUI_TERRAIN_SIZE_X, GUI_TERRAIN_SIZE_Y, 32, 0, 0, 0, 0);
	SDL_FillRect(gui.terrain, NULL, SDL_MapRGB(gui.ecran->format, 200, 255, 200));
	gui.posTerrain = getPixelCoordWithSize(0, 0, TABLE_X, TABLE_Y);

	gui.robot = SDL_CreateRGBSurface(SDL_HWSURFACE, GUI_ROBOT_SIZE, GUI_ROBOT_SIZE, 32, 0, 0, 0, 0);
	SDL_FillRect(gui.robot, NULL, SDL_MapRGB(gui.ecran->format, 0, 0, 0));

	gui.marker = SDL_CreateRGBSurface(SDL_HWSURFACE, GUI_MARKER_SIZE_X, GUI_MARKER_SIZE_Y, 32, 0, 0, 0, 0);
	SDL_FillRect(gui.marker, NULL, SDL_MapRGB(gui.ecran->format, 0, 255, 0));
	gui.posMarker = getPixelCoordWithSize(CALIB_X-MARKER_SIZE/2, CALIB_Y-MARKER_SIZE/2, MARKER_SIZE, MARKER_SIZE);

	gui.lidar = SDL_CreateRGBSurface(SDL_HWSURFACE, GUI_LIDAR_SIZE_X, GUI_LIDAR_SIZE_Y, 32, 0, 0, 0, 0);
	gui.point = SDL_CreateRGBSurface(SDL_HWSURFACE, GUI_POINT_SIZE, GUI_POINT_SIZE, 32, 0, 0, 0, 0);


}

void
blitMap(Pt_t positionLidar){
	SDL_FillRect(gui.ecran, NULL, SDL_MapRGB(gui.ecran->format, 255, 255, 255));
	SDL_BlitSurface(gui.terrain, NULL, gui.ecran, &(gui.posTerrain));
	SDL_BlitSurface(gui.marker, NULL, gui.ecran, &(gui.posMarker));
}

void
blitLidar(Pt_t positionLidar, struct color c){
	SDL_Rect p = getPixelCoord(positionLidar.x-LIDAR_SIZE/2, positionLidar.y+LIDAR_SIZE/2);
	SDL_FillRect(gui.lidar, NULL, SDL_MapRGB(gui.ecran->format, c.r, c.g, c.b));
	SDL_BlitSurface(gui.lidar, NULL, gui.ecran, &p);
}

void
blitRobots(Cluster_t *robots, int nRobots, struct color c){
	SDL_FillRect(gui.robot, NULL, SDL_MapRGB(gui.ecran->format, c.r, c.g, c.b));
	int i, maxsize = 0;
	for (i=0; i<nRobots; i++) {
		if (robots[i].size > maxsize) {
			maxsize = robots[i].size;
		}
	}
	for(i=0; i<nRobots; i++){
		int size_to_blit = GUI_ROBOT_SIZE * ((float)robots[i].size / (float)maxsize);
		SDL_Rect p = getPixelCoord(robots[i].center.x-size_to_blit/2, robots[i].center.y+size_to_blit/2);
		SDL_BlitSurface(gui.robot, NULL, gui.ecran, &p );
	}
}

void
blitPoints(Pt_t *points, int nPoints, struct color c){
	SDL_FillRect(gui.point, NULL, SDL_MapRGB(gui.ecran->format, c.r, c.g, c.b));
	for(int i=0; i<nPoints; i++){
		/*#ifndef DEBUG_DO_NOT_REMOVE_POINTS
		if( points[i].x == 0 && points[i].y == 0) continue;
		#endif*/
		SDL_Rect p = getPixelCoord(points[i].x, points[i].y);
		SDL_BlitSurface(gui.point, NULL, gui.ecran, &p );
	}

}


void
waitScreen(){
	do{
		SDL_Event event;
		while( SDL_PollEvent(&event) ){ // On doit utiliser PollEvent car il ne faut pas attendre d'évènement de l'utilisateur pour mettre à jour la fenêtre
			switch(event.type) {
				case SDL_QUIT:
					exit(EXIT_SUCCESS);
					break;
				case SDL_KEYDOWN:
			        if(event.key.keysym.sym == SDLK_ESCAPE) exit(EXIT_SUCCESS);
			        break;
			}
		}
	}while(SDL_Flip(gui.ecran)!=0);

	//SDL_Delay(1000);

}

