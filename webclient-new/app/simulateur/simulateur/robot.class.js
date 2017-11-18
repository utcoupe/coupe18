/**
 * Gestion des robots
 * 
 * @author Mindstan
 */

"use strict";
/**
 * Permet une gestion plus précise des robots dans le simulateur
 * 
 * @class Robot
 * @extends {Object3d}
 */
class Robot extends Object3d
{
    /**
     * Crée un nouveau robot
     * 
     * @param {Object} params 
     * @param {String} ressourcesPath
     * @param {Function} onCreateFinished Fonction appelée losrque le robot a fini d'être initialisé
     * @param {Number} radius Rayon du robot (utilisé pour enlever les objets)
     * @param {Function} autotrash Fonction appelée pour supprimer des objets proches
     */
    constructor (params, ressourcesPath, onCreateFinished, radius, autotrash)
    {
        // On appelle le constructeur parent
        super(params, ressourcesPath);

        /**
         * @type {Number}
         * @const
         */
        this.PATH_MAX_POINTS = 10;

        /**
         * Rayons représentant le robot sur le plan (xOz)
         * @type {Number}
         */
        this.radius = radius;

        /**
         * Fonction appelée sur le controlleur lorsque le robot bouge,
         * avec en paramètre la position et le rayon du robot
         * @type {Function}
         */
        this.autoTrash = autotrash;

        this.initPath(onCreateFinished);
    }

    /**
     * Initialise l'afichage du chemin que suivra le robot
     * 
     * @param {Function} onCreateFinished
     */
    initPath(onCreateFinished)
    {
        var pathGeometry = new THREE.Geometry();
        var pathMaterial = new THREE.LineBasicMaterial({
            color: 0x000000,
            linewidth: 3
        });

        for(var idVertice = 0; idVertice < this.PATH_MAX_POINTS; idVertice++)
            pathGeometry.vertices.push(new THREE.Vector3( 0, 0, 0 ));

        /** @type {external:THREE.Line} */
        this.pathLine = new THREE.Line(pathGeometry, pathMaterial);
        onCreateFinished(this.pathLine); // Callback
    }

    /**
     * Met à jour et affiche le chemin passé en paramètre
     * 
     * @param {Array<Position>} newPath
     */
    showPath(newPath)
    {
        //console.log(newPath);
        var idVertice = 0;
        while(idVertice < newPath.length && idVertice < this.PATH_MAX_POINTS)
        {
            var pos = newPath[idVertice];
            var v = new THREE.Vector3(pos.x, pos.y, pos.z);
            this.pathLine.geometry.vertices[idVertice] = v;
            idVertice++;
        }
        var lastVertice = newPath[newPath.length-1];
        while(idVertice < this.PATH_MAX_POINTS)
        {
            this.pathLine.geometry.vertices[idVertice] = lastVertice;
            idVertice++;
        }
        this.pathLine.geometry.verticesNeedUpdate = true;
    }

    /**
     * Supprime le chemin de ce robot
     */
    clearPath()
    {
        for(var idVertice = 0; idVertice < this.PATH_MAX_POINTS; idVertice++) {
            this.pathLine.geometry.vertices[idVertice] = new THREE.Vector3( 0, 0, 0 );
        }
        
        this.pathLine.geometry.verticesNeedUpdate = true;
    }

    /**
     * Met à jour la position et la rotation de l'objet 3D.
     * Si certains paramètres ne sont pas définis, ils ne seront pas modifiés.
     * Supprime automatiquement les objets proches.
     * 
     * @param {Object} params Paramètres divers de l'objet 3D
     */
    updateParams (params)
    {
        super.updateParams(params);
        this.autoTrash(this.position, this.radius);
    }
}