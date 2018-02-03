/**
 * @file Gestion des objects 3d dans le simulateur
 * @author Mindstan
 * 
 * @requires THREE
 * @requires THREE.ColladaLoader
 * @requires Position
 */

"use strict";

/**
 * Gère les meshes
 * 
 */
class Object3d
{
    /**
     * Constructeur de Object3d
     * 
     * @param {Object} params Paramètres divers de l'objet 3d
     * @param {String} ressourcesPath Répertoire de travail
     */
    constructor (params, ressourcesPath)
    {
        /** @type {String} */
        this.ressourcesPath = ressourcesPath;

        /** @type {external:THREE.ColladaLoader} */
        this.loader = new THREE.ColladaLoader();
        this.loader.options.convertUpAxis = true;

        /** @type {Object} */
        this.mesh = null;

        this.setParams(params);
    }

    /**
     * Répartit les valeurs contenues dans params dans les bonnes variables membres
     * 
     * /!\ Ecrase les données existantes si elles existent ! /!\
     * 
     * @param {Object} params Paramètres divers de l'objet 3D
     */
    setParams (params)
    {
        //console.log("Loading params for " + params.name);
        /** @type {Object} */
        this.params = params;

        /** @type {String} */
        this.name = params.name;

        /** @type {Position} */
        this.position = new Position(params.pos.x, params.pos.y, params.pos.z);

        /** @type {Position} */
        this.rotation = new Position(params.rotation.x, params.rotation.y, params.rotation.z);
        this.rotation.makeRotationFromDegrees();

        /** @type {String} */
        this.source = params.source;

        /** @type {String} */
        this.color = params.color;

        /** @type {String} */
        this.type = params.type;
    }

    /**
     * Met à jour la position et la rotation de l'objet 3D.
     * Si certains paramètres ne sont pas définis, ils ne seront pas modifiés.
     * 
     * @param {Object} params Paramètres divers de l'objet 3D
     */
    updateParams (params)
    {
        if(params.pos)
            this.setPosition(params.pos);
        if(params.rotation)
            this.setRotation(params.rotation);
    }

    /**
     * Charge le mesh contenu dans le fichier collada spécifié par {@link Object3d#source}
     * 
     * Appelle la fonction onSucess avec en paramètre la scène (c'est-à-dire l'objet) losque le chargement est terminé.
     * 
     * @param {function} onSuccess Callback appelé lorsque le chargement du collada est terminé
     */
    loadMesh (onSuccess)
    {
        this.loader.load(this.ressourcesPath + this.source, (collada) => {
            this.mesh = collada.scene;
            this.mesh.position.set(this.position.x, this.position.y, this.position.z);
            this.mesh.rotation.set(this.rotation.x, this.rotation.y, this.rotation.z);
            this.mesh.scale.set(1, 1, 1);
            //this.debug_scene();
            onSuccess(this.mesh);
        });
    }

    /**
     * Change la position de l'objet
     * 
     * @param {Position} pos Nouvelle position
     */
    setPosition (pos)
    {
        this.position = pos;
        if ( this.mesh )
        {
            this.mesh.position.set(pos.x, pos.y, pos.z);
        }
    }

    /**
     * Change la rotation de l'objet
     * 
     * @param {Position} rotation Nouvelle rotation
     */
    setRotation (rotation)
    {
        this.rotation = rotation;
        if ( this.mesh )
        {
            this.mesh.rotation.set(rotation.x, -rotation.y, rotation.z);
        }
    }

    /**
     * Affiche l'objet en mode fils de fer
     * (utile lorsque les textures ne chargent pas ou quand les faces sont transparentes)
     */
    debug_scene ()
    {
        this.mesh.traverse(function ( object ) { 
            if ( object.material ) {
                object.material = new THREE.MeshBasicMaterial( { wireframe: true } );
            }
        } );
    }
}