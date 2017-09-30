/**
 * @file Controlleur du simulateur
 * @author Mindstan
 * 
 * @requires THREE
 * @requires THREE.OrbitControls
 * @requires position
 * @requires object3d
 */

"use strict";

/**
 * Gère le simulateur
 */
class Controller {
    /**
     * Constructeur du controlleur
     * 
     * Prend en paramètre le chemin d'accès à la configuration et le chemin d'accès aux ressources.
     * 
     * @param {String} configPath Chemin d'accès au fichier de coniguration relatif à {@link Controller.ressourcesPath}
     * @param {String} ressourcesPath Chemin d'accès aux ressources appelées par le simulateur
     */
    constructor(configPath, ressourcesPath) {
        /**
         * Chemin d'accès au fichier contenant la configuration du simulateur.
         * @see Controller#ressourcesPath
         * @type {String}
         */
        this.configPath = configPath;

        /**
         * Chemin d'accès aux différentes ressources chargées par le simulateur.
         * @type {String}
         */
        this.ressourcesPath = ressourcesPath;

        // A charger dynamiquement du fichier
        /**
         * Id de la balise qui contiendra le simulateur.
         * @type {String}
         * */
        this.container = document.getElementById("simulateur_container");

        /** @type {Map<String, Object3d>} */
        this.objects3d = new Map();

        /** @type {Array<external:THREE.DirectionalLight>} */
        this.directionLights = [];

        /**
         * @type {Number}
         * @const
         */
        this.RADIUS_PR = 0.1;

        /**
         * @type {Number}
         * @const
         */
        this.RADIUS_GR = 0.2;
    }

    /**
     * Charge tous les paramètres
     * @see {@link Controller#configPath}
     * @see {@link Controller#ressourcesPath}
     */
    loadParameters() {
        getParsedJSONFromFile(
            this.ressourcesPath + this.configPath,
            (objects) => { this.create3dObjects(objects) }
        );
    }

    /**
     * Crée tous les objets 3D passés en paramètres
     * 
     * @param {Array<Object>} objects liste de tous les objets à créer
     * @see {@link Controller#ressourcesPath}
     */
    create3dObjects(objects) {
        console.log("Creating the 3D objects...");
        for (var idObject = 0; idObject < objects.length; idObject++) {
            var name = objects[idObject].name;
            //console.log("Creating " + name);
            switch (objects[idObject].type) {
            case "pr":
                this.objects3d.set(
                    name,
                    new Robot(objects[idObject],
                        this.ressourcesPath,
                        (pathLine) => { this.scene.add(pathLine); },
                        this.RADIUS_PR,
                        this.autotrash.bind(this)
                    ));
                break;

            case "gr":
                this.objects3d.set(
                    name,
                    new Robot(objects[idObject],
                        this.ressourcesPath,
                        (pathLine) => { this.scene.add(pathLine); },
                        this.RADIUS_GR,
                        this.autotrash.bind(this)
                    ));
                break;

            default:
                this.objects3d.set(name, new Object3d(objects[idObject], this.ressourcesPath));
            }
            this.objects3d.get(name).loadMesh((scene) => {
                this.scene.add(scene);
            });
        }
    }

    /**
     * Crée le moteur de rendu
     */
    createRenderer() {
        // hauteur de la zone de rendu
        var height = Math.max(
            $('body').height() - $('#div_menu').outerHeight() - 2 * $('#simu_before').outerHeight(),
            200
        );
        //alert($('body').height() - $('#div_menu').outerHeight() - 2*$('#simu_before').outerHeight());
        // largeur de la zone de rendu
        var width = $('#simulateur_container').width();

        /** @type {external:THREE.Scene} */
        this.scene = new THREE.Scene();
        /** @type {external:THREE.PerspectiveCamera} */
        this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 10);

        /** @type {external:THREE.WebGLRenderer} */
        this.renderer = new THREE.WebGLRenderer();
        this.renderer.setSize(width, height);
        this.renderer.setClearColor(0x272525, 0.5);
        this.container.appendChild(this.renderer.domElement);

        /** @type {external:THREE.OrbitControls} */
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);

        /** @type {external:THREE.AxisHelper} */
        this.axisHelper = new THREE.AxisHelper(5);
        this.scene.add(this.axisHelper);

        this.createLights();
        this.selectView("front");

        // Permet de changer la taille du canva de THREE en fonction de la taille de la fenêtre
        window.addEventListener('resize', () => {
            const HEIGHT = Math.max(
                $('body').height() - $('#div_menu').outerHeight() - 2 * $('#simu_before').outerHeight(),
                200
            );
            const WIDTH = $('#simulateur_container').width();
            this.renderer.setSize(WIDTH, HEIGHT);
            this.camera.aspect = WIDTH / HEIGHT;
            this.camera.updateProjectionMatrix();
        });

        // On lance le rendu
        this.render();
    }

    /**
     * Met à jour le rendu lorsque la page est rechargée par angular
     */
    updateRenderer() {
        this.container = document.getElementById("simulateur_container");
        this.container.appendChild(this.renderer.domElement);
    }

    /**
     * Crée et insert des lumières directionnelles dans la scène
     */
    createLights() {
        var largeurTable = 2;
        var longueurTable = 3;
        var heightLights = 2;
        // Les lumières sont disposés haut-dessus des quattres coins du plateau, avec un offset de 1
        var posLights = [
            new Position(-largeurTable / 2, heightLights, -longueurTable / 2),
            new Position(largeurTable * 3 / 2, heightLights, -longueurTable / 2),
            new Position(-largeurTable / 2, heightLights, longueurTable * 3 / 2),
            new Position(largeurTable * 3 / 2, heightLights, longueurTable * 3 / 2)
        ];

        posLights.forEach(function (pos) {
            var light = new THREE.DirectionalLight(0xffffff, 1);
            light.position.set(pos.x, pos.y, pos.z);
            light.intensity = 0.5;
            this.directionLights.push(light);
            this.scene.add(this.directionLights[this.directionLights.length - 1]);
        }, this);
    }

    /**
     * Crée une boucle et met à jour le rendu à 60fps
     */
    render() {
        requestAnimationFrame(() => {
            this.render();
        });
        this.renderer.render(this.scene, this.camera);
    }

    test() {
        var geometry = new THREE.BoxGeometry(1, 1, 1);
        var material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        var cube = new THREE.Mesh(geometry, material);
        this.scene.add(cube);

        this.camera.position.z = 5;
    }

    /**
     * Affiche la vue désirée
     * 
     * Valeurs possibles : "front", "top", "behind", "left", "right"
     * 
     * @param {String} view Vue désirée
     */
    selectView(view) {
        const X_MAX = 3;
        const Z_MAX = 2;
        console.log("Changement de vue : " + view);
        switch (view) {
            case "front":
                this.controls.reset();
                this.camera.position.set(1.5, 1.5, 3.5);
                this.camera.rotation.set(-0.5, 0, 0);
                this.controls.target.set(X_MAX/2, 0, Z_MAX/2);
                break;
            
            case "top":
                this.controls.reset();
                this.camera.position.set(1.5, 3, 1);
                this.camera.rotation.set(-1.6, 0, 0);
                this.controls.target.set(X_MAX/2, 0, Z_MAX/2);
                break;

            case "behind":
                this.controls.reset();
                this.camera.position.set(1.5, 1, -1.5);
                this.camera.rotation.set(-2.5, 0, 3.0);
                this.controls.target.set(X_MAX/2, 0, Z_MAX/2);
                break;

            case "left":
                this.controls.reset();
                this.camera.position.set(-0.8, 1.5, 1);
                this.camera.rotation.set(-1.6, -1, -1.6);
                this.controls.target.set(X_MAX/2, 0, Z_MAX/2);
                break;
            
            case "right":
                this.controls.reset();
                this.camera.position.set(4, 0.5, 1);
                this.camera.rotation.set(-1.5, 1.5, 1.5);
                this.controls.target.set(X_MAX/2, 0, Z_MAX/2);
                break;

            default: // Invalide
                console.warn("Attention : \"" + view + "\" n'est pas une vue valide.");
        }
    }

    /**
     * Met à jour les données des objets dans param. Cette fonction est destinée à être appelée depuis le webclient.
     * 
     * @param {Object} params 
     */
    updateObjects(params) {
        params.forEach(function (object3d) {
            this.objects3d.get(object3d.name).updateParams(object3d);
        }, this);
    }

    /**
     * Fonction destinée a être appelée par les robots pour enlever (mettre invisible) des éléments du simulateur
     * 
     * @param {any} pos 
     * @param {any} radius
     */
    autotrash(pos, radius) {
        /*if (!this.objects3d) // Pour que la fonction ne soit pas lancée lorsque la table n'est pas construite
            return;*/
        var blacklist = ["pr", "gr", "plateau"];
        this.objects3d.forEach((object3d) => {
            if (blacklist.indexOf(object3d.type) == -1) {
                if (object3d.position.get2dDistance(pos) <= radius && object3d.mesh) {
                    //console.log("Make invisible : " + object3d.name);
                    object3d.mesh.traverse( ( object ) => { object.visible = false; } );
                }
            }
        }, this)
    }
}