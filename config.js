/**
 * Configuration globale pour tous les modules
 *
 * @module config
 * @type {Object}
 */

module.exports = {
	/**
	 * Adresse du serveur node hébergeant
	 * @type {String}
	 */
	server: "127.0.0.1:3128",
	// server: "192.168.1.131:3128",

	/**
	 * Commande pour démarrer les hokuyos
	 * @type {String}
	 */
	hokuyo_command: "$UTCOUPE_WORKSPACE/bin/hokuyo"
}
