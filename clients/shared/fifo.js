/**
 *  @file       Describes the fifo module.
 *  @date       01/04/2017
 *  @module     clients/shared/fifo
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

"use strict";

/**
 * The Fifo class implements a First In First Out structure.
 * This structure handles order stacking.
 * The use of this structure is easy, you have to create new orders. At each new order, a new order is called (if available).
 * For each order added to the Fifo objects, you have to call the orderFinished method in the callback send when creating a new order.
 * This is mandatory to make the module work properly.
 */
class Fifo {
    /**
     * Creates an empty fifo object.
     */
    constructor() {
        this.logger = require('log4js').getLogger('Fifo');
        this.clean();
        this.current_order_name = null;
    }

    /**
     * Clean the fifo, remove all the stacked orders.
     * @public
     */
    clean() {
        this.fifo = [];
        this.order_in_progress = false;
        this.current_order_name = null;
    }

    /**
     * This method has to be called by each order added to the fifo, when the order is finished.
     * When an order is finished, the fifo object calls the next order.
     * @public
     */
    orderFinished() {
        this.logger.info(this.current_order_name + " done !");
        this.current_order_name = null;
        this.order_in_progress = false;
        this.nextOrder();
    }

    /**
     * Add a new order in the fifo.
     * This method will also call the next order.
     * @public
     * @param {function} callback   The function to stack in the fifo
     * @param {string} name         The name of the order
     */
    newOrder(callback, name) {
        if (name === undefined)
            name = "";
        this.fifo.push({callback: callback, name: name});
        this.nextOrder();
    }

    /**
     * Call the next order of the fifo.
     * @private
     */
    nextOrder() {
        if (!this.order_in_progress && this.fifo.length > 0) {
            // logger.debug(this.fifo.length);
            this.order_in_progress = true;
            var object = this.fifo.shift();
            this.current_order_name = object.name;
            this.logger.info("Doing " + this.current_order_name);
            object.callback();
        }
    }
}

module.exports = Fifo;
