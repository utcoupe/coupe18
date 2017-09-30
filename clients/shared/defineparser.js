/**
 *  @file       Describes the define parser module.
 *  <pre>
 *  Converts #define from a .h file to a json object :
 *  #define VALUEA
 *  #define VALUEB 'b'
 *  #define VALUEC 42
 *  In :
 *  {
 *	    VALUEA : null,
 *	    VALUEB : "b",
 *	    VALUEC : 42
 *  }</pre>
 *  @date       01/04/2017
 *  @module     clients/shared/defineParser
 *  @copyright  Copyright (c) 2017 UTCoupe All rights reserved.
 *  @licence    See licence.txt file
 */

module.exports = (function () {
    var logger = require('log4js').getLogger('DefineParser');
    var fs = require('fs');
    // Regular expression to be used to identify define
    var reg = /#define[ \t]+(\S+)[ \t]+(\S+)/g;

    return function parse(file) {
        var parsed = {};
        fs.readFile(file, 'utf8', function (err, data) {
            if (err) {
                logger.fatal("cant read file:\"" + file + "\"");
                return {};
            }
            var nb = 0;
            while (findings = reg.exec(data)) {
                try {
                    parsed[findings[1]] = eval(findings[2]);
                } catch (e) {
                }
                // try to evaluate calculus...
                nb++;
            }
            logger.info("done parsing \"" + file + "\" with " + nb + " defines");
            //should be quick enough
        });
        return parsed;
    }
})();