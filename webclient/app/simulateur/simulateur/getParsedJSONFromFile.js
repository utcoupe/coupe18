/**
 * @file Défini une fonction qui télécharge via AJAX le fichier JSON, puis retourne son contenu converti en objet.
 * @author Mindstan
 * 
 * @requires {JQuery}
 */

/**
 * Télécharge via AJAX le fichier JSON demandé, puis appelle la fonction passée en paramètre avec son contenu converti en objet.
 * 
 * @param {String} fileName Nom du fichier distant
 * @param {function} onSuccess Fonction appelée lorsque la requête a abouti
 * @returns {Object}
 */
function getParsedJSONFromFile(fileName, onSuccess)
{
    /* Ne fonctionne pas en local, retourne une réponse xml à la place de json
    var xmlhttp = new XMLHttpRequest();
    xmlhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            this.overrideMimeType('application/json');
            onSuccess(JSON.parse(this.responseText));
        }
    };
    xmlhttp.open("GET", fileName, true);
    xmlhttp.send();
    */

    // Plutôt ceci :
    $.ajax({
        dataType: "json",
        url: fileName,
        mimeType: "application/json",
        success: function(result){
            console.log("Successfuly retrieved " + fileName + " from the server.");
            onSuccess(result);
        }
    });
    // Fonction de JQUERY, ne fonctionne pas en local
    //$.getJSON(fileName, onSuccess);
}