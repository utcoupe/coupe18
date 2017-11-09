# Definitions package

This package is used by others to get the full path to the definitions files they need. These files are all stores in the src/definitions directory. They are arranged by domain (ai, navigation, ...), and each definition file has to start with a capital letter.

The service request needs the domain and the name. The file is fetched following the pattern `definitions/<domain>/<name>`, transforming the name with `title()`.
