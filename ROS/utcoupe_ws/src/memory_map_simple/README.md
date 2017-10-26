This is a simple version of the map package.
It simply loads the map.yml description file into a python dict.

__CAUTION__ : Strictly no data validation nor error handling yet!

# Getting data from the database

## Request

Use a string that gives the path to the desired value with the appropriate format. For instance, to get the color of the object `cube_1`, type `/objects/cube_1/userdata/color`.
This is equivalent to writing in python `MapDict["objects"]["cube_1"]["userdata"]["color"]`

## Response

The module returns a JSON with all the content localted inside the last element the request path gave.

# Setting data