# Vision tracking readme

This is the vision tracking code for 2013. It uses Roborealm, a vision-processing
tool.

The camera sends images via the router to the computer (black-and-white, 320x240 resolution 
with 50% compression), which then runs the Roborealms script to detect the targets,
their center coordinates, sizes, and distance from the robot. It then transfers this
data to the robot using NetworkTables.

The 'detect_rectangles.robo` is the Roborealm script file. You can open it and view it
to see how the various filters work (it's just an xml file). The python script is
essentially a filter called by the Roborealm file to calculate the distances of the 
and prepares the data for transfer via NetworkTables.

