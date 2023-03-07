# This example is used to demonstrate the running example in the esp-idf V5.0 environment.
The build steps are as follows:

```
cd Enfield_Smart_Display

get_idf # Get the current idf environment

idf.py set-target esp32s3

idf.py build flash monitor 
