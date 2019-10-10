#
# Main Makefile. This is basically the same as a component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_SRCDIRS := . ./esp32-ds18b20 ./esp32-owb
COMPONENT_ADD_INCLUDEDIRS := . ./include ./esp32-ds18b20/include ./esp32-owb/include