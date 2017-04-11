***** Build environment *****

This build process has been done to work under Linux OS but without any
gurantees.

Build process options:
  make all              Build library for all target
  make NUCLEO_F429ZI    Build library for Nucleo F429ZI target
  make DISCO_F407VG     Build library for STM32F407G-DISC1 target
  make clean            Delete all files created (.o, .a)

The build process create one library per target and copy the library inside the
variant folder of each target.
This library is used by Arduino IDE to generate the compiled code.
