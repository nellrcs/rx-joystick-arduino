  * Implementation of externally accessible (in other applications) virtual joysticks on operating systems that do not use uinput (like `MacOs` or Windows).
  * Make the automatic Arduino detection platform independent
> Currently this works only on Linux, it may also work on `MacOs`, but that is not tested.
  * Improve the topology of the PWM/PPM switch, in the current design, it's not safe to set the switch to BAT/PPM, when both the D.S.C. cable is connected and the Ch7 connector is used.
  * Improve performance on the computer-side
> On an Intel i5, the program uses 0.8% of one cpu core, which is much IMO, given that the functions used of the python `serial` module are interrupt-based.
  * Make more use of OO programming in the python code
  * Test with more receivers (currently tested with Graupner JR `R700` and Futaba R142JE)