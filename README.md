# flying-brick

This is a sample aircraft for MSFS built from scratch, especially for
demonstrating how to override the "flight model" of MSFS with your own
code that completely overrides how the aircraft behaves.

The 3D model here is *not* intended to be very good. On the contrary.

The .cfg files in particular are intended to be extremely verbosely
(and visually nicely) commented. Much improved upon the MSFS SDK
samples. The Aircraft Editor in MSFS has not been allowed to re-write
them.

When building the standalone Wasm modukle you need to have the
MSFSLayoutGenerator executable in your PATH. Build it from
https://github.com/HughesMDflyer4/MSFSLayoutGenerator . It is used to
automatically update the layout.json file of the package that the VS
projects plops the resulting .wasm file into. So you should do "Build
All" in the MSFS Project Editor first.

For more information about this, it is a good idea to read the git
log, as many commit messages contain useful information. (Well, after
the initial few.)

Patches are welcome (why else would this be on GitHub), but: Please
stay with the same style for code and configuration files:

- The C++ code should *not* use any Hungarian Notation. (Except for
  parameter names to callback functions that are documented using that
  convention.)
- No TAB characters anywhere.
- Verbose and useful commit messages, written in the present tense.
  (Sure, my own commit messages until now were not like that, but that
  was during the very initial hacking when commits were basically just
  arbitrary steps on the path up to making this public.)
