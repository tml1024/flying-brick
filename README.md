# flying-brick

## HELLO THERE!

There are now over 30 people who have cloned this repo. But not many
contacts. Please, if this is useful to you, or if you have improvement
suggestions, don't be shy to open a discussion just to tell how it has
helped you, file an issue, or even submit a PR here on GitHub. (But no
personal email expecting personal replies please. That is not how open
source works. Unless you have some business (contracting) proposal,
but I am not really expecting such.)

## Current state

For some reason this thing seems to have become broken at some stage.
Usually when you start flying it doesn't behave as it should. Both the
horizontal and vertical speeds behave very oddly. Occastionally it
does work as it used to, though. Oh well.

## General

This is a sample aircraft for MSFS built from scratch, especially for
demonstrating how to override the "flight model" of MSFS with your own
code that completely overrides how the aircraft behaves.

The 3D model here is *not* intended to be any good. On the contrary,
it is a flying yellow box.

The .cfg files in particular are intended to be extremely verbosely
(and visually nicely) commented. Much improved upon the MSFS SDK
samples. The Aircraft Editor in MSFS has not been allowed to re-write
them.

For more information, it is a good idea to read the git log, as many
commit messages contain useful information. (Well, after the initial
few.)

## Contributing

Patches are welcome (why else would this be on GitHub), but: Please
keep the existing style for code and configuration files:

- The C++ code should *not* use any Hungarian Notation. (Except for
  parameter names to callback functions that are documented using that
  convention.)
- No TAB characters anywhere.
- Verbose and useful commit messages, written in the present tense.
  (Sure, my own commit messages until now were not like that, but that
  was during the very initial hacking when commits were basically just
  arbitrary steps on the path up to making this public.)

## Flying the thing

You need to use a control stick (or yoke that returns to the centre
position if you let go of it), rudder pedals (or twistable joystick),
and a throttle lever.

When you push the stick forward, the aircraft goes forward. When you
pull it backwards, it goes backwars. Push to the side and it goes
sidewards.

Use the rudder pedals (or twist the joystick) and it turns around the
vertical axis.

To control vertical movement, use the throttle. Throttle in the middle
keeps its altidue. Increase throttle and the vertical speed increases.
Decrease throttle and you go downwards.

Very simple. This is a sample and a toy with quite unrelatistic
behaviour. No inertia or moment of inertia is taken into account. The
controls affect the velocities directly.

## Problems

The behaviour when "landing" is slightly broken. The state management
in the Wasm module needs work.

There is occasionally some stutter in the view, especially close to
ground. But it seems to be better now than before the last MSFS Sim
Update.
