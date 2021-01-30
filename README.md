# J-ZX81
The original Java ZX81 emulator!

I wrote this during the 98/99 academic year for my final year university project 22 years ago :-)

Now available again with source code, and patched to compile and run with the latest version
of Java (SE 15).  I had to make a few classes public as calling methods in non public classes via
a public reference is no longer allowed, but surprisingly no other code changes were required.

To try simply download and run the [jar](https://github.com/RobinSergeant/J-ZX81/releases/latest/download/J_ZX81.jar) file.

## Windows 10 display fix

If the display looks wrong (font not scaled properly) find the Java exectutable (e.g. C:\Program Files\Java\jdk-15.0.2\bin) and
set "High DPI scaling override" to "System" (right click exe, select Properties, then select "Change high dpi settings" from the
Compatibility tab).  Now restart the emulator from the command line:

    java -jar J_ZX81.jar

## Quick start instructions

Those not familiar with the ZX81 keyboard will probably find the layout very confusing at first. Selecting
'Keyboard Layout' from the help menu will display a scanned image of the original keyboard.

![Alt text](/data/zx81keyb.jpg?raw=true)

The emulator maps your keyboard to this layout, except for the following exceptions/additions:

- Shift maps to Shift.
- Enter maps to NewLine.
- The BackSpace key can be used in place of Shift+0 (Rubout).
- The SpaceBar maps to the Space key and can be used in place of the key next to M.
- The cursor keys can be used in place of the 5, 6, 7, and 8 keys.

## Playing games!

- Download your favourite games in standard .p file format.
- Set the tape file directory (under the Settings menu) by selecting any file in the directory.
- Load using the Sinclair BASIC load command, e.g. to load the game "3D Monster Maze" which can be found on the
internet as the file "3dmaze.p" you would need to enter LOAD "3DMAZE" (J, Shift+P etc.).
