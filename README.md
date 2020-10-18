# Lean back to scroll

This is Arduino code that lets you scroll websites and documents by leaning back in your chair. It's meant to run on Adafruit's [Feather nRF52840 Sense](https://www.adafruit.com/product/4516) board that's attached to the back of the chair.

[Demo video.](https://www.youtube.com/watch?v=mv5tD-7EkHM)

It uses the sensors on the board to figure out the orientation and works as a Bluetooth mouse that only does one thing - scroll down. Doing 9DOF sensor fusion just to get the pitch angle is a bit of an overkill when one accelerometer value would do the trick, but I already had that code ready for other projects so I just reused it.

Press the "user switch" button on the board to set the "zero" position.

Have fun!
