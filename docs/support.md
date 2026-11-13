# Support

Help with the openrz67 apps.

The apps control the openrz67 trigger, a Bluetooth remote release you build yourself for the
Mamiya RZ67. They need the trigger to do anything. The [build guide](build-guide.md) shows how
to build one.

## The app stays on "Scanning for devices"

- Press the button on the trigger to wake it. A short blink every 2 seconds means it is
  waiting for a phone. It goes back to sleep after 3 minutes without one.
- Check that Bluetooth is on and that the app is allowed to use it. On iPhone: Settings →
  Apps → OpenRZ67 → Bluetooth (on iOS 17: Settings → OpenRZ67).
- The trigger talks to one phone at a time. Close the app on any other phone that is connected
  to it.

## The connection dropped

The app reconnects on its own when the trigger comes back in range. The status line says
"Waiting for trigger" meanwhile. **Reconnect** makes the app forget the trigger and scan again,
which helps if you switched to a different one.

## The shutter does not fire

- The LED is off while the trigger is connected and idle. It lights for 2 seconds after a
  release and blinks once a second while it holds the shutter open in bulb.
- Check the camera cable and that pin 1 is the right way round at both ends.

## Bulb mode takes a normal picture

The trigger only holds the release. Set the shutter-speed dial on the camera to `B`, or it
takes one exposure at the dialled speed. The camera closes the shutter by itself after about
60 seconds.

## Contact

Report bugs and ask questions in the issue tracker for the
[iOS app](https://github.com/mhellevang/openrz67-ios/issues), the
[Android app](https://github.com/mhellevang/openrz67-android/issues) or the
[trigger](https://github.com/mhellevang/openrz67-trigger/issues).
