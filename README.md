# Robot with GTSAM

This is a robot project that demonstrates usage of the GTSAM vendordep.

It simulates a robot moving around, and uses GTSAM to localize it.

There are three inputs:

* A single camera and one April Tag. the camera can usually, but not always,
  see the tag.  Each corner of the tag is handled with a separate factor.
* Odometry measurements at each wheel.  These are turned into a single
  "twist" measurement, used by a "Between" factor.
* A gyro.  The old gyro is modeled as a prior.  The new one is uses the
  "PlanarGyroFactor" which is like "between".
  
There are several items plotted on Field2d:

* The (fixed, not uncertain) tag position.
* The robot ground-truth pose (used to compute the camera/odometry/gyro inputs).
* The "mean" pose estimate from GTSAM.
* A bunch of samples using the estimated covariance of the pose estimate.

## Details

The outer loop is handled by `Sim`, which steps the `CircleSimulator`
once per run, and then runs the `Estimate` solver.


## Building

If you've built the gtsam-vendordep locally, then the artifacts will be in $HOME/releases/maven.

To use them here, copy to the wpilib maven, e.g.:

```
cp -r releases/maven/release/org/team100/gtsam-vendordep wpilib/2026/maven/org/team100/
```

## Running

Use "Simulate Robot" to see what this does.

The pose prior is very wide, so the uncertainty at startup is very high,
and the so the solver "looks around" very widely, including at possible
poses where the landmark is "behind" the camera.  (On a real field, of course
most of the landmarks are behind most of the cameras most of the time.)  The
way the "pinhole camera" model works, a "behind" projection will yield a
deceiving result -- equivalent to the reciprocal direction in the front.

To prevent these errors, the GTSAM camera projection throws an exception,
and we catch it in the C++ factor.  At the moment, we print an error when
this happens.  (We should stop that, or make it switchable.  So at startup
there are lots of these messages, as the solver wanders into territory
that yields these "behind" conditions.  After a few iterations, the
pose uncertainty is much improved, and so the solver doesn't wander
as far, and the error messages stop.



## Gradle and JDK version

Because the GTSAM vendordep uses java 25, I upgraded the gradle here to 9.4.1,
by typing this (twice, so the gradle jar is updated):

```
./gradlew wrapper --gradle-version 9.4.1
```

Once gradle is updated, you can update the JDK version in the wpilib tree,
e.g.

```
mv jdk jdk17
ln -sfn /usr/lib/jvm/java-25-openjdk-amd64 jdk
```