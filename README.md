# Robot with GTSAM

This is a robot project that demonstrates usage of the GTSAM vendordep.

It simulates a robot moving around, and uses GTSAM to localize it.

There are three inputs:

* A single camera and one April Tag. the camera can usually, but not always,
  see the tag.  Each corner of the tag is handled with a separate factor.
* Odometry measurements at each wheel.  These are turned into a single
  "twist" measurement, used by a "Between" factor.
* A gyro, currently modeled as a prior.  This should be changed to the new
  "between" type.

There are several items plotted on Field2d:

* The (fixed, not uncertain) tag position.
* The robot ground-truth pose (used to compute the camera/odometry/gyro inputs).
* The "mean" pose estimate from GTSAM.


## Building

If you've built the gtsam-vendordep locally, then the artifacts will be in $HOME/releases/maven.

To use them here, copy to the wpilib maven, e.g.:

```
cp -r releases/maven/release/org/team100/gtsam-vendordep wpilib/2026/maven/org/team100/
```

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