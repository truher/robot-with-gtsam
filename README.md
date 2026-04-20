# Robot with GTSAM

This is a robot project that demonstrates usage of the GTSAM vendordep.

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