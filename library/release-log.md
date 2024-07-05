# release log for the KK Physics library

## Version 0.2.0 released on TBD

This was another proof-of-concept pre-release.

+ Switched from the jolt-java JVM bindings to jolt-jni.  This expanded support
  to include MacOS and Windows platforms and eliminated the dependencies on
  JDK 19 preview features.
+ Added 2 collision shapes:  `HullCollisionShape` and `MeshCollisionShape`.
+ Added `BulletAppState` accessors to configure debug visualization.
+ Increased the maximum number of bodies in a `PhysicsSpace` to 15,000.
+ Added `PhysicsSpace` configuration for broadphase optimization frequency.
+ Did some preliminary performance tuning.

## Version 0.1.0 released on 23 June 2024

This was a proof-of-concept pre-release,
dependent on v0.1.0 of the jolt-java library for its JVM bindings.

Features:

+ static and dynamic rigid bodies with friction and restitution
+ 4 collision shapes (box, Y-axis capsule, Y-axis cylinder, and sphere)
+ `BulletAppState` and `RigidBodyControl`
+ debug visualization
+ `PhysicsDumper` and `PhysicsDescriber`
+ mimics the APIs of Minie and jme3-jbullet

Known issues:

+ required Java 19 preview features
+ only worked on Linux
+ `getLinearVelocity()` often returned a wrong value