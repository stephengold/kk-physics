# release log for the KK Physics library

## Version 0.3.1 released on TBD

Pre-release for limited testing.

+ Updated the jolt-jni library to v0.9.5.
+ Added the `PlaneCollisionShape` class.
+ Made contact response optional for rigid bodies.
+ Added another `MeshCollisionShape` constructor.
+ Moved the `PhysicsSystem` from `PhysicsSpace` to `CollisionSpace`.
+ Added package javadoc.

## Version 0.3.0 released on 3 August 2024

Pre-release for limited testing.

+ Deferred the creation of jolt-jni bodies until the `PhysicsSystem` is stepped.
+ Added a cleaner thread for Jolt physics objects.
+ Implemented kinematic bodies, `CollisionShapeFactory`, `RigidBodyControl`,
  collision margins, and custom debug materials.
+ Added 3 collision shapes:
  + `CompoundCollisionShape`
  + `HeightfieldCollisionShape`
  + `SimplexCollisionShape`
+ Implemented alternative orientations of capsule and cylinder shapes.
+ Implemented debug visualization for bounding boxes.
+ Replaced `RigidBodySnapshot` with `BodyCreationSettings`.
+ Updated the jolt-jni library to v0.5.0,
  picking up support for Linux-on-ARM platforms.

## Version 0.2.0 released on 5 July 2024

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