/*
 * Copyright (c) 2009-2019 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jme3.bullet.objects;

import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.infos.RigidBodySnapshot;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Vec3d;
import java.lang.foreign.MemorySession;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jolt.math.FVec3;
import jolt.math.Quat;
import jolt.physics.body.BodyCreationSettings;
import jolt.physics.body.BodyInterface;
import jolt.physics.body.MotionType;
import jolt.physics.body.MutableBody;
import jolt.physics.body.MutableMotionProperties;

/**
 * A collision object to simulate a rigid body, based on jolt-java's
 * {@code MutableBody}.
 *
 * @author normenhansen
 */
public class PhysicsRigidBody {
    // *************************************************************************
    // constants and loggers

    /**
     * magic mass value used to specify a static rigid body
     */
    final public static float massForStatic = 0f;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsRigidBody.class.getName());
    // *************************************************************************
    // fields

    /**
     * current shape of this body
     */
    private CollisionShape shape;
    /**
     * JVM copy of the mass (&gt;0) of a dynamic body, or 0 for a static body
     */
    protected float mass = 1f;
    /**
     * underlying jolt-java object
     */
    private MutableBody joltBody;
    /**
     * scene object that's using this collision object. The scene object is
     * typically a PhysicsControl, PhysicsLink, or Spatial. Used by physics
     * controls.
     */
    private Object userObject = null;
    /**
     * where this body has been added, or null if removed or never added
     */
    private PhysicsSpace addedToSpace = null;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected PhysicsRigidBody() {
        // do nothing
    }

    /**
     * Instantiate a responsive, dynamic body with mass=1 and the specified
     * CollisionShape. The new body is not added to any PhysicsSpace.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public PhysicsRigidBody(CollisionShape shape) {
        this(shape, 1f);
        assert mass == 1f : mass;
    }

    /**
     * Instantiate a responsive dynamic or static body with the specified
     * CollisionShape and mass. The new body is not added to any PhysicsSpace.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass if 0, a static body is created; otherwise a dynamic body is
     * created (&ge;0, default=1)
     */
    public PhysicsRigidBody(CollisionShape shape, float mass) {
        this(shape, mass, Vec3d.ZERO, Quaternion.IDENTITY);
    }

    /**
     * Instantiate a responsive dynamic or static body with the specified
     * CollisionShape, mass, and initial position. The new body is not added to
     * any PhysicsSpace.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass if 0, a static body is created; otherwise a dynamic body is
     * created (&ge;0, default=1)
     * @param location the desired initial location (in physics-space
     * coordinates, not null, unaffected)
     * @param orientation the desired initial orientation (in physics-space
     * coordinates, not null, unaffected)
     */
    public PhysicsRigidBody(CollisionShape shape, float mass, Vec3d location,
            Quaternion orientation) {
        Validate.nonNull(shape, "shape");
        Validate.nonNegative(mass, "mass");
        Validate.nonNull(location, "location");
        Validate.nonZero(orientation, "orientation");

        this.shape = shape;
        this.mass = mass;
        this.joltBody = createRigidBody(shape, mass, location, orientation);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy this body's angular velocity. The body must be in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (radians per second in physics-space
     * coordinates, either storeResult or a new vector, not null)
     */
    public Vec3d getAngularVelocityDp(Vec3d storeResult) {
        assert isDynamic();
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        MutableMotionProperties properties = joltBody.getMotionProperties();
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena);
        properties.getAngularVelocity(fvec3);
        result.set(fvec3.getX(), fvec3.getY(), fvec3.getZ());

        return result;
    }

    /**
     * Access the space where this body has been added.
     *
     * @return the pre-existing instance, or null if not added
     */
    public PhysicsSpace getCollisionSpace() {
        return addedToSpace;
    }

    /**
     * Copy the linear velocity of this body's center of mass. The body must be
     * in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (physics-space units per second in
     * physics-space coordinates, either storeResult or a new vector, not null)
     */
    public Vec3d getLinearVelocityDp(Vec3d storeResult) {
        assert isDynamic();
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        MutableMotionProperties properties = joltBody.getMotionProperties();
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena);
        properties.getLinearVelocity(fvec3);
        result.set(fvec3.getX(), fvec3.getY(), fvec3.getZ());

        return result;
    }

    /**
     * Return the body's mass.
     *
     * @return the mass (&gt;0) or zero for a static body
     */
    public float getMass() {
        return mass;
    }

    /**
     * Copy the location of this object's center to a Vector3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, finite)
     */
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena);
        BodyInterface bodyInterface = addedToSpace.getBodyInterface();
        int bodyId = joltBody.getId();
        bodyInterface.getCenterOfMassPosition(bodyId, fvec3);
        result.x = fvec3.getX();
        result.y = fvec3.getY();
        result.z = fvec3.getZ();

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the location of this object's center to a Vec3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    public Vec3d getPhysicsLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena);
        BodyInterface bodyInterface = addedToSpace.getBodyInterface();
        int bodyId = joltBody.getId();
        bodyInterface.getCenterOfMassPosition(bodyId, fvec3);
        result.x = fvec3.getX();
        result.y = fvec3.getY();
        result.z = fvec3.getZ();

        assert result.isFinite();
        return result;
    }

    /**
     * Copy the orientation (rotation) of this object to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation Quaternion (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        MemorySession arena = PhysicsSpace.getArena();
        Quat wr = Quat.of(arena);
        joltBody.getRotation(wr);
        result.set(wr.getX(), wr.getY(), wr.getZ(), wr.getW());

        return result;
    }

    /**
     * Access the scene object that's using this collision object, typically a
     * PhysicsControl, PhysicsLink, or Spatial. Used by physics controls.
     *
     * @return the pre-existing instance, or null if none
     */
    public Object getUserObject() {
        return userObject;
    }

    /**
     * Test whether this body is in dynamic mode.
     *
     * @return true if in dynamic mode, otherwise false (static/kinematic mode)
     */
    public boolean isDynamic() {
        boolean result = (joltBody.getMotionProperties() != null);
        return result;
    }

    /**
     * Return the ID of the assigned native object, assuming that one is
     * assigned.
     *
     * @return the native identifier
     */
    public long nativeId() {
        long result = joltBody.getId();
        return result;
    }

    /**
     * Rebuild this rigid body with a new native object.
     */
    public void rebuildRigidBody() {
        MutableBody oldBody = joltBody;
        Vec3d location = new Vec3d();
        Quaternion orientation = new Quaternion();
        RigidBodySnapshot snapshot = null;
        PhysicsSpace removedFrom = addedToSpace;

        if (oldBody != null) {
            getPhysicsLocationDp(location);
            getPhysicsRotation(orientation);
            snapshot = new RigidBodySnapshot(this);

            if (removedFrom != null) {
                removedFrom.removeCollisionObject(this);
            }

            logger2.log(Level.INFO, "Clearing {0}.", this);
        }

        this.joltBody = createRigidBody(shape, mass, location, orientation);
        if (logger2.isLoggable(Level.INFO)) {
            if (oldBody != null) {
                logger2.log(
                        Level.INFO, "Created {0}.", joltBody);
            } else {
                logger2.log(
                        Level.INFO, "Substituted {0} for {1}.", new Object[]{
                            joltBody, oldBody
                        });
            }
        }

        if (removedFrom != null) {
            removedFrom.addCollisionObject(this);
        }
        if (snapshot != null) {
            snapshot.applyTo(this);
        }
        // TODO kinematic flag, gravity, physics joints
    }

    /**
     * Rebuild this rigid body in a new position.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, not zero, unaffected)
     */
    public void reposition(Vector3f location, Quaternion orientation) {
        RigidBodySnapshot snapshot = new RigidBodySnapshot(this);
        MutableBody oldBody = joltBody;
        PhysicsSpace removedFrom = addedToSpace;

        if (removedFrom != null) {
            removedFrom.removeCollisionObject(this);
        }

        Vec3d vec3d = new Vec3d(location);
        this.joltBody = createRigidBody(shape, mass, vec3d, orientation);
        if (logger2.isLoggable(Level.INFO)) {
            if (oldBody != null) {
                logger2.log(
                        Level.INFO, "Created {0}.", joltBody);
            } else {
                logger2.log(
                        Level.INFO, "Substituted {0} for {1}.", new Object[]{
                            joltBody, oldBody
                        });
            }
        }

        if (removedFrom != null) {
            removedFrom.addCollisionObject(this);
        }
        if (snapshot != null) {
            snapshot.applyTo(this);
        }
        // TODO kinematic flag, gravity, physics joints
    }

    /**
     * Alter the {@code addedToSpace} field. Internal use only.
     *
     * @param physicsSpace (may be null, alias created)
     */
    public void setAddedToSpaceInternal(PhysicsSpace physicsSpace) {
        this.addedToSpace = physicsSpace;
    }

    /**
     * Alter this body's angular velocity.
     *
     * @param omega the desired angular velocity (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setAngularVelocityDp(Vec3d omega) {
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena, (float) omega.x, (float) omega.y,
                (float) omega.z);
        joltBody.setAngularVelocity(fvec3);
    }

    /**
     * Alter the linear velocity of this body's center of mass.
     *
     * @param velocity the desired velocity (physics-space units per second in
     * physics-space coordinates, not null, unaffected)
     */
    public void setLinearVelocityDp(Vec3d velocity) {
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena, (float) velocity.x, (float) velocity.y,
                (float) velocity.z);
        joltBody.setLinearVelocity(fvec3);
    }

    /**
     * Associate a "user" with this collision object. Used by physics controls.
     *
     * @param user the desired scene object (alias created, default=null)
     * @see #getUserObject()
     */
    public void setUserObject(Object user) {
        this.userObject = user;
    }
    // *************************************************************************
    // private methods

    /**
     *
     * @param shape
     * @param mass
     * @param location
     * @param orientation
     * @return
     */
    private static MutableBody createRigidBody(CollisionShape shape, float mass,
            Vec3d location, Quaternion orientation) {
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena, (float) location.x, (float) location.y,
                (float) location.z);
        Quat quat = Quat.of(arena, orientation.getX(), orientation.getY(),
                orientation.getZ(), orientation.getW());

        BodyCreationSettings settings;
        if (mass > 0f) {
            settings = BodyCreationSettings.of(arena, shape.getJoltShape(),
                    fvec3, quat, MotionType.DYNAMIC,
                    PhysicsSpace.OBJ_LAYER_MOVING);
            settings.setGravityFactor(1f);
        } else {
            assert mass == 0f : mass;
            settings = BodyCreationSettings.of(arena, shape.getJoltShape(),
                    fvec3, quat, MotionType.STATIC,
                    PhysicsSpace.OBJ_LAYER_NON_MOVING);
            settings.setGravityFactor(0f);
        }

        PhysicsSpace physicsSpace
                = (PhysicsSpace) CollisionSpace.getCollisionSpace();
        BodyInterface bodyInterface = physicsSpace.getBodyInterface();
        MutableBody result = bodyInterface.createBody(settings);

        return result;
    }
}
