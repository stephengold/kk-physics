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
public class PhysicsRigidBody extends PhysicsBody {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(PhysicsRigidBody.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the mass (&gt;0) of a dynamic body, or 0 for a static body
     */
    protected float mass = 1f;
    /**
     * underlying jolt-java object
     */
    private MutableBody joltBody;
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

        if (mass != massForStatic) {
            validateDynamicShape(shape);
        }
        super.setCollisionShape(shape);
        this.mass = mass;
        this.joltBody = createRigidBody(shape, mass, location, orientation);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Apply a central impulse to the body.
     *
     * @param impulse the impulse vector (mass times physics-space units per
     * second in physics-space coordinates, not null, unaffected)
     */
    public void applyCentralImpulse(Vector3f impulse) {
        Validate.finite(impulse, "impulse");

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena, impulse.x, impulse.y, impulse.z);
        joltBody.addImpulse(fvec3);
    }

    /**
     * Copy the body's angular velocity. The body must be in dynamic mode.
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
     * Copy the linear velocity of the body's center of mass. The body must be
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
     * Copy the location of the object's center to a Vector3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, finite)
     */
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena);
        BodyInterface bodyInterface
                = ((PhysicsSpace) getCollisionSpace()).getBodyInterface();
        int bodyId = joltBody.getId();
        bodyInterface.getCenterOfMassPosition(bodyId, fvec3);
        result.x = fvec3.getX();
        result.y = fvec3.getY();
        result.z = fvec3.getZ();

        assert Vector3f.isValidVector(result);
        return result;
    }

    /**
     * Copy the location of the object's center to a Vec3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    public Vec3d getPhysicsLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena);
        BodyInterface bodyInterface
                = ((PhysicsSpace) getCollisionSpace()).getBodyInterface();
        int bodyId = joltBody.getId();
        bodyInterface.getCenterOfMassPosition(bodyId, fvec3);
        result.x = fvec3.getX();
        result.y = fvec3.getY();
        result.z = fvec3.getZ();

        assert result.isFinite();
        return result;
    }

    /**
     * Copy the orientation (rotation) of the object to a Quaternion.
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
     * Test whether a jolt-java object is assigned to this instance.
     *
     * @return true if one is assigned, otherwise false
     */
    final public boolean hasAssignedNativeObject() {
        if (joltBody == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test whether the body is in dynamic mode.
     *
     * @return true if in dynamic mode, otherwise false (static/kinematic mode)
     */
    public boolean isDynamic() {
        boolean result;
        if (joltBody == null) {
            result = (mass != massForStatic);
        } else if (joltBody.getMotionProperties() == null) {
            result = false;
        } else {
            result = true;
        }

        return result;
    }

    /**
     * Return the ID of the assigned jolt-java object, assuming that one is
     * assigned.
     *
     * @return the jolt-java identifier
     */
    public long nativeId() {
        long result = joltBody.getId();
        return result;
    }

    /**
     * Rebuild the rigid body with a new jolt-java object.
     */
    public void rebuildRigidBody() {
        MutableBody oldBody = joltBody;
        Vec3d location = new Vec3d();
        Quaternion orientation = new Quaternion();
        RigidBodySnapshot snapshot = null;
        PhysicsSpace removedFrom = (PhysicsSpace) getCollisionSpace();
        if (oldBody != null) {
            if (removedFrom != null) {
                getPhysicsLocationDp(location);
                getPhysicsRotation(orientation);
                removedFrom.removeCollisionObject(this);
            }
            snapshot = new RigidBodySnapshot(this);

            logger2.log(Level.INFO, "Clearing {0}.", this);
        }

        CollisionShape shape = getCollisionShape();
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
     * Rebuild the rigid body in a new position.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, not zero, unaffected)
     */
    public void reposition(Vector3f location, Quaternion orientation) {
        RigidBodySnapshot snapshot = new RigidBodySnapshot(this);
        MutableBody oldBody = joltBody;
        PhysicsSpace removedFrom = (PhysicsSpace) getCollisionSpace();

        if (removedFrom != null) {
            removedFrom.removeCollisionObject(this);
        }

        CollisionShape shape = getCollisionShape();
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
     * Alter the body's angular velocity.
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
     * Apply the specified CollisionShape to the body. The body gets rebuilt on
     * the jolt-java side.
     *
     * @param desiredShape the shape to apply (not null, alias created)
     */
    public void setCollisionShape(CollisionShape desiredShape) {
        Validate.nonNull(desiredShape, "collision shape");

        if (desiredShape == getCollisionShape()) {
            return;
        }
        if (isDynamic()) {
            validateDynamicShape(desiredShape);
        }
        super.setCollisionShape(desiredShape);
        rebuildRigidBody();
    }

    /**
     * Alter the linear velocity of the body's center of mass.
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
    // *************************************************************************
    // PhysicsBody methods

    /**
     * Return the body's mass.
     *
     * @return the mass (&gt;0) or zero for a static body
     */
    @Override
    public float getMass() {
        return mass;
    }

    /**
     * Relocate the body's center of mass.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, finite, unaffected)
     */
    @Override
    public void setPhysicsLocation(Vector3f location) {
        Validate.finite(location, "location");

        Quaternion orientation = getPhysicsRotation(null);
        reposition(location, orientation);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the specified {@code MutableBody}.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass if 0, a static body is created; otherwise a dynamic body is
     * created (&ge;0, default=1)
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     * @param orientation the desired orientation (in physics-space coordinates,
     * not null, not zero, unaffected)
     * @return a new instance (not null)
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

    /**
     * Validate a shape as suitable for a dynamic body.
     *
     * @param shape (not null, unaffected)
     */
    private static void validateDynamicShape(CollisionShape shape) {
        assert shape != null;

        if (shape.isNonMoving()) {
            throw new IllegalStateException(
                    "Dynamic rigid body can't have a non-moving shape!");
        }
    }
}
