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

import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyId;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.EMotionType;
import com.github.stephengold.joltjni.MotionProperties;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.Vec3Arg;
import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.objects.infos.RigidBodySnapshot;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Quatd;
import com.simsilica.mathd.Vec3d;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A collision object to simulate a rigid body, based on jolt-jni's {@code Body}
 * class.
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
     * underlying jolt-jni object
     */
    private Body joltBody;
    /**
     * jolt-jni body ID, or null if not added to a space
     */
    private BodyId bodyId;
    /**
     * jolt-jni interface for modifications, or null if not added to a space
     */
    private BodyInterface bodyInterface;
    /**
     * copy of the mass (&gt;0) of a dynamic body, or 0 for a static body
     */
    protected float mass = 1f;
    /**
     * jolt-jni motion properties, or null if not created
     */
    private MotionProperties motionProperties;
    /**
     * temporary storage for properties
     */
    private RigidBodySnapshot snapshot = new RigidBodySnapshot();
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

        this.bodyId = joltBody.getId();
        this.motionProperties = joltBody.getMotionProperties();
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

        if (isDynamic()) {
            Vector3f velocity = getLinearVelocity(null);
            MyVector3f.accumulateScaled(velocity, impulse, 1f / mass);
            setLinearVelocity(velocity);
        }
    }

    /**
     * Return the body's Jolt ID.
     *
     * @return the pre-existing instance, or null if none
     */
    public BodyId findBodyId() {
        return bodyId;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new velocity vector (radians per second in physics-space
     * coordinates, not null)
     */
    public Vector3f getAngularVelocity() {
        assert isDynamic();
        return getAngularVelocity(null);
    }

    /**
     * Copy the body's angular velocity. The body must be in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (radians per second in physics-space
     * coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f getAngularVelocity(Vector3f storeResult) {
        assert isDynamic();
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        if (motionProperties == null) {
            Vec3d vec3d = new Vec3d();
            snapshot.getAngularVelocity(vec3d);
            result.set((float) vec3d.x, (float) vec3d.y, (float) vec3d.z);
        } else {
            Vec3 vec3 = motionProperties.getAngularVelocity();
            result.set(vec3.getX(), vec3.getY(), vec3.getZ());
        }

        return result;
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

        if (motionProperties == null) {
            snapshot.getAngularVelocity(result);
        } else {
            Vec3 vec3 = joltBody.getAngularVelocity();
            result.set(vec3.getX(), vec3.getY(), vec3.getZ());
        }

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new velocity vector (physics-space units per second in
     * physics-space coordinates, not null)
     */
    public Vector3f getLinearVelocity() {
        assert isDynamic();
        return getLinearVelocity(null);
    }

    /**
     * Copy the linear velocity of the body's center of mass. The body must be
     * in dynamic mode.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a velocity vector (physics-space units per second in
     * physics-space coordinates, either storeResult or a new vector, not null)
     */
    public Vector3f getLinearVelocity(Vector3f storeResult) {
        assert isDynamic();
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        if (motionProperties == null) {
            Vec3d vec3d = new Vec3d();
            snapshot.getLinearVelocity(vec3d);
            result.set((float) vec3d.x, (float) vec3d.y, (float) vec3d.z);
        } else {
            Vec3Arg vec3 = joltBody.getLinearVelocity();
            result.set(vec3.getX(), vec3.getY(), vec3.getZ());
        }

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

        if (motionProperties == null) {
            snapshot.getLinearVelocity(result);
        } else {
            Vec3 vec3 = joltBody.getLinearVelocity();
            result.set(vec3.getX(), vec3.getY(), vec3.getZ());
        }

        return result;
    }

    /**
     * Test whether the body is in dynamic mode.
     *
     * @return true if in dynamic mode, otherwise false (static/kinematic mode)
     */
    public boolean isDynamic() {
        boolean result = (mass > massForStatic);
        return result;
    }

    /**
     * Rebuild the rigid body with a new jolt-jni object.
     */
    public void rebuildRigidBody() {
        Body oldBody = joltBody;
        Vec3d location = new Vec3d();
        Quaternion orientation = new Quaternion();
        PhysicsSpace removedFrom = (PhysicsSpace) getCollisionSpace();
        RigidBodySnapshot nextSnapshot = new RigidBodySnapshot(this);

        if (oldBody != null) {
            if (removedFrom != null) {
                getPhysicsLocationDp(location);
                getPhysicsRotation(orientation);
                removedFrom.removeCollisionObject(this);
            }
            logger2.log(Level.INFO, "Clearing {0}.", this);
        }
        this.snapshot = nextSnapshot;

        CollisionShape shape = getCollisionShape();
        this.joltBody = createRigidBody(shape, mass, location, orientation);
        this.bodyId = joltBody.getId();
        this.motionProperties = joltBody.getMotionProperties();
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
        snapshot.applyTo(this);
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
    public void reposition(Vec3d location, Quaternion orientation) {
        Body oldBody = joltBody;
        PhysicsSpace removedFrom = (PhysicsSpace) getCollisionSpace();
        RigidBodySnapshot nextSnapshot = new RigidBodySnapshot(this);

        if (removedFrom != null) {
            removedFrom.removeCollisionObject(this);
        }
        logger2.log(Level.INFO, "Clearing {0}.", this);
        this.snapshot = nextSnapshot;

        CollisionShape shape = getCollisionShape();
        this.joltBody = createRigidBody(shape, mass, location, orientation);
        this.bodyId = joltBody.getId();
        this.motionProperties = joltBody.getMotionProperties();
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
        snapshot.applyTo(this);
        // TODO kinematic flag, gravity, physics joints
    }

    /**
     * Alter the body's angular velocity.
     *
     * @param omega the desired angular velocity (in physics-space coordinates,
     * not null, finite, unaffected)
     */
    public void setAngularVelocity(Vector3f omega) {
        Validate.finite(omega, "angular velocity");
        assert isDynamic();

        if (bodyInterface == null) {
            Vec3d vec3d = new Vec3d(omega);
            snapshot.setAngularVelocity(vec3d);
        } else {
            Vec3 vec3 = new Vec3((float) omega.x, (float) omega.y,
                    (float) omega.z);
            bodyInterface.setAngularVelocity(bodyId, vec3);
        }
    }

    /**
     * Alter the body's angular velocity.
     *
     * @param omega the desired angular velocity (in physics-space coordinates,
     * not null, unaffected)
     */
    public void setAngularVelocityDp(Vec3d omega) {
        Validate.nonNull(omega, "angular velocity");
        assert isDynamic();

        Vec3Arg vec3 = new Vec3(
                (float) omega.x, (float) omega.y, (float) omega.z);
        if (bodyInterface == null) {
            joltBody.setAngularVelocity(vec3);
        } else {
            bodyInterface.setAngularVelocity(bodyId, vec3);
        }
    }

    /**
     * Alter the linear velocity of the body's center of mass.
     *
     * @param velocity the desired velocity (physics-space units per second in
     * physics-space coordinates, not null, finite, unaffected)
     */
    public void setLinearVelocity(Vector3f velocity) {
        Validate.finite(velocity, "velocity");
        assert isDynamic();

        Vec3 vec3 = new Vec3(
                (float) velocity.x, (float) velocity.y, (float) velocity.z);
        if (bodyInterface == null) {
            joltBody.setLinearVelocity(vec3);
        } else {
            bodyInterface.setLinearVelocity(bodyId, vec3);
        }
    }

    /**
     * Alter the linear velocity of the body's center of mass.
     *
     * @param velocity the desired velocity (physics-space units per second in
     * physics-space coordinates, not null, unaffected)
     */
    public void setLinearVelocityDp(Vec3d velocity) {
        Validate.nonNull(velocity, "velocity");
        assert isDynamic();

        Vec3 vec3 = new Vec3(velocity.x, velocity.y, velocity.z);
        if (bodyInterface == null) {
            joltBody.setLinearVelocity(vec3);
        } else {
            bodyInterface.setLinearVelocity(bodyId, vec3);
        }
    }

    /**
     * Directly relocate the body's center of mass.
     *
     * @param location the desired location (in physics-space coordinates, not
     * null, unaffected)
     */
    public void setPhysicsLocationDp(Vec3d location) {
        Validate.nonNull(location, "location");

        Quaternion orientation = getPhysicsRotation(null);
        reposition(location, orientation);
    }

    /**
     * Directly alter the body's orientation.
     *
     * @param orientation the desired orientation (rotation matrix relative to
     * physics-space coordinates, not null, unaffected)
     */
    public void setPhysicsRotation(Matrix3f orientation) {
        Validate.nonNull(orientation, "orientation");

        Vec3d location = getPhysicsLocationDp(null);
        Quaternion quaternion
                = new Quaternion().fromRotationMatrix(orientation);
        reposition(location, quaternion);
    }

    /**
     * Directly reorient the body.
     *
     * @param orientation the desired orientation (relative to physics-space
     * coordinates, not null, not zero, unaffected)
     */
    public void setPhysicsRotation(Quaternion orientation) {
        Validate.nonZero(orientation, "orientation");

        Vec3d location = getPhysicsLocationDp(null);
        reposition(location, orientation);
    }

    /**
     * Directly reorient the body.
     *
     * @param orientation the desired orientation (relative to physics-space
     * coordinates, not null, unaffected)
     */
    public void setPhysicsRotationDp(Quatd orientation) {
        Validate.nonNull(orientation, "orientation");

        Vec3d location = getPhysicsLocationDp(null);
        Quaternion quaternion = orientation.toQuaternion();
        reposition(location, quaternion);
    }

    /**
     * Return the total force applied to the body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the total force (either storeResult or a new vector, mass times
     * physics-space units per second squared in physics-space coordinates)
     */
    public Vector3f totalAppliedForce(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        Vec3 vec3 = joltBody.getAccumulatedForce();
        result.set(vec3.getX(), vec3.getY(), vec3.getZ());

        return result;
    }

    /**
     * Return the total torque applied to the body.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the total torque (either storeResult or a new vector, mass times
     * physics-space units squared per second squared in physics-space
     * coordinates)
     */
    public Vector3f totalAppliedTorque(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        Vec3 vec3 = joltBody.getAccumulatedTorque();
        result.set(vec3.getX(), vec3.getY(), vec3.getZ());

        return result;
    }
    // *************************************************************************
    // PhysicsBody methods

    /**
     * Reactivate the body if it has been deactivated due to lack of motion.
     *
     * @param forceFlag true to force activation (ignored)
     */
    @Override
    public void activate(boolean forceFlag) {
        bodyInterface.activateBody(bodyId);
    }

    /**
     * Return the body's friction parameter.
     *
     * @return the parameter value (&ge;0)
     */
    @Override
    public float getFriction() {
        float result;
        if (joltBody == null) {
            result = snapshot.getFriction();
        } else {
            result = joltBody.getFriction();
        }

        assert result >= 0f : result;
        return result;
    }

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
     * Copy the location of the body's center to a Vector3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, finite)
     */
    @Override
    public Vector3f getPhysicsLocation(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        RVec3 rvec3 = joltBody.getCenterOfMassPosition();
        result.x = rvec3.x();
        result.y = rvec3.y();
        result.z = rvec3.z();

        if (!Vector3f.isValidVector(result)) {
            throw new RuntimeException("result = " + result);
        }
        return result;
    }

    /**
     * Copy the location of the body's center to a Vec3d.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    @Override
    public Vec3d getPhysicsLocationDp(Vec3d storeResult) {
        Vec3d result = (storeResult == null) ? new Vec3d() : storeResult;

        RVec3 rvec3 = joltBody.getCenterOfMassPosition();
        result.x = rvec3.xx();
        result.y = rvec3.yy();
        result.z = rvec3.zz();

        assert result.isFinite();
        return result;
    }

    /**
     * Copy the orientation (rotation) of the body to a {@code Quaternion}.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation Quaternion (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    @Override
    public Quaternion getPhysicsRotation(Quaternion storeResult) {
        Quaternion result
                = (storeResult == null) ? new Quaternion() : storeResult;

        Quat wr = joltBody.getRotation();
        result.set(wr.getX(), wr.getY(), wr.getZ(), wr.getW());

        return result;
    }

    /**
     * Copy the orientation (rotation) of the body to a {@code Matrix3f}.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (in physics-space coordinates, either
     * storeResult or a new matrix, not null)
     */
    @Override
    public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult) {
        Matrix3f result = (storeResult == null) ? new Matrix3f() : storeResult;

        Quat wr = joltBody.getRotation();
        Quaternion quaternion
                = new Quaternion(wr.getX(), wr.getY(), wr.getZ(), wr.getW());
        result.set(quaternion);

        return result;
    }

    /**
     * Return the body's restitution parameter.
     *
     * @return the parameter value (&ge;0)
     */
    @Override
    public float getRestitution() {
        float result;
        if (joltBody == null) {
            result = snapshot.getRestitution();
        } else {
            result = joltBody.getRestitution();
        }

        assert result >= 0f : result;
        return result;
    }

    /**
     * Test whether a jolt-jni object is assigned to this body.
     *
     * @return true if one is assigned, otherwise false
     */
    @Override
    public boolean hasAssignedNativeObject() {
        if (joltBody == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Test whether the body has been deactivated due to lack of motion.
     *
     * @return true if the body is still active, false if it's deactivated
     */
    @Override
    public boolean isActive() {
        boolean result = joltBody.isActive();
        return result;
    }

    /**
     * Test whether the body is static (immobile).
     *
     * @return true if static, otherwise false
     */
    @Override
    public boolean isStatic() {
        boolean result = (mass == massForStatic);
        return result;
    }

    /**
     * Return the address of the assigned jolt-jni object, assuming that one is
     * assigned.
     *
     * @return the virtual address (not zero)
     */
    @Override
    public long nativeId() {
        long result = joltBody.va();
        return result;
    }

    /**
     * Alter the {@code addedToSpace} field. Internal use only.
     *
     * @param physicsSpace (alias created if not null)
     */
    @Override
    public void setAddedToSpaceInternal(PhysicsSpace physicsSpace) {
        super.setAddedToSpaceInternal(physicsSpace);

        if (physicsSpace == null) {
            this.bodyInterface = null;
        } else {
            this.bodyInterface = physicsSpace.getBodyInterface();
            snapshot.applyTo(this);
        }
    }

    /**
     * Apply the specified CollisionShape to the body. The body gets rebuilt on
     * the jolt-jni side.
     *
     * @param desiredShape the shape to apply (not null, alias created)
     */
    @Override
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
     * Alter the body's friction.
     *
     * @param friction the desired friction value (default=0.5)
     * <p>
     * Note: the jolt-jni default is 0.2 .
     */
    @Override
    public void setFriction(float friction) {
        if (joltBody == null) {
            snapshot.setFriction(friction);
        } else {
            joltBody.setFriction(friction);
        }
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

        Vec3d vec3d = new Vec3d(location);
        Quaternion orientation = getPhysicsRotation(null);
        reposition(vec3d, orientation);
    }

    /**
     * Alter the body's restitution.
     *
     * @param restitution the desired restitution value (default=0)
     */
    @Override
    public void setRestitution(float restitution) {
        Validate.nonNegative(restitution, "restitution");

        if (joltBody == null) {
            snapshot.setRestitution(restitution);
        } else {
            joltBody.setRestitution(restitution);
        }
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
    private static Body createRigidBody(CollisionShape shape, float mass,
            Vec3d location, Quaternion orientation) {
        RVec3 rvec3 = new RVec3(location.x, location.y, location.z);
        Quat quat = new Quat(orientation.getX(), orientation.getY(),
                orientation.getZ(), orientation.getW());

        BodyCreationSettings settings;
        if (mass > 0f) {
            settings = new BodyCreationSettings(shape.getJoltShape(),
                    rvec3, quat, EMotionType.Dynamic,
                    PhysicsSpace.OBJ_LAYER_MOVING);
            settings.setGravityFactor(1f);
        } else {
            assert mass == 0f : mass;
            settings = new BodyCreationSettings(shape.getJoltShape(),
                    rvec3, quat, EMotionType.Static,
                    PhysicsSpace.OBJ_LAYER_NON_MOVING);
            settings.setGravityFactor(0f);
        }

        PhysicsSpace physicsSpace
                = (PhysicsSpace) CollisionSpace.getCollisionSpace();
        BodyInterface bodyInterface = physicsSpace.getBodyInterface();
        Body result = bodyInterface.createBody(settings);

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
