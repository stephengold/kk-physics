/*
 * Copyright (c) 2009-2021 jMonkeyEngine
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
package com.jme3.bullet;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.control.PhysicsControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.util.SafeArrayList;
import java.lang.foreign.MemorySession;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jolt.Jolt;
import jolt.core.JobSystem;
import jolt.core.TempAllocator;
import jolt.math.FVec3;
import jolt.physics.Activation;
import jolt.physics.PhysicsSystem;
import jolt.physics.body.BodyInterface;
import jolt.physics.collision.ObjectLayerPairFilter;
import jolt.physics.collision.ObjectLayerPairFilterFn;
import jolt.physics.collision.broadphase.BroadPhaseLayerInterface;
import jolt.physics.collision.broadphase.BroadPhaseLayerInterfaceFn;
import jolt.physics.collision.broadphase.ObjectVsBroadPhaseLayerFilter;
import jolt.physics.collision.broadphase.ObjectVsBroadPhaseLayerFilterFn;

/**
 * A CollisionSpace to simulate dynamic physics, with its own
 * {@code PhysicsSystem}.
 *
 * @author normenhansen
 */
public class PhysicsSpace extends CollisionSpace {
    // *************************************************************************
    // constants and loggers

    /**
     * index of the broadphase layer for non-moving objects
     */
    public static final byte BP_LAYER_NON_MOVING = 0;
    /**
     * index of the broadphase layer for moving objects
     */
    public static final byte BP_LAYER_MOVING = 1;
    /**
     * index of the X axis
     */
    final public static int AXIS_X = 0;
    /**
     * index of the Y axis
     */
    final public static int AXIS_Y = 1;
    /**
     * index of the Z axis
     */
    final public static int AXIS_Z = 2;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsSpace.class.getName());
    /**
     * index of the object layer for non-moving objects
     */
    public static final short OBJ_LAYER_NON_MOVING = 0;
    /**
     * index of the object layer for moving objects
     */
    public static final short OBJ_LAYER_MOVING = 1;
    // *************************************************************************
    // fields

    /**
     * time step (in seconds, &gt;0) ignored when maxSubSteps=0
     */
    private float accuracy = 1f / 60f;
    /**
     * maximum time step (in seconds, &gt;0) ignored when maxSubSteps>0
     */
    private float maxTimeStep = 0.1f;
    /**
     * simulation lag (for {@code maxSubSteps>0} in seconds, &ge;0)
     */
    private float physicsLag = 0f;
    /**
     * maximum number of simulation steps per frame (&gt;0) or 0 for a variable
     * time step
     */
    private int maxSubSteps = 4;
    /**
     * list of registered tick listeners
     */
    final private Collection<PhysicsTickListener> tickListeners
            = new SafeArrayList<>(PhysicsTickListener.class);
    /**
     * manage contact/collision listeners and events
     */
    private ContactManager manager = new DefaultContactManager(this);
    /**
     *
     */
    private final JobSystem jobSystem;
    private final PhysicsSystem physicsSystem;
    private final TempAllocator tempAllocator;
    /**
     * copy of the gravity-acceleration vector for newly-added bodies (default
     * is 9.81 in the -Y direction, approximating Earth-normal gravity in MKS
     * units for a Y-up coordinate system)
     */
    final private Vector3f gravity = new Vector3f(0, -9.81f, 0);

    static {
        Jolt.load();
        Jolt.registerDefaultAllocator();
        Jolt.createFactory();
        Jolt.registerTypes();
        System.out.println("Features: " + Jolt.featureSet());
        Jolt.assertSinglePrecision();
    }
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsSpace with the specified number of worker threads.
     * Must be invoked on the designated physics thread.
     *
     * @param numSolvers the desired number of worker threads (&ge;1, &le;64)
     */
    public PhysicsSpace(int numSolvers) {
        super(numSolvers);

        this.tempAllocator = TempAllocator.of(10 * 1024 * 1024);
        MemorySession arena = getArena();

        this.jobSystem = JobSystem.of(
                JobSystem.MAX_PHYSICS_JOBS,
                JobSystem.MAX_PHYSICS_BARRIERS,
                numSolvers);

        BroadPhaseLayerInterfaceFn bplif = new BroadPhaseLayerInterfaceFn() {
            @Override
            public int getNumBroadPhaseLayers() {
                return 2;
            }

            @Override
            public byte getBroadPhaseLayer(short layer) {
                return switch (layer) {
                    case OBJ_LAYER_NON_MOVING ->
                        BP_LAYER_NON_MOVING;
                    case OBJ_LAYER_MOVING ->
                        BP_LAYER_MOVING;
                    default ->
                        throw new IllegalArgumentException(
                                "Invalid object layer " + layer);
                };
            }
        };
        BroadPhaseLayerInterface bpli
                = BroadPhaseLayerInterface.of(arena, bplif);

        ObjectVsBroadPhaseLayerFilterFn ovbplff
                = (short layer1, byte layer2) -> switch (layer1) {
            case OBJ_LAYER_NON_MOVING ->
                layer2 == BP_LAYER_MOVING;
            case OBJ_LAYER_MOVING ->
                true;
            default ->
                false;
        };
        ObjectVsBroadPhaseLayerFilter ovbplf
                = ObjectVsBroadPhaseLayerFilter.of(arena, ovbplff);

        ObjectLayerPairFilterFn olpff
                = (short layer1, short layer2) -> switch (layer1) {
            case OBJ_LAYER_NON_MOVING ->
                layer2 == OBJ_LAYER_MOVING;
            case OBJ_LAYER_MOVING ->
                true;
            default ->
                false;
        };
        ObjectLayerPairFilter olpf = ObjectLayerPairFilter.of(arena, olpff);

        int maxBodies = 44444;
        int numBodyMutexes = 0;
        int maxBodyPairs = 1024;
        int maxContactConstraints = 1024;
        this.physicsSystem = PhysicsSystem.of(maxBodies, numBodyMutexes,
                maxBodyPairs, maxContactConstraints, bpli, ovbplf, olpf);

        FVec3 defaultGravity = FVec3.of(arena, gravity.x, gravity.y, gravity.z);
        physicsSystem.setGravity(defaultGravity);

        initThread();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Register the specified tick listener with the space.
     * <p>
     * Tick listeners are notified before and after each simulation step. A
     * simulation step is not necessarily the same as a frame; it is more
     * influenced by the accuracy of the PhysicsSpace.
     *
     * @see #setAccuracy(float)
     *
     * @param listener the listener to register (not null, alias created)
     */
    public void addTickListener(PhysicsTickListener listener) {
        Validate.nonNull(listener, "listener");
        assert !tickListeners.contains(listener);

        tickListeners.add(listener);
    }

    /**
     * Count how many active bodies are in the space.
     *
     * @return the count (&ge;0)
     */
    public int countActiveBodies() {
        int result = physicsSystem.getNumActiveBodies();
        return result;
    }

    /**
     * Count how many tick listeners are registered with the space.
     *
     * @return the count (&ge;0)
     */
    public int countTickListeners() {
        int count = tickListeners.size();
        return count;
    }

    /**
     * Return the simulation accuracy: the time step used when maxSubSteps&gt;0.
     *
     * @return the time step (in seconds, &gt;0)
     */
    public float getAccuracy() {
        return accuracy;
    }

    /**
     * Access the jolt-java BodyInterface. Internal use only.
     *
     * @return an instance (not null)
     */
    public BodyInterface getBodyInterface() {
        BodyInterface result = physicsSystem.getBodyInterface();
        assert result != null;
        return result;
    }

    /**
     * Access the current ContactManager.
     *
     * @return the pre-existing instance (not null)
     */
    public ContactManager getContactManager() {
        assert manager != null;
        return manager;
    }

    /**
     * Return the type of contact-and-constraint solver in use.
     *
     * @return an enum value (not null)
     */
    public SolverType getSolverType() {
        return SolverType.SI;
    }

    /**
     * Return the maximum number of simulation steps per frame.
     *
     * @return the number of steps (&gt;1) or 0 for a variable time step
     */
    public int maxSubSteps() {
        assert maxSubSteps >= 0 : maxSubSteps;
        return maxSubSteps;
    }

    /**
     * Return the maximum time step (imposed when maxSubSteps=0).
     *
     * @return the maximum time step (in seconds, &gt;0, default=0.1)
     */
    public float maxTimeStep() {
        assert maxTimeStep > 0f : maxTimeStep;
        return maxTimeStep;
    }

    /**
     * De-register the specified tick listener.
     *
     * @see #addTickListener(com.jme3.bullet.PhysicsTickListener)
     * @param listener the listener to de-register (not null, unaffected)
     */
    public void removeTickListener(PhysicsTickListener listener) {
        Validate.nonNull(listener, "listener");

        boolean success = tickListeners.remove(listener);
        assert success;
    }

    /**
     * Alter the accuracy (time step used when maxSubSteps&gt;0).
     * <p>
     * In general, the smaller the time step, the more accurate (and
     * compute-intensive) the simulation will be.
     *
     * @param accuracy the desired time step (in seconds, &gt;0, default=1/60)
     */
    public void setAccuracy(float accuracy) {
        Validate.positive(accuracy, "accuracy");
        this.accuracy = accuracy;
    }

    /**
     * Replace the current ContactManager with the specified one.
     *
     * @param manager the desired manager (not null)
     */
    public void setContactManager(ContactManager manager) {
        Validate.nonNull(manager, "manager");
        this.manager = manager;
    }

    /**
     * Alter the gravitational acceleration acting on newly-added bodies.
     * <p>
     * Typically, when a body is added to a space, the body's gravity gets set
     * to that of the space. Thus, it is preferable to set the space's gravity
     * before adding any bodies to the space.
     *
     * @param gravity the desired acceleration vector (in physics-space
     * coordinates, not null, unaffected, default=(0,-9.81,0))
     */
    public void setGravity(Vector3f gravity) {
        this.gravity.set(gravity);
    }

    /**
     * Alter the maximum number of simulation steps per frame.
     * <p>
     * Extra simulation steps help maintain determinism when the render fps
     * drops below 1/accuracy. For example a value of 2 can compensate for frame
     * rates as low as 30fps, assuming the physics has an accuracy of 1/60 sec.
     * <p>
     * Setting this value too high can depress the frame rate.
     *
     * @param steps the desired maximum number of steps (&ge;1) or 0 for a
     * variable time step (default=4)
     */
    public void setMaxSubSteps(int steps) {
        Validate.nonNegative(steps, "steps");
        this.maxSubSteps = steps;
    }

    /**
     * Alter the maximum time step (imposed when maxSubSteps=0).
     * <p>
     * In general, the smaller the time step, the more accurate (and
     * compute-intensive) the simulation will be.
     *
     * @param maxTimeStep the desired maximum time step (in seconds, &gt;0,
     * default=0.1)
     */
    public void setMaxTimeStep(float maxTimeStep) {
        Validate.positive(maxTimeStep, "max time step");
        this.maxTimeStep = maxTimeStep;
    }

    /**
     * Update the space. Can be used to single-step the physics simulation, if
     * maxSubSteps is set to 0 or 1. This method should be invoked on the thread
     * that created the space.
     *
     * @see #setMaxSubSteps(int)
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     */
    public void update(float timeInterval) {
        assert Validate.nonNegative(timeInterval, "time interval");

        float interval;
        if (maxSubSteps == 0) {
            interval = Math.min(timeInterval, maxTimeStep);
        } else {
            interval = timeInterval;
            assert maxSubSteps > 0 : maxSubSteps;
        }
        manager.update(interval, maxSubSteps);
    }

    /**
     * Update the space. This method should be invoked on the thread that
     * created the space.
     *
     * @param timeInterval the time interval to simulate (in seconds, &ge;0)
     * @param maxSteps the maximum number of steps of size {@code accuracy}
     * (&ge;1) or 0 for a single step of size {@code timeInterval}
     */
    public void update(float timeInterval, int maxSteps) {
        assert Validate.nonNegative(timeInterval, "time interval");
        assert Validate.nonNegative(maxSteps, "max steps");

        physicsSystem.optimizeBroadPhase();

        float timePerStep;
        int numSubSteps;
        if (maxSubSteps == 0) {
            timePerStep = timeInterval;
            numSubSteps = 1;

        } else {
            assert accuracy > 0f : accuracy;
            timePerStep = accuracy;

            assert physicsLag >= 0f : physicsLag;
            float timeSinceStep = physicsLag + timeInterval;
            numSubSteps = (int) FastMath.floor(timeSinceStep / accuracy);
            assert numSubSteps >= 0 : numSubSteps;
            this.physicsLag = timeSinceStep - numSubSteps * timePerStep;
            assert physicsLag >= 0f : physicsLag;

            if (numSubSteps > maxSubSteps) {
                numSubSteps = maxSubSteps;
            }
        }

        for (int i = 0; i < numSubSteps; ++i) {
            preTick(timePerStep);

            // Single-step the physics system:
            int collisionSteps = 1;
            int integrationSubSteps = 1;
            physicsSystem.update(timePerStep, collisionSteps,
                    integrationSubSteps, tempAllocator, jobSystem);

            postTick(timePerStep);
        }
    }
    // *************************************************************************
    // CollisionSpace methods

    /**
     * Add the specified object to the space. For compatibility with the
     * jme3-jbullet library.
     * <p>
     * The jme3-jbullet version allows the argument to be null.
     *
     * @param object the PhysicsControl, Spatial-with-PhysicsControl, collision
     * object, or PhysicsJoint to add (not null)
     */
    @Override
    public void add(Object object) {
        Validate.nonNull(object, "object");

        if (object instanceof PhysicsControl) {
            ((PhysicsControl) object).setPhysicsSpace(this);
        } else if (object instanceof Spatial) {
            Spatial spatial = (Spatial) object;
            for (int i = 0; i < spatial.getNumControls(); ++i) {
                if (spatial.getControl(i) instanceof PhysicsControl) {
                    add(spatial.getControl(i));
                }
            }
        } else {
            super.add(object);
        }
    }

    /**
     * Add the specified collision object to the space.
     *
     * @param pco the collision object to add (not null)
     */
    @Override
    public void addCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        if (pco instanceof PhysicsRigidBody) {
            addRigidBody((PhysicsRigidBody) pco);
        } else {
            super.addCollisionObject(pco);
        }
    }

    /**
     * Remove the specified object from the space. For compatibility with the
     * jme3-jbullet library.
     * <p>
     * The jme3-jbullet version allows the argument to be null.
     *
     * @param object the PhysicsControl, Spatial-with-PhysicsControl, collision
     * object, or PhysicsJoint to remove, or null
     */
    @Override
    public void remove(Object object) {
        if (object instanceof PhysicsControl) {
            ((PhysicsControl) object).setPhysicsSpace(null);
        } else if (object instanceof Spatial) {
            Spatial spatial = (Spatial) object;
            for (int i = 0; i < spatial.getNumControls(); ++i) {
                if (spatial.getControl(i) instanceof PhysicsControl) {
                    remove((spatial.getControl(i)));
                }
            }
        } else {
            super.remove(object);
        }
    }

    /**
     * Remove the specified collision object from the space.
     *
     * @param pco the collision object to remove (not null)
     */
    @Override
    public void removeCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        if (pco instanceof PhysicsRigidBody) {
            removeRigidBody((PhysicsRigidBody) pco);
        } else {
            super.removeCollisionObject(pco);
        }
    }
    // *************************************************************************
    // Java private methods

    /**
     * Add the specified PhysicsRigidBody to the space.
     * <p>
     * NOTE: When a rigid body is added, its gravity gets set to that of the
     * space.
     *
     * @param rigidBody the body to add (not null, modified)
     */
    private void addRigidBody(PhysicsRigidBody rigidBody) {
        assert rigidBody.getCollisionSpace() == null;

        BodyInterface bodyInterface = getBodyInterface();
        int bodyId = (int) rigidBody.nativeId();
        if (rigidBody.isDynamic()) {
            bodyInterface.addBody(bodyId, Activation.ACTIVATE);
        } else {
            bodyInterface.addBody(bodyId, Activation.DONT_ACTIVATE);
        }

        rigidBody.setAddedToSpaceInternal(this);
    }

    /**
     * Invoked just after the physics is stepped.
     *
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    private void postTick(float timeStep) {
        for (PhysicsTickListener listener : tickListeners) {
            listener.physicsTick(this, timeStep);
        }
    }

    /**
     * Invoked just before the physics is stepped.
     *
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    private void preTick(float timeStep) {
        for (PhysicsTickListener listener : tickListeners) {
            listener.prePhysicsTick(this, timeStep);
        }
    }

    /**
     * Remove the specified PhysicsRigidBody from the space.
     *
     * @param rigidBody the body to remove (not null, modified)
     */
    private void removeRigidBody(PhysicsRigidBody rigidBody) {
        assert rigidBody.getCollisionSpace() == this;

        BodyInterface bodyInterface = getBodyInterface();
        int bodyId = (int) rigidBody.nativeId();
        bodyInterface.removeBody(bodyId);

        rigidBody.setAddedToSpaceInternal(null);
    }
}
