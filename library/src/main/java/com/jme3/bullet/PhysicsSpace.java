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

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
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
 * A space to simulate dynamic physics.
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
    private final JobSystem jobSystem;
    private final PhysicsSystem physicsSystem;
    private final TempAllocator tempAllocator;
    /**
     * copy of gravity-acceleration vector for newly-added bodies (default is
     * 9.81 in the -Y direction, approximating Earth-normal in MKS units)
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
     * Instantiate a PhysicsSpace.
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
     * Add the specified collision object to this space.
     *
     * @param pco the collision object to add (not null, modified)
     */
    public void addCollisionObject(PhysicsRigidBody pco) {
        BodyInterface bodyInterface = getBodyInterface();
        int bodyId = (int) pco.nativeId();
        float mass = pco.getMass();
        if (mass > 0f) {
            bodyInterface.addBody(bodyId, Activation.ACTIVATE);
        } else {
            bodyInterface.addBody(bodyId, Activation.DONT_ACTIVATE);
        }
        pco.setAddedToSpaceInternal(this);
    }

    /**
     * Return the number of active bodies.
     *
     * @return the count (&ge;0)
     */
    public int countActiveBodies() {
        int result = physicsSystem.getNumActiveBodies();
        return result;
    }

    /**
     * Access the jolt-java BodyInterface.
     *
     * @return an instance (not null)
     */
    public BodyInterface getBodyInterface() {
        BodyInterface result = physicsSystem.getBodyInterface();
        return result;
    }

    /**
     * Read the maximum number of simulation steps per frame.
     *
     * @return the number of steps (&gt;1) or 0 for a variable time step
     */
    public int maxSubSteps() {
        assert maxSubSteps >= 0 : maxSubSteps;
        return maxSubSteps;
    }

    /**
     * Remove the specified collision object from this space.
     *
     * @param rbc the collision object to remove (not null, modified)
     */
    public void removeCollisionObject(PhysicsRigidBody rbc) {
        BodyInterface bodyInterface = getBodyInterface();
        int bodyId = (int) rbc.nativeId();
        bodyInterface.removeBody(bodyId);

        rbc.setAddedToSpaceInternal(null);
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
     * Update this space. Can be used to single-step the physics simulation, if
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
        update(interval, maxSubSteps);
    }

    /**
     * Update this space. This method should be invoked on the thread that
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
    // private methods

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
}
