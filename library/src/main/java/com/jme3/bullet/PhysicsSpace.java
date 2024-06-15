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

import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.math.Vector3f;
import java.lang.foreign.MemorySession;
import jme3utilities.math.MyMath;
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
public class PhysicsSpace {
    // *************************************************************************
    // constants

    public static final byte BP_LAYER_NON_MOVING = 0;
    public static final byte BP_LAYER_MOVING = 1;

    public static final short OBJ_LAYER_NON_MOVING = 0;
    public static final short OBJ_LAYER_MOVING = 1;
    // *************************************************************************
    // fields

    private final JobSystem jobSystem;
    private final PhysicsSystem physicsSystem;
    private final TempAllocator tempAllocator;
    private static final ThreadLocal<MemorySession> tlArenas
            = new ThreadLocal<>();
    /**
     * copy of gravity-acceleration vector for newly-added bodies (default is
     * 9.81 in the -Y direction, corresponding to Earth-normal in MKS units)
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

    public PhysicsSpace() {
        this.tempAllocator = TempAllocator.of(10 * 1024 * 1024);
        MemorySession arena = getArena();

        int numCpus = Runtime.getRuntime().availableProcessors();
        int numThreads = MyMath.clamp(numCpus - 1, 1, 16);
        this.jobSystem = JobSystem.of(
                JobSystem.MAX_PHYSICS_JOBS,
                JobSystem.MAX_PHYSICS_BARRIERS,
                numThreads);

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

        FVec3 defaultGravity = FVec3.of(arena, 0f, -10f, 0f);
        physicsSystem.setGravity(defaultGravity);
    }
    // *************************************************************************
    // new methods exposed

    public void addBody(RigidBodyControl rbc) {
        BodyInterface bodyInterface = getBodyInterface();
        int id = rbc.getId();
        bodyInterface.addBody(id, Activation.ACTIVATE);
    }

    public int countActiveBodies() {
        int result = physicsSystem.getNumActiveBodies();
        return result;
    }

    final public static synchronized MemorySession getArena() {
        MemorySession result = tlArenas.get();
        if (result == null) {
            //System.out.println("Lazily creating a confined memory session.");
            result = MemorySession.openConfined();
            assert result != null;
            tlArenas.set(result);
        }

        return result;
    }

    public BodyInterface getBodyInterface() {
        BodyInterface result = physicsSystem.getBodyInterface();
        return result;
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

    public void setMaxSubSteps(int maxSubSteps) {
    }

    public void update(float tpf) {
        physicsSystem.optimizeBroadPhase();

        float deltaTime = 1 / 60f;
        int collisionSteps = 1;
        int integrationSubSteps = 1;
        physicsSystem.update(deltaTime, collisionSteps, integrationSubSteps,
                tempAllocator, jobSystem);
    }
}
