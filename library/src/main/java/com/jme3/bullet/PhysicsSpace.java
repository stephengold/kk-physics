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

import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.ConstBodyId;
import com.github.stephengold.joltjni.EBodyType;
import com.github.stephengold.joltjni.JobSystem;
import com.github.stephengold.joltjni.JobSystemThreadPool;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.MapObj2Bp;
import com.github.stephengold.joltjni.ObjVsBpFilter;
import com.github.stephengold.joltjni.ObjVsObjFilter;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.TempAllocatorImpl;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.Vec3Arg;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.control.PhysicsControl;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.util.SafeArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;

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
     * list of registered tick listeners
     */
    final private Collection<PhysicsTickListener> tickListeners
            = new SafeArrayList<>(PhysicsTickListener.class);
    /**
     * manage contact/collision listeners and events
     */
    private ContactManager manager = new DefaultContactManager(this);
    /**
     * count collision objects added or removed since the previous broadphase
     * optimization
     */
    private int addRemoveCount = 0;
    /**
     * number of adds or removes to trigger broadphase optimization
     */
    private int bpoThreshold = 5;
    /**
     * maximum number of simulation steps per frame (&gt;0) or 0 for a variable
     * time step
     */
    private int maxSubSteps = 4;
    /**
     * schedule simulation jobs
     */
    private final JobSystem jobSystem;
    /**
     * map native IDs to rigid bodies that are queued up to be added to the
     * {@code PhysicsSystem}
     */
    final private Map<Long, PhysicsRigidBody> queuedMap
            = new ConcurrentHashMap<>(64);
    /**
     * map native IDs to rigid bodies that have been added to the
     * {@code PhysicsSystem}
     */
    final private Map<Long, PhysicsRigidBody> rigidMap
            = new ConcurrentHashMap<>(64);
    /**
     * underlying jolt-jni object
     */
    private final PhysicsSystem physicsSystem;
    /**
     * allocate temporary memory for physics simulation
     */
    private final TempAllocator tempAllocator;
    /**
     * copy of the gravity-acceleration vector for newly-added bodies (default
     * is 9.81 in the -Y direction, approximating Earth-normal gravity in MKS
     * units for a Y-up coordinate system)
     */
    final private Vector3f gravity = new Vector3f(0f, -9.81f, 0f);

    static {
        NativeLibrary.load();
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

        this.tempAllocator = new TempAllocatorImpl(150 * 1024 * 1024);

        int maxJobs = Jolt.cMaxPhysicsJobs;
        int maxBarriers = Jolt.cMaxPhysicsBarriers;
        this.jobSystem
                = new JobSystemThreadPool(maxJobs, maxBarriers, numSolvers);

        int numObjLayers = 2;
        int numBpLayers = 2;
        MapObj2Bp mapObj2Bp = new MapObj2Bp(numObjLayers, numBpLayers)
                .add(OBJ_LAYER_NON_MOVING, BP_LAYER_NON_MOVING)
                .add(OBJ_LAYER_MOVING, BP_LAYER_MOVING);

        ObjVsBpFilter objVsBpFilter
                = new ObjVsBpFilter(numObjLayers, numBpLayers)
                        .disablePair(OBJ_LAYER_NON_MOVING, BP_LAYER_NON_MOVING);

        ObjVsObjFilter objVsObjFilter = new ObjVsObjFilter(numObjLayers)
                .disablePair(OBJ_LAYER_NON_MOVING, BP_LAYER_NON_MOVING);

        this.physicsSystem = new PhysicsSystem();

        int maxBodies = 15_000;
        int numBodyMutexes = 0; // 0 means "use the default value"
        int maxBodyPairs = maxBodies;
        int maxContactConstraints = 2 * maxBodies;
        physicsSystem.init(
                maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints,
                mapObj2Bp, objVsBpFilter, objVsObjFilter);

        Vec3Arg defaultGravity = new Vec3(gravity.x, gravity.y, gravity.z);
        physicsSystem.setGravity(defaultGravity);

        initThread();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Activate all rigid bodies in the space.
     *
     * @param forceFlag true to force activation
     */
    public void activateAll(boolean forceFlag) {
        for (PhysicsRigidBody rigidBody : rigidMap.values()) {
            rigidBody.activate(forceFlag);
        }
    }

    /**
     * Add all physics controls in the specified subtree of the scene graph to
     * the space (e.g. after loading from disk). For compatibility with the
     * jme3-jbullet library.
     * <p>
     * Does not add joints unless they are managed by a PhysicsControl; the
     * jme3-jbullet version attempts to add ALL joints.
     * <p>
     * Note: recursive!
     *
     * @param spatial the root of the subtree (not null)
     */
    public void addAll(Spatial spatial) {
        add(spatial);

        if (spatial instanceof Node) {
            List<Spatial> children = ((Node) spatial).getChildren();
            for (Spatial child : children) {
                addAll(child);
            }
        }
    }

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
     * Count how many active rigid bodies are in the space.
     *
     * @return the count (&ge;0)
     */
    public int countActiveRigidBodies() {
        int result = physicsSystem.getNumActiveBodies(EBodyType.RigidBody);
        return result;
    }

    /**
     * Count the rigid bodies in the space.
     *
     * @return count (&ge;0)
     */
    public int countRigidBodies() {
        int count = queuedMap.size() + rigidMap.size();
        return count;
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
     * Enqueue the specified body for addition to the {@code PhysicsSystem}
     * before the next simulation step. Internal use only.
     *
     * @param body the body to be added (not null)
     */
    public void enqueueForAdditionToSystemInternal(PhysicsRigidBody body) {
        long nativeId = body.nativeId();
        assert !queuedMap.containsKey(nativeId);
        assert !rigidMap.containsKey(nativeId);

        queuedMap.put(nativeId, body);
        body.setAddedToSpaceInternal(this);
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
     * Access the jolt-jni {@code BodyInterface}. Internal use only.
     *
     * @return an instance (not null)
     */
    public BodyInterface getBodyInterface() {
        BodyInterface result = physicsSystem.getBodyInterface();

        assert result != null;
        return result;
    }

    /**
     * Return the number of adds or removes required to trigger broadphase
     * optimization.
     *
     * @return the threshold value (&ge;0)
     */
    public int getBpoThreshold() {
        return bpoThreshold;
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
     * Copy the gravitational acceleration for newly-added bodies.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the acceleration vector (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    public Vector3f getGravity(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        assert checkGravity();
        result.set(gravity);

        return result;
    }

    /**
     * Access the PhysicsSpace <b>running on this thread</b>. For parallel
     * physics, this may be invoked from the OpenGL thread.
     *
     * @return the pre-existing PhysicsSpace running on this thread
     */
    public static PhysicsSpace getPhysicsSpace() {
        CollisionSpace result = getCollisionSpace();
        return (PhysicsSpace) result;
    }

    /**
     * Enumerate rigid bodies (including vehicles) that have been added to this
     * space and not yet removed.
     *
     * @return a new unmodifiable collection of pre-existing instances (not
     * null)
     */
    public Collection<PhysicsRigidBody> getRigidBodyList() {
        int size = countRigidBodies();
        Collection<PhysicsRigidBody> result = new HashSet<>(size);
        result.addAll(rigidMap.values());
        result.addAll(queuedMap.values());
        result = Collections.unmodifiableCollection(result);

        return result;
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
     * Alter the number of adds or removes required to trigger broadphase
     * optimization.
     *
     * @param threshold the desired threshold (&ge;0, default=5)
     */
    public void setBpoThreshold(int threshold) {
        this.bpoThreshold = threshold;
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
        Vec3Arg vec3 = new Vec3(gravity.x, gravity.y, gravity.z);
        physicsSystem.setGravity(vec3);
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

        float timePerStep;
        int numSubSteps;
        if (maxSteps == 0) {
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

            if (numSubSteps > maxSteps) {
                numSubSteps = maxSteps;
            }
        }

        for (int i = 0; i < numSubSteps; ++i) {
            preTick(timePerStep);

            // Add queued bodies to the physics system:
            for (PhysicsRigidBody body : queuedMap.values()) {
                body.addToSystemInternal();

                long newId = body.nativeId();
                PhysicsRigidBody previousBody = rigidMap.put(newId, body);
                assert previousBody == null : previousBody;
            }
            queuedMap.clear();

            if (addRemoveCount >= bpoThreshold) {
                physicsSystem.optimizeBroadPhase();
                this.addRemoveCount = 0;
            }

            // Single-step the physics system:
            int collisionSteps = 1;
            physicsSystem.update(
                    timePerStep, collisionSteps, tempAllocator, jobSystem);

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
     * Test whether the specified collision object is added to the space.
     *
     * @param pco the object to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    @Override
    public boolean contains(PhysicsCollisionObject pco) {
        boolean result;
        long pcoId = pco.nativeId();
        if (pco instanceof PhysicsRigidBody) {
            result =
                    rigidMap.containsKey(pcoId) || queuedMap.containsKey(pcoId);
        } else {
            result = super.contains(pco);
        }

        return result;
    }

    /**
     * Remove all collision objects and physics joints.
     */
    @Override
    public void destroy() {
        super.destroy();

        for (PhysicsRigidBody rigidBody : rigidMap.values()) {
            removeRigidBody(rigidBody);
        }
        queuedMap.clear();
    }

    /**
     * Enumerate collision objects that have been added to the space and not yet
     * removed.
     *
     * @return a new modifiable collection of pre-existing instances (not null)
     */
    @Override
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = super.getPcoList();
        result.addAll(queuedMap.values());
        result.addAll(rigidMap.values());

        return result;
    }

    /**
     * Test whether the space is empty.
     *
     * @return true if empty, otherwise false
     */
    @Override
    public boolean isEmpty() {
        boolean result = super.isEmpty() && rigidMap.isEmpty();
        return result;
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
     * Add the specified {@code PhysicsRigidBody} to the space, but not to the
     * {@code PhysicsSystem}.
     * <p>
     * TODO: When a rigid body is added, its gravity gets set to that of the
     * space.
     *
     * @param rigidBody the body to add (not null, modified)
     */
    private void addRigidBody(PhysicsRigidBody rigidBody) {
        assert rigidBody.getCollisionSpace() == null;

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Adding {0} to {1}.",
                    new Object[]{rigidBody, this});
        }
        enqueueForAdditionToSystemInternal(rigidBody);
        rigidBody.setAddedToSpaceInternal(this);
    }

    /**
     * Compare jolt-jni's gravity vector to the local copy.
     *
     * @return true if scale factors are exactly equal, otherwise false
     */
    private boolean checkGravity() {
        Vec3Arg joltGravity = physicsSystem.getGravity();

        boolean result = (joltGravity.getX() == gravity.x
                && joltGravity.getY() == gravity.y
                && joltGravity.getZ() == gravity.z);
        if (!result) {
            logger.log(Level.WARNING,
                    "mismatch detected: space={0} copy={1} jolt={2}",
                    new Object[]{this, gravity, joltGravity});
        }

        return result;
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
     * Remove the specified rigid body from the space and its physics system.
     *
     * @param rigidBody the body to remove (not null, modified)
     */
    private void removeRigidBody(PhysicsRigidBody rigidBody) {
        assert rigidBody.getCollisionSpace() == this;

        if (logger.isLoggable(Level.FINE)) {
            logger.log(Level.FINE, "Removing {0} from {1}.",
                    new Object[]{rigidBody, this});
        }

        PhysicsRigidBody removedBody;
        long id = rigidBody.nativeId();
        if (queuedMap.containsKey(id)) {
            removedBody = queuedMap.remove(id);

        } else {
            removedBody = rigidMap.remove(id);
            if (removedBody == null) {
                logger.log(Level.WARNING, "{0} does not exist in {1}.",
                        new Object[]{rigidBody, this});
                return;
            }

            BodyInterface bodyInterface = getBodyInterface();
            ConstBodyId bodyId = rigidBody.findBodyId();
            bodyInterface.removeBody(bodyId);
            ++addRemoveCount;
        }

        assert removedBody == rigidBody : removedBody;
        rigidBody.setAddedToSpaceInternal(null);
    }
}
