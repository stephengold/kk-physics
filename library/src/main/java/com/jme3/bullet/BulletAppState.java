/*
 * Copyright (c) 2009-2018 jMonkeyEngine
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

import com.jme3.app.Application;
import com.jme3.app.state.AppStateManager;
import com.jme3.app.state.BaseAppState;
import com.jme3.bullet.debug.DebugConfiguration;
import com.jme3.bullet.util.NativeLibrary;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An AppState to manage a single PhysicsSpace.
 *
 * @author normenhansen
 */
public class BulletAppState extends BaseAppState {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(BulletAppState.class.getName());
    // *************************************************************************
    // fields

    /**
     * true if-and-only-if the physics simulation is running (started but not
     * yet stopped)
     */
    private volatile boolean isRunning = false;
    /**
     * configuration for debug visualization
     */
    final private DebugConfiguration debugConfig = new DebugConfiguration();
    /**
     * simulation speed multiplier (paused=0)
     */
    private float speed = 1f;
    /**
     * number of threads to create in the thread-safe pool
     */
    private int numSolvers = NativeLibrary.countThreads();
    /**
     * created {@code PhysicsSpace}
     */
    private PhysicsSpace physicsSpace;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled app state to manage a new space.
     * <p>
     * Use {@code getStateManager().addState(bulletAppState)} to start
     * simulating physics.
     */
    public BulletAppState() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count the solvers in the thread-safe pool.
     *
     * @return the count
     */
    public int countSolvers() {
        return numSolvers;
    }

    /**
     * Access the PhysicsSpace managed by this state. Normally there is none
     * until the state is attached.
     *
     * @return the pre-existing instance, or null if no simulation running
     */
    public PhysicsSpace getPhysicsSpace() {
        return physicsSpace;
    }

    /**
     * Determine the physics simulation speed.
     *
     * @return the speedup factor (&ge;0, default=1)
     */
    public float getSpeed() {
        assert speed >= 0f : speed;
        return speed;
    }

    /**
     * Test whether the physics simulation is running (started but not yet
     * stopped).
     *
     * @return true if running, otherwise false
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Enable or disable debug visualization. Changes take effect on the next
     * update.
     *
     * @param debugEnabled true &rarr; enable, false &rarr; disable
     * (default=false)
     */
    public void setDebugEnabled(boolean debugEnabled) {
        debugConfig.setEnabled(debugEnabled);
    }

    /**
     * Alter the number of solvers in the thread-safe pool.
     *
     * @param numSolvers the desired number of solvers in the thread-safe pool
     * (&ge;1, &le;64, default=numThreads)
     */
    public void setNumSolvers(int numSolvers) {
        Validate.inRange(numSolvers, "number of solvers", 1, 64);
        assert !isRunning();

        this.numSolvers = numSolvers;
    }

    /**
     * Alter the physics simulation speed.
     *
     * @param speed the desired speedup factor (&ge;0, default=1)
     */
    public void setSpeed(float speed) {
        Validate.nonNegative(speed, "speed");
        this.speed = speed;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the configuration for debug visualization.
     *
     * @return the pre-existing instance (not null)
     */
    protected DebugConfiguration getDebugConfiguration() {
        assert debugConfig != null;
        return debugConfig;
    }

    /**
     * Alter which PhysicsSpace is managed by this state.
     *
     * @param newSpace the space to be managed (may be null)
     */
    protected void setPhysicsSpace(PhysicsSpace newSpace) {
        debugConfig.setSpace(newSpace);
    }

    /**
     * Alter whether the physics simulation is running (started but not yet
     * stopped).
     *
     * @param desiredSetting true&rarr;running, false&rarr;not running
     */
    protected void setRunning(boolean desiredSetting) {
        this.isRunning = desiredSetting;
    }
    // *************************************************************************
    // BaseAppState methods

    /**
     * Invoked after the app state is detached or during application shutdown if
     * the state is still attached. onDisable() is called before this cleanup()
     * method if the state is enabled at the time of cleanup.
     *
     * @param app the application (not null)
     */
    @Override
    protected void cleanup(Application app) {
    }

    /**
     * Initialize this state prior to its first update. Should be invoked only
     * by a subclass or by the AppStateManager.
     *
     * @param app the application which owns this state (not null)
     */
    @Override
    protected void initialize(Application app) {
    }

    /**
     * Transition this state from enabled to disabled.
     */
    @Override
    protected void onDisable() {
    }

    /**
     * Transition this state from disabled to enabled.
     */
    @Override
    protected void onEnable() {
    }

    /**
     * Callback invoked when a request is made to attach the AppState.
     *
     * @param stateManager the manager instance (not null)
     */
    @Override
    public void stateAttached(AppStateManager stateManager) {
        super.stateAttached(stateManager);

        assert physicsSpace == null;
        this.physicsSpace = new PhysicsSpace(numSolvers);
    }

    /**
     * Update this state prior to rendering. Should be invoked only by a
     * subclass or by the AppStateManager. Invoked once per frame, provided the
     * state is attached and enabled.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        super.update(tpf);
        assert isEnabled();
        physicsSpace.update(tpf);
    }
}
