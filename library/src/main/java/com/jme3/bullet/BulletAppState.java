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
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.debug.DebugConfiguration;
import com.jme3.bullet.debug.DebugInitListener;
import com.jme3.bullet.util.NativeLibrary;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * An AppState to manage a single PhysicsSpace.
 *
 * @author normenhansen
 */
public class BulletAppState
        extends BaseAppState
        implements PhysicsTickListener {
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
     * AppState to manage the debug visualization, or null if none
     */
    private BulletDebugAppState debugAppState;
    /**
     * configuration for debug visualization
     */
    final private DebugConfiguration debugConfig = new DebugConfiguration();
    /**
     * simulation speed multiplier (0&rarr;paused)
     */
    private float speed = 1f;
    /**
     * time interval between frames (in seconds) from the most recent update
     */
    private float tpf;
    /**
     * number of worker threads to use
     */
    private int numSolvers = NativeLibrary.numThreads();
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
     * Determine the length of the debug axis arrows.
     *
     * @return length (in physics-space units, &ge;0)
     */
    public float debugAxisLength() {
        float result = debugConfig.axisArrowLength();
        return result;
    }

    /**
     * Determine the line width of the debug axis arrows.
     *
     * @return width (in pixels, &ge;1) or 0 for solid arrows
     */
    public float debugAxisLineWidth() {
        float result = debugConfig.axisLineWidth();
        return result;
    }

    /**
     * Access the Camera used for debug visualization.
     *
     * @return the pre-existing instance, or null if unknown
     */
    public Camera getDebugCamera() {
        Camera result = debugConfig.getCamera();
        return result;
    }

    /**
     * Access the PhysicsSpace managed by this state. Normally there is none
     * until the state is attached.
     *
     * @return the pre-existing instance, or null if no simulation running
     */
    public PhysicsSpace getPhysicsSpace() {
        PhysicsSpace result = debugConfig.getSpace();
        return result;
    }

    /**
     * Return the physics-simulation speed.
     *
     * @return the speedup factor (&ge;0, default=1, 0&rarr;paused)
     */
    public float getSpeed() {
        assert speed >= 0f : speed;
        return speed;
    }

    /**
     * Test whether debug visualization is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isDebugEnabled() {
        boolean result = debugConfig.isEnabled();
        return result;
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
     * Alter which angular velocities are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * angular velocities (default=null)
     */
    public void setDebugAngularVelocityFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setAngularVelocityFilter(filter);
        } else {
            debugAppState.setAngularVelocityFilter(filter);
        }
    }

    /**
     * Alter the length of the debug axis arrows.
     *
     * @param length the desired length (in physics-space units, &ge;0)
     */
    public void setDebugAxisLength(float length) {
        Validate.nonNegative(length, "length");
        debugConfig.setAxisArrowLength(length);
    }

    /**
     * Alter the line width for debug axis arrows.
     *
     * @param width the desired width (in pixels, &ge;1) or 0 for solid arrows
     * (default=1)
     */
    public void setDebugAxisLineWidth(float width) {
        Validate.inRange(width, "width", 0f, Float.MAX_VALUE);
        debugConfig.setAxisLineWidth(width);
    }

    /**
     * Replace the Camera used for debug visualization.
     *
     * @param camera the Camera to use (alias created) or null for unknown
     * (defaults to the application's main camera)
     */
    public void setDebugCamera(Camera camera) {
        debugConfig.setCamera(camera);
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
     * Alter which objects are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize all
     * objects (default=null)
     */
    public void setDebugFilter(BulletDebugAppState.DebugAppStateFilter filter) {
        debugConfig.setFilter(filter);
    }

    /**
     * Replace or remove the init listener for the BulletDebugAppState.
     *
     * @param listener the listener to register, or null to de-register the
     * current listener (default=null)
     */
    public void setDebugInitListener(DebugInitListener listener) {
        debugConfig.setInitListener(listener);
    }

    /**
     * Alter the shadow mode of the debug root node.
     *
     * @param mode the desired value (not null, default=Off)
     */
    public void setDebugShadowMode(RenderQueue.ShadowMode mode) {
        Validate.nonNull(mode, "mode");

        if (debugAppState != null) {
            Node node = debugAppState.getRootNode();
            node.setShadowMode(mode);
        }
        debugConfig.setShadowMode(mode);
    }

    /**
     * Alter which velocity vectors are included in the debug visualization.
     *
     * @param filter the filter to use (alias created) or null to visualize no
     * velocity vectors (default=null)
     */
    public void setDebugVelocityVectorFilter(
            BulletDebugAppState.DebugAppStateFilter filter) {
        if (debugAppState == null) {
            debugConfig.setVelocityVectorFilter(filter);
        } else {
            debugAppState.setVelocityVectorFilter(filter);
        }
    }

    /**
     * Alter which view ports will render the debug visualization.
     *
     * @param viewPorts (not null, aliases created)
     */
    public void setDebugViewPorts(ViewPort... viewPorts) {
        Validate.nonNull(viewPorts, "view ports");
        debugConfig.setViewPorts(viewPorts);
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
     * @param speed the desired speedup factor (&ge;0, default=1, 0&rarr;paused)
     */
    public void setSpeed(float speed) {
        Validate.nonNegative(speed, "speed");
        this.speed = speed;
    }

    /**
     * Allocate a PhysicsSpace and start simulating physics.
     * <p>
     * Simulation starts automatically after the state is attached. To start it
     * sooner, invoke this method.
     */
    public void startPhysics() {
        if (isRunning) {
            return;
        }

        PhysicsSpace pSpace;
        pSpace = createPhysicsSpace();
        debugConfig.setSpace(pSpace);
        pSpace.addTickListener(this);
        setRunning(true);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Create the configured debug-visualization app state.
     *
     * @return a new instance (not null)
     */
    protected BulletDebugAppState createDebugAppState() {
        BulletDebugAppState appState = new BulletDebugAppState(debugConfig);
        return appState;
    }

    /**
     * Create the configured {@code PhysicsSpace}.
     *
     * @return a new instance (not null)
     */
    protected PhysicsSpace createPhysicsSpace() {
        PhysicsSpace result = new PhysicsSpace(numSolvers);
        return result;
    }

    /**
     * Access the AppState to manage the debug visualization.
     *
     * @return the pre-existing instance, or null if none
     */
    protected BulletDebugAppState getDebugAppState() {
        return debugAppState;
    }

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
        debugConfig.initialize(app);
        startPhysics();
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
     * Render this state. Should be invoked only by a subclass or by the
     * AppStateManager. Invoked once per frame, provided the state is attached
     * and enabled.
     *
     * @param rm the render manager (not null)
     */
    @Override
    public void render(RenderManager rm) {
        assert isRunning;
        super.render(rm);

        PhysicsSpace pSpace = debugConfig.getSpace();
        pSpace.update(isEnabled() ? tpf * speed : 0f);
    }

    /**
     * Transition this state from detached to initializing. Should be invoked
     * only by a subclass or by the AppStateManager.
     *
     * @param stateManager the manager instance (not null)
     */
    @Override
    public void stateAttached(AppStateManager stateManager) {
        super.stateAttached(stateManager);

        if (!isRunning) {
            startPhysics();
        }
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
        this.tpf = tpf;

        boolean enable = debugConfig.isEnabled();
        if (enable && debugAppState == null) {
            // Start debug visualization.
            this.debugAppState = createDebugAppState();
            getStateManager().attach(debugAppState);

        } else if (!enable && debugAppState != null) {
            // Stop debug visualization.
            getStateManager().detach(debugAppState);
            this.debugAppState = null;
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback invoked just after the physics has been stepped. A good time to
     * clear/apply forces. Meant to be overridden.
     *
     * @param space the space that was just stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }

    /**
     * Callback invoked just before the physics is stepped. A good time to
     * clear/apply forces. Meant to be overridden.
     *
     * @param space the space that's about to be stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSpace space, float timeStep) {
        // do nothing
    }
}
