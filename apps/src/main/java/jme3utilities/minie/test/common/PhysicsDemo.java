/*
 Copyright (c) 2018-2024 Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.common;

import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.material.Material;
import com.jme3.math.FastMath;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.shape.Box;
import java.util.Map;
import java.util.TreeMap;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.mesh.Prism;
import jme3utilities.minie.DumpFlags;
import jme3utilities.minie.FilterAll;
import jme3utilities.minie.PhysicsDumper;
import jme3utilities.minie.test.shape.ShapeGenerator;
import jme3utilities.ui.AcorusDemo;

/**
 * An AcorusDemo with additional data and methods to test and/or demonstrate the
 * capabilities of Minie.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract public class PhysicsDemo extends AcorusDemo {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerP
            = Logger.getLogger(PhysicsDemo.class.getName());
    /**
     * action string to dump the GUI scene graph
     */
    final public static String asDumpGui = "dump gui";
    /**
     * action string to dump the physics space
     */
    final public static String asDumpSpace = "dump space";
    /**
     * action string to dump the main scene graph
     */
    final public static String asDumpScene = "dump scene";
    /**
     * action string to dump the render manager (all viewports)
     */
    final public static String asDumpScenes = "dump scenes";
    /**
     * action string to dump the main viewport
     */
    final public static String asDumpViewport = "dump viewport";
    /**
     * action string to toggle debug visualization of collision-object bounding
     * boxes
     */
    final public static String asToggleAabbs = "toggle aabbs";
    /**
     * action string to toggle debug visualization of CCD swept spheres
     */
    final public static String asToggleCcdSpheres = "toggle ccdSpheres";
    /**
     * action string to toggle debug visualization
     */
    final public static String asToggleDebug = "toggle debug";
    /**
     * action string to toggle debug visualization of gravity vectors
     */
    final public static String asToggleGArrows = "toggle gArrows";
    /**
     * action string to toggle debug visualization of collision-object axes
     */
    final public static String asTogglePcoAxes = "toggle pcoAxes";
    /**
     * action string to toggle debug visualization of velocity vectors
     */
    final public static String asToggleVArrows = "toggle vArrows";
    /**
     * action string to toggle debug visualization of angular-velocity vectors
     */
    final public static String asToggleWArrows = "toggle wArrows";
    // *************************************************************************
    // fields

    /**
     * filter to control visualization of axis-aligned bounding boxes
     */
    private FilterAll aabbsFilter = null;
    /**
     * filter to control visualization of velocity vectors
     */
    private FilterAll vArrowsFilter = null;
    /**
     * filter to control visualization of angular velocities
     */
    private FilterAll wArrowsFilter = null;
    /**
     * library of named physics collision shapes
     */
    final private Map<String, CollisionShape> namedShapes = new TreeMap<>();
    /**
     * dump debugging information to {@code System.out}
     */
    final private PhysicsDumper dumper = new PhysicsDumper();
    /**
     * enhanced pseudo-random generator
     */
    final private ShapeGenerator generator = new ShapeGenerator();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a generic PhysicsDemo.
     */
    protected PhysicsDemo() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Activate all rigid bodies in the PhysicsSpace.
     */
    public void activateAll() {
        PhysicsSpace space = getPhysicsSpace();
        space.activateAll(true);
    }

    /**
     * Add the specified collision object to the PhysicsSpace.
     *
     * @param pco the object to add (not null, not in world)
     */
    public void addCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "pco");
        Validate.require(!pco.isInWorld(), "not in world");

        PhysicsSpace space = getPhysicsSpace();
        space.addCollisionObject(pco);

        postAdd(pco);
    }

    /**
     * Configure the specified platform body and add it to the PhysicsSpace.
     *
     * @param body the body to add (not null, static, not in world)
     */
    public void addPlatform(PhysicsBody body) {
        Validate.nonNull(body, "body");
        Validate.require(!body.isInWorld(), "not in world");
        Validate.require(body.isStatic(), "static");

        Material material = findMaterial("platform");
        assert material != null;
        body.setApplicationData(material);

        addCollisionObject(body);
    }

    /**
     * Add a platform to the PhysicsSpace.
     *
     * @param platformType the name of the desired platform type (not null)
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    public void addPlatform(String platformType, float topY) {
        Validate.nonNull(platformType, "platform type");

        float topRadius = 20f;
        float thickness = 0.2f * topRadius;
        switch (platformType) {
            case "box":
                addBoxPlatform(topY, topRadius, thickness);
                break;

            case "cylinder":
                addCylinderPlatform(topY, topRadius);
                break;

            case "hull":
                addHullPlatform(topY, topRadius, thickness);
                break;

            case "plane":
                addPlanePlatform(topY);
                break;

            case "triangle":
                addPlatform(platformType, MeshNormals.Facet, topY);
                break;

            default:
                String message
                        = "platformType = " + MyString.quote(platformType);
                throw new RuntimeException(message);
        }
    }

    /**
     * Add a static rigid body with the named shape to the PhysicsSpace, to
     * serve as a platform.
     *
     * @param shapeName (not null)
     * @param normals which normals to include in the debug mesh
     * @param centerY the desired Y coordinate of the center (in physics-space
     * coordinates)
     */
    public void addPlatform(
            String shapeName, MeshNormals normals, float centerY) {
        Validate.nonNull(shapeName, "shape name");

        CollisionShape shape = findShape(shapeName);
        assert shape != null : shapeName;
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(normals);
        body.setPhysicsLocation(new Vector3f(0f, centerY, 0f));

        addPlatform(body);
    }

    /**
     * Add a static cube to the scene, to serve as a platform.
     *
     * @param halfExtent half the desired extent of the cube (&gt;0)
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @return the new physics control (not null)
     */
    public RigidBodyControl attachCubePlatform(float halfExtent, float topY) {
        Validate.positive(halfExtent, "half extent");

        Mesh mesh = new Box(halfExtent, halfExtent, halfExtent);
        Geometry geometry = new Geometry("cube platform", mesh);
        rootNode.attachChild(geometry);

        float centerY = topY - halfExtent;
        geometry.move(0f, centerY, 0f);
        Material platformMaterial = findMaterial("platform");
        assert platformMaterial != null;
        geometry.setMaterial(platformMaterial);
        geometry.setShadowMode(RenderQueue.ShadowMode.Receive);

        BoxCollisionShape shape = new BoxCollisionShape(halfExtent);
        RigidBodyControl control
                = new RigidBodyControl(shape, PhysicsBody.massForStatic);
        geometry.addControl(control);
        control.setApplyScale(true);
        control.setFriction(0.1f);

        PhysicsSpace space = getPhysicsSpace();
        control.setPhysicsSpace(space);
        geometry.addControl(control);

        return control;
    }

    /**
     * Configure the PhysicsDumper. Invoke during startup.
     */
    public void configureDumper() {
        dumper.setEnabled(DumpFlags.ShadowModes, true);
        dumper.setEnabled(DumpFlags.Transforms, true);
    }

    /**
     * Describe the current physics debug options, assuming debug is enabled.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    public String describePhysicsDebugOptions() {
        String result = "Physics";

        if (aabbsFilter != null) {
            result += "+AABBs";
        }

        BulletAppState bulletAppState = getBulletAppState();
        if (bulletAppState.debugAxisLength() > 0f) {
            result += "+PcoAxes";
        }

        if (vArrowsFilter != null) {
            result += "+VArrows";
        }
        if (wArrowsFilter != null) {
            result += "+WArrows";
        }

        return result;
    }

    /**
     * Find the named CollisionShape in the library.
     *
     * @param name the name of the shape to find (not null)
     * @return the pre-existing instance, or null if not found
     */
    public CollisionShape findShape(String name) {
        Validate.nonNull(name, "name");

        CollisionShape result = namedShapes.get(name);
        return result;
    }

    /**
     * Initialize the library of named collision shapes. Invoke during startup.
     */
    public void generateShapes() {
        // do nothing
    }

    /**
     * Access the PhysicsDumper.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsDumper getDumper() {
        return dumper;
    }

    /**
     * Access the ShapeGenerator (enhanced pseudo-random generator).
     *
     * @return the pre-existing instance (not null)
     */
    public ShapeGenerator getGenerator() {
        return generator;
    }

    /**
     * Access the current PhysicsSpace.
     *
     * @return the pre-existing instance (not null)
     */
    public PhysicsSpace getPhysicsSpace() {
        BulletAppState bas = getBulletAppState();
        PhysicsSpace result = bas.getPhysicsSpace();

        assert result != null;
        return result;
    }

    /**
     * Callback invoked after adding a collision object to the PhysicsSpace.
     * Meant to be overridden.
     *
     * @param pco the object that was added (not null)
     */
    public void postAdd(PhysicsCollisionObject pco) {
        // do nothing
    }

    /**
     * Add a CollisionShape to the library.
     *
     * @param name the key that will be used to find the shape (not null)
     * @param shape (not null, alias created)
     */
    public void registerShape(String name, CollisionShape shape) {
        Validate.nonNull(name, "name");
        Validate.nonNull(shape, "shape");
        assert !namedShapes.containsKey(name);

        namedShapes.put(name, shape);
    }

    /**
     * Alter the damping fractions of all rigid bodies in the PhysicsSpace.
     *
     * @param damping the desired fraction (&ge;0, &le;1)
     */
    public void setDampingAll(float damping) {
        Validate.fraction(damping, "damping");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsRigidBody rigidBody : physicsSpace.getRigidBodyList()) {
            rigidBody.setDamping(damping, damping);
        }
    }

    /**
     * Alter the friction of all collision objects in the PhysicsSpace.
     *
     * @param friction the desired friction coefficient (&ge;0)
     */
    public void setFrictionAll(float friction) {
        Validate.nonNegative(friction, "friction");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            pco.setFriction(friction);
        }
    }

    /**
     * Alter the gravity vectors of the PhysicsSpace.
     *
     * @param gravity the desired magnitude (&ge;0)
     */
    public void setGravityAll(float gravity) {
        Validate.nonNegative(gravity, "gravity");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        Vector3f gravityVector = new Vector3f(0f, -gravity, 0f);
        physicsSpace.setGravity(gravityVector);
    }

    /**
     * Alter the restitution of all collision objects in the PhysicsSpace.
     *
     * @param restitution the desired restitution fraction (&ge;0, &le;1)
     */
    public void setRestitutionAll(float restitution) {
        Validate.fraction(restitution, "restitution");

        PhysicsSpace physicsSpace = getPhysicsSpace();
        for (PhysicsCollisionObject pco : physicsSpace.getPcoList()) {
            pco.setRestitution(restitution);
        }
    }

    /**
     * Toggle physics-debug visualization on/off.
     */
    public void togglePhysicsDebug() {
        BulletAppState bulletAppState = getBulletAppState();
        boolean enabled = bulletAppState.isDebugEnabled();
        bulletAppState.setDebugEnabled(!enabled);
    }

    /**
     * Remove a CollisionShape from the library, if it's been registered.
     *
     * @param key the key used to register the shape (not null)
     */
    public void unregisterShape(String key) {
        Validate.nonNull(key, "key");
        namedShapes.remove(key);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the active BulletAppState.
     *
     * @return the pre-existing instance (not null)
     */
    abstract protected BulletAppState getBulletAppState();

    /**
     * Determine the length of debug axis arrows (when they're visible).
     *
     * @return the desired length (in physics-space units, &ge;0)
     */
    abstract protected float maxArrowLength();
    // *************************************************************************
    // AcorusDemo methods

    /**
     * Process an action that wasn't handled by the active InputMode.
     *
     * @param actionString textual description of the action (not null)
     * @param ongoing true if the action is ongoing, otherwise false
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void onAction(String actionString, boolean ongoing, float tpf) {
        if (ongoing) {
            switch (actionString) {
                case asDumpGui:
                    dumper.dump(guiNode);
                    return;
                case asDumpScene:
                    dumper.dump(rootNode);
                    return;
                case asDumpScenes:
                    dumper.dump(renderManager);
                    return;
                case asDumpSpace:
                    PhysicsSpace physicsSpace = getPhysicsSpace();
                    dumper.dump(physicsSpace);
                    return;
                case asDumpViewport:
                    dumper.dump(viewPort);
                    return;

                case asToggleAabbs:
                    toggleAabbs();
                    return;
                case asToggleDebug:
                    togglePhysicsDebug();
                    return;
                case asTogglePcoAxes:
                    togglePcoAxes();
                    return;
                case asToggleVArrows:
                    toggleVelocityArrows();
                    return;
                case asToggleWArrows:
                    toggleAngularVelocityArrows();
                    return;

                default:
            }
        }
        super.onAction(actionString, ongoing, tpf);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a large, static box to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topHalfExtent half the desired extent of the top surface (&gt;0)
     * @param thickness the desired thickness (in physics-space units, &gt;0)
     */
    private void addBoxPlatform(
            float topY, float topHalfExtent, float thickness) {
        Validate.positive(topHalfExtent, "top half extent");
        Validate.positive(thickness, "thickness");

        CollisionShape shape = new BoxCollisionShape(
                topHalfExtent, thickness / 2f, topHalfExtent);
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(MeshNormals.Facet);
        body.setPhysicsLocation(new Vector3f(0f, topY - thickness / 2f, 0f));

        addPlatform(body);
    }

    /**
     * Add a large, static cylinder to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topRadius the desired radius of the top surface (&gt;0)
     */
    private void addCylinderPlatform(float topY, float topRadius) {
        Validate.positive(topRadius, "top radius");

        float height = 3f * topRadius; // tall to mitigate smoothed normals
        CylinderCollisionShape shape = new CylinderCollisionShape(
                topRadius, height, PhysicsSpace.AXIS_Y);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(MeshNormals.Smooth);
        body.setPhysicsLocation(new Vector3f(0f, topY - height / 2f, 0f));

        addPlatform(body);
    }

    /**
     * Add a large, static pentagonal prism to the PhysicsSpace, to serve as a
     * platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     * @param topRadius the desired radius of the top surface (&gt;0)
     * @param thickness the desired thickness (in physics-space units, &gt;0)
     */
    private void addHullPlatform(float topY, float topRadius, float thickness) {
        Validate.positive(topRadius, "top radius");
        Validate.positive(thickness, "thickness");

        boolean normals = false;
        Mesh mesh = new Prism(5, topRadius, thickness, normals);
        HullCollisionShape shape = new HullCollisionShape(mesh);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(MeshNormals.Facet);
        body.setPhysicsLocation(new Vector3f(0f, topY - thickness / 2f, 0f));

        addPlatform(body);
    }

    /**
     * Add a static plane to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the surface (in physics-space
     * coordinates)
     */
    private void addPlanePlatform(float topY) {
        Plane plane = new Plane(Vector3f.UNIT_Y, topY);
        PlaneCollisionShape shape = new PlaneCollisionShape(plane);

        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(MeshNormals.Facet);

        addPlatform(body);

        Material material = findMaterial("greenTile");
        body.setApplicationData(material);
        body.setDebugMaterial(material);
    }

    /**
     * Add a rounded rectangle to the PhysicsSpace, to serve as a platform.
     *
     * @param topY the desired Y coordinate of the top surface (in physics-space
     * coordinates)
     */
    private void addRoundedRectangle(float topY) {
        CollisionShape shape = findShape("roundedRectangle");
        PhysicsRigidBody body
                = new PhysicsRigidBody(shape, PhysicsBody.massForStatic);

        body.setDebugMeshNormals(MeshNormals.Facet);
        Quaternion rotation
                = new Quaternion().fromAngles(FastMath.HALF_PI, 0f, 0f);
        body.setPhysicsRotation(rotation);
        body.setPhysicsLocation(new Vector3f(0f, topY, 0f));

        addPlatform(body);
    }

    /**
     * Toggle visualization of collision-object bounding boxes.
     */
    private void toggleAabbs() {
        if (aabbsFilter == null) {
            this.aabbsFilter = new FilterAll(true);
        } else {
            this.aabbsFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugBoundingBoxFilter(aabbsFilter);
    }

    /**
     * Toggle visualization of rigid-body angular velocities.
     */
    private void toggleAngularVelocityArrows() {
        if (wArrowsFilter == null) {
            this.wArrowsFilter = new FilterAll(true);
        } else {
            this.wArrowsFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugAngularVelocityFilter(wArrowsFilter);
    }

    /**
     * Toggle visualization of collision-object axes.
     */
    private void togglePcoAxes() {
        BulletAppState bulletAppState = getBulletAppState();

        float axisLength = bulletAppState.debugAxisLength();
        if (axisLength != 0f) {
            axisLength = 0f;
        } else {
            axisLength = maxArrowLength();
        }
        bulletAppState.setDebugAxisLength(axisLength);
    }

    /**
     * Toggle visualization of rigid-body velocities.
     */
    private void toggleVelocityArrows() {
        if (vArrowsFilter == null) {
            this.vArrowsFilter = new FilterAll(true);
        } else {
            this.vArrowsFilter = null;
        }

        BulletAppState bulletAppState = getBulletAppState();
        bulletAppState.setDebugVelocityVectorFilter(vArrowsFilter);
    }
}
