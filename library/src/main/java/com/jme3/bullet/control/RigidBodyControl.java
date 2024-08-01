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
package com.jme3.bullet.control;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Sphere;
import com.simsilica.mathd.Vec3d;
import java.io.IOException;
import java.util.logging.Logger;
import jme3utilities.MySpatial;

/**
 * A PhysicsControl to link a PhysicsRigidBody to a Spatial.
 *
 * @author normenhansen
 */
public class RigidBodyControl
        extends PhysicsRigidBody
        implements PhysicsControl {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger3
            = Logger.getLogger(RigidBodyControl.class.getName());
    // *************************************************************************
    // fields

    /**
     * true&rarr;body is added to a PhysicsSpace, false&rarr;not added
     */
    private boolean added = false;
    /**
     * true&rarr;Control is enabled, false&rarr;Control is disabled
     */
    private boolean enabled = true;
    /**
     * space to which the body is (or would be) added
     */
    private PhysicsSpace space = null;
    /**
     * Spatial to which this Control is added, or null if none
     */
    private Spatial spatial;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled control. The new instance is incomplete because it
     * lacks a collision shape, so it CANNOT be immediately added to a space.
     *
     * Its shape will be auto-generated when it is added to a Spatial. If the
     * controlled spatial is a Geometry with a box or sphere mesh, a matching
     * box or sphere {@code CollisionShape} will be generated.
     *
     * @param mass the desired mass (&ge;0)
     */
    public RigidBodyControl(float mass) {
        this.mass = mass;
    }

    /**
     * Instantiate an enabled Control with an active/responsive dynamic rigid
     * body, mass=1, and the specified shape.
     *
     * @param shape the desired shape (not null, alias created)
     */
    public RigidBodyControl(CollisionShape shape) {
        super(shape);
    }

    /**
     * Instantiate an enabled {@code Control} with a dynamic or static rigid
     * body and the specified shape and mass.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass the desired mass (&ge;0)
     */
    public RigidBodyControl(CollisionShape shape, float mass) {
        super(shape, mass);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the controlled spatial.
     *
     * @return the pre-existing {@code Spatial}, or null if none
     */
    public Spatial getSpatial() {
        return spatial;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Set the body's CollisionShape based on the controlled spatial.
     */
    protected void createCollisionShape() {
        if (spatial == null) {
            return;
        }

        CollisionShape shape = null;
        if (spatial instanceof Geometry) {
            Mesh mesh = ((Geometry) spatial).getMesh();
            if (mesh instanceof Sphere) {
                float radius = ((Sphere) mesh).getRadius();
                shape = new SphereCollisionShape(radius);
            } else if (mesh instanceof Box) {
                Box box = (Box) mesh;
                shape = new BoxCollisionShape(
                        box.getXExtent(), box.getYExtent(), box.getZExtent());
            }
        }
        if (shape == null) {
            assert false; // TODO use CollisionShapeFactory
        }
        setCollisionShape(shape);
    }
    // *************************************************************************
    // PhysicsControl methods

    /**
     * Clone this Control for a different Spatial. No longer used as of JME 3.1.
     *
     * @param spatial (unused)
     * @return never
     * @throws UnsupportedOperationException always
     */
    @Override
    public Control cloneForSpatial(Spatial spatial) {
        throw new UnsupportedOperationException(
                "cloneForSpatial() isn't implemented.");
    }

    /**
     * Access the PhysicsSpace to which the body is (or would be) added.
     *
     * @return the pre-existing space, or null for none
     */
    @Override
    public PhysicsSpace getPhysicsSpace() {
        return space;
    }

    /**
     * Test whether this Control is enabled.
     *
     * @return true if enabled, otherwise false
     */
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * De-serialize this Control from the specified importer, for example when
     * loading from a J3O file.
     *
     * @param importer (not null)
     * @throws IOException from the importer
     */
    @Override
    public void read(JmeImporter importer) throws IOException {
        throw new UnsupportedOperationException("read() isn't implemented.");
    }

    /**
     * Render this Control. Invoked once per ViewPort per frame, provided the
     * Control is added to a scene. Should be invoked only by a subclass or by
     * the RenderManager.
     *
     * @param rm the RenderManager (unused)
     * @param vp the ViewPort to render (unused)
     */
    @Override
    public void render(RenderManager rm, ViewPort vp) {
        // do nothing
    }

    /**
     * Enable or disable this Control.
     * <p>
     * When the Control is disabled, the body is removed from PhysicsSpace. When
     * the Control is enabled again, the body is moved to the current position
     * of the Spatial and then added to the PhysicsSpace.
     *
     * @param enabled true&rarr;enable the Control, false&rarr;disable it
     */
    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (space != null) {
            if (enabled && !added) {
                if (spatial != null) {
                    Vector3f location = spatial.getWorldTranslation(); // alias
                    Vec3d vec3d = new Vec3d(location);
                    Quaternion orientation
                            = spatial.getWorldRotation(); // alias
                    reposition(vec3d, orientation);
                }
                space.addCollisionObject(this);
                this.added = true;
            } else if (!enabled && added) {
                space.removeCollisionObject(this);
                this.added = false;
                // TODO also remove all joints
            }
        }
    }

    /**
     * If enabled, add this control's body to the specified PhysicsSpace. If not
     * enabled, alter where the body would be added. The body is removed from
     * any other space it's currently in.
     *
     * @param newSpace where to add, or null to simply remove
     */
    @Override
    public void setPhysicsSpace(PhysicsSpace newSpace) {
        if (space == newSpace) {
            return;
        }
        if (added) {
            space.removeCollisionObject(this);
            this.added = false;
        }
        if (newSpace != null && isEnabled()) {
            newSpace.addCollisionObject(this);
            this.added = true;
        }
        /*
         * If this Control isn't enabled, its body will be
         * added to the new space when the Control becomes enabled.
         */
        this.space = newSpace;
    }

    /**
     * Alter which Spatial is controlled. Invoked when the Control is added to
     * or removed from a Spatial. Should be invoked only by a subclass or from
     * Spatial. Do not invoke directly from user code.
     *
     * @param controlledSpatial the Spatial to control (or null)
     */
    @Override
    public void setSpatial(Spatial controlledSpatial) {
        if (spatial == controlledSpatial) {
            return;
        }

        this.spatial = controlledSpatial;
        setUserObject(controlledSpatial); // link from collision object

        if (controlledSpatial != null) {
            if (getCollisionShape() == null) {
                createCollisionShape();
                rebuildRigidBody();
            }
            Vector3f location = spatial.getWorldTranslation(); // alias
            Vec3d vec3d = new Vec3d(location);
            Quaternion orientation = spatial.getWorldRotation(); // alias
            reposition(vec3d, orientation);
        }
    }

    /**
     * Update this Control. Invoked once per frame, during the logical-state
     * update, provided the Control is added to a scene. Do not invoke directly
     * from user code.
     *
     * @param tpf the time interval between frames (in seconds, &ge;0)
     */
    @Override
    public void update(float tpf) {
        if (!enabled) {
            return;
        }

        if (isKinematic()) {
            Vector3f location = spatial.getWorldTranslation();
            Quaternion orientation = spatial.getWorldRotation();
            repositionKinematic(location, orientation, tpf);

        } else if (!MySpatial.isIgnoringTransforms(spatial)) {
            Vector3f location = getPhysicsLocation(null);
            MySpatial.setWorldLocation(spatial, location);

            Quaternion orientation = getPhysicsRotation(null);
            MySpatial.setWorldOrientation(spatial, orientation);
        }
    }

    /**
     * Serialize this Control to the specified exporter, for example when saving
     * to a J3O file.
     *
     * @param exporter (not null)
     * @throws IOException from the exporter
     */
    @Override
    public void write(JmeExporter exporter) throws IOException {
        throw new UnsupportedOperationException("write() isn't implemented.");
    }
}
