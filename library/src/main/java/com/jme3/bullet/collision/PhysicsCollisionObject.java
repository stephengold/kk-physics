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
package com.jme3.bullet.collision;

import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.Validate;

/**
 * The abstract base class for collision objects based on Bullet's
 * {@code btCollisionObject}.
 * <p>
 * Subclasses include MultiBodyCollider, PhysicsBody, PhysicsCharacter, and
 * PhysicsGhostObject.
 *
 * @author normenhansen
 */
abstract public class PhysicsCollisionObject {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsCollisionObject.class.getName());
    // *************************************************************************
    // fields

    /**
     * shape of this object, or null if none
     */
    private CollisionShape collisionShape;
    /**
     * which normals to generate for new debug meshes
     */
    private MeshNormals debugMeshNormals = MeshNormals.None;
    /**
     * scene object that's using this collision object. The scene object is
     * typically a PhysicsControl, PhysicsLink, or Spatial. Used by physics
     * controls.
     */
    private Object userObject = null;
    /**
     * space where this collision object has been added, or null if removed or
     * never added
     */
    private PhysicsSpace addedToSpace = null;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision object with no shape or user object.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    protected PhysicsCollisionObject() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Reactivate the collision object if it has been deactivated due to lack of
     * motion.
     *
     * @param forceFlag true to force activation
     */
    abstract public void activate(boolean forceFlag);

    /**
     * Determine which normals to include in new debug meshes.
     *
     * @return an enum value (not null)
     */
    public MeshNormals debugMeshNormals() {
        assert debugMeshNormals != null;
        return debugMeshNormals;
    }

    /**
     * Access the shape of this object.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    /**
     * Access the space where this object is added.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionSpace getCollisionSpace() {
        return addedToSpace;
    }

    /**
     * Access the scene object that's using this collision object, typically a
     * PhysicsControl, PhysicsLink, or Spatial. Used by physics controls.
     *
     * @return the pre-existing instance, or null if none
     * @see #setUserObject(java.lang.Object)
     */
    public Object getUserObject() {
        return userObject;
    }

    /**
     * Test whether the collision object has been deactivated due to lack of
     * motion.
     *
     * @return true if object still active, false if deactivated
     */
    abstract public boolean isActive();

    /**
     * Test whether this object is added to a space.
     *
     * @return true&rarr;added to a space, false&rarr;not added to a space
     */
    final public boolean isInWorld() {
        if (addedToSpace == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Alter the {@code addedToSpace} field. Internal use only.
     *
     * @param physicsSpace (may be null, alias created)
     */
    public void setAddedToSpaceInternal(PhysicsSpace physicsSpace) {
        this.addedToSpace = physicsSpace;
    }

    /**
     * Apply the specified shape to this object. Meant to be overridden.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        this.collisionShape = collisionShape;
    }

    /**
     * Alter which normals to include in new debug meshes.
     *
     * @param newSetting an enum value (not null, default=None)
     */
    public void setDebugMeshNormals(MeshNormals newSetting) {
        Validate.nonNull(newSetting, "new setting");
        this.debugMeshNormals = newSetting;
    }

    /**
     * Associate a "user" with this collision object. Used by physics controls.
     *
     * @param user the desired scene object (alias created, default=null)
     * @see #getUserObject()
     */
    public void setUserObject(Object user) {
        this.userObject = user;
    }
}
