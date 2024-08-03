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

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.material.Material;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.simsilica.mathd.Vec3d;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.Validate;

/**
 * The abstract base class for collision objects.
 * <p>
 * Subclasses include {@code PhysicsBody}.
 *
 * @author normenhansen
 */
abstract public class PhysicsCollisionObject
        implements Comparable<PhysicsCollisionObject> {
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
     * number of visible sides for default debug materials (&ge;0, &le;2)
     */
    private int debugNumSides = 1;
    /**
     * custom material for the debug shape, or null to use the default material
     */
    private Material debugMaterial = null;
    /**
     * which normals to generate for new debug meshes
     */
    private MeshNormals debugMeshNormals = MeshNormals.None;
    /**
     * application-specific data of this collision object. Untouched.
     */
    private Object applicationData = null;
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
     * Calculate an axis-aligned bounding box for this object, based on its
     * collision shape. Note: the calculated bounds are seldom minimal; they are
     * typically larger than necessary due to centering constraints and
     * collision margins.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a bounding box in physics-space coordinates (either storeResult
     * or a new instance)
     */
    public BoundingBox boundingBox(BoundingBox storeResult) {
        BoundingBox result
                = (storeResult == null) ? new BoundingBox() : storeResult;

        Vector3f translation = getPhysicsLocation(null);
        Matrix3f rotation = getPhysicsRotationMatrix(null);
        collisionShape.boundingBox(translation, rotation, result);

        return result;
    }

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
     * Determine how many sides of this object's default debug materials are
     * visible.
     *
     * @return the number of sides (&ge;0, &le;2)
     */
    public int debugNumSides() {
        assert debugNumSides >= 0 : debugNumSides;
        assert debugNumSides <= 2 : debugNumSides;
        return debugNumSides;
    }

    /**
     * Access any application-specific data associated with the collision
     * object.
     *
     * @return the pre-existing instance, or null if none
     * @see #setApplicationData(java.lang.Object)
     */
    public Object getApplicationData() {
        return applicationData;
    }

    /**
     * Access the shape of the collision object.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    /**
     * Access the space where the collision object is added.
     *
     * @return the pre-existing instance, or null if none
     */
    public CollisionSpace getCollisionSpace() {
        return addedToSpace;
    }

    /**
     * Access the custom debug material, if specified.
     *
     * @return the pre-existing instance, or null for default materials
     */
    public Material getDebugMaterial() {
        return debugMaterial;
    }

    /**
     * Return the collision object's friction ratio.
     *
     * @return the ratio
     */
    abstract public float getFriction();

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @return a new location vector (in physics-space coordinates, not null)
     */
    public Vector3f getPhysicsLocation() {
        return getPhysicsLocation(null);
    }

    /**
     * Locate the collision object's center.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, finite)
     */
    abstract public Vector3f getPhysicsLocation(Vector3f storeResult);

    /**
     * Locate the collision object's center in double precision.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector (in physics-space coordinates, either
     * storeResult or a new vector, not null, finite)
     */
    abstract public Vec3d getPhysicsLocationDp(Vec3d storeResult);

    /**
     * Copy the orientation (rotation) of the collision object to a Quaternion.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation Quaternion (in physics-space coordinates, either
     * storeResult or a new instance, not null)
     */
    abstract public Quaternion getPhysicsRotation(Quaternion storeResult);

    /**
     * Copy the orientation of the collision object (the basis of its local
     * coordinate system) to a Matrix3f.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a rotation matrix (in physics-space coordinates, either
     * storeResult or a new matrix, not null)
     */
    abstract public Matrix3f getPhysicsRotationMatrix(Matrix3f storeResult);

    /**
     * Return the collision object's restitution (bounciness).
     *
     * @return restitution value
     */
    abstract public float getRestitution();

    /**
     * Access the scene object that's using the collision object, typically a
     * PhysicsControl, PhysicsLink, or Spatial. Used by physics controls.
     *
     * @return the pre-existing instance, or null if none
     * @see #getApplicationData()
     * @see #setUserObject(java.lang.Object)
     */
    public Object getUserObject() {
        return userObject;
    }

    /**
     * Test whether a jolt-jni object is assigned to this collision object.
     *
     * @return true if one is assigned, otherwise false
     */
    abstract public boolean hasAssignedNativeObject();

    /**
     * Test whether the collision object has been deactivated due to lack of
     * motion.
     *
     * @return true if object still active, false if deactivated
     */
    abstract public boolean isActive();

    /**
     * Test whether the collision object is added to a space.
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
     * Test whether the collision object is static (immobile).
     *
     * @return true if static, otherwise false
     */
    abstract public boolean isStatic();

    /**
     * Return the address of the assigned native object, assuming one is
     * assigned.
     *
     * @return the virtual address (not zero)
     */
    abstract public long nativeId();

    /**
     * Alter the {@code addedToSpace} field. Internal use only.
     *
     * @param physicsSpace (may be null, alias created)
     */
    public void setAddedToSpaceInternal(PhysicsSpace physicsSpace) {
        this.addedToSpace = physicsSpace;
    }

    /**
     * Associate application-specific data with the collision object. KK Physics
     * never touches application-specific data.
     *
     * @param data the desired data object (alias created, default=null)
     * @see #getApplicationData()
     */
    public void setApplicationData(Object data) {
        this.applicationData = data;
    }

    /**
     * Apply the specified shape to the collision object. Meant to be
     * overridden.
     *
     * @param collisionShape the shape to apply (not null, alias created)
     */
    public void setCollisionShape(CollisionShape collisionShape) {
        Validate.nonNull(collisionShape, "collision shape");
        this.collisionShape = collisionShape;
    }

    /**
     * Apply or remove a custom debug material.
     *
     * @param material the Material to use (alias created) or null to use the
     * default debug materials (default=null)
     */
    public void setDebugMaterial(Material material) {
        this.debugMaterial = material;
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
     * Alter how many sides of this object's default debug materials are
     * visible. This setting has no effect on custom debug materials.
     *
     * @param numSides the desired number of sides (&ge;0, &le;2, default=1)
     */
    public void setDebugNumSides(int numSides) {
        Validate.inRange(numSides, "number of sides", 0, 2);
        this.debugNumSides = numSides;
    }

    /**
     * Alter the collision object's friction ratio.
     *
     * @param friction the desired ratio (default=0.5)
     */
    abstract public void setFriction(float friction);

    /**
     * Alter the collision object's restitution (bounciness). For perfect
     * elasticity, set restitution=1.
     *
     * @param restitution the desired value (default=0)
     */
    abstract public void setRestitution(float restitution);

    /**
     * Associate a "user" with the collision object. Used by physics controls.
     *
     * @param user the desired scene object (alias created, default=null)
     * @see #getUserObject()
     * @see #setApplicationData(java.lang.Object)
     */
    public void setUserObject(Object user) {
        this.userObject = user;
    }
    // *************************************************************************
    // Comparable methods

    /**
     * Compare (by ID) with another collision object.
     *
     * @param other (not null, unaffected)
     * @return 0 if the objects have the same native ID; negative if this comes
     * before other; positive if this comes after other
     */
    @Override
    public int compareTo(PhysicsCollisionObject other) {
        long objectId = nativeId();
        long otherId = other.nativeId();
        int result = Long.compare(objectId, otherId);

        return result;
    }
    // *************************************************************************
    // Object methods

    /**
     * Represent the collision object as a {@code String}.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = getClass().getSimpleName();
        result = result.replace("Body", "");
        result = result.replace("Control", "C");
        result = result.replace("Physics", "");
        result = result.replace("Object", "");
        if (hasAssignedNativeObject()) {
            long objectId = nativeId();
            result += "#" + Long.toHexString(objectId);
        } else {
            result += "#unassigned";
        }

        return result;
    }
}
