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
package com.jme3.bullet.collision.shapes;

import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * An axis-aligned, rectangular-solid collision shape based on jolt-jni's
 * {@code BoxShape}.
 *
 * @author normenhansen
 */
public class BoxCollisionShape extends CollisionShape {
    // *************************************************************************
    // loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(BoxCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled half extents for each local axis (in shape units,
     * not null, no negative component)
     */
    private Vector3f halfExtents = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected BoxCollisionShape() {
    }

    /**
     * Instantiate a cube shape with the specified half extent.
     *
     * @param halfExtent the desired half extent on each local axis before
     * scaling (&ge;0)
     */
    public BoxCollisionShape(float halfExtent) {
        Validate.nonNegative(halfExtent, "half extent");

        halfExtents.set(halfExtent, halfExtent, halfExtent);
        createShape();
    }

    /**
     * Instantiate a box with the specified half extents.
     *
     * @param xHalfExtent the desired half extent on the local X axis before
     * scaling (&ge;0)
     * @param yHalfExtent the desired half extent on the local Y axis before
     * scaling (&ge;0)
     * @param zHalfExtent the desired half extent on the local Z axis before
     * scaling (&ge;0)
     */
    public BoxCollisionShape(
            float xHalfExtent, float yHalfExtent, float zHalfExtent) {
        Validate.nonNegative(xHalfExtent, "half extent on X");
        Validate.nonNegative(yHalfExtent, "half extent on Y");
        Validate.nonNegative(zHalfExtent, "half extent on Z");

        halfExtents.set(xHalfExtent, yHalfExtent, zHalfExtent);
        createShape();
    }

    /**
     * Instantiate a box with the specified half extents.
     *
     * @param halfExtents the desired half extents on each local axis before
     * scaling (not null, all components &ge;0, unaffected)
     */
    public BoxCollisionShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtents.set(halfExtents);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the half extents of the box.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, all components &ge;0)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        Vector3f result;
        if (storeResult == null) {
            result = halfExtents.clone();
        } else {
            result = storeResult.set(halfExtents);
        }
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Return the collision margin of the shape, according to Jolt Physics.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    @Override
    protected float nativeMargin() {
        ShapeRefC ref = getUnscaledShape();
        BoxShape boxShape = (BoxShape) ref.getPtr();
        float result = boxShape.getConvexRadius();

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code BoxShape}.
     */
    private void createShape() {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        Vec3Arg vec3 = new Vec3(halfExtents.x, halfExtents.y, halfExtents.z);
        float margin = getDefaultMargin();
        BoxShape shape = new BoxShape(vec3, margin);
        setNativeObject(shape.toRefC());
    }
}
