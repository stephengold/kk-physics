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

import com.github.stephengold.joltjni.CylinderShape;
import com.github.stephengold.joltjni.ShapeRefC;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A cylindrical collision shape based on jolt-jni's {@code CylinderShape}.
 *
 * @author normenhansen
 */
public class CylinderCollisionShape extends CollisionShape {
    // *************************************************************************
    // loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CylinderCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * index of the main (height) axis (0&rarr;X, 1&rarr;Y, 2&rarr;Z)
     */
    private int axis;
    /**
     * copy of the unscaled half extents for each local axis (not null, no
     * negative component)
     */
    private Vector3f halfExtents = new Vector3f(0.5f, 0.5f, 0.5f);
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CylinderCollisionShape() {
    }

    /**
     * Instantiate a cylinder shape around the specified main (height) axis.
     *
     * @param radius the desired unscaled radius (in shape units, &ge;0)
     * @param height the desired unscaled height (in shape units, &ge;0)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z (default=2)
     */
    public CylinderCollisionShape(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");
        Validate.axisIndex(axisIndex, "axis index");

        this.axis = axisIndex;
        halfExtents.set(radius, radius, radius);
        halfExtents.set(axisIndex, height / 2f);
        createShape();
    }

    /**
     * Instantiate a Z-axis cylinder shape with the specified half extents.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     */
    public CylinderCollisionShape(Vector3f halfExtents) {
        Validate.nonNegative(halfExtents, "half extents");

        this.halfExtents.set(halfExtents);
        this.axis = PhysicsSpace.AXIS_Z;
        createShape();
    }

    /**
     * Instantiate a cylinder shape around the specified main (height) axis.
     *
     * @param halfExtents the desired unscaled half extents (not null, no
     * negative component, unaffected)
     * @param axisIndex which local axis to use for the height: 0&rarr;X,
     * 1&rarr;Y, 2&rarr;Z (default=2)
     */
    public CylinderCollisionShape(Vector3f halfExtents, int axisIndex) {
        Validate.nonNegative(halfExtents, "half extents");
        Validate.axisIndex(axisIndex, "axis index");

        this.halfExtents.set(halfExtents);
        this.axis = axisIndex;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the main (height) axis of the cylinder.
     *
     * @return the axis index: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     */
    public int getAxis() {
        assert axis == PhysicsSpace.AXIS_X
                || axis == PhysicsSpace.AXIS_Y
                || axis == PhysicsSpace.AXIS_Z : axis;
        return axis;
    }

    /**
     * Copy the half extents of the cylinder.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
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

    /**
     * Return the height of the cylinder.
     *
     * @return the unscaled height (&ge;0)
     */
    public float getHeight() {
        float result = 2f * halfExtents.get(axis);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scale factors can be applied to this shape.
     * For cylinder shapes, radial scaling must be uniform. The shape is
     * unaffected.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale);
        if (canScale) {
            if (axis == PhysicsSpace.AXIS_X && scale.y != scale.z) {
                canScale = false;
            } else if (axis == PhysicsSpace.AXIS_Y && scale.z != scale.x) {
                canScale = false;
            } else if (axis == PhysicsSpace.AXIS_Z && scale.x != scale.y) {
                canScale = false;
            }
        }

        return canScale;
    }

    /**
     * Return the collision margin of the shape, according to Jolt Physics.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    @Override
    protected float nativeMargin() {
        ShapeRefC ref = getUndecoratedShapeRef();
        CylinderShape cylinderShape = (CylinderShape) ref.getPtr();
        float result = cylinderShape.getConvexRadius();

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code CylinderShape}.
     */
    private void createShape() {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        this.margin = getDefaultMargin();

        float radius;
        switch (axis) {
            case PhysicsSpace.AXIS_X:
                rotation.fromAngles(0f, 0f, -FastMath.HALF_PI);
                radius = halfExtents.y;
                assert radius == halfExtents.z;
                break;
            case PhysicsSpace.AXIS_Y:
                radius = halfExtents.z;
                assert radius == halfExtents.x;
                break;
            case PhysicsSpace.AXIS_Z:
                rotation.fromAngles(FastMath.HALF_PI, 0f, 0f);
                radius = halfExtents.x;
                assert radius == halfExtents.y;
                break;
            default:
                throw new IllegalStateException("axis = " + axis);
        }

        float halfHeight = halfExtents.get(axis);
        CylinderShape shape = new CylinderShape(halfHeight, radius, margin);
        ShapeRefC shapeRef = shape.toRefC();
        setNativeObject(shapeRef);
    }
}
