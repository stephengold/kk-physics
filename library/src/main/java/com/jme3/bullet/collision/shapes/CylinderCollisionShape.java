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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import java.lang.foreign.MemorySession;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jolt.Jolt;
import jolt.physics.collision.shape.CylinderShapeSettings;
import jolt.physics.collision.shape.Shape;

/**
 * A cylindrical collision shape based on jolt-java's {@code CylinderShape}.
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
     * copy of the half extents for each local axis (not null, no negative
     * component)
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
     * @param radius the desired radius (&ge;0)
     * @param height the desired height (&ge;0)
     * @param axisIndex 1
     */
    public CylinderCollisionShape(float radius, float height, int axisIndex) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");
        Validate.inRange(axisIndex, "axis index", 1, 1);

        float halfHeight = height / 2f;
        halfExtents.set(radius, halfHeight, radius);
        createShape();
    }

    /**
     * Instantiate a cylinder shape around the specified axis.
     *
     * @param halfExtents the desired half extents (not null, no negative
     * component, unaffected)
     * @param axisIndex 1
     */
    public CylinderCollisionShape(Vector3f halfExtents, int axisIndex) {
        Validate.nonNegative(halfExtents, "half extents");
        Validate.require(
                halfExtents.x == halfExtents.z, "equal X and Z extents");
        Validate.inRange(axisIndex, "axis index", 1, 1);

        this.halfExtents.set(halfExtents);
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the main (height) axis of the cylinder.
     *
     * @return 1
     */
    public int getAxis() {
        return PhysicsSpace.AXIS_Y;
    }

    /**
     * Copy the half extents of the cylinder.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the half extent for each local axis (either storeResult or a new
     * vector, not null, no negative component)
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
     * @return the height (&ge;0)
     */
    public float getHeight() {
        float result = 2f * halfExtents.get(PhysicsSpace.AXIS_Y);

        assert result >= 0f : result;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code CylinderShape}.
     */
    private void createShape() {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;
        assert halfExtents.x == halfExtents.z : halfExtents;

        float halfHeight = halfExtents.y;
        float radius = halfExtents.x;
        CylinderShapeSettings css
                = CylinderShapeSettings.of(halfHeight, radius);
        MemorySession arena = PhysicsSpace.getArena();
        Shape shape = Jolt.use(css, settings -> {
            return settings.create(arena);
        }).orThrow();
        setNativeObject(shape);
    }
}
