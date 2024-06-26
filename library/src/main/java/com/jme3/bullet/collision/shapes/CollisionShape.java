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
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;
import jolt.math.FVec3;
import jolt.physics.collision.shape.ScaledShape;
import jolt.physics.collision.shape.Shape;

/**
 * The abstract base class for collision shapes based on jolt-java's
 * {@code Shape} class.
 *
 * @author normenhansen
 */
abstract public class CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(CollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * underlying scaled jolt-java object
     */
    private ScaledShape joltShape;
    /**
     * underlying unscaled jolt-java object
     */
    private Shape unscaledShape;
    /**
     * copy of the scale factors, one for each local axis
     */
    protected Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision shape with no underlying jolt-java objects.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    protected CollisionShape() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether the specified scale factors can be applied to the shape.
     * Subclasses that restrict scaling should override this method.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    public boolean canScale(Vector3f scale) {
        boolean result;
        if (scale == null) {
            result = false;
        } else {
            result = MyVector3f.isAllPositive(scale);
        }

        return result;
    }

    /**
     * Generate vertex indices for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of indices (capacity a
     * multiple of 3)
     */
    abstract public IntBuffer copyIndices();

    /**
     * Generate un-indexed triangles for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    abstract public FloatBuffer copyTriangles();

    /**
     * Generate vertex positions for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    abstract public FloatBuffer copyVertexPositions();

    /**
     * Access the underlying jolt-java Shape.
     *
     * @return the pre-existing instance (not null)
     */
    public Shape getJoltShape() {
        assert joltShape != null;
        return joltShape;
    }

    /**
     * Copy the scale factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the scale factor for each local axis (either storeResult or a new
     * vector, not null, all components positive)
     */
    public Vector3f getScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        assert checkScale();
        result.set(scale);

        assert MyVector3f.isAllPositive(result);
        return result;
    }

    /**
     * Test whether the shape can be applied to a dynamic rigid body.
     *
     * @return true if non-moving, false otherwise
     */
    public boolean isNonMoving() {
        boolean result = joltShape.mustBeStatic();
        return result;
    }

    /**
     * Return the ID of the assigned jolt-java object.
     *
     * @return the raw long value associated with the unscaled shape
     */
    public long nativeId() {
        long result = unscaledShape.address().toRawLongValue();
        return result;
    }

    /**
     * Alter the scale of this shape to a uniform factor. CAUTION: Not all
     * shapes can be scaled.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param factor the desired scale factor for all axes (&gt;0, default=1)
     */
    public void setScale(float factor) {
        assert Validate.positive(factor, "factor");

        Vector3f scaleVector
                = new Vector3f(factor, factor, factor); // TODO garbage
        setScale(scaleVector);
    }

    /**
     * Alter the scale of this shape. CAUTION: Not all shapes can be scaled
     * arbitrarily.
     * <p>
     * Note that if the shape is shared (between collision objects and/or
     * compound shapes) changes can have unintended consequences.
     *
     * @param scale the desired scale factor for each local axis (not null, all
     * components positive, unaffected, default=(1,1,1))
     */
    public void setScale(Vector3f scale) {
        if (!canScale(scale)) {
            String typeName = getClass().getCanonicalName();
            String message = String.format("%s cannot be scaled to (%s,%s,%s)",
                    typeName, scale.x, scale.y, scale.z);
            throw new IllegalArgumentException(message);
        }

        this.scale.set(scale);

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena, scale.x, scale.y, scale.z);
        this.joltShape = ScaledShape.of(unscaledShape, fvec3);

        logger.log(Level.FINE, "Scaling {0}.", this);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Initialize the underlying jolt-java objects.
     *
     * @param unscaled the unscaled shape to use
     */
    protected void setNativeObject(Shape unscaled) {
        Validate.nonNull(unscaled, "unscaled jolt shape");
        assert this.joltShape == null : this.joltShape;
        assert this.unscaledShape == null : this.unscaledShape;

        this.unscaledShape = unscaled;

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 fvec3 = FVec3.of(arena, scale.x, scale.y, scale.z);
        this.joltShape = ScaledShape.of(unscaledShape, fvec3);
    }
    // *************************************************************************
    // Java private methods

    /**
     * Compare jolt-java's scale factors to the local copies.
     *
     * @return true if the factors match exactly, otherwise false
     */
    private boolean checkScale() {
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 joltScale = FVec3.of(arena);
        joltShape.getScale(joltScale);

        boolean result = (joltScale.getX() == scale.x
                && joltScale.getY() == scale.y
                && joltScale.getZ() == scale.z);
        if (!result) {
            logger.log(Level.WARNING,
                    "mismatch detected: shape={0} copy={1} jolt={2}",
                    new Object[]{this, scale, joltScale});
        }

        return result;
    }
}
