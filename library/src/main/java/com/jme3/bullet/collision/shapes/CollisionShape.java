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

import com.github.stephengold.joltjni.ConvexShape;
import com.github.stephengold.joltjni.ScaledShape;
import com.github.stephengold.joltjni.ScaledShapeSettings;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;

/**
 * The abstract base class for collision shapes based on jolt-jni's
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
     * default margin (in physics-space units, &gt;0)
     */
    private static float defaultMargin = 0.04f;
    /**
     * underlying scaled jolt-jni object
     */
    private ShapeRefC joltShape;
    /**
     * underlying unscaled jolt-jni object
     */
    private ShapeRefC unscaledShape;
    /**
     * copy of the scale factors, one for each local axis
     */
    protected Vector3f scale = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a collision shape with no underlying jolt-jni objects.
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
     * Generate un-indexed triangles for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    public FloatBuffer copyTriangles() {
        int numTriangles = joltShape.countDebugTriangles();
        int numFloats = 9 * numTriangles;
        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);
        joltShape.copyDebugTriangles(result);
        MyBuffer.scale(result, 0, numFloats, scale);

        return result;
    }

    /**
     * Return the default margin for new shapes.
     *
     * @return the margin thickness (in physics-space units, &gt;0)
     */
    public static float getDefaultMargin() {
        assert defaultMargin > 0f : defaultMargin;
        return defaultMargin;
    }

    /**
     * Access the underlying jolt-jni ScaledShape.
     *
     * @return the pre-existing instance (not null)
     */
    public ShapeRefC getJoltShape() {
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
     * Test whether the shape has convex type.
     *
     * @return true if convex type, false otherwise
     */
    public boolean isConvex() {
        boolean result = (unscaledShape.getPtr() instanceof ConvexShape);
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
     * Return the ID of the assigned jolt-jni object.
     *
     * @return the raw long value associated with the unscaled shape
     */
    public long nativeId() {
        long result = unscaledShape.va();
        return result;
    }

    /**
     * Alter the default margin for new shapes.
     *
     * @param margin the desired margin thickness (in physics-space units,
     * &gt;0, default=0.04)
     */
    public static void setDefaultMargin(float margin) {
        Validate.positive(margin, "margin");
        defaultMargin = margin;
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

        Vec3Arg vec3 = new Vec3(scale.x, scale.y, scale.z);
        ScaledShapeSettings settings
                = new ScaledShapeSettings(unscaledShape, vec3);
        this.joltShape = settings.create().get();

        logger.log(Level.FINE, "Scaling {0}.", this);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the underlying jolt-jni Shape.
     *
     * @return the pre-existing instance (not null)
     */
    protected ShapeRefC getUnscaledShape() {
        assert unscaledShape != null;
        return unscaledShape;
    }

    /**
     * Initialize the underlying jolt-jni objects.
     *
     * @param unscaled the unscaled shape to use
     */
    protected void setNativeObject(ShapeRefC unscaled) {
        Validate.nonNull(unscaled, "unscaled jolt shape");
        assert this.joltShape == null : this.joltShape;
        assert this.unscaledShape == null : this.unscaledShape;

        this.unscaledShape = unscaled;

        Vec3Arg vec3 = new Vec3(scale.x, scale.y, scale.z);
        ScaledShapeSettings settings
                = new ScaledShapeSettings(unscaledShape, vec3);
        this.joltShape = settings.create().get();
    }
    // *************************************************************************
    // Java private methods

    /**
     * Compare jolt-jni's scale factors with the local copies.
     *
     * @return true if the factors match exactly, otherwise false
     */
    private boolean checkScale() {
        ScaledShape ss = (ScaledShape) joltShape.getPtr();
        Vec3Arg joltScale = ss.getScale();

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
