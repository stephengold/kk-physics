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
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RotatedTranslatedShapeSettings;
import com.github.stephengold.joltjni.ScaledShape;
import com.github.stephengold.joltjni.ScaledShapeSettings;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyQuaternion;
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
     * default collision margin for most shapes (in physics-space units, &ge;0)
     */
    private static float defaultMargin = 0.04f;
    /**
     * copy of collision margin (in physics-space units, &ge;0, default=0.04)
     */
    protected float margin = defaultMargin;
    /**
     * copy of the shape rotation
     */
    protected Quaternion rotation = new Quaternion();
    /**
     * reference to the rotated-and-scaled jolt-jni shape, which might be the
     * rotated shape or the undecorated shape
     */
    private ShapeRefC joltShapeRef;
    /**
     * reference to the rotated-but-unscaled jolt-jni shape, which might be the
     * undecorated shape
     */
    private ShapeRefC rotatedShapeRef;
    /**
     * reference to the undecorated jolt-jni shape
     */
    private ShapeRefC undecoratedShapeRef;
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
        int numTriangles = undecoratedShapeRef.countDebugTriangles();
        int numFloats = 9 * numTriangles;
        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);
        undecoratedShapeRef.copyDebugTriangles(result);

        MyBuffer.rotate(result, 0, numFloats, rotation);
        MyBuffer.scale(result, 0, numFloats, scale);

        return result;
    }

    /**
     * Return the default collision margin for most new shapes.
     *
     * @return the margin thickness (in physics-space units, &gt;0)
     */
    public static float getDefaultMargin() {
        assert defaultMargin > 0f : defaultMargin;
        return defaultMargin;
    }

    /**
     * Access the underlying jolt-jni {@code Shape}.
     *
     * @return the pre-existing reference (not null)
     */
    public ShapeRefC getJoltShapeRef() {
        assert joltShapeRef != null;
        return joltShapeRef;
    }

    /**
     * Return the (copied) collision margin of the shape.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    public float getMargin() {
        assert margin >= 0f : margin;
        assert margin == nativeMargin() : margin + " != " + nativeMargin();
        return margin;
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
        boolean result = (undecoratedShapeRef.getPtr() instanceof ConvexShape);
        return result;
    }

    /**
     * Test whether the shape can be applied to a dynamic rigid body.
     *
     * @return true if non-moving, false otherwise
     */
    public boolean isNonMoving() {
        boolean result = joltShapeRef.mustBeStatic();
        return result;
    }

    /**
     * Return the ID of the assigned jolt-jni object.
     *
     * @return the raw long value associated with the unscaled shape
     */
    public long nativeId() {
        long result = undecoratedShapeRef.va();
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
        if (this.scale.equals(scale)) {
            return;
        } else if (!canScale(scale)) {
            String typeName = getClass().getCanonicalName();
            String message = String.format("%s cannot be scaled to (%s,%s,%s)",
                    typeName, scale.x, scale.y, scale.z);
            throw new IllegalArgumentException(message);
        }

        this.scale.set(scale);

        if (MyVector3f.isScaleIdentity(scale)) {
            this.joltShapeRef = rotatedShapeRef;
        } else {
            Vec3Arg vec3 = new Vec3(scale.x, scale.y, scale.z);
            ScaledShapeSettings settings
                    = new ScaledShapeSettings(rotatedShapeRef, vec3);
            this.joltShapeRef = settings.create().get();
        }

        logger.log(Level.FINE, "Scaling {0}.", this);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Access the undecorated jolt-jni {@code Shape}.
     *
     * @return the pre-existing reference (not null)
     */
    protected ShapeRefC getUndecoratedShapeRef() {
        assert undecoratedShapeRef != null;
        return undecoratedShapeRef;
    }

    /**
     * Return the collision margin of the shape, according to Jolt Physics.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    abstract protected float nativeMargin();

    /**
     * Initialize the underlying jolt-jni objects.
     *
     * @param undecorated a reference to the undecorated shape to use (not null)
     */
    protected void setNativeObject(ShapeRefC undecorated) {
        Validate.nonNull(undecorated, "undecorated jolt-jni shape");
        assert joltShapeRef == null : joltShapeRef;
        assert rotatedShapeRef == null : rotatedShapeRef;
        assert undecoratedShapeRef == null : undecoratedShapeRef;

        this.undecoratedShapeRef = undecorated;

        if (MyQuaternion.isRotationIdentity(rotation)) {
            this.rotatedShapeRef = undecoratedShapeRef;
        } else {
            QuatArg quat = new Quat(rotation.getX(), rotation.getY(),
                    rotation.getZ(), rotation.getW());
            Vec3Arg zeroOffset = new Vec3();
            RotatedTranslatedShapeSettings settings
                    = new RotatedTranslatedShapeSettings(
                            zeroOffset, quat, undecoratedShapeRef);
            this.rotatedShapeRef = settings.create().get();
        }

        if (MyVector3f.isScaleIdentity(scale)) {
            this.joltShapeRef = rotatedShapeRef;
        } else {
            Vec3Arg vec3 = new Vec3(scale.x, scale.y, scale.z);
            ScaledShapeSettings settings
                    = new ScaledShapeSettings(rotatedShapeRef, vec3);
            this.joltShapeRef = settings.create().get();
        }
    }
    // *************************************************************************
    // Java private methods

    /**
     * Compare jolt-jni's scale factors with the local copies.
     *
     * @return true if the factors match exactly, otherwise false
     */
    private boolean checkScale() {
        ConstShape shape = joltShapeRef.getPtr();
        Vec3Arg joltScale;
        if (shape instanceof ScaledShape) {
            ScaledShape ss = (ScaledShape) joltShapeRef.getPtr();
            joltScale = ss.getScale();
        } else {
            joltScale = new Vec3(1f, 1f, 1f);
        }

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
