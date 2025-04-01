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

import com.github.stephengold.joltjni.MutableCompoundShape;
import com.github.stephengold.joltjni.MutableCompoundShapeSettings;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.ShapeRef;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;
import java.util.ArrayList;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A collision shape formed by combining child shapes, based on Jolt JNI's
 * {@code MutableCompoundShape}.
 *
 * @author normenhansen
 */
public class CompoundCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * default initial allocation for children
     */
    final private static int defaultCapacity = 6;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CompoundCollisionShape.class.getName());
    /**
     * local copy of {@link com.jme3.math.Matrix3f#IDENTITY}
     */
    final private static Matrix3f matrixIdentity = new Matrix3f();
    /**
     * local copy of {@link com.jme3.math.Vector3f#ZERO}
     */
    final private static Vector3f translateIdentity = new Vector3f(0f, 0f, 0f);
    // *************************************************************************
    // fields

    /**
     * children that make up this shape
     */
    private ArrayList<ChildCollisionShape> children;
    /**
     * reference to the undecorated, mutable Jolt-JNI shape
     */
    private ShapeRef mutableShapeRef;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an empty compound shape (with an initial capacity of 6 and no
     * children).
     */
    public CompoundCollisionShape() {
        this.children = new ArrayList<>(defaultCapacity);
        createEmpty();
    }

    /**
     * Instantiate an empty compound shape with the specified initial capacity
     * (and no children).
     *
     * @param initialCapacity the number of children to allocate (&gt;0,
     * default=6)
     */
    public CompoundCollisionShape(int initialCapacity) {
        Validate.positive(initialCapacity, "initial capacity");

        this.children = new ArrayList<>(initialCapacity);
        createEmpty();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a child shape without transforming its coordinates.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     */
    public void addChildShape(CollisionShape childShape) {
        Validate.nonNull(childShape, "child shape");
        addChildShape(childShape, translateIdentity, matrixIdentity);
    }

    /**
     * Add a child shape with the specified local translation.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param offsetX the local X coordinate of the child shape's origin
     * @param offsetY the local Y coordinate of the child shape's origin
     * @param offsetZ the local Z coordinate of the child shape's origin
     */
    public void addChildShape(CollisionShape childShape, float offsetX,
            float offsetY, float offsetZ) {
        Validate.nonNull(childShape, "child shape");

        Vector3f offset
                = new Vector3f(offsetX, offsetY, offsetZ); // TODO garbage
        addChildShape(childShape, offset, matrixIdentity);
    }

    /**
     * Add a child shape with the specified local translation.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param offset the local coordinates of the child shape's origin (not
     * null, unaffected)
     */
    public void addChildShape(CollisionShape childShape, Vector3f offset) {
        Validate.nonNull(childShape, "child shape");
        Validate.nonNull(offset, "offset");

        addChildShape(childShape, offset, matrixIdentity);
    }

    /**
     * Add a child shape with the specified local translation and orientation.
     *
     * @param childShape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param offset the local coordinates of the child shape's origin (not
     * null, unaffected)
     * @param rotation the local orientation of the child shape (not null,
     * unaffected)
     */
    public void addChildShape(
            CollisionShape childShape, Vector3f offset, Matrix3f rotation) {
        if (childShape instanceof CompoundCollisionShape) {
            throw new IllegalArgumentException(
                    "A CompoundCollisionShape cannot have"
                    + " a CompoundCollisionShape child!");
        }

        MutableCompoundShape compound
                = (MutableCompoundShape) mutableShapeRef.getPtr();
        Vec3Arg vec3 = new Vec3(offset.x, offset.y, offset.z);
        Quaternion quaternion = new Quaternion();
        quaternion.fromRotationMatrix(rotation);
        QuatArg quat = new Quat(quaternion.getX(), quaternion.getY(),
                quaternion.getZ(), quaternion.getW());
        ShapeRefC subShapeRef = childShape.getJoltShapeRef();
        int index = compound.addShape(vec3, quat, subShapeRef);
        assert index == children.size() : index;

        ChildCollisionShape child
                = new ChildCollisionShape(offset, rotation, childShape);
        children.add(child);
    }

    /**
     * Add a child shape with the specified local transform. The transform's
     * scale is ignored.
     *
     * @param shape the child shape to add (not null, not a compound shape,
     * alias created)
     * @param transform the local transform of the child shape (not null,
     * unaffected)
     */
    public void addChildShape(CollisionShape shape, Transform transform) {
        Vector3f offset = transform.getTranslation(); // alias
        Matrix3f rotationMatrix = transform.getRotation().toRotationMatrix();
        addChildShape(shape, offset, rotationMatrix);
    }

    /**
     * Count the child shapes.
     *
     * @return the count (&ge;0)
     */
    public int countChildren() {
        int numChildren = children.size();
        MutableCompoundShape compound
                = (MutableCompoundShape) mutableShapeRef.getPtr();
        assert numChildren == compound.getNumSubShapes() : numChildren;

        return numChildren;
    }

    /**
     * Find the first child with the specified shape.
     *
     * @param childShape the shape to search for (unaffected)
     * @return the index of the child if found, otherwise -1
     */
    public int findIndex(CollisionShape childShape) {
        int result = -1;
        for (int index = 0; index < children.size(); ++index) {
            ChildCollisionShape ccs = children.get(index);
            CollisionShape baseShape = ccs.getShape();
            if (baseShape == childShape) {
                result = index;
                break;
            }
        }

        return result;
    }

    /**
     * Enumerate the child shapes.
     *
     * @return a new array of pre-existing child shapes (not null)
     */
    public ChildCollisionShape[] listChildren() {
        int numChildren = children.size();
        ChildCollisionShape[] result = new ChildCollisionShape[numChildren];
        children.toArray(result);

        return result;
    }

    /**
     * Purge all references to the specified child shape from this shape.
     *
     * @param childShape the collision shape to remove (not null)
     */
    public void removeChildShape(CollisionShape childShape) {
        MutableCompoundShape compound
                = (MutableCompoundShape) mutableShapeRef.getPtr();
        for (int index = children.size() - 1; index >= 0; --index) {
            ChildCollisionShape childCollisionShape = children.get(index);
            if (childCollisionShape.getShape() == childShape) {
                children.remove(index);
                compound.removeShape(index);
            }
        }
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scale factors can be applied to the shape. For
     * compound shapes, scaling must be unity.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale = super.canScale(scale)
                && MyVector3f.isScaleIdentity(scale);
        return canScale;
    }

    /**
     * Return the collision margin of the shape, according to Jolt Physics.
     *
     * @return the margin thickness (in physics-space units, &ge;0)
     */
    @Override
    protected float nativeMargin() {
        return 0f;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate an empty {@code MutableCompoundShape}.
     */
    private void createEmpty() {
        this.margin = 0f;

        MutableCompoundShapeSettings settings
                = new MutableCompoundShapeSettings();
        ShapeRefC shapeRefC = settings.create().get();
        MutableCompoundShape shape = (MutableCompoundShape) shapeRefC.getPtr();
        this.mutableShapeRef = shape.toRef();
        setNativeObject(shapeRefC);
    }
}
