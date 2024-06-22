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
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;
import java.lang.foreign.MemorySession;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.mesh.CapsuleMesh;
import jolt.Jolt;
import jolt.physics.collision.shape.CapsuleShapeSettings;
import jolt.physics.collision.shape.Shape;

/**
 * A capsule collision shape based on jolt-java's {@code CapsuleShape}.
 *
 * @author normenhansen
 */
public class CapsuleCollisionShape extends CollisionShape {
    // *************************************************************************
    // loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(CapsuleCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled height of the cylindrical portion (in shape units,
     * &ge;0)
     */
    private float height;
    /**
     * copy of the unscaled radius (in shape units, &ge;0)
     */
    private float radius;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected CapsuleCollisionShape() {
    }

    /**
     * Instantiate a Y-axis capsule shape with the specified radius and height.
     *
     * @param radius the desired unscaled radius (in shape units, &ge;0)
     * @param height the desired unscaled height of the cylindrical portion (in
     * shape units, &ge;0)
     */
    public CapsuleCollisionShape(float radius, float height) {
        Validate.nonNegative(radius, "radius");
        Validate.nonNegative(height, "height");

        this.radius = radius;
        this.height = height;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the main (height) axis of the capsule.
     *
     * @return 1
     */
    public int getAxis() {
        return PhysicsSpace.AXIS_Y;
    }

    /**
     * Return the height of the cylindrical portion.
     *
     * @return the unscaled height (in shape units, &ge;0)
     */
    public float getHeight() {
        assert height >= 0f : height;
        return height;
    }

    /**
     * Return the radius of the capsule.
     *
     * @return the unscaled radius (in shape units, &ge;0)
     */
    public float getRadius() {
        assert radius >= 0f : radius;
        return radius;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scale factors can be applied to the shape. For
     * capsule shapes, scaling must be uniform.
     *
     * @param scale the desired scale factor for each local axis (may be null,
     * unaffected)
     * @return true if applicable, otherwise false
     */
    @Override
    public boolean canScale(Vector3f scale) {
        boolean canScale
                = super.canScale(scale) && MyVector3f.isScaleUniform(scale);
        return canScale;
    }

    /**
     * Generate vertex indices for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of indices (capacity a
     * multiple of 3)
     */
    @Override
    public IntBuffer copyIndices() {
        float h = height * scale.x;
        float r = radius * scale.x;
        Mesh mesh = new CapsuleMesh(2, r, h);

        IndexBuffer ib = mesh.getIndicesAsList();
        int numInts = ib.size();
        IntBuffer result = BufferUtils.createIntBuffer(numInts);
        for (int ii = 0; ii < numInts; ++ii) {
            int index = ib.get(ii);
            result.put(ii, index);
        }

        assert (result.limit() == result.capacity());
        assert result.isDirect();
        assert (result.capacity() % 3) == 0;
        return result;
    }

    /**
     * Generate un-indexed triangles for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    @Override
    public FloatBuffer copyTriangles() {
        float h = height * scale.x;
        float r = radius * scale.x;
        Mesh mesh = new CapsuleMesh(2, r, h);

        IndexBuffer ib = mesh.getIndicesAsList();
        int numTriangles = mesh.getTriangleCount();
        int numIndices = 3 * numTriangles;
        int numFloats = 3 * numIndices;
        FloatBuffer pb = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f tmpVector = new Vector3f();

        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);
        for (int ii = 0; ii < numIndices; ++ii) {
            int index = ib.get(ii);
            MyBuffer.get(pb, 3 * index, tmpVector);
            MyBuffer.put(result, 3 * ii, tmpVector);
        }

        assert (result.limit() == result.capacity());
        assert result.isDirect();
        assert (result.capacity() % 9) == 0;
        return result;
    }

    /**
     * Generate vertex positions for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    @Override
    public FloatBuffer copyVertexPositions() {
        float h = height * scale.x;
        float r = radius * scale.x;
        Mesh mesh = new CapsuleMesh(2, r, h);

        FloatBuffer result = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        assert (result.limit() == result.capacity());
        assert result.isDirect();
        assert (result.capacity() % 3) == 0;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code CapsuleShape}.
     */
    private void createShape() {
        assert radius >= 0f : radius;
        assert height >= 0f : height;

        float halfHeight = height / 2f;
        CapsuleShapeSettings css = CapsuleShapeSettings.of(halfHeight, radius);
        MemorySession arena = PhysicsSpace.getArena();
        Shape shape = Jolt.use(css, settings -> {
            return settings.create(arena);
        }).orThrow();
        setNativeObject(shape);
    }
}
