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
import com.jme3.scene.shape.Box;
import com.jme3.util.BufferUtils;
import java.lang.foreign.MemorySession;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jolt.Jolt;
import jolt.math.FVec3;
import jolt.physics.collision.shape.BoxShapeSettings;
import jolt.physics.collision.shape.Shape;

/**
 * An axis-aligned, rectangular-solid collision shape based on jolt-java's
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
     * Generate vertex indices for a debug-visualization mesh.
     *
     * @return a new, unflipped, direct buffer full of indices (capacity a
     * multiple of 3)
     */
    @Override
    public IntBuffer copyIndices() {
        Vector3f he = halfExtents.mult(scale);
        Mesh mesh = new Box(he.x, he.y, he.z);

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
        Vector3f he = halfExtents.mult(scale);
        Mesh mesh = new Box(he.x, he.y, he.z);

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
        Vector3f he = halfExtents.mult(scale);
        Mesh mesh = new Box(he.x, he.y, he.z);

        FloatBuffer result = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        assert (result.limit() == result.capacity());
        assert result.isDirect();
        assert (result.capacity() % 3) == 0;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code BoxShape}.
     */
    private void createShape() {
        assert MyVector3f.isAllNonNegative(halfExtents) : halfExtents;

        MemorySession arena = PhysicsSpace.getArena();
        FVec3 vec3
                = FVec3.of(arena, halfExtents.x, halfExtents.y, halfExtents.z);
        BoxShapeSettings bss = BoxShapeSettings.of(vec3);
        Shape shape = Jolt.use(bss, settings -> {
            return settings.create(arena);
        }).orThrow();
        setNativeObject(shape);
    }
}
