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
import jme3utilities.mesh.Icosphere;
import jolt.Jolt;
import jolt.physics.collision.shape.Shape;
import jolt.physics.collision.shape.SphereShapeSettings;

/**
 * A spherical collision shape based on jolt-java's {@code SphereShape}.
 *
 * @author normenhansen
 */
public class SphereCollisionShape extends CollisionShape {
    // *************************************************************************
    // loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(SphereCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * copy of the unscaled radius (in shape units, &ge;0)
     */
    private float radius;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SphereCollisionShape() {
    }

    /**
     * Instantiate a sphere shape with the specified radius.
     *
     * @param radius the desired unscaled radius (in shape units, &ge;0)
     */
    public SphereCollisionShape(float radius) {
        Validate.nonNegative(radius, "radius");

        this.radius = radius;
        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the radius of the sphere.
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
     * sphere shapes, scaling must be uniform.
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
        float r = radius * scale.x;
        Mesh mesh = new Icosphere(2, r);

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
        float r = radius * scale.x;
        Mesh mesh = new Icosphere(2, r);

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
        float r = radius * scale.x;
        Mesh mesh = new Icosphere(2, r);

        FloatBuffer result = mesh.getFloatBuffer(VertexBuffer.Type.Position);

        assert (result.limit() == result.capacity());
        assert result.isDirect();
        assert (result.capacity() % 3) == 0;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code SphereShape}.
     */
    private void createShape() {
        assert radius >= 0f : radius;

        SphereShapeSettings bss = SphereShapeSettings.of(radius);
        MemorySession arena = PhysicsSpace.getArena();
        Shape shape = Jolt.use(bss, settings -> {
            return settings.create(arena);
        }).orThrow();
        setNativeObject(shape);
    }
}
