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

import com.github.stephengold.joltjni.HeightFieldShapeSettings;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import com.jme3.terrain.Terrain;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A collision shape for terrain defined by a matrix of height values, based on
 * jolt-jni's {@code HeightFieldShape}. Should be more efficient than an
 * equivalent {@code MeshCollisionShape}. Not for use in dynamic bodies.
 *
 * @author Brent Owens
 */
public class HeightfieldCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HeightfieldCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * unflipped direct buffer containing the heightfield samples
     */
    private FloatBuffer heightBuffer;
    /**
     * copy of number of rows/columns in the heightfield (&ge;2)
     */
    private int edgeLength;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected HeightfieldCollisionShape() {
    }

    /**
     * Instantiate a square shape for the specified array of heights.
     *
     * @param heightArray (not null, length&ge;4, length a perfect square,
     * unaffected)
     */
    public HeightfieldCollisionShape(float[] heightArray) {
        Validate.inRange(heightArray.length, "number of heights",
                4, Integer.MAX_VALUE);
        createShape(heightArray);
    }

    /**
     * Instantiate a square shape for the specified terrain and scale vector.
     *
     * @param terrain (not null, size &ge;2, unaffected)
     * @param scale the desired scale factor for each local axis (not null, no
     * negative component, unaffected, default=(1,1,1))
     */
    public HeightfieldCollisionShape(Terrain terrain, Vector3f scale) {
        Validate.nonNegative(scale, "scale");
        float[] heightArray = terrain.getHeightMap();
        Validate.inRange(heightArray.length, "number of heights",
                4, Integer.MAX_VALUE);

        createShape(heightArray);
        setScale(scale);
    }

    /**
     * Instantiate a square shape for the specified HeightMap. If the HeightMap
     * isn't populated, invoke its load() method.
     *
     * @param heightMap (not null, size&ge;2)
     */
    public HeightfieldCollisionShape(HeightMap heightMap) {
        float[] heightArray = heightMap.getHeightMap();
        if (heightArray == null) { // not populated
            boolean success = heightMap.load();
            assert success;
            heightArray = heightMap.getHeightMap();
            assert heightArray != null;
        }
        Validate.inRange(heightArray.length, "number of heights",
                4, Integer.MAX_VALUE);

        createShape(heightArray);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many columns are in the heightfield.
     *
     * @return the count (&ge;2)
     */
    public int countColumns() {
        assert edgeLength >= 2 : edgeLength;
        return edgeLength;
    }

    /**
     * Count how many heights are in the heightfield.
     *
     * @return the count (&ge;4)
     */
    public int countMeshVertices() {
        int count = heightBuffer.capacity();

        assert count >= 4 : count;
        return count;
    }

    /**
     * Count how many rows are in the heightfield.
     *
     * @return the count (&ge;2)
     */
    public int countRows() {
        assert edgeLength >= 2 : edgeLength;
        return edgeLength;
    }

    /**
     * Return the index of the height axis.
     *
     * @return 1
     */
    public int upAxis() {
        return PhysicsSpace.AXIS_Y;
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
        return 0f;
    }
    // *************************************************************************
    // private methods

    /**
     * Instantiate a square {@code HeightFieldShape}.
     *
     * @param heightArray (not null, length&ge;4, length a perfect square,
     * unaffected)
     */
    private void createShape(float[] heightArray) {
        this.margin = 0f;

        int numHeights = heightArray.length;
        this.edgeLength = (int) Math.round(Math.sqrt(numHeights));
        Validate.require(edgeLength * edgeLength == numHeights,
                "the number of heights be a perfect square");

        this.heightBuffer = BufferUtils.createFloatBuffer(numHeights);
        for (int i = 0; i < numHeights; ++i) {
            float height = heightArray[i];
            if (!Float.isFinite(height)) {
                throw new IllegalArgumentException("illegal height: " + height);
            }
            heightBuffer.put(i, height);
        }

        float xzOffset = 0.5f * (edgeLength - 1);
        Vec3Arg offset = new Vec3(-xzOffset, 0f, -xzOffset);
        Vec3Arg inScale = new Vec3(1f, 1f, 1f);
        HeightFieldShapeSettings settings = new HeightFieldShapeSettings(
                heightBuffer, offset, inScale, edgeLength);
        ShapeRefC shape = settings.create().get();
        setNativeObject(shape);
    }
}
