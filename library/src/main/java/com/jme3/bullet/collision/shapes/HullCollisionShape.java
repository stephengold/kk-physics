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

import com.github.stephengold.joltjni.ConstShape;
import com.github.stephengold.joltjni.ConvexHullShape;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.Vec3;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A convex-hull collision shape based on jolt-jni's {@code ConvexHullShape}.
 */
public class HullCollisionShape extends CollisionShape {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes in a vector
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger2
            = Logger.getLogger(HullCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * non-flipped direct buffer for passing vertices to jolt-jni
     */
    private FloatBuffer directBuffer;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a shape based on the specified collection of locations.
     *
     * @param locations a collection of location vectors on which to base the
     * shape (not null, not empty, unaffected)
     */
    public HullCollisionShape(Collection<Vector3f> locations) {
        Validate.nonEmpty(locations, "locations");

        int numLocations = locations.size();
        int numFloats = numAxes * numLocations;
        this.directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (Vector3f location : locations) {
            if (!Vector3f.isValidVector(location)) {
                throw new IllegalArgumentException(
                        "illegal coordinates: " + location);
            }
            directBuffer.put(location.x).put(location.y).put(location.z);
        }

        createShape();
    }

    /**
     * Instantiate a shape based on the specified array of coordinates.
     *
     * @param coordinates an array of coordinates on which to base the shape
     * (not null, not empty, length a multiple of 3, unaffected)
     */
    public HullCollisionShape(float... coordinates) {
        Validate.nonEmpty(coordinates, "coordinates");
        int numFloats = coordinates.length;
        Validate.require(
                numFloats % numAxes == 0, "length a multiple of 3");

        this.directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (float f : coordinates) {
            if (!Float.isFinite(f)) {
                throw new IllegalArgumentException("illegal coordinate: " + f);
            }
            directBuffer.put(f);
        }

        createShape();
    }

    /**
     * Instantiate a shape based on a flipped buffer containing coordinates.
     *
     * @param flippedBuffer the coordinates on which to base the shape (not
     * null, limit&gt;0, limit a multiple of 3, unaffected)
     */
    public HullCollisionShape(FloatBuffer flippedBuffer) {
        Validate.nonNull(flippedBuffer, "flipped buffer");
        int numFloats = flippedBuffer.limit();
        Validate.positive(numFloats, "limit");
        Validate.require(numFloats % numAxes == 0, "limit a multiple of 3");

        this.directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (int i = 0; i < numFloats; ++i) {
            float f = flippedBuffer.get(i);
            if (!Float.isFinite(f)) {
                throw new IllegalArgumentException("illegal coordinate: " + f);
            }
            directBuffer.put(f);
        }

        createShape();
    }

    /**
     * Instantiate a shape based on the specified JME mesh(es).
     *
     * @param meshes the mesh(es) on which to base the shape (all non-null, at
     * least one vertex, unaffected)
     */
    public HullCollisionShape(Mesh... meshes) {
        Validate.nonEmpty(meshes, "meshes");
        this.directBuffer = getPoints(meshes);
        Validate.require(directBuffer.capacity() > 0, "at least one vertex");

        createShape();
    }

    /**
     * Instantiate a shape based on the specified array of locations.
     *
     * @param locations an array of location vectors (in shape coordinates, not
     * null, not empty, unaffected)
     */
    public HullCollisionShape(Vector3f... locations) {
        Validate.nonEmpty(locations, "locations");

        int numFloats = numAxes * locations.length;
        this.directBuffer = BufferUtils.createFloatBuffer(numFloats);
        for (Vector3f location : locations) {
            if (!Vector3f.isValidVector(location)) {
                throw new IllegalArgumentException(
                        "illegal coordinates: " + location);
            }
            directBuffer.put(location.x).put(location.y).put(location.z);
        }

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the unscaled vertex locations of the optimized convex hull.
     *
     * @return a new array (not null)
     */
    public float[] copyHullVertices() {
        ConvexHullShape shape = (ConvexHullShape) getUnscaledShape();
        int numHullVertices = shape.getNumPoints();
        float[] result = new float[numHullVertices * numAxes];
        for (int vi = 0; vi < numHullVertices; ++vi) {
            Vec3 point = shape.getPoint(vi);
            result[numAxes * vi] = point.getX();
            result[numAxes * vi + 1] = point.getY();
            result[numAxes * vi + 2] = point.getZ();
        }

        return result;
    }

    /**
     * Count the number of vertices in the optimized convex hull.
     *
     * @return the count (&ge;0)
     */
    public int countHullVertices() {
        ConvexHullShape shape = (ConvexHullShape) getUnscaledShape();
        int result = shape.getNumPoints();

        return result;
    }

    /**
     * Count the vertices used to generate the hull.
     *
     * @return the count (&gt;0)
     */
    public int countMeshVertices() {
        int length = directBuffer.capacity();
        assert (length % numAxes == 0) : length;
        int result = length / numAxes;

        assert result > 0 : result;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code ConvexHullShape}.
     */
    private void createShape() {
        int numFloats = directBuffer.capacity();
        assert numFloats != 0;
        assert (numFloats % numAxes == 0) : numFloats;
        int numVertices = numFloats / numAxes;

        ConvexHullShapeSettings settings
                = new ConvexHullShapeSettings(numVertices, directBuffer);
        ConstShape shape = settings.create().get().getPtr();
        setNativeObject(shape);
    }

    /**
     * Copy vertex positions from the specified JME mesh(es).
     *
     * @param meshes the mesh(es) to read (not null)
     * @return a new array (not null, length a multiple of 3)
     */
    private static FloatBuffer getPoints(Mesh... meshes) {
        int numVectors = 0;
        for (Mesh mesh : meshes) {
            numVectors += mesh.getVertexCount();
        }
        int numFloats = numAxes * numVectors;
        FloatBuffer result = BufferUtils.createFloatBuffer(numFloats);

        for (Mesh mesh : meshes) {
            FloatBuffer buffer = mesh.getFloatBuffer(Type.Position);
            int bufNumFloats = numAxes * mesh.getVertexCount();
            for (int bufPos = 0; bufPos < bufNumFloats; ++bufPos) {
                float f = buffer.get(bufPos);
                result.put(f);
            }
        }
        assert result.position() == numFloats;

        return result;
    }
}
