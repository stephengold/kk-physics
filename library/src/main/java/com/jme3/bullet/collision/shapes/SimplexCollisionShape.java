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

import com.github.stephengold.joltjni.ConvexHullShape;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.TriangleShape;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.AbstractTriangle;
import com.jme3.math.FastMath;
import com.jme3.math.LineSegment;
import com.jme3.math.Vector3f;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.MyVolume;

/**
 * A simple point, line-segment, triangle, or tetrahedron collision shape based
 * on jolt-jni's {@code TriangleShape} or {@code ConvexHullShape}.
 *
 * @author normenhansen
 */
public class SimplexCollisionShape extends CollisionShape {
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
            = Logger.getLogger(SimplexCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * vertex locations
     */
    private Vector3f[] locations;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected SimplexCollisionShape() {
    }

    /**
     * Instantiate a point shape based on the specified location.
     *
     * @param location the location of the point (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f location) {
        this.locations = new Vector3f[1];
        locations[0] = location.clone();
        createShape();
    }

    /**
     * Instantiate a line-segment shape based on the specified endpoints.
     *
     * @param point1 the location of first endpoint (in shape coordinates, not
     * null, unaffected)
     * @param point2 the location of 2nd endpoint (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f point1, Vector3f point2) {
        this.locations = new Vector3f[2];
        locations[0] = point1.clone();
        locations[1] = point2.clone();
        createShape();
    }

    /**
     * Instantiate a triangular shape based on the specified vertices.
     *
     * @param vertex1 the location of first vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex2 the location of 2nd vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex3 the location of 3rd vertex (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f vertex1, Vector3f vertex2,
            Vector3f vertex3) {
        this.locations = new Vector3f[3];
        locations[0] = vertex1.clone();
        locations[1] = vertex2.clone();
        locations[2] = vertex3.clone();
        createShape();
    }

    /**
     * Instantiate a tetrahedral shape based on the specified vertices.
     *
     * @param vertex1 the location of first vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex2 the location of 2nd vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex3 the location of 3rd vertex (in shape coordinates, not
     * null, unaffected)
     * @param vertex4 the location of 4th vertex (in shape coordinates, not
     * null, unaffected)
     */
    public SimplexCollisionShape(Vector3f vertex1, Vector3f vertex2,
            Vector3f vertex3, Vector3f vertex4) {
        this.locations = new Vector3f[4];
        locations[0] = vertex1.clone();
        locations[1] = vertex2.clone();
        locations[2] = vertex3.clone();
        locations[3] = vertex4.clone();
        createShape();
    }

    /**
     * Instantiate a triangular shape based on the specified AbstractTriangle.
     *
     * @param triangle the triangle (not null, unaffected)
     */
    public SimplexCollisionShape(AbstractTriangle triangle) {
        Validate.nonNull(triangle, "triangle");

        this.locations = new Vector3f[3];
        locations[0] = triangle.get1().clone();
        locations[1] = triangle.get2().clone();
        locations[2] = triangle.get3().clone();
        createShape();
    }

    /**
     * Instantiate a simplex shape based on the specified FloatBuffer range.
     *
     * @param buffer the buffer that contains the vertex locations (not null,
     * unaffected)
     * @param startPosition the buffer position at which the vertex locations
     * start (&ge;0, &le;endPosition-3)
     * @param endPosition the buffer position at which the vertex locations end
     * (&ge;startPosition+3, &le;capacity)
     */
    public SimplexCollisionShape(FloatBuffer buffer, int startPosition,
            int endPosition) {
        Validate.nonNull(buffer, "buffer");
        Validate.inRange(startPosition, "start position", 0,
                endPosition - numAxes);
        Validate.inRange(endPosition, "end position", startPosition + numAxes,
                buffer.capacity());
        int numFloats = endPosition - startPosition;
        Validate.require(numFloats % numAxes == 0, "range a multiple of 3");
        int numVertices = numFloats / numAxes;
        assert numVertices >= 1 : numVertices;
        assert numVertices <= 4 : numVertices;

        this.locations = new Vector3f[numVertices];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            locations[vertexIndex] = new Vector3f();
            MyBuffer.get(buffer, startPosition + vertexIndex * numAxes,
                    locations[vertexIndex]);
        }

        createShape();
    }

    /**
     * Instantiate a line-segment shape based on the specified LineSegment.
     *
     * @param segment the segment (not null, unaffected)
     */
    public SimplexCollisionShape(LineSegment segment) {
        Validate.nonNull(segment, "segment");

        this.locations = new Vector3f[2];
        locations[0] = segment.getNegativeEnd(null);
        locations[1] = segment.getPositiveEnd(null);

        createShape();
    }

    /**
     * Instantiate a simplex shape based on the specified array.
     *
     * @param vertices an array of vertex locations (not null, not empty,
     * unaffected)
     */
    public SimplexCollisionShape(Vector3f[] vertices) {
        Validate.nonEmpty(vertices, "vertices");
        int numVertices = vertices.length;
        assert numVertices <= 4 : numVertices;

        this.locations = new Vector3f[numVertices];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            locations[vertexIndex] = vertices[vertexIndex].clone();
        }

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy the indexed vertex.
     *
     * @param index (&ge;0, &lt;4)
     * @param storeResult storage for the result (modified if not null)
     * @return the location of the vertex (either storeResult or a new instance)
     */
    public Vector3f copyVertex(int index, Vector3f storeResult) {
        int numVertices = locations.length;
        Validate.inRange(index, "index", 0, numVertices - 1);

        Vector3f result;
        if (storeResult == null) {
            result = locations[index].clone();
        } else {
            result = storeResult.set(locations[index]);
        }

        return result;
    }

    /**
     * Copy the unscaled vertex locations.
     *
     * @return a new array (not null)
     */
    public float[] copyVertices() {
        int numVertices = locations.length;
        float[] result = new float[numVertices * numAxes];
        for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
            int floatIndex = vertexIndex * numAxes;
            Vector3f location = locations[vertexIndex];
            result[floatIndex + PhysicsSpace.AXIS_X] = location.x;
            result[floatIndex + PhysicsSpace.AXIS_Y] = location.y;
            result[floatIndex + PhysicsSpace.AXIS_Z] = location.z;
        }

        return result;
    }

    /**
     * Count the points used to generate the simplex.
     *
     * @return the count (&ge;1, &le;4)
     */
    public int countMeshVertices() {
        int result = locations.length;

        assert result >= 1 : result;
        assert result <= 4 : result;
        return result;
    }

    /**
     * Calculate the unscaled half extents of the simplex.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return the unscaled half extent for each local axis (either storeResult
     * or a new vector, not null, no negative component)
     */
    public Vector3f getHalfExtents(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        result.zero();
        for (Vector3f location : locations) {
            float x = FastMath.abs(location.x);
            if (x > result.x) {
                result.x = x;
            }
            float y = FastMath.abs(location.y);
            if (y > result.y) {
                result.y = y;
            }
            float z = FastMath.abs(location.z);
            if (z > result.z) {
                result.z = z;
            }
        }

        assert MyVector3f.isAllNonNegative(result) : result;
        return result;
    }

    /**
     * Calculate the unscaled volume of the simplex.
     *
     * @return the volume (in shape units cubed, &ge;0)
     */
    public float unscaledVolume() {
        float volume = 0f;
        if (locations.length > 3) {
            volume = (float) MyVolume.tetrahedronVolume(locations[0],
                    locations[1], locations[2], locations[3]);
        }

        assert volume >= 0f : volume;
        return volume;
    }
    // *************************************************************************
    // CollisionShape methods

    /**
     * Test whether the specified scale factors can be applied to the shape. For
     * simplex shapes, scaling must be unity.
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
        ShapeRefC ref = getUndecoratedShapeRef();
        ConstShape shape = ref.getPtr();
        float result;
        if (shape instanceof ConvexHullShape) {
            ConvexHullShape hullShape = (ConvexHullShape) shape;
            result = hullShape.getConvexRadius();
        } else {
            TriangleShape triangleShape = (TriangleShape) shape;
            result = triangleShape.getConvexRadius();
        }

        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured shape.
     */
    private void createShape() {
        int numVertices = locations.length;
        switch (numVertices) {
            case 1:
                createTriangle(locations[0], locations[0], locations[0]);
                break;
            case 2:
                createTriangle(locations[0], locations[1], locations[1]);
                break;
            case 3:
                createTriangle(locations[0], locations[1], locations[2]);
                break;
            case 4:
                createHull(locations[0], locations[1], locations[2],
                        locations[3]);
                break;
            default:
                String message = "numVertices = " + numVertices;
                throw new IllegalArgumentException(message);
        }
    }

    /**
     * Instantiate the configured {@code ConvexHullShape}.
     *
     * @param a the first vertex (not null, unaffected)
     * @param b the 2nd vertex (not null, unaffected)
     * @param c the 3rd vertex (not null, unaffected)
     * @param d the 4rd vertex (not null, unaffected)
     */
    private void createHull(Vector3f a, Vector3f b, Vector3f c, Vector3f d) {
        int numVertices = 4;
        int numFloats = numAxes * numVertices;
        FloatBuffer directBuffer = BufferUtils.createFloatBuffer(numFloats);
        directBuffer.put(a.x).put(a.y).put(a.z);
        directBuffer.put(b.x).put(b.y).put(b.z);
        directBuffer.put(c.x).put(c.y).put(c.z);
        directBuffer.put(d.x).put(d.y).put(d.z);

        ConvexHullShapeSettings settings
                = new ConvexHullShapeSettings(numVertices, directBuffer);
        float defaultMargin = getDefaultMargin();
        settings.setMaxConvexRadius(defaultMargin);
        ShapeRefC shapeRef = settings.create().get();

        ConvexHullShape hullShape = (ConvexHullShape) shapeRef.getPtr();
        this.margin = hullShape.getConvexRadius();
        setNativeObject(shapeRef);
    }

    /**
     * Instantiate the configured {@code TriangleShape}.
     *
     * @param a the first vertex (not null, unaffected)
     * @param b the 2nd vertex (not null, unaffected)
     * @param c the 3rd vertex (not null, unaffected)
     */
    private void createTriangle(Vector3f a, Vector3f b, Vector3f c) {
        Vec3Arg v1 = new Vec3(a.x, a.y, a.z);
        Vec3Arg v2 = new Vec3(b.x, b.y, b.z);
        Vec3Arg v3 = new Vec3(c.x, c.y, c.z);
        this.margin = getDefaultMargin();

        TriangleShape shape = new TriangleShape(v1, v2, v3, margin);
        ShapeRefC shapeRef = shape.toRefC();
        setNativeObject(shapeRef);
    }
}
