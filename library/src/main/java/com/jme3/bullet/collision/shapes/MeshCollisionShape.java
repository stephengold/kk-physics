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

import com.github.stephengold.joltjni.Float3;
import com.github.stephengold.joltjni.IndexedTriangle;
import com.github.stephengold.joltjni.IndexedTriangleList;
import com.github.stephengold.joltjni.MeshShapeSettings;
import com.github.stephengold.joltjni.Shape;
import com.github.stephengold.joltjni.VertexList;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.MyMesh;
import jme3utilities.Validate;
import jme3utilities.math.MyVector3f;

/**
 * A mesh collision shape based on jolt-jni's {@code MeshShape}. Not for use in
 * dynamic bodies.
 *
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape {
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
            = Logger.getLogger(MeshCollisionShape.class.getName());
    // *************************************************************************
    // fields

    /**
     * sequence of vertex indices
     */
    private IndexedTriangleList itList;
    /**
     * positions of the mesh vertices
     */
    private VertexList vertexList;
    // *************************************************************************
    // constructors

    /**
     * No-argument constructor needed by SavableClassUtil.
     */
    protected MeshCollisionShape() {
    }

    /**
     * Instantiate a shape based on the specified JME mesh.
     *
     * @param jmeMesh the mesh on which to base the shape (must contain at least
     * one triangle, unaffected)
     * @param useCompression true to use quantized AABB compression
     */
    public MeshCollisionShape(Mesh jmeMesh, boolean useCompression) {
        Validate.nonNull(jmeMesh, "mesh");
        Validate.require(MyMesh.hasTriangles(jmeMesh),
                "mode=Triangles/TriangleFan/TriangleStrip");
        int numVertices = jmeMesh.getVertexCount();
        Validate.nonNegative(numVertices, "number of vertices in mesh");

        FloatBuffer floatBuffer
                = jmeMesh.getFloatBuffer(VertexBuffer.Type.Position);
        this.vertexList = new VertexList();
        vertexList.resize(numVertices);
        for (int vi = 0; vi < numVertices; ++vi) {
            int startPosition = numAxes * vi;
            float x = floatBuffer.get(startPosition + MyVector3f.xAxis);
            float y = floatBuffer.get(startPosition + MyVector3f.yAxis);
            float z = floatBuffer.get(startPosition + MyVector3f.zAxis);
            Float3 location = new Float3(x, y, z);
            this.vertexList.set(vi, location);
        }

        IndexBuffer triangleIndices = jmeMesh.getIndicesAsList();
        int numIndices = triangleIndices.size();
        assert (numIndices % MyMesh.vpt) == 0 : numIndices;
        int numTriangles = numIndices / MyMesh.vpt;
        this.itList = new IndexedTriangleList();
        itList.resize(numTriangles);
        for (int triIndex = 0; triIndex < numTriangles; ++triIndex) {
            int startPosition = MyMesh.vpt * triIndex;
            int vi0 = triangleIndices.get(startPosition);
            int vi1 = triangleIndices.get(startPosition + 1);
            int vi2 = triangleIndices.get(startPosition + 2);
            IndexedTriangle triangle = new IndexedTriangle(vi0, vi1, vi2);
            itList.set(triIndex, triangle);
        }

        createShape();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many triangles are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshTriangles() {
        int result = itList.size();
        return result;
    }

    /**
     * Count how many vertices are in the mesh.
     *
     * @return the count (&ge;0)
     */
    public int countMeshVertices() {
        int numVertices = vertexList.size();
        return numVertices;
    }

    /**
     * Count how many submeshes are in the mesh.
     *
     * @return 1
     */
    public int countSubmeshes() {
        int result = 1;
        return result;
    }
    // *************************************************************************
    // Java private methods

    /**
     * Instantiate the configured {@code MeshShape}.
     */
    private void createShape() {
        MeshShapeSettings settings = new MeshShapeSettings(vertexList, itList);
        Shape shape = settings.createShape();
        setNativeObject(shape);
    }
}
