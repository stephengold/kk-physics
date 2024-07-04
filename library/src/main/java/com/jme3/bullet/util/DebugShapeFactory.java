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
package com.jme3.bullet.util;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import java.nio.FloatBuffer;
import java.util.Collections;
import java.util.Map;
import java.util.WeakHashMap;
import java.util.logging.Logger;
import jme3utilities.MeshNormals;
import jme3utilities.MyMesh;
import jme3utilities.Validate;

/**
 * A utility class to generate debug meshes for jolt-jni collision shapes.
 *
 * @author CJ Hare, normenhansen
 */
final public class DebugShapeFactory {
    // *************************************************************************
    // constants and loggers

    /**
     * number of axes
     */
    final private static int numAxes = 3;
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(DebugShapeFactory.class.getName());
    // *************************************************************************
    // fields

    /**
     * largest debug mesh to index (0&rarr;never index, MAX_VALUE&rarr;always
     * index)
     */
    private static int maxVerticesToIndex = 6_000;
    /**
     * map keys to previously generated debug meshes, for reuse
     * <p>
     * Synchronized so that it can be updated from the "Physics Cleaner" thread.
     */
    final private static Map<DebugMeshKey, Mesh> cache
            = Collections.synchronizedMap(
                    new WeakHashMap<DebugMeshKey, Mesh>(200));
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private DebugShapeFactory() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Forget all previously generated debug meshes.
     */
    public static void clearCache() {
        cache.clear();
    }

    /**
     * Count how many debug meshes are cached.
     *
     * @return the count (&ge;0)
     */
    public static int countCachedMeshes() {
        int result = cache.size();
        return result;
    }

    /**
     * Determine vertex locations for the specified collision shape.
     *
     * @param shape the input shape (not null, unaffected)
     * @param meshResolution ignored
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 3)
     */
    public static FloatBuffer debugVertices(
            CollisionShape shape, int meshResolution) {
        Validate.nonNull(shape, "shape");
        FloatBuffer result = shape.copyTriangles();

        assert (result.capacity() % numAxes) == 0 : result.capacity();
        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param shape the shape to visualize (not null, unaffected)
     * @return a new Triangles-mode Mesh (no indices, no normals)
     */
    public static Mesh getDebugMesh(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        FloatBuffer floatBuffer = getDebugTriangles(shape);
        Mesh result = new Mesh();
        result.setBuffer(VertexBuffer.Type.Position, numAxes, floatBuffer);
        result.updateBound();

        return result;
    }

    /**
     * For compatibility with the jme3-jbullet library.
     *
     * @param shape the shape to visualize (may be null, unaffected)
     * @return a new Spatial or null
     */
    public static Spatial getDebugShape(CollisionShape shape) {
        Spatial result;
        if (shape == null) {
            result = null;
        } else {
            result = createGeometry(shape, MeshNormals.None);
        }

        return result;
    }

    /**
     * Create a Spatial for visualizing the specified collision object.
     * <p>
     * This is mostly used internally. To enable debug visualization, use
     * {@link com.jme3.bullet.BulletAppState#setDebugEnabled(boolean)}.
     *
     * @param pco the object to visualize (not null, unaffected)
     * @return a new tree of nodes and geometries, or null
     */
    public static Spatial getDebugShape(PhysicsCollisionObject pco) {
        CollisionShape shape = pco.getCollisionShape();
        MeshNormals normals = pco.debugMeshNormals();
        Spatial result = createGeometry(shape, normals);

        return result;
    }

    /**
     * Generate vertex locations for triangles to visualize the specified
     * collision shape.
     *
     * @param shape the shape to visualize (not null, unaffected)
     * @return a new, unflipped, direct buffer full of scaled shape coordinates
     * (capacity a multiple of 9)
     */
    public static FloatBuffer getDebugTriangles(CollisionShape shape) {
        Validate.nonNull(shape, "shape");
        FloatBuffer result = shape.copyTriangles();

        assert (result.capacity() % 9) == 0 : result.capacity();
        return result;
    }

    /**
     * Forget all previously generated debug meshes for the identified shape.
     *
     * @param shapeId the ID of the shape to remove
     */
    public static void removeShapeFromCache(long shapeId) {
        synchronized (cache) {
            for (DebugMeshKey key : cache.keySet()) {
                if (key.shapeId() == shapeId) {
                    cache.remove(key);
                }
            }
        }
    }

    /**
     * Alter whether to index new debug meshes. (Doesn't affect cached meshes.)
     * Indexing might boost performance when there are many small meshes; it
     * isn't recommended for very large meshes.
     *
     * @param setting true&rarr;always index, false&rarr;never index
     */
    public static void setIndexBuffers(boolean setting) {
        maxVerticesToIndex = setting ? Integer.MAX_VALUE : -1;
    }

    /**
     * Alter whether to index new debug meshes. (Doesn't affect cached meshes.)
     * Indexing might boost performance when there are many small meshes; it
     * isn't recommended for very large meshes.
     *
     * @param maxVertices the largest mesh to be indexed (vertex count, &ge;-1,
     * default=6000)
     */
    public static void setIndexBuffers(int maxVertices) {
        Validate.inRange(maxVertices, "maxVertices", -1, Integer.MAX_VALUE);
        maxVerticesToIndex = maxVertices;
    }
    // *************************************************************************
    // private methods

    /**
     * Create a Geometry for visualizing the specified collision shape.
     *
     * @param shape (not null, unaffected)
     * @param normals which normals to generate (not null)
     * @return a new Geometry (not null)
     */
    private static Geometry createGeometry(
            CollisionShape shape, MeshNormals normals) {
        assert shape != null;
        assert normals != null;

        DebugMeshKey key = new DebugMeshKey(shape, normals);
        Mesh mesh;
        synchronized (cache) {
            mesh = cache.get(key);
            if (mesh == null) {
                mesh = createMesh(shape, normals);
                cache.put(key, mesh);
            }
        }

        Geometry geometry = new Geometry("Physics debug", mesh);
        geometry.updateModelBound();

        return geometry;
    }

    /**
     * Create a Mesh for visualizing the specified collision shape.
     *
     * @param shape (not null, unaffected)
     * @param normals which normals to generate (not null)
     * @return a new Mesh (not null)
     */
    private static Mesh createMesh(CollisionShape shape, MeshNormals normals) {
        FloatBuffer positions = shape.copyTriangles();
        Mesh mesh = new Mesh();
        mesh.setBuffer(VertexBuffer.Type.Position, numAxes, positions);

        switch (normals) {
            case Facet: // always start with a non-indexed mesh
                MyMesh.generateFacetNormals(mesh);
                break;

            case None:
                break;

            case Smooth:
                MyMesh.generateFacetNormals(mesh);
                MyMesh.smoothNormals(mesh);
                break;

            case Sphere:
                MyMesh.addSphereNormals(mesh);
                break;

            default:
                String message = "normals = " + normals;
                throw new IllegalArgumentException(message);
        }

        // If the mesh is not too big, generate an index buffer.
        if (!MyMesh.hasIndices(mesh)
                && mesh.getVertexCount() <= maxVerticesToIndex) {
            mesh = MyMesh.addIndices(mesh);
        }

        mesh.updateBound();
        mesh.setStatic();

        return mesh;
    }
}
