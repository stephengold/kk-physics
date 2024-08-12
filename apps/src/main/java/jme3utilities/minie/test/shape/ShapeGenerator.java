/*
 Copyright (c) 2020-2024 Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities.minie.test.shape;

import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.Mesh.Mode;
import com.jme3.util.BufferUtils;
import java.nio.FloatBuffer;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyVector3f;
import jme3utilities.math.noise.Generator;
import jme3utilities.mesh.Cone;
import jme3utilities.mesh.Dodecahedron;
import jme3utilities.mesh.Icosahedron;
import jme3utilities.mesh.Octahedron;
import jme3utilities.mesh.Prism;

/**
 * Generate pseudo-random collision shapes for use in MinieExamples.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ShapeGenerator extends Generator {
    // *************************************************************************
    // constants and loggers

    /**
     * square root of 3
     */
    final public static float root3 = FastMath.sqrt(3f);
    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(ShapeGenerator.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Explicit no-arg constructor to avoid javadoc warnings from JDK 18+.
     */
    public ShapeGenerator() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a box shape.
     *
     * @return a new shape (not null)
     */
    public BoxCollisionShape nextBox() {
        float rx = nextFloat(0.5f, 1.5f);
        float ry = nextFloat(0.5f, 1.5f);
        float rz = nextFloat(0.5f, 1.5f);
        Vector3f halfExtents = new Vector3f(rx, ry, rz);
        BoxCollisionShape result = new BoxCollisionShape(halfExtents);

        return result;
    }

    /**
     * Generate a capsule shape.
     *
     * @return a new shape (not null)
     */
    public CapsuleCollisionShape nextCapsule() {
        float radius = nextFloat(0.2f, 1.2f);
        float height = nextFloat(0.5f, 1.5f);
        int axis = nextInt(3);
        CapsuleCollisionShape result
                = new CapsuleCollisionShape(radius, height, axis);

        return result;
    }

    /**
     * Generate a cylinder shape.
     *
     * @return a new shape (not null)
     */
    public CylinderCollisionShape nextCylinder() {
        float baseRadius = nextFloat(0.5f, 1.5f);
        float height = nextFloat(1f, 4f);
        int axisIndex = nextInt(3);
        CylinderCollisionShape result
                = new CylinderCollisionShape(baseRadius, height, axisIndex);

        return result;
    }

    /**
     * Generate a centered HullCollisionShape, using the origin plus 4-to-19
     * pseudo-random vertices.
     *
     * @return a new shape
     */
    public HullCollisionShape nextHull() {
        int numVertices = nextInt(5, 20);

        FloatBuffer buffer = BufferUtils.createFloatBuffer(
                MyVector3f.numAxes * numVertices);
        buffer.put(0f).put(0f).put(0f);
        Vector3f tmpLocation = new Vector3f();
        for (int vertexI = 1; vertexI < numVertices; ++vertexI) {
            nextUnitVector3f(tmpLocation);
            tmpLocation.multLocal(1.5f);
            buffer.put(tmpLocation.x).put(tmpLocation.y).put(tmpLocation.z);
        }

        // Use arithmetic mean to center the vertices.
        int start = 0;
        int end = buffer.limit();
        Vector3f offset = MyBuffer.mean(buffer, start, end, null);
        offset.negateLocal();
        MyBuffer.translate(buffer, start, end, offset);

        HullCollisionShape result = new HullCollisionShape(buffer);

        return result;
    }

    /**
     * Generate a Platonic solid.
     *
     * @return a new shape (not null)
     */
    public CollisionShape nextPlatonic() {
        Mesh mesh;
        float radius;
        boolean noNormals = false;

        CollisionShape result;
        int solidType = nextInt(1, 4);
        switch (solidType) {
            case 1: // cube or regular hexahedron
                radius = 1.4f * nextFloat(0.5f, 1.5f);
                float halfExtent = radius / root3;
                result = new BoxCollisionShape(halfExtent);
                break;

            case 2: // regular octahedron
                radius = 1.4f * nextFloat(0.5f, 1.5f);
                mesh = new Octahedron(radius, noNormals);
                result = new HullCollisionShape(mesh);
                break;

            case 3: // regular dodecahedron
                radius = 1.1f * nextFloat(0.5f, 1.5f);
                mesh = new Dodecahedron(radius, Mode.Triangles);
                result = new HullCollisionShape(mesh);
                break;

            case 4: // regular icosahedron
                radius = 1.13f * nextFloat(0.5f, 1.5f);
                mesh = new Icosahedron(radius, noNormals);
                result = new HullCollisionShape(mesh);
                break;

            default:
                throw new RuntimeException("solidType = " + solidType);
        }

        return result;
    }

    /**
     * Generate a prism.
     *
     * @return a new shape (not null)
     */
    public HullCollisionShape nextPrism() {
        int numSides = nextInt(3, 9);
        float radius = nextFloat(0.6f, 2f);
        float height = nextFloat(0.6f, 1.6f);
        boolean noNormals = false;
        Mesh mesh = new Prism(numSides, radius, height, noNormals);
        HullCollisionShape result = new HullCollisionShape(mesh);

        return result;
    }

    /**
     * Generate a pyramid.
     *
     * @return a new shape (not null)
     */
    public HullCollisionShape nextPyramid() {
        int numSides = nextInt(3, 9);
        float baseRadius = nextFloat(0.8f, 1.8f);
        float yHeight = nextFloat(1f, 2.5f);
        boolean generatePyramid = true;
        Mesh mesh = new Cone(numSides, baseRadius, yHeight, generatePyramid);
        HullCollisionShape result = new HullCollisionShape(mesh);

        return result;
    }

    /**
     * Generate an instance of the named shape.
     *
     * @param shapeName the type of shape to generate (not null, not empty)
     * @return a new shape (not null)
     */
    public CollisionShape nextShape(String shapeName) {
        Validate.nonEmpty(shapeName, "shape name");

        CollisionShape result;
        switch (shapeName) {
            case "box":
                result = nextBox();
                break;

            case "capsule":
                result = nextCapsule();
                break;

            case "cylinder":
                result = nextCylinder();
                break;

            case "hull":
                result = nextHull();
                break;

            case "platonic":
                result = nextPlatonic();
                break;

            case "prism":
                result = nextPrism();
                break;

            case "pyramid":
                result = nextPyramid();
                break;

            case "sphere":
                result = nextSphere();
                break;

            default:
                String message = "shapeName = " + MyString.quote(shapeName);
                throw new IllegalArgumentException(message);
        }

        return result;
    }

    /**
     * Generate a sphere shape.
     *
     * @return a new shape (not null)
     */
    public SphereCollisionShape nextSphere() {
        float radius = nextFloat(0.5f, 1.5f);
        SphereCollisionShape result = new SphereCollisionShape(radius);

        return result;
    }
}
