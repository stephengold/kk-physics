/*
 Copyright (c) 2019-2024 Stephen Gold
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
package jme3utilities.minie.test;

import com.jme3.bullet.CollisionSpace;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.objects.PhysicsBody;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.shape.CenterQuad;
import java.util.ArrayList;
import java.util.List;
import jme3utilities.MeshNormals;
import org.junit.Assert;
import org.junit.Test;

/**
 * Verify the defaults for physics objects.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestDefaults {
    // *************************************************************************
    // fields

    private static BoxCollisionShape box; // initialized by testShapes()
    private static PhysicsRigidBody rigidA;
    // *************************************************************************
    // new methods exposed

    /**
     * Verify the defaults for physics objects.
     */
    @Test
    public void testDefaults() {
        PhysicsSpace pSpace = new PhysicsSpace(1);
        // deactivation deadline
        float deadline = pSpace.getDeactivationDeadline();
        Assert.assertEquals(2f, deadline, 0f);

        // deactivation enabled flag
        boolean enabled = pSpace.isDeactivationEnabled();
        Assert.assertTrue(enabled);

        pSpace.setDeactivationEnabled(false);
        enabled = pSpace.isDeactivationEnabled();
        Assert.assertFalse(enabled);

        CollisionSpace cSpace = new CollisionSpace(1);
        testCollisionSpace(cSpace);

        testPhysicsSpace(pSpace);

        testShapes();
        testCollisionObjects();

        RigidBodyControl dynamicRbc = new RigidBodyControl(box);
        testRigidBody(dynamicRbc);
    }
    // *************************************************************************
    // private methods

    private static void testCollisionObjects() {
        PhysicsRigidBody staticPrb
                = new PhysicsRigidBody(box, PhysicsBody.massForStatic);
        testRigidBody(staticPrb);
        RigidBodyControl staticRbc
                = new RigidBodyControl(box, PhysicsBody.massForStatic);
        testRigidBody(staticRbc);

        rigidA = new PhysicsRigidBody(box);
        testRigidBody(rigidA);
    }

    /**
     * Verify defaults common to all newly-created collision spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private static void testCollisionSpace(CollisionSpace space) {
        Assert.assertNotNull(space);
    }

    /**
     * Test the defaults that are common to all newly-created convex shapes.
     *
     * @param shape the shape to test (not null, unaffected)
     */
    private static void testConvexShape(CollisionShape shape) {
        testShape(shape);

        Assert.assertFalse(shape.isNonMoving());
    }

    /**
     * Test the defaults that are common to all newly-created collision objects.
     *
     * @param pco the object to test (not null, unaffected)
     */
    private static void testPco(PhysicsCollisionObject pco) {
        Assert.assertFalse(pco.isInWorld());

        Assert.assertNull(pco.getCollisionSpace());

        Assert.assertSame(MeshNormals.None, pco.debugMeshNormals());

        Utils.assertEquals(0f, 0f, 0f, pco.getPhysicsLocation(null), 0f);
        Utils.assertEquals(0., 0., 0., pco.getPhysicsLocationDp(null), 0.);
        Utils.assertEquals(0f, 0f, 0f, 1f, pco.getPhysicsRotation(null), 0f);
        Assert.assertEquals(
                Matrix3f.IDENTITY, pco.getPhysicsRotationMatrix(null));

        Assert.assertEquals(0f, pco.getRestitution(), 0f);
        Assert.assertNull(pco.getUserObject());
    }

    /**
     * Verify defaults common to all newly-created physics spaces.
     *
     * @param space the space to test (not null, unaffected)
     */
    private static void testPhysicsSpace(PhysicsSpace space) {
        testCollisionSpace(space);

        Assert.assertEquals(0, space.countTickListeners());

        Assert.assertEquals(1 / 60f, space.getAccuracy(), 0f);
        Utils.assertEquals(0f, -9.81f, 0f, space.getGravity(null), 0f);
        Assert.assertEquals(4, space.maxSubSteps());
        Assert.assertEquals(0.1f, space.maxTimeStep(), 0f);
    }

    /**
     * Test the defaults that are common to all newly-created rigid bodies.
     *
     * @param prb the body to test (not null, unaffected)
     */
    private static void testRigidBody(PhysicsRigidBody prb) {
        Assert.assertNotNull(prb);
        testPco(prb);

        Assert.assertEquals(0f, prb.getAngularDamping(), 0f);
        Assert.assertEquals(0f, prb.getLinearDamping(), 0f);
        Assert.assertFalse(prb.isKinematic());
        Utils.assertEquals(0f, 0f, 0f, prb.totalAppliedForce(null), 0f);
        Utils.assertEquals(0f, 0f, 0f, prb.totalAppliedTorque(null), 0f);
        if (prb.getMass() > 0f) {
            Utils.assertEquals(0f, 0f, 0f, prb.getAngularVelocity(null), 0f);
            Utils.assertEquals(0f, 0f, 0f, prb.getLinearVelocity(null), 0f);
            Assert.assertEquals(1f, prb.getMass(), 0f);
            Assert.assertTrue(prb.isDynamic());
            Assert.assertFalse(prb.isStatic());

        } else {
            Assert.assertEquals(0f, prb.getMass(), 0f);
            Assert.assertFalse(prb.isDynamic());
            Assert.assertTrue(prb.isStatic());
        }

        if (prb instanceof RigidBodyControl) {
            RigidBodyControl rbc = (RigidBodyControl) prb;
            Assert.assertNull(rbc.getSpatial());
        }
    }

    /**
     * Test the defaults that are common to all newly-created collision shapes.
     *
     * @param shape the shape to test (not null, unaffected)
     */
    private static void testShape(CollisionShape shape) {
        Assert.assertNotNull(shape);
        Assert.assertNotEquals(0L, shape.nativeId());
        Utils.assertEquals(1f, 1f, 1f, shape.getScale(null), 0f);
    }

    /**
     * Verify the defaults for all collision shapes.
     */
    private static void testShapes() {
        testShapesConcave();
        testShapesConvex1();
        testShapesConvex2();
    }

    private static void testShapesConcave() {
        // MeshCollisionShape:
        Mesh quad = new CenterQuad(1f, 1f);
        MeshCollisionShape mesh2 = new MeshCollisionShape(quad, false);
        testShape(mesh2);
        Assert.assertEquals(2, mesh2.countMeshTriangles());
        Assert.assertEquals(4, mesh2.countMeshVertices());
        Assert.assertEquals(1, mesh2.countSubmeshes());
        Assert.assertTrue(mesh2.isNonMoving());
    }

    private static void testShapesConvex1() {
        // BoxCollisionShape
        box = new BoxCollisionShape(1f);
        testConvexShape(box);
        Assert.assertEquals(0.04f, box.getMargin(), 0f);

        // CapsuleCollisionShape
        CapsuleCollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        testConvexShape(capsule);
        Assert.assertEquals(PhysicsSpace.AXIS_Y, capsule.getAxis());
        Assert.assertEquals(1f, capsule.getHeight(), 0f);
        Assert.assertEquals(0f, capsule.getMargin(), 0f);

        // Z-axis CylinderCollisionShape
        CylinderCollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        testConvexShape(cylinder);
        Assert.assertEquals(PhysicsSpace.AXIS_Z, cylinder.getAxis());
        Assert.assertEquals(2f, cylinder.getHeight(), 0f);
        Assert.assertEquals(0.04f, cylinder.getMargin(), 0f);

        // HullCollisionShape
        List<Vector3f> prismVertices = new ArrayList<>(6);
        prismVertices.add(new Vector3f(1f, 1f, 1f));
        prismVertices.add(new Vector3f(1f, 1f, -1f));
        prismVertices.add(new Vector3f(-1f, 1f, 0f));
        prismVertices.add(new Vector3f(1f, -1f, 1f));
        prismVertices.add(new Vector3f(1f, -1f, -1f));
        prismVertices.add(new Vector3f(-1f, -1f, 0f));
        HullCollisionShape hull = new HullCollisionShape(prismVertices);
        testConvexShape(hull);
        Assert.assertEquals(6, hull.countHullVertices());
        Assert.assertEquals(6, hull.countMeshVertices());
        Assert.assertEquals(0.0345f, hull.getMargin(), 0.0001f);
    }

    private static void testShapesConvex2() {
        // SimplexCollisionShape of 1 vertex
        SimplexCollisionShape simplex1
                = new SimplexCollisionShape(new Vector3f(0f, 0f, 0f));
        testSimplexShape(simplex1);
        Assert.assertEquals(1, simplex1.countMeshVertices());
        Utils.assertEquals(0f, 0f, 0f, simplex1.copyVertex(0, null), 0f);
        Utils.assertEquals(0f, 0f, 0f, simplex1.getHalfExtents(null), 0f);
        Assert.assertEquals(0f, simplex1.unscaledVolume(), 0f);

        // SimplexCollisionShape of 2 vertices
        SimplexCollisionShape simplex2 = new SimplexCollisionShape(
                new Vector3f(1f, 0f, 0f), new Vector3f(-1, 0f, 0f));
        testSimplexShape(simplex2);
        Assert.assertEquals(2, simplex2.countMeshVertices());
        Utils.assertEquals(1f, 0f, 0f, simplex2.copyVertex(0, null), 0f);
        Utils.assertEquals(-1f, 0f, 0f, simplex2.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, 0f, 0f, simplex2.getHalfExtents(null), 0f);
        Assert.assertEquals(0f, simplex2.unscaledVolume(), 0f);

        // SimplexCollisionShape of 3 vertices
        SimplexCollisionShape simplex3 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(1f, 1f, 0f),
                new Vector3f(1f, 0f, 1f)
        );
        testSimplexShape(simplex3);
        Assert.assertEquals(3, simplex3.countMeshVertices());
        Utils.assertEquals(0f, 1f, 1f, simplex3.copyVertex(0, null), 0f);
        Utils.assertEquals(1f, 1f, 0f, simplex3.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, 0f, 1f, simplex3.copyVertex(2, null), 0f);
        Utils.assertEquals(1f, 1f, 1f, simplex3.getHalfExtents(null), 0f);
        Assert.assertEquals(0f, simplex3.unscaledVolume(), 0f);

        // SimplexCollisionShape of 4 vertices
        SimplexCollisionShape simplex4 = new SimplexCollisionShape(
                new Vector3f(0f, 1f, 1f),
                new Vector3f(0f, 1f, -1f),
                new Vector3f(1f, -1f, 0f),
                new Vector3f(-1f, -1f, 0f)
        );
        testSimplexShape(simplex4);
        Assert.assertEquals(4, simplex4.countMeshVertices());
        Utils.assertEquals(0f, 1f, 1f, simplex4.copyVertex(0, null), 0f);
        Utils.assertEquals(0f, 1f, -1f, simplex4.copyVertex(1, null), 0f);
        Utils.assertEquals(1f, -1f, 0f, simplex4.copyVertex(2, null), 0f);
        Utils.assertEquals(-1f, -1f, 0f, simplex4.copyVertex(3, null), 0f);
        Utils.assertEquals(1f, 1f, 1f, simplex4.getHalfExtents(null), 0f);
        Assert.assertEquals(4f / 3f, simplex4.unscaledVolume(), 1e-6f);

        // SphereCollisionShape
        SphereCollisionShape sphere = new SphereCollisionShape(1f);
        testConvexShape(sphere);
        Assert.assertEquals(0f, sphere.getMargin(), 0f);
    }

    /**
     * Test the defaults that are common to all newly-created simplex shapes.
     *
     * @param simplex the shape to test (not null, unaffected)
     */
    private static void testSimplexShape(SimplexCollisionShape simplex) {
        testConvexShape(simplex);
    }
}
