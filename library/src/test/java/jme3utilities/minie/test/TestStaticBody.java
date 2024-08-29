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

import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.ModelKey;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HeightfieldCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.PlaneCollisionShape;
import com.jme3.bullet.collision.shapes.SimplexCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.export.binary.BinaryLoader;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.Plane;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.terrain.heightmap.HeightMap;
import com.jme3.terrain.heightmap.ImageBasedHeightMap;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.plugins.AWTLoader;
import jme3utilities.MyAsset;
import org.junit.Test;

/**
 * Test the creation of static rigid bodies with various shapes.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestStaticBody {
    // *************************************************************************
    // fields

    /**
     * AssetManager to load Jaime and Simple_height.png
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    // *************************************************************************
    // new methods exposed

    /**
     * Test the creation of static rigid bodies with various shapes.
     */
    @Test
    public void testStaticBody() {
        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(".", FileLocator.class);
        assetManager.registerLocator(null, ClasspathLocator.class);

        PhysicsSpace space = new PhysicsSpace(1);

        // BoxCollisionShape
        CollisionShape box = new BoxCollisionShape(1f);
        PhysicsRigidBody boxBody = new PhysicsRigidBody(box, 0f);
        space.addCollisionObject(boxBody);

        // CapsuleCollisionShape
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        PhysicsRigidBody capsuleBody = new PhysicsRigidBody(capsule, 0f);
        space.addCollisionObject(capsuleBody);

        // CompoundCollisionShape of a capsule
        CompoundCollisionShape compound = new CompoundCollisionShape(1);
        compound.addChildShape(capsule, 0f, 1f, 0f);
        PhysicsRigidBody compoundBody = new PhysicsRigidBody(compound, 0f);
        space.addCollisionObject(compoundBody);

        // CylinderCollisionShape
        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(1f, 1f, 1f));
        PhysicsRigidBody cylinderBody = new PhysicsRigidBody(cylinder, 0f);
        space.addCollisionObject(cylinderBody);

        // HeightfieldCollisionShape
        Texture heightTexture = MyAsset.loadTexture(
                assetManager, "Textures/BumpMapTest/Simple_height.png", false);
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        HeightMap heightMap = new ImageBasedHeightMap(heightImage, heightScale);
        CollisionShape hcs = new HeightfieldCollisionShape(heightMap);
        PhysicsRigidBody hcsBody = new PhysicsRigidBody(hcs, 0f);
        space.addCollisionObject(hcsBody);

        // HullCollisionShape
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape hull = new HullCollisionShape(mesh);
        PhysicsRigidBody hullBody = new PhysicsRigidBody(hull, 0f);
        space.addCollisionObject(hullBody);

        // MeshCollisionShape without compression
        CollisionShape uncompressed = new MeshCollisionShape(mesh);
        PhysicsRigidBody uncompressedBody
                = new PhysicsRigidBody(uncompressed, 0f);
        space.addCollisionObject(uncompressedBody);

        // PlaneCollisionShape
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        PhysicsRigidBody pcsBody = new PhysicsRigidBody(pcs, 0f);
        space.addCollisionObject(pcsBody);

        // SimplexCollisionShape
        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        PhysicsRigidBody simplexBody = new PhysicsRigidBody(simplex, 0f);
        space.addCollisionObject(simplexBody);

        // SphereCollisionShape
        CollisionShape sphere = new SphereCollisionShape(1f);
        PhysicsRigidBody sphereBody = new PhysicsRigidBody(sphere, 0f);
        space.addCollisionObject(sphereBody);

        System.gc();
    }
}
