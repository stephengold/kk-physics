/*
 Copyright (c) 2018-2024 Stephen Gold
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
import com.jme3.bullet.util.NativeLibrary;
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
import org.junit.Assert;
import org.junit.Test;

/**
 * Test the setScale() function on collision shapes of all types.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TestSetScale {
    // *************************************************************************
    // fields

    /**
     * AssetManager to load Jaime and Simple_height.png
     */
    final private static AssetManager assetManager = new DesktopAssetManager();
    /*
     * identity scaling vector
     */
    final private static Vector3f ident = new Vector3f(1f, 1f, 1f);
    /*
     * uniform scaling vector
     */
    final private static Vector3f uni = new Vector3f(9f, 9f, 9f);
    /*
     * non-uniform scaling vector with x=y
     */
    final private static Vector3f non = new Vector3f(9f, 9f, 1f);
    /*
     * non-uniform scaling vector with 3 unequal components
     */
    final private static Vector3f non2 = new Vector3f(1f, 2f, 3f);
    // *************************************************************************
    // new methods exposed

    /**
     * Test the setScale() function on collision shapes of all types.
     */
    @Test
    public void testSetScale() {
        NativeLibrary.load();

        assetManager.registerLoader(AWTLoader.class, "jpg", "png");
        assetManager.registerLoader(BinaryLoader.class, "j3o");
        assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
        assetManager.registerLocator(null, ClasspathLocator.class);

        setScaleConcave();
        setScaleConvex();

        // CompoundCollisionShape of a box
        CollisionShape childBox = new BoxCollisionShape(1f);
        CompoundCollisionShape compound = new CompoundCollisionShape(1);
        compound.addChildShape(childBox, 0f, 1f, 0f);
        Utils.assertEquals(ident, compound.getScale(null), 0f);

        System.gc();
    }
    // *************************************************************************
    // private methods

    private static void setScaleConcave() {
        // HeightfieldCollisionShape
        Texture heightTexture = MyAsset.loadTexture(
                assetManager, "Textures/BumpMapTest/Simple_height.png", false);
        Image heightImage = heightTexture.getImage();
        float heightScale = 1f;
        HeightMap heightMap = new ImageBasedHeightMap(heightImage, heightScale);
        CollisionShape hcs = new HeightfieldCollisionShape(heightMap);
        Utils.assertEquals(ident, hcs.getScale(null), 0f);
        hcs.setScale(uni);
        Utils.assertEquals(uni, hcs.getScale(null), 0f);
        hcs.setScale(non);
        Utils.assertEquals(non, hcs.getScale(null), 0f);
        hcs.setScale(non2);
        Utils.assertEquals(non2, hcs.getScale(null), 0f);

        // MeshCollisionShape
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape mcs = new MeshCollisionShape(mesh);
        Utils.assertEquals(ident, mcs.getScale(null), 0f);
        mcs.setScale(uni);
        Utils.assertEquals(uni, mcs.getScale(null), 0f);
        mcs.setScale(non);
        Utils.assertEquals(non, mcs.getScale(null), 0f);
        mcs.setScale(non2);
        Utils.assertEquals(non2, mcs.getScale(null), 0f);

        // PlaneCollisionShape
        Plane plane = new Plane(new Vector3f(0f, 1f, 0f), 0f);
        CollisionShape pcs = new PlaneCollisionShape(plane);
        Utils.assertEquals(ident, pcs.getScale(null), 0f);
        pcs.setScale(uni);
        Utils.assertEquals(uni, pcs.getScale(null), 0f);
        pcs.setScale(non);
        Utils.assertEquals(non, pcs.getScale(null), 0f);
        pcs.setScale(non2);
        Utils.assertEquals(non2, pcs.getScale(null), 0f);
    }

    private static void setScaleConvex() {
        // BoxCollisionShape
        CollisionShape box = new BoxCollisionShape(2f);
        Utils.assertEquals(ident, box.getScale(null), 0f);
        box.setScale(uni);
        Utils.assertEquals(uni, box.getScale(null), 0f);
        box.setScale(non);
        Utils.assertEquals(non, box.getScale(null), 0f);
        box.setScale(non2);
        Utils.assertEquals(non2, box.getScale(null), 0f);

        // Y-axis CapsuleCollisionShape
        CollisionShape capsule = new CapsuleCollisionShape(1f, 1f);
        Utils.assertEquals(ident, capsule.getScale(null), 0f);
        capsule.setScale(uni);
        Utils.assertEquals(uni, capsule.getScale(null), 0f);

        // Z-axis CylinderCollisionShape
        CollisionShape cylinder
                = new CylinderCollisionShape(new Vector3f(3f, 3f, 3f));
        Assert.assertEquals(ident, cylinder.getScale(null));
        cylinder.setScale(uni);
        Assert.assertEquals(uni, cylinder.getScale(null));
        cylinder.setScale(non);
        Assert.assertEquals(non, cylinder.getScale(null));

        // HullCollisionShape
        ModelKey key = new ModelKey("Models/Jaime/Jaime.j3o");
        Node model = (Node) assetManager.loadModel(key);
        Geometry geo = (Geometry) model.getChild(0);
        Mesh mesh = geo.getMesh();
        CollisionShape hull = new HullCollisionShape(mesh);
        Utils.assertEquals(ident, hull.getScale(null), 0f);
        hull.setScale(uni);
        Utils.assertEquals(uni, hull.getScale(null), 0f);
        hull.setScale(non);
        Utils.assertEquals(non, hull.getScale(null), 0f);
        hull.setScale(non2);
        Utils.assertEquals(non2, hull.getScale(null), 0f);

        // SimplexCollisionShape of 3 vertices
        Vector3f p1 = new Vector3f(0f, 1f, 1f);
        Vector3f p2 = new Vector3f(1f, 0f, 1f);
        Vector3f p3 = new Vector3f(1f, 1f, 0f);
        CollisionShape simplex = new SimplexCollisionShape(p1, p2, p3);
        Utils.assertEquals(ident, simplex.getScale(null), 0f);

        // SphereCollisionShape
        CollisionShape sphere = new SphereCollisionShape(1f);
        Utils.assertEquals(ident, sphere.getScale(null), 0f);
        sphere.setScale(uni);
        Utils.assertEquals(uni, sphere.getScale(null), 0f);
    }
}
