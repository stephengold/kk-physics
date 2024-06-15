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
package com.jme3.bullet.control;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.control.AbstractControl;
import java.lang.foreign.MemorySession;
import jme3utilities.MySpatial;
import jolt.Jolt;
import jolt.math.FVec3;
import jolt.math.Quat;
import jolt.physics.Activation;
import jolt.physics.body.BodyCreationSettings;
import jolt.physics.body.BodyInterface;
import jolt.physics.body.MotionType;
import jolt.physics.body.MutableBody;

/**
 * An AbstractControl to link a rigid body to a Spatial.
 *
 * @author normenhansen
 */
public class RigidBodyControl extends AbstractControl {
    // *************************************************************************
    // fields

    final private BoxCollisionShape shape;
    final private float mass;
    private MutableBody body;
    private PhysicsSpace physicsSpace;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an enabled Control with an active/responsive rigid body and
     * the specified shape and mass.
     *
     * @param shape the desired shape (not null, alias created)
     * @param mass the desired mass (&ge;0)
     */
    public RigidBodyControl(BoxCollisionShape shape, float mass) {
        this.shape = shape;
        this.mass = mass;
    }
    // *************************************************************************
    // new methods exposed

    public int getId() {
        int result = body.getId();
        return result;
    }

    public void setPhysicsSpace(PhysicsSpace physicsSpace) {
        this.physicsSpace = physicsSpace;
    }
    // *************************************************************************
    // AbstractControl methods

    @Override
    protected void controlUpdate(float f) {
        MemorySession arena = PhysicsSpace.getArena();
        if (body == null && spatial != null && physicsSpace != null) {

            Vector3f wt = spatial.getWorldTranslation(); // alias
            FVec3 location = FVec3.of(arena, wt.x, wt.y, wt.z);

            Quaternion wr = spatial.getWorldRotation(); // alias
            Quat orientation = Quat.of(
                    arena, wr.getX(), wr.getY(), wr.getZ(), wr.getW());

            BodyCreationSettings bcs;
            if (mass > 0f) {
                bcs = BodyCreationSettings.of(arena, shape.getShape(),
                        location, orientation, MotionType.DYNAMIC,
                        PhysicsSpace.OBJ_LAYER_MOVING);
                bcs.setGravityFactor(1f);
            } else {
                assert mass == 0 : mass;
                bcs = BodyCreationSettings.of(arena, shape.getShape(),
                        location, orientation, MotionType.STATIC,
                        PhysicsSpace.OBJ_LAYER_NON_MOVING);
                bcs.setGravityFactor(0f);
            }

            BodyInterface bodyInterface = physicsSpace.getBodyInterface();
            this.body = bodyInterface.createBody(bcs);

            int id = body.getId();
            if (mass > 0f) {
                FVec3 v = FVec3.of(arena, 0f, 1f, 0f);
                body.setLinearVelocity(v);
                bodyInterface.addBody(id, Activation.ACTIVATE);
            } else {
                bodyInterface.addBody(id, Activation.DONT_ACTIVATE);
            }
        }

        if (body != null && spatial != null && physicsSpace != null) {
            Jolt.assertSinglePrecision();
            BodyInterface bodyInterface = physicsSpace.getBodyInterface();
            int bodyId = body.getId();
            FVec3 wt = FVec3.of(arena);
            bodyInterface.getCenterOfMassPosition(bodyId, wt);
            Vector3f loc = new Vector3f(
                    (float) wt.getX(), (float) wt.getY(), (float) wt.getZ());
            MySpatial.setWorldLocation(spatial, loc);

            Quat wr = Quat.of(arena);
            body.getRotation(wr);
            Quaternion ori = new Quaternion(
                    wr.getX(), wr.getY(), wr.getZ(), wr.getW());
            MySpatial.setWorldOrientation(spatial, ori);
        }
    }

    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }
}
