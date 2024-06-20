/*
 * Copyright (c) 2022-2024 jMonkeyEngine
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
package com.jme3.bullet.objects.infos;

import com.jme3.bullet.objects.PhysicsRigidBody;
import com.simsilica.mathd.Vec3d;
import java.util.logging.Logger;

/**
 * Copy certain properties of a {@code PhysicsRigidBody} in order to re-apply
 * them later.
 * <p>
 * Snapshots are used for rebuilding, so they don't include the collision shape,
 * mass, physics space, or position.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RigidBodySnapshot {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(RigidBodySnapshot.class.getName());
    // *************************************************************************
    // fields

    /**
     * friction parameter
     */
    private float friction;
    /**
     * angular velocity (in physics-space coordinates)
     */
    final private Vec3d angularVelocity;
    /**
     * linear velocity (in physics-space coordinates)
     */
    final private Vec3d linearVelocity;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a snapshot with default parameters.
     */
    public RigidBodySnapshot() {
        this.friction = 0.5f;
        this.angularVelocity = new Vec3d();
        this.linearVelocity = new Vec3d();
    }

    /**
     * Instantiate a snapshot of the specified body.
     *
     * @param body the body to capture (not null)
     */
    public RigidBodySnapshot(PhysicsRigidBody body) {
        this.friction = body.getFriction();

        // Vec3d
        if (body.isDynamic()) {
            this.angularVelocity = body.getAngularVelocityDp(null);
            this.linearVelocity = body.getLinearVelocityDp(null);
        } else {
            this.angularVelocity = new Vec3d();
            this.linearVelocity = new Vec3d();
        }
    }
    // *********************************************************************
    // new methods exposed

    /**
     * Apply the properties to the specified body.
     *
     * @param body the target body (not null, modified)
     */
    public void applyTo(PhysicsRigidBody body) {
        body.setFriction(friction);
        if (body.isDynamic()) {
            body.setAngularVelocityDp(angularVelocity);
            body.setLinearVelocityDp(linearVelocity);
        }
    }

    /**
     * Copy the snapshot's angular velocity.
     *
     * @param result (not null, unaffected)
     */
    public void getAngularVelocity(Vec3d result) {
        result.set(angularVelocity);
    }

    /**
     * Copy the snapshot's linear velocity.
     *
     * @param result (not null, unaffected)
     */
    public void getLinearVelocity(Vec3d result) {
        result.set(linearVelocity);
    }

    /**
     * Alter the snapshot's angular velocity.
     *
     * @param vec3d (not null, modified)
     */
    public void setAngularVelocity(Vec3d vec3d) {
        angularVelocity.set(vec3d);
    }

    /**
     * Alter the snapshot's linear velocity.
     *
     * @param vec3d (not null, modified)
     */
    public void setLinearVelocity(Vec3d vec3d) {
        linearVelocity.set(vec3d);
    }
}
