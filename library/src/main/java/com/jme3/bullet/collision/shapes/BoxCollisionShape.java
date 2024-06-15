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

import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import java.lang.foreign.MemorySession;
import jolt.Jolt;
import jolt.math.FVec3;
import jolt.physics.collision.shape.BoxShapeSettings;
import jolt.physics.collision.shape.Shape;

/**
 * An axis-aligned, rectangular-solid collision shape.
 *
 * @author normenhansen
 */
public class BoxCollisionShape {
    // *************************************************************************
    // fields

    final private Shape shape;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a box with the specified half extents.
     *
     * @param halfExtents the desired half extents on each local axis (not null,
     * all components &ge;0, unaffected)
     */
    public BoxCollisionShape(Vector3f halfExtents) {
        MemorySession arena = PhysicsSpace.getArena();
        FVec3 vec3
                = FVec3.of(arena, halfExtents.x, halfExtents.y, halfExtents.z);
        BoxShapeSettings bss = BoxShapeSettings.of(vec3);
        this.shape = Jolt.use(bss, settings -> {
            return settings.create(arena);
        }).orThrow();
    }
    // *************************************************************************
    // new methods exposed

    public Shape getShape() {
        return shape;
    }
}
