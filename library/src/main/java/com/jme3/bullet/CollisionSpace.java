/*
 * Copyright (c) 2009-2024 jMonkeyEngine
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
package com.jme3.bullet;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import java.util.Collection;
import java.util.TreeSet;
import java.util.logging.Logger;
import jme3utilities.Validate;

/**
 * A Jolt-JME collision space.
 *
 * @author normenhansen
 */
public class CollisionSpace {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger loggerC
            = Logger.getLogger(CollisionSpace.class.getName());
    // *************************************************************************
    // fields

    /**
     * number of worker threads (&ge;1, &le;64)
     */
    final private int numSolvers;
    /**
     * collision-space reference for each thread
     * <p>
     * When a collision space is created, the current thread automatically
     * becomes associated with it. For the space to be garbage collected, the
     * same thread should null out its reference (using
     * {@code setLocalThreadPhysicsSpace()}) before terminating.
     */
    final private static ThreadLocal<CollisionSpace> physicsSpaceTL
            = new ThreadLocal<>();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a CollisionSpace. Must be invoked on the designated physics
     * thread.
     *
     * @param numSolvers the desired number of worker threads (&ge;1, &le;64)
     */
    public CollisionSpace(int numSolvers) {
        Validate.inRange(numSolvers, "number of solvers", 1, 64);
        this.numSolvers = numSolvers;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add the specified object to the space.
     *
     * @param object the object to add (not null)
     */
    public void add(Object object) {
        Validate.nonNull(object, "object");

        if (object instanceof PhysicsCollisionObject) {
            addCollisionObject((PhysicsCollisionObject) object);
        } else {
            String typeName = object.getClass().getCanonicalName();
            String msg = "Cannot add a " + typeName + " to a collision space.";
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Add the specified collision object to the space.
     *
     * @param pco the collision object to add (not null, modified)
     */
    public void addCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        String typeName = pco.getClass().getCanonicalName();
        String msg = "Unknown type of collision object: " + typeName;
        throw new IllegalArgumentException(msg);
    }

    /**
     * Test whether the specified collision object is added to the space.
     *
     * @param pco the object to test (not null, unaffected)
     * @return true if currently added, otherwise false
     */
    public boolean contains(PhysicsCollisionObject pco) {
        String typeName = pco.getClass().getCanonicalName();
        String msg = "Unknown type of collision object: " + typeName;
        throw new IllegalArgumentException(msg);
    }

    /**
     * Count the worker threads.
     *
     * @return the count (&ge;1, &le;64)
     */
    public int countSolvers() {
        assert numSolvers >= 1 : numSolvers;
        assert numSolvers <= 64 : numSolvers;
        return numSolvers;
    }

    /**
     * Access the CollisionSpace associated with the current thread.
     *
     * @return the pre-existing CollisionSpace, or {@code null} if none
     */
    public static CollisionSpace getCollisionSpace() {
        CollisionSpace result = physicsSpaceTL.get();
        return result;
    }

    /**
     * Enumerate collision objects that have been added to the space and not yet
     * removed.
     *
     * @return a new modifiable collection of pre-existing instances (not null)
     */
    public Collection<PhysicsCollisionObject> getPcoList() {
        Collection<PhysicsCollisionObject> result = new TreeSet<>();
        return result;
    }

    /**
     * Remove the specified object from the space.
     *
     * @param object the object to remove, or null
     */
    public void remove(Object object) {
        if (object == null) {
            return;
        } else if (object instanceof PhysicsCollisionObject) {
            removeCollisionObject((PhysicsCollisionObject) object);
        } else {
            String typeName = object.getClass().getCanonicalName();
            String msg = "Cannot remove a " + typeName
                    + " from a collision space.";
            throw new IllegalArgumentException(msg);
        }
    }

    /**
     * Remove the specified collision object from the space.
     *
     * @param pco the collision object to remove (not null, modified)
     */
    public void removeCollisionObject(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        String typeName = pco.getClass().getCanonicalName();
        String msg = "Unknown type of collision object: " + typeName;
        throw new IllegalArgumentException(msg);
    }
    // *************************************************************************
    // new protected methods

    /**
     * Must be invoked on the designated physics thread.
     */
    protected void initThread() {
        physicsSpaceTL.set(this);
    }
}
