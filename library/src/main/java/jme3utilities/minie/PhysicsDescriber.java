/*
 Copyright (c) 2013-2024 Stephen Gold
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
package jme3utilities.minie;

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.collision.shapes.CylinderCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.SphereCollisionShape;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.Control;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.debug.Describer;

/**
 * Generate compact textual descriptions of physics objects for debugging
 * purposes.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PhysicsDescriber extends Describer {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsDescriber.class.getName());
    // *************************************************************************
    // constructors

    /**
     * Instantiate a describer with the default separator.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public PhysicsDescriber() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Generate a textual description for a CollisionShape.
     *
     * @param shape (not null, unaffected)
     * @return description (not null)
     */
    public String describe(CollisionShape shape) {
        Validate.nonNull(shape, "shape");

        StringBuilder result = new StringBuilder(80);

        String desc = MyShape.describeType(shape);
        result.append(desc);

        if (shape instanceof BoxCollisionShape) {
            Vector3f he = ((BoxCollisionShape) shape).getHalfExtents(null);
            desc = describeHalfExtents(he);
            result.append(desc);

        } else if (shape instanceof CapsuleCollisionShape) {
            CapsuleCollisionShape capsule = (CapsuleCollisionShape) shape;
            int axis = capsule.getAxis();
            desc = MyString.axisName(axis);
            result.append(desc);

            float height = capsule.getHeight();
            float radius = capsule.getRadius();
            desc = describeHeightAndRadius(height, radius);
            result.append(desc);

        } else if (shape instanceof CylinderCollisionShape) {
            CylinderCollisionShape cylinder = (CylinderCollisionShape) shape;
            int axis = cylinder.getAxis();
            desc = MyString.axisName(axis);
            result.append(desc);

            Vector3f he = cylinder.getHalfExtents(null);
            desc = describeHalfExtents(he);
            result.append(desc);

        } else if (shape instanceof HullCollisionShape) {

        } else if (shape instanceof MeshCollisionShape) {
            MeshCollisionShape meshShape = (MeshCollisionShape) shape;
            int numV = meshShape.countMeshVertices();
            desc = String.format("[numV=%d]", numV);
            result.append(desc);

        } else if (shape instanceof SphereCollisionShape) {
            SphereCollisionShape sphere = (SphereCollisionShape) shape;
            result.append(" r=");
            float radius = sphere.getRadius();
            result.append(MyString.describe(radius));

        } else {
            result.append('?');
        }

        return result.toString();
    }

    /**
     * Describe the application data of a collision object.
     *
     * @param pco the collision object to describe (not null, unaffected)
     * @return a descriptive string (not null, may be empty)
     */
    public String describeApplicationData(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        String result = "";
        Object aData = pco.getApplicationData();
        if (aData != null) {
            StringBuilder builder = new StringBuilder(64);
            builder.append(" aData=");
            appendObjectDescription(builder, aData);
            result = builder.toString();
        }

        return result;
    }

    /**
     * Describe a collision object in the context of another PCO.
     *
     * @param pco the object to describe (not null, unaffected)
     * @param forceId true to force inclusion of the native ID, false to include
     * it only if there's no user or application data
     * @return descriptive text (not null, not empty)
     */
    String describePco(PhysicsCollisionObject pco, boolean forceId) {
        StringBuilder result = new StringBuilder(80);
        appendPco(result, pco, forceId);
        return result.toString();
    }

    /**
     * Describe the user of a collision object.
     *
     * @param pco the collision object to describe (not null, unaffected)
     * @return a descriptive string (not null, may be empty)
     */
    public String describeUser(PhysicsCollisionObject pco) {
        Validate.nonNull(pco, "collision object");

        String result = "";
        Object user = pco.getUserObject();
        if (user != null) {
            StringBuilder builder = new StringBuilder(64);
            builder.append(" user=");
            appendObjectDescription(builder, user);
            result = builder.toString();
        }

        return result;
    }
    // *************************************************************************
    // Describer methods

    /**
     * Create a copy of this PhysicsDescriber.
     *
     * @return a new instance, equivalent to this one
     * @throws CloneNotSupportedException if the superclass isn't cloneable
     */
    @Override
    public PhysicsDescriber clone() throws CloneNotSupportedException {
        PhysicsDescriber clone = (PhysicsDescriber) super.clone();
        return clone;
    }

    /**
     * Generate a textual description of a scene-graph control.
     *
     * @param control (not null)
     * @return description (not null)
     */
    @Override
    protected String describe(Control control) {
        Validate.nonNull(control, "control");
        String result = MyControlP.describe(control);
        return result;
    }

    /**
     * Test whether a scene-graph control is enabled.
     *
     * @param control control to test (not null)
     * @return true if the control is enabled, otherwise false
     */
    @Override
    protected boolean isControlEnabled(Control control) {
        Validate.nonNull(control, "control");

        boolean result = !MyControlP.canDisable(control)
                || MyControlP.isEnabled(control);

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Append a description of an arbitrary Object, such as a PCO's user.
     *
     * @param builder the StringBuilder to append to (not null, modified)
     * @param subject the Object to describe (not null, unaffected)
     */
    private static void appendObjectDescription(
            StringBuilder builder, Object subject) {
        String className = subject.getClass().getSimpleName();

        String desc;
        if (subject instanceof Material) {
            builder.append(className);
            desc = ((Material) subject).getName();
        } else if (subject instanceof Spatial) {
            builder.append(className);
            desc = ((Spatial) subject).getName();
        } else if (subject instanceof String) {
            builder.append("String");
            desc = (String) subject;
        } else {
            desc = subject.toString();
        }

        if (desc != null) {
            if (desc.length() > 50) {
                desc = desc.substring(0, 47) + "...";
            }
            builder.append(MyString.quote(desc));
        }
    }

    /**
     * Append a description of a collision object in the context of another PCO.
     *
     * @param builder the StringBuilder to append to (not null, modified)
     * @param pco the object to describe (not null, unaffected)
     * @param forceId true to force inclusion of the native ID, false to include
     * it only if there's no user or application data
     */
    private void appendPco(StringBuilder builder, PhysicsCollisionObject pco,
            boolean forceId) {
        String desc;
        if (pco.getApplicationData() == null && pco.getUserObject() == null) {
            desc = pco.toString();
            builder.append(desc);

        } else {
            builder.append('[');

            if (forceId) {
                desc = pco.toString();
            } else {
                desc = pco.getClass().getSimpleName();
                desc = desc.replace("Body", "");
                desc = desc.replace("Control", "C");
                desc = desc.replace("Physics", "");
                desc = desc.replace("Object", "");
            }
            builder.append(desc);

            desc = describeApplicationData(pco);
            builder.append(desc);

            desc = describeUser(pco);
            builder.append(desc);

            builder.append(']');
        }

        if (!pco.isInWorld()) {
            builder.append("_NOT_IN_WORLD");
        }
    }

    /**
     * Describe the specified height and radius.
     *
     * @param height the height of the shape
     * @param radius the radius of the shape
     * @return a bracketed description (not null, not empty)
     */
    private static String describeHeightAndRadius(float height, float radius) {
        String hText = MyString.describe(height);
        String rText = MyString.describe(radius);
        String result = String.format(" h=%s r=%s", hText, rText);

        return result;
    }
}
