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

import com.github.stephengold.joltjni.EMotionQuality;
import com.jme3.app.state.AppState;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SolverType;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.debug.BulletDebugAppState;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import java.io.PrintStream;
import java.nio.FloatBuffer;
import java.util.Collection;
import java.util.logging.Logger;
import jme3utilities.MyString;
import jme3utilities.Validate;
import jme3utilities.debug.Describer;
import jme3utilities.debug.Dumper;
import jme3utilities.math.MyBuffer;
import jme3utilities.math.MyQuaternion;
import jme3utilities.math.MyVector3f;

/**
 * Dump Minie data structures for debugging purposes.
 * <p>
 * The level of detail can be configured dynamically.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class PhysicsDumper extends Dumper {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PhysicsDumper.class.getName());
    /**
     * local copy of {@link com.jme3.math.Vector3f#UNIT_XYZ}
     */
    final private static Vector3f scaleIdentity = new Vector3f(1f, 1f, 1f);
    // *************************************************************************
    // fields

    /**
     * enable dumping of children in compound collision shapes
     */
    private boolean dumpChildShapes = false;
    /**
     * enable dumping of clusters in soft bodies
     */
    private boolean dumpClustersInSofts = false;
    /**
     * enable dumping of ignored objects in collision objects
     */
    private boolean dumpIgnores = false;
    /**
     * enable dumping of physics joints in bodies
     */
    private boolean dumpJointsInBodies = false;
    /**
     * enable dumping of joints in physics spaces
     */
    private boolean dumpJointsInSpaces = false;
    /**
     * enable dumping of motors in physics joints
     */
    private boolean dumpMotors = false;
    /**
     * enable dumping native IDs of physics objects
     */
    private boolean dumpNativeIDs = false;
    /**
     * enable dumping of soft-body nodes in clusters
     */
    private boolean dumpNodesInClusters = false;
    /**
     * enable dumping of nodes in soft bodies
     */
    private boolean dumpNodesInSofts = false;
    /**
     * enable dumping of collision objects in physics spaces
     */
    private boolean dumpPcos = true;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a PhysicsDumper that uses {@code System.out} for output.
     */
    public PhysicsDumper() {
        super();
        PhysicsDescriber newDescriber = new PhysicsDescriber();
        setDescriber(newDescriber);
    }

    /**
     * Instantiate a PhysicsDumper that uses the specified output stream.
     *
     * @param printStream output stream (not null)
     */
    public PhysicsDumper(PrintStream printStream) {
        super(printStream);
        PhysicsDescriber newDescriber = new PhysicsDescriber();
        setDescriber(newDescriber);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Dump the specified BulletAppState.
     *
     * @param appState the app state to dump (not null, unaffected)
     */
    public void dump(BulletAppState appState) {
        Validate.nonNull(appState, "app state");
        dumpBas(appState, "");
    }

    /**
     * Dump the specified CollisionShape.
     *
     * @param shape the shape to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(CollisionShape shape, String indent) {
        Validate.nonNull(shape, "shape");
        Validate.nonNull(indent, "indent");

        addLine(indent);

        PhysicsDescriber describer = getDescriber();
        String desc = describer.describe(shape);
        stream.print(desc);

        Vector3f scale = shape.getScale(null);
        desc = describer.describeScale(scale);
        addDescription(desc);

        long objectId = shape.nativeId();
        addNativeId(objectId);
    }

    /**
     * Dump the specified PhysicsRigidBody.
     *
     * @param body the rigid body to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dump(PhysicsRigidBody body, String indent) {
        Validate.nonNull(body, "body");
        Validate.nonNull(indent, "indent");

        addLine(indent);
        stream.print("Rigid ");

        String desc = MyPco.describe(body);
        stream.print(desc);

        PhysicsDescriber describer = getDescriber();
        desc = describer.describeApplicationData(body);
        stream.print(desc);
        desc = describer.describeUser(body);
        stream.print(desc);

        Vector3f location = body.getPhysicsLocation(null);
        String locString = MyVector3f.describe(location);
        stream.printf(" loc[%s]", locString);

        Quaternion orientation = body.getPhysicsRotation(null);
        if (!MyQuaternion.isRotationIdentity(orientation)) {
            String orientText = MyQuaternion.describe(orientation);
            stream.printf(" orient[%s]", orientText);
        }

        long objectId = body.nativeId();
        addNativeId(objectId);

        // 2nd line: activation state and contact parameters
        addLine(indent);
        addActivationState(body);
        addContactParameters(body);

        if (body.isDynamic()) {
            // The next 3 lines describes the dynamic properties.
            addDynamicProperties(body, indent);
        }
        /*
         * The next line has the shape and scale.
         * There may be additional lines for child shapes.
         */
        CollisionShape shape = body.getCollisionShape();
        dump(shape, indent + " ");
    }

    /**
     * Dump the specified PhysicsSpace.
     *
     * @param space the PhysicsSpace to dump (not null, unaffected)
     */
    public void dump(PhysicsSpace space) {
        dump(space, "", null);
    }

    /**
     * Dump the specified PhysicsSpace with the specified filter. TODO dump a
     * CollisionSpace
     *
     * @param space the PhysicsSpace to dump (not null, unaffected)
     * @param indent (not null, may be empty)
     * @param filter determines which physics objects are dumped, or null to
     * dump all (unaffected)
     */
    public void dump(PhysicsSpace space, String indent,
            BulletDebugAppState.DebugAppStateFilter filter) {
        Validate.nonNull(indent, "indent");

        String type = space.getClass().getSimpleName();
        stream.printf("%n%s%s with ", indent, type);

        Collection<PhysicsRigidBody> rigidBodies = space.getRigidBodyList();
        int numRigids = rigidBodies.size();
        stream.printf("%d rigid%s", numRigids, (numRigids == 1) ? "" : "s");

        // 2nd line
        addLine(indent);

        Vector3f grav = space.getGravity(null);
        stream.printf(" grav[%s] timeStep[", MyVector3f.describe(grav));
        int maxSS = space.maxSubSteps();
        if (maxSS == 0) {
            float maxTimeStep = space.maxTimeStep();
            String mtsDesc = MyString.describe(maxTimeStep);
            stream.printf("VAR max=%s", mtsDesc);
        } else {
            float accuracy = space.getAccuracy();
            String accuDesc = MyString.describe(accuracy);
            stream.printf("%s maxSS=%d", accuDesc, maxSS);
        }

        int tCount = space.countTickListeners();
        stream.printf("] listeners[t=%d]", tCount);

        // 3rd line: solver type and info
        addLine(indent);
        SolverType solverType = space.getSolverType();
        stream.printf(" solver[%s]", solverType);

        if (dumpPcos) {
            String moreIndent = indent + indentIncrement();
            for (PhysicsRigidBody rigid : rigidBodies) {
                if (filter == null || filter.displayObject(rigid)) {
                    dump(rigid, moreIndent);
                }
            }
        }

        stream.println();
    }

    /**
     * Dump the specified BulletAppState.
     *
     * @param appState the app state to dump (not null, unaffected)
     * @param indent (not null)
     */
    public void dumpBas(BulletAppState appState, String indent) {
        Validate.nonNull(indent, "indent");

        String className = appState.getClass().getSimpleName();
        stream.print(className);

        if (appState.isEnabled()) {
            stream.print(" enabled ");

            if (!appState.isDebugEnabled()) {
                stream.print("NO");
            }
            stream.print("debug ");

            float speed = appState.getSpeed();
            String speedString = MyString.describe(speed);
            stream.printf("speed=%s", speedString);

            PhysicsSpace space = appState.getPhysicsSpace();
            String moreIndent = indent + indentIncrement();
            dump(space, moreIndent, null);
        } else {
            stream.println(" disabled");
        }
    }

    /**
     * Test whether the specified dump flag is set.
     *
     * @param dumpFlag which flag to test (not null)
     * @return true if output is enabled, otherwise false
     */
    public boolean isEnabled(DumpFlags dumpFlag) {
        boolean result;

        switch (dumpFlag) {
            case BoundsInSpatials:
                result = isDumpBounds();
                break;

            case Buckets:
                result = isDumpBucket();
                break;

            case CullHints:
                result = isDumpCull();
                break;

            case MatParams:
                result = isDumpMatParam();
                break;

            case NativeIDs:
                result = dumpNativeIDs;
                break;

            case Overrides:
                result = isDumpOverride();
                break;

            case Pcos:
                result = dumpPcos;
                break;

            case ShadowModes:
                result = isDumpShadow();
                break;

            case Transforms:
                result = isDumpTransform();
                break;

            case UserData:
                result = isDumpUser();
                break;

            case VertexData:
                result = isDumpVertex();
                break;

            default:
                throw new IllegalArgumentException("dumpFlag = " + dumpFlag);
        }

        return result;
    }

    /**
     * Configure the specified dump flag.
     *
     * @param dumpFlag which flag to set (not null)
     * @param newValue true to enable output, false to disable it
     * @return this instance for chaining
     */
    public PhysicsDumper setEnabled(DumpFlags dumpFlag, boolean newValue) {
        switch (dumpFlag) {
            case BoundsInSpatials:
                setDumpBounds(newValue);
                break;

            case Buckets:
                setDumpBucket(newValue);
                break;

            case CullHints:
                setDumpCull(newValue);
                break;

            case MatParams:
                setDumpMatParam(newValue);
                break;

            case NativeIDs:
                this.dumpNativeIDs = newValue;
                break;

            case Overrides:
                setDumpOverride(newValue);
                break;

            case Pcos:
                this.dumpPcos = newValue;
                break;

            case ShadowModes:
                setDumpShadow(newValue);
                break;

            case Transforms:
                setDumpTransform(newValue);
                break;

            case UserData:
                setDumpUser(newValue);
                break;

            case VertexData:
                setDumpVertex(newValue);
                break;

            default:
                throw new IllegalArgumentException("dumpFlag = " + dumpFlag);
        }

        return this;
    }
    // *************************************************************************
    // Dumper methods

    /**
     * Create a deep copy of this dumper.
     *
     * @return a new instance, equivalent to this one, with its own Describer
     * @throws CloneNotSupportedException if the superclass isn't cloneable
     */
    @Override
    public PhysicsDumper clone() throws CloneNotSupportedException {
        PhysicsDumper clone = (PhysicsDumper) super.clone();
        return clone;
    }

    /**
     * Dump the specified AppState.
     *
     * @param appState the AppState to dump (not null, unaffected)
     * @param indent (not null)
     */
    @Override
    public void dump(AppState appState, String indent) {
        Validate.nonNull(appState, "app state");
        Validate.nonNull(indent, "indent");

        if (appState instanceof BulletAppState) {
            dumpBas((BulletAppState) appState, indent);
        } else {
            super.dump(appState, indent);
        }
    }

    /**
     * Access the Describer used by this Dumper.
     *
     * @return the pre-existing instance (not null)
     */
    @Override
    public PhysicsDescriber getDescriber() {
        Describer describer = super.getDescriber();
        PhysicsDescriber result = (PhysicsDescriber) describer;

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Print the activation state of the specified rigid body.
     *
     * @param body (not null, unaffected)
     */
    private void addActivationState(PhysicsRigidBody body) {
        boolean isActive = body.isActive();
        stream.print(" act=");
        stream.print(isActive);
    }

    /**
     * Print the contact parameters of the specified rigid body.
     *
     * @param body (not null, unaffected)
     */
    private void addContactParameters(PhysicsRigidBody body) {
        float fric = body.getFriction();
        stream.print(" contact[fric=");
        stream.print(MyString.describe(fric));

        float rest = body.getRestitution();
        stream.print(" rest=");
        stream.print(MyString.describe(rest));

        stream.print(']');
    }

    /**
     * Print dynamic properties of the specified rigid body.
     *
     * @param rigidBody (not null, unaffected)
     * @param indent (not null)
     */
    private void addDynamicProperties(
            PhysicsRigidBody rigidBody, String indent) {
        // first line: gravity factor, motion quality, and damping
        addLine(indent);

        float fFactor = rigidBody.getGravityFactor();
        stream.print(" gFactor=");
        stream.print(MyString.describe(fFactor));

        EMotionQuality quality = rigidBody.getMotionQuality();
        stream.print(" quality=");
        stream.print(quality);

        float angularDamping = rigidBody.getAngularDamping();
        float linearDamping = rigidBody.getLinearDamping();
        stream.print(" damp[l=");
        stream.print(MyString.describe(linearDamping));
        stream.print(" a=");
        stream.print(MyString.describe(angularDamping));
        stream.print(']');

        // 2nd line: linear velocity, applied force
        addLine(indent);

        Vector3f v = rigidBody.getLinearVelocity(null);
        stream.printf(" v[%s]", MyVector3f.describe(v));
        Vector3f force = rigidBody.totalAppliedForce(null);
        stream.printf(" force[%s]", MyVector3f.describe(force));

        // 3rd line: angular velocity, applied torque
        addLine(indent);

        Vector3f angularVelocity = rigidBody.getAngularVelocity(null);
        stream.printf(" w[%s]", MyVector3f.describe(angularVelocity));
        Vector3f torq = rigidBody.totalAppliedTorque(null);
        stream.printf(" torq[%s]", MyVector3f.describe(torq));
    }

    /**
     * Add a native ID, if the flag is set.
     *
     * @param id the unique identifier (not zero)
     */
    private void addNativeId(long id) {
        if (dumpNativeIDs) {
            stream.print(" #");
            String hex = Long.toHexString(id);
            stream.print(hex);
        }
    }

    /**
     * Generate a textual description of the indexed vector in the specified
     * FloatBuffer.
     *
     * @param buffer the buffer to read (not null, unaffected)
     * @param vectorIndex the index of the vector in the buffer (&ge;0)
     * @return descriptive text (not null, not empty)
     */
    private static String describeVector(FloatBuffer buffer, int vectorIndex) {
        Vector3f vector = new Vector3f();
        MyBuffer.get(buffer, MyVector3f.numAxes * vectorIndex, vector);
        String locString = MyVector3f.describe(vector);

        return locString;
    }
}
