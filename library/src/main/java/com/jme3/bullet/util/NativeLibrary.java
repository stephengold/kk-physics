/*
 * Copyright (c) 2019-2024 jMonkeyEngine
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
package com.jme3.bullet.util;

import com.github.stephengold.joltjni.Jolt;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.system.Platform;
import java.util.logging.Logger;
import jme3utilities.math.MyMath;

/**
 * Static interface to the jolt-jni library.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class NativeLibrary {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(NativeLibrary.class.getName());
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private NativeLibrary() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many threads are available for task scheduling.
     *
     * @return the count (&ge;1, &le;64)
     */
    public static int countThreads() {
        int result = Runtime.getRuntime().availableProcessors() - 1;
        result = MyMath.clamp(result, 1, 64);

        return result;
    }

    /**
     * Test whether the native library uses double-precision arithmetic.
     *
     * @return true if double-precision, false if single-precision
     */
    public static boolean isDoublePrecision() {
        boolean result = Jolt.isDoublePrecision();
        return result;
    }

    /**
     * Load the appropriate jolt-jni native library from the classpath and
     * initialize it.
     */
    public static void load() {
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.Linux64,
                "linux/x86-64/com/github/stephengold/libjoltjni.so");
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.MacOSX64,
                "osx/x86-64/com/github/stephengold/libjoltjni.dylib");
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.MacOSX_ARM64,
                "osx/aarch64/com/github/stephengold/libjoltjni.dylib");
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.Windows64,
                "windows/x86-64/com/github/stephengold/joltjni.dll");

        NativeLibraryLoader.loadNativeLibrary("jolt-jni", true);
        String buildType = Jolt.buildType();
        if (!buildType.equals("Release")) {
            System.out.print(buildType + "-");
        }
        if (Jolt.isDoublePrecision()) {
            System.out.print("Dp-");
        }
        System.out.println(
                "jolt-jni version " + Jolt.versionString() + " initializing");

        Jolt.registerDefaultAllocator();
        Jolt.newFactory();
        Jolt.registerTypes();
    }

    /**
     * Return the number of worker threads to use.
     *
     * @return the recommended number (&ge;1)
     */
    public static int numThreads() {
        int result = Runtime.getRuntime().availableProcessors() - 1;
        if (result < 1) {
            result = 1;
        }

        return result;
    }

    /**
     * Determine the native library's version string.
     *
     * @return the version string (typically of the form Major.Minor.Patch)
     */
    public static String versionNumber() {
        String result = Jolt.versionString();
        return result;
    }
}
