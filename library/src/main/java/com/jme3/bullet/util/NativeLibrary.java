/*
 * Copyright (c) 2019-2025 jMonkeyEngine
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
import com.github.stephengold.joltjni.JoltPhysicsObject;
import com.jme3.system.JmeSystem;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.system.Platform;
import java.util.Collection;
import java.util.List;
import java.util.Locale;
import java.util.TreeSet;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import jme3utilities.math.MyMath;
import oshi.SystemInfo;
import oshi.hardware.CentralProcessor;
import oshi.hardware.HardwareAbstractionLayer;

/**
 * Static interface to the jolt-jni native library.
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
    /**
     * expected version string of the native library
     */
    final public static String expectedVersion = "0.9.7";
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
        Platform platform = JmeSystem.getPlatform();
        if (platform == Platform.Linux64) {
            String assetPath = hasFmaFeatures()
                    ? "linux/x86-64-fma/com/github/stephengold/libjoltjni.so"
                    : "linux/x86-64/com/github/stephengold/libjoltjni.so";
            NativeLibraryLoader.registerNativeLibrary(
                    "jolt-jni", platform, assetPath);
        }
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.Linux_ARM32,
                "linux/armhf/com/github/stephengold/libjoltjni.so");
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.Linux_ARM64,
                "linux/aarch64/com/github/stephengold/libjoltjni.so");
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.MacOSX64,
                "osx/x86-64/com/github/stephengold/libjoltjni.dylib");
        NativeLibraryLoader.registerNativeLibrary("jolt-jni",
                Platform.MacOSX_ARM64,
                "osx/aarch64/com/github/stephengold/libjoltjni.dylib");
        if (platform == Platform.Windows64) {
            String assetPath = hasAvx2Features()
                    ? "windows/x86-64-avx2/com/github/stephengold/joltjni.dll"
                    : "windows/x86-64/com/github/stephengold/joltjni.dll";
            NativeLibraryLoader.registerNativeLibrary(
                    "jolt-jni", platform, assetPath);
        }

        NativeLibraryLoader.loadNativeLibrary("jolt-jni", true);
        String buildType = Jolt.buildType();
        if (!buildType.equals("Release")) {
            System.out.print(buildType + "-");
        }
        if (Jolt.isDoublePrecision()) {
            System.out.print("Dp-");
        }
        String jjVersion = Jolt.versionString();
        System.out.println(
                "jolt-jni version " + jjVersion + " initializing");

        if (!jjVersion.equals(expectedVersion)) {
            logger.warning("Expected a v" + expectedVersion
                    + " native library but loaded v" + jjVersion + "!");
        }

        //Jolt.setTraceAllocations(true); // to debug native memory allocation
        JoltPhysicsObject.startCleaner();

        Jolt.registerDefaultAllocator();
        Jolt.installDefaultAssertCallback();
        Jolt.installDefaultTraceCallback();
        boolean success = Jolt.newFactory();
        assert success;
        Jolt.registerTypes();
    }

    /**
     * Return the recommended number of worker threads to use.
     *
     * @return the count (&ge;1, &le;64)
     */
    public static int numThreads() {
        int numCpus = Runtime.getRuntime().availableProcessors();
        int result = (int) Math.floor(0.9 * numCpus);
        result = MyMath.clamp(result, 1, 64);

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
    // *************************************************************************
    // private methods

    /**
     * Test for the presence of four x86_64 ISA extensions that Jolt Physics can
     * exploit, including AVX2.
     *
     * @return {@code true} if those ISA extensions are present, otherwise
     * {@code false}
     */
    private static boolean hasAvx2Features() {
        boolean result = hasCpuFeatures("avx", "avx2", "sse4_1", "sse4_2");
        return result;
    }

    /**
     * Test whether all of the named CPU features are present.
     *
     * @param requiredFeatures the names of the features to test for
     * @return {@code true} if all are present, otherwise {@code false}
     */
    private static boolean hasCpuFeatures(String... requiredFeatures) {
        // Obtain the list of CPU feature strings from OSHI:
        SystemInfo si = new SystemInfo();
        HardwareAbstractionLayer hal = si.getHardware();
        CentralProcessor cpu = hal.getProcessor();
        List<String> oshiList = cpu.getFeatureFlags();

        Pattern pattern = Pattern.compile("[a-z][a-z0-9_]*");

        // Convert the list to a collection of feature names:
        Collection<String> presentFeatures = new TreeSet<>();
        for (String oshiString : oshiList) {
            String lcString = oshiString.toLowerCase(Locale.ROOT);
            Matcher matcher = pattern.matcher(lcString);
            while (matcher.find()) {
                String featureName = matcher.group();
                presentFeatures.add(featureName);
            }
        }

        // Test for each required CPU feature:
        for (String featureName : requiredFeatures) {
            String linuxName = featureName.toLowerCase(Locale.ROOT);
            String windowsName = "pf_" + linuxName + "_instructions_available";
            boolean isPresent = presentFeatures.contains(linuxName)
                    || presentFeatures.contains(windowsName);
            if (!isPresent) {
                return false;
            }
        }

        return true;
    }

    /**
     * Test for the presence of seven x86_64 ISA extensions that Jolt Physics
     * can exploit, including AVX2 and FMA.
     *
     * @return {@code true} if those ISA extensions are present, otherwise
     * {@code false}
     */
    private static boolean hasFmaFeatures() {
        boolean result = hasCpuFeatures(
                "avx", "avx2", "bmi1", "f16c", "fma", "sse4_1", "sse4_2");
        return result;
    }
}
