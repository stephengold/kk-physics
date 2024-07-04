// Gradle script to build the "apps" subproject of KK Physics

plugins {
    `application` // to build JVM applications
}

java {
    sourceCompatibility = JavaVersion.VERSION_19
    targetCompatibility = JavaVersion.VERSION_19
}

application {
    mainClass = "jme3utilities.minie.test.issue.TestManyBoxes"
}
tasks.named<Jar>("jar") {
    manifest {
        attributes["Main-Class"] = application.mainClass
    }
}

val btf = "ReleaseSp"
dependencies {
    implementation(libs.jme3.core)
    implementation(libs.jme3.desktop)
    runtimeOnly(libs.jme3.awt.dialogs)
    runtimeOnly(libs.jme3.lwjgl3)
    runtimeOnly(libs.jme3.testdata)

    implementation(libs.heart)
    runtimeOnly(variantOf(libs.jolt.jni.linux64){ classifier(btf) })
    runtimeOnly(variantOf(libs.jolt.jni.macosx64){ classifier(btf) })
    runtimeOnly(variantOf(libs.jolt.jni.macosxarm64){ classifier(btf) })
    runtimeOnly(variantOf(libs.jolt.jni.windows64){ classifier(btf) })

    implementation(project(":library")) // for latest sourcecode
}

configurations.all {
    resolutionStrategy.cacheChangingModulesFor(0, "seconds") // to disable caching of snapshots
}

tasks.withType<JavaCompile>().all { // Java compile-time options:
    options.compilerArgs.add("-Xdiags:verbose")
    options.compilerArgs.add("-Xlint:unchecked")
    options.encoding = "UTF-8"
    options.release = 19
    options.setDeprecation(true) // to provide detailed deprecation warnings
}

tasks.withType<Javadoc>().all { // Javadoc runtime options:
    (options as CoreJavadocOptions).apply {
        addStringOption("-release", "19")
    }
}

tasks.withType<JavaExec>().all { // Java runtime options:
    classpath = sourceSets.main.get().getRuntimeClasspath()
    enableAssertions = true
    jvmArgs("--enable-native-access=ALL-UNNAMED")
    jvmArgs("--enable-preview")
}

// Register cleanup tasks:

tasks.named("clean") {
    dependsOn("cleanDLLs", "cleanDyLibs", "cleanLogs", "cleanSOs")
}
tasks.register<Delete>("cleanDLLs") { // extracted Windows native libraries
    delete(fileTree(".").matching{ include("*.dll") })
}
tasks.register<Delete>("cleanDyLibs") { // extracted macOS native libraries
    delete(fileTree(".").matching{ include("*.dylib") })
}
tasks.register<Delete>("cleanLogs") { // JVM crash logs
    delete(fileTree(".").matching{ include("hs_err_pid*.log") })
}
tasks.register<Delete>("cleanSOs") { // extracted Linux native libraries
    delete(fileTree(".").matching{ include("*.so") })
}
