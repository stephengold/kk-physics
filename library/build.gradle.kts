// Gradle script to build the "library" subproject of KK Physics

import org.gradle.nativeplatform.platform.internal.DefaultNativePlatform

plugins {
    `checkstyle`  // to analyze Java sourcecode for style violations
    `java-library`  // to build JVM libraries
    `maven-publish` // to publish artifacts to Maven repositories
    `signing`       // to sign artifacts for publication
}

val group = "com.github.stephengold"
val artifact = "kk-physics"
val libraryVersion = rootProject.ext["version"] as String
val baseName = "${artifact}-${libraryVersion}" // for artifacts
val javaVendor = System.getProperty("java.vendor")
val javaVersion = JavaVersion.current()
val websiteUrl = "https://github.com/stephengold/kk-physics"

java {
    sourceCompatibility = JavaVersion.VERSION_11
    targetCompatibility = JavaVersion.VERSION_11
}

val btf = "DebugSp"
val os = DefaultNativePlatform.getCurrentOperatingSystem()

dependencies {
    api(libs.jme3.core)
    api(libs.jme3.terrain)
    api(libs.jolt.jni.windows64)
    api(libs.sim.math)
    implementation(libs.heart)
    implementation(libs.jme3.desktop)
    implementation(libs.oshi.core)

    testImplementation(libs.junit4)
    testImplementation(libs.log4j.impl)
    testRuntimeOnly(libs.jme3.testdata)
    if (os.isLinux()) {
        testRuntimeOnly(variantOf(libs.jolt.jni.linux64){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.linux64fma){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.linuxarm32hf){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.linuxarm64){ classifier(btf) })
    }
    if (os.isMacOsX()) {
        testRuntimeOnly(variantOf(libs.jolt.jni.macosx64){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.macosxarm64){ classifier(btf) })
    }
    if (os.isWindows()) {
        testRuntimeOnly(variantOf(libs.jolt.jni.windows64){ classifier(btf) })
        testRuntimeOnly(variantOf(libs.jolt.jni.windows64avx2){ classifier(btf) })
    }
}

configurations.all {
    resolutionStrategy.cacheChangingModulesFor(0, "seconds") // to disable caching of snapshots
}

checkstyle {
    toolVersion = libs.versions.checkstyle.get()
}

tasks.withType<JavaCompile>().all { // Java compile-time options:
    options.compilerArgs.add("-Xdiags:verbose")
    options.compilerArgs.add("-Xlint:unchecked")
    options.encoding = "UTF-8"
    options.release = 11
    //options.setDeprecation(true) // to provide detailed deprecation warnings
}

tasks.withType<Javadoc>().all { // Javadoc runtime options:
    (options as CoreJavadocOptions).apply {
    }
}

// Register publishing tasks:

tasks.register("install") {
    dependsOn("publishMavenPublicationToMavenLocal")
    description = "Installs the Maven artifacts to the local repository."
}
tasks.register("release") {
    dependsOn("publishMavenPublicationToOSSRHRepository")
    description = "Stages the Maven artifacts to Sonatype OSSRH."
}

tasks.jar {
    archiveBaseName.set(baseName)
    doLast {
        println("built using Java $javaVersion ($javaVendor)")
    }
    manifest {
        attributes["Created-By"] = "$javaVersion ($javaVendor)"
    }
}

java.withJavadocJar()
tasks.named<Jar>("javadocJar") { archiveBaseName.set(baseName) }
tasks.register<Jar>("sourcesJar") {
    archiveBaseName.set(baseName)
    archiveClassifier.set("sources")
    description = "Creates a JAR of Java sourcecode."
    from(sourceSets.main.get().java) // default is ".allSource", which includes resources
}

tasks.named("assemble") { dependsOn("module", "moduleAsc", "pom", "pomAsc") }
tasks.register<Copy>("module") {
    dependsOn("generateMetadataFileForMavenPublication")
    description = "Copies the module metadata to build/libs."
    from("build/publications/maven/module.json")
    into("build/libs")
    rename("module.json", baseName + ".module")
}
tasks.register<Copy>("moduleAsc") {
    dependsOn("signMavenPublication")
    description = "Copies the signature of the module metadata to build/libs."
    from("build/publications/maven/module.json.asc")
    into("build/libs")
    rename("module.json.asc", baseName + ".module.asc")
}
tasks.register<Copy>("pom") {
    dependsOn("generatePomFileForMavenPublication")
    description = "Copies the Maven POM to build/libs."
    from("build/publications/maven/pom-default.xml")
    into("build/libs")
    rename("pom-default.xml", baseName + ".pom")
}
tasks.register<Copy>("pomAsc") {
    dependsOn("signMavenPublication")
    description = "Copies the signature of the Maven POM to build/libs."
    from("build/publications/maven/pom-default.xml.asc")
    into("build/libs")
    rename("pom-default.xml.asc", baseName + ".pom.asc")
}

publishing {
    publications {
        create<MavenPublication>("maven") {
            artifact(tasks.named("sourcesJar"))
            artifactId = artifact
            from(components["java"])
            groupId = group
            pom {
                description = "A modern physics library for JMonkeyEngine"
                developers {
                    developer {
                        email = "sgold@sonic.net"
                        name = "Stephen Gold"
                    }
                }
                inceptionYear = "2024"
                licenses {
                    license {
                        distribution = "repo"
                        name = "New BSD (3-clause) License"
                        url = websiteUrl + "/blob/master/LICENSE"
                    }
                }
                name = "${group}:${artifact}"
                scm {
                    connection = "scm:git:git://github.com/stephengold/kk-physics.git"
                    developerConnection = "scm:git:ssh://github.com:stephengold/kk-physics.git"
                    url = websiteUrl + "/tree/master"
                }
                url = websiteUrl
            }
            version = libraryVersion
        }
    }
    // Staging to OSSRH relies on the existence of 2 properties
    // (ossrhUsername and ossrhPassword)
    // which should be stored in ~/.gradle/gradle.properties
    repositories {
        maven {
            credentials {
                username = if (hasProperty("ossrhUsername")) property("ossrhUsername") as String else "Unknown user"
                password = if (hasProperty("ossrhPassword")) property("ossrhPassword") as String else "Unknown password"
            }
            name = "OSSRH"
            url = uri("https://s01.oss.sonatype.org/service/local/staging/deploy/maven2")
        }
    }
}
tasks.named("generateMetadataFileForMavenPublication") { dependsOn("pom") }
tasks.named("publishMavenPublicationToMavenLocal") {
    dependsOn("assemble")
    doLast { println("installed locally as " + baseName) }
}
tasks.named("publishMavenPublicationToOSSRHRepository") { dependsOn("assemble") }

// Register signing tasks:

// Signing relies on the existence of 3 properties
// (signing.keyId, signing.password, and signing.secretKeyRingFile)
// which should be stored in ~/.gradle/gradle.properties

signing {
    sign(publishing.publications["maven"])
}
tasks.withType<Sign>().all {
    onlyIf { project.hasProperty("signing.keyId") }
}
tasks.named("signMavenPublication") { dependsOn("module") }
