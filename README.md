The [KK Physics Project][project] is about integrating
[the Jolt Physics engine][jolt] into
[the jMonkeyEngine (JME) game engine][jme].

This project is currently under development
and is not intended for production use.

It contains 2 subprojects:

 1. library: the KK Physics [JVM] runtime library and its automated tests
 2. apps: non-automated test software

Complete source code (in [Java]) is provided under
[a 3-clause BSD license][license].


<a name="toc"></a>

## Contents of this document

+ [How to add KK Physics to an existing project](#add)
+ [How to build KK Physics from source](#build)
+ [External links](#links)
+ [Acknowledgments](#acks)


<a name="add"></a>

## How to add KK Physics to an existing project

KK Physics comes pre-built as a single JVM library
that can be downloaded from Maven Central or GitHub.

KK Physics replaces (and is thus incompatible with)
the jme3-bullet, jme3-jbullet, and Minie libaries.
Before adding KK Physics to a project,
you should remove those libraries
so they won’t interfere with KK Physics.

For projects built using [Maven] or [Gradle], it is sufficient to add
dependencies on the KK Physics library and appropriate Jolt-JNI native libraries.
The build tool should automatically resolve the remaining dependencies.

Current Jolt-JNI releases provide 32 desktop native libraries,
each specific to a particular platform, build type, and build flavor.

Eight desktop platforms are supported:
+ "Linux64" (Linux on x86_64 CPUs)
+ "Linux64_fma" (Linux on x86_64 CPUs with AVX2 and FMA extensions)
+ "Linux_ARM32hf" (Linux on 32-bit ARM CPUs with hardware floating-point)
+ "Linux_ARM64" (Linux on aarch64 CPUs)
+ "MacOSX64" (macOS on Intel CPUs)
+ "MacOSX_ARM64" (macOS on "Apple Silicon")
+ "Windows64" (MS Windows on x86_64 CPUs)
+ "Windows64_avx2" (MS Windows on x86_64 CPUs with AVX2 extensions)

Your runtime classpath should include
the JVM library plus 1-to-8 native libraries:
a native library for each platform on which the code will run.

Build types:  use "Debug" native libraries for development and troubleshooting,
then switch to "Release" libraries for performance testing and production.

Build flavors:  use "Dp" to simulate large worlds (>1000 meters in diameter)
otherwise use "Sp".

### Gradle-built projects

Add to the project’s "build.gradle" or "build.gradle.kts" file:

    repositories {
        mavenCentral()
    }
    dependencies {
        // JVM library:
        implementation("com.github.stephengold:kk-physics:0.3.1")

        // native libraries:
        runtimeOnly("com.github.stephengold:jolt-jni-Linux64:0.9.5:DebugSp")
        // Native libraries for other platforms could be added.
    }

+ The "Linux64" platform name may be replaced
  with the name of another desktop platform.
+ The "DebugSp" classifier
  may be replaced by "DebugDp", "ReleaseSp", or "ReleaseDp".
+ For some older versions of Gradle,
  it's necessary to replace `implementation` with `compile`.

### Maven-built projects

Add to the project’s "pom.xml" file:

    <repositories>
      <repository>
        <id>mvnrepository</id>
        <url>https://repo1.maven.org/maven2/</url>
      </repository>
    </repositories>

    <dependency>
      <groupId>com.github.stephengold</groupId>
      <artifactId>kk-physics</artifactId>
      <version>0.3.1</version>
    </dependency>
    <dependency>
      <groupId>com.github.stephengold</groupId>
      <artifactId>jolt-jni-Linux64</artifactId>
      <version>0.9.5</version>
      <classifier>DebugSp</classifier>
    </dependency>


<a name="build"></a>

## How to build KK Physics from source

### Initial build

1. Install a [Java Development Kit (JDK)][adoptium],
   version 17 or higher,
   if you don't already have one.
2. Point the `JAVA_HOME` environment variable to your JDK installation:
   (In other words, set it to the path of a directory/folder
   containing a "bin" that contains a Java executable.
   That path might look something like
   "C:\Program Files\Eclipse Adoptium\jdk-17.0.3.7-hotspot"
   or "/usr/lib/jvm/java-17-openjdk-amd64/" or
   "/Library/Java/JavaVirtualMachines/zulu-17.jdk/Contents/Home" .)
  + using Bash or Zsh: `export JAVA_HOME="` *path to installation* `"`
  + using [Fish]: `set -g JAVA_HOME "` *path to installation* `"`
  + using Windows Command Prompt: `set JAVA_HOME="` *path to installation* `"`
  + using PowerShell: `$env:JAVA_HOME = '` *path to installation* `'`
3. Download and extract the KK Physics source code from GitHub:
  + using [Git]:
    + `git clone https://github.com/stephengold/kk-physics.git`
    + `cd kk-physics`
    + `git checkout -b latest 0.3.1`
4. Run the [Gradle] wrapper:
  + using Bash or Fish or PowerShell or Zsh: `./gradlew build`
  + using Windows Command Prompt: `.\gradlew build`

After a successful build,
Maven artifacts will be found in "library/build/libs".

### Other tasks

You can install the artifacts to your local Maven repository:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew install`
+ using Windows Command Prompt: `.\gradlew install`

You can restore the project to a pristine state:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew clean`
+ using Windows Command Prompt: `.\gradlew clean`

[Jump to the table of contents](#toc)


<a name="links"></a>

## External links

+ [Jolt Physics Multicore Scaling (2022)](https://jrouwe.nl/jolt/JoltPhysicsMulticoreScaling.pdf)
+ [Architecting Jolt Physics (2022)](https://gdcvault.com/play/1027891/Architecting-Jolt-Physics-for-Horizon),
  also in a different form [here](https://jrouwe.nl/architectingjolt/ArchitectingJoltPhysics_Rouwe_Jorrit_Notes.pdf)
+ [Minie](https://stephengold.github.io/Minie/minie/overview.html),
  a similar project that integrates Bullet Physics into jMonkeyEngine
+ [The physics section of the jMonkeyEngine Wiki (2020)](https://wiki.jmonkeyengine.org/docs/3.4/physics/physics.html)
+ [Alan Chou's game-physics tutorial (2013)](http://allenchou.net/game-physics-series/)

[Jump to the table of contents](#toc)


<a name="history"></a>

## History

The evolution of this project is chronicled in
[its release log][log].

The API and much of the source code is derived from [Minie].

[Jump to the table of contents](#toc)


<a name="acks"></a>

## Acknowledgments

Like most projects, the KK Physics Project builds on the work of many who
have gone before.  I therefore acknowledge the following
software developers:

+ Jorrit Rouwe (aka "jrouwe") for creating the Jolt Physics Engine
+ Normen Hansen (aka "normen") for creating most of the `jme3-bullet` library
  (from which the KK Physics API is derived)
+ "aecsocket" for creating the [JoltJava] bindings to the Jolt Physics Engine
+ plus the creators of (and contributors to) the following software:
    + the [Firefox] web browser
    + the [Git] revision-control system and GitK commit viewer
    + the [GitKraken] client
    + the [Gradle] build tool
    + the [Java] compiler, standard doclet, and runtime environment
    + [jMonkeyEngine][jme] and the jME3 Software Development Kit
    + the [Jolt Physics][jolt] Engine
    + the [Linux Mint][mint] operating system
    + [LWJGL], the Lightweight Java Game Library
    + the [Markdown] document-conversion tool
    + the [Meld] visual merge tool
    + Microsoft Windows
    + the [NetBeans] integrated development environment

I am grateful to [GitHub] and [Sonatype]
for providing free hosting for this project
and many other open-source projects.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know, so I can
correct the situation: sgold@sonic.net

[Jump to the table of contents](#toc)


[adoptium]: https://adoptium.net/temurin/releases/ "Adoptium Project"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[fish]: https://fishshell.com/ "Fish command-line shell"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gitkraken]: https://www.gitkraken.com "GitKraken client"
[gradle]: https://gradle.org "Gradle Project"
[java]: https://en.wikipedia.org/wiki/Java_(programming_language) "Java programming language"
[jme]: https://jmonkeyengine.org "jMonkeyEngine Project"
[jolt]: https://jrouwe.github.io/JoltPhysics/index.html "the Jolt Physics Engine"
[joltjava]: https://github.com/aecsocket/jolt-java?tab=readme-ov-file#readme "JoltJava Project"
[jvm]: https://en.wikipedia.org/wiki/Java_virtual_machine "Java virtual machine"
[license]: https://github.com/stephengold/kk-physics/blob/master/LICENSE "KK Physics license"
[log]: https://github.com/stephengold/kk-physics/blob/master/library/release-log.md "release log"
[lwjgl]: https://www.lwjgl.org "Lightweight Java Game Library"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[maven]: https://maven.apache.org "Maven Project"
[meld]: https://meldmerge.org "Meld merge tool"
[minie]: https://stephengold.github.io/Minie/minie/overview.html "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[project]: https://stephengold.github.io/kk-physics "KK Physics Project"
[sonatype]: https://www.sonatype.com "Sonatype"
