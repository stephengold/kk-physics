The [KK Physics Project][project] is about integrating
[the Jolt Physics engine][jolt] into
[the jMonkeyEngine (JME) game engine][jme].

This project is currently a proof of concept
and is not intended for production use.

It contains 2 subprojects:

 1. library: the KK Physics [JVM] runtime library and its automated tests
 2. apps: non-automated test software

Complete source code (in Java) is provided under
[a 3-clause BSD license][license].


<a name="toc"></a>

## Contents of this document

+ [How to add KK Physics to an existing project](#add)
+ [How to build KK Physics from source](#build)
+ [External links](#links)
+ [Acknowledgments](#acks)


<a name="add"></a>

## How to add KK Physics to an existing project

KK Physics comes pre-built as a single JVM library.

For projects built using [Maven] or [Gradle], it is sufficient to add
dependencies on the KK Physics library and appropriate jolt-jni native libraries.
The build tool should automatically resolve the remaining dependencies.

In the following examples:
+ "Linux64" may be replaced by "MacOSX64", "MacOSX_ARM64", or "Windows64".
+ "DebugSp" may be replaced by "DebugDp", "ReleaseSp", or "ReleaseDp". 

### Gradle-built projects

Add to the project’s "build.gradle" file:

    repositories {
        mavenCentral()
    }
    dependencies {
        implementation("com.github.stephengold:kk-physics:0.2.0")
        runtimeOnly("com.github.stephengold:jolt-jni-Linux64:0.1.10:DebugSp")
    }

For some older versions of Gradle,
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
      <version>0.2.0</version>
    </dependency>
    <dependency>
      <groupId>com.github.stephengold</groupId>
      <artifactId>jolt-jni-Linux64</artifactId>
      <version>0.1.10</version>
      <classifier>DebugSp</classifier>
    </dependency>


<a name="build"></a>

## How to build KK Physics from source

1. Install a [Java Development Kit (JDK)][adoptium],
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
    + `git checkout -b latest 0.2.0`
4. Run the [Gradle] wrapper:
  + using Bash or Fish or PowerShell or Zsh: `./gradlew build`
  + using Windows Command Prompt: `.\gradlew build`

After a successful build,
Maven artifacts will be found in "library/build/libs".

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

The API and much of the source code is derived from [Minie].

[Jump to the table of contents](#toc)


<a name="acks"></a>

## Acknowledgments

Like most projects, the KK Physics Project builds on the work of many who
have gone before.  I therefore acknowledge the following
software developers:

+ "aecsocket" for creating the [jolt-java] bindings to the Jolt Physics Engine
+ Normen Hansen (aka "normen") for creating most of the `jme3-bullet` library
  (from which the KK Physics API is derived)
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
[jolt-java]: https://github.com/aecsocket/jolt-java?tab=readme-ov-file#readme "Jolt-java Project"
[jvm]: https://en.wikipedia.org/wiki/Java_virtual_machine "Java virtual machine"
[license]: https://github.com/stephengold/kk-physics/blob/master/LICENSE "KK Physics license"
[lwjgl]: https://www.lwjgl.org "Lightweight Java Game Library"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[maven]: https://maven.apache.org "Maven Project"
[meld]: https://meldmerge.org "Meld merge tool"
[minie]: https://stephengold.github.io/Minie/minie/overview.html "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[project]: https://stephengold.github.io/kk-physics "KK Physics Project"
[sonatype]: https://www.sonatype.com "Sonatype"
