// global build settings shared by all KK Physics subprojects

rootProject.name = "kk-physics"

dependencyResolutionManagement {
    repositories {
        //mavenLocal() // to find libraries installed locally
        mavenCentral() // to find libraries released to the Maven Central repository
    }
}

// subprojects:
include("apps")
include("library")
