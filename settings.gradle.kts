// global build settings shared by all KK Physics subprojects

rootProject.name = "kk-physics"

dependencyResolutionManagement {
    repositories {
        //mavenLocal() // to find libraries installed locally
        mavenCentral() // to find libraries released to the Maven Central repository
        //maven { url = uri("https://s01.oss.sonatype.org/content/groups/staging") } // to find libraries staged but not yet released
    }
}

// subprojects:
include("apps")
include("library")
