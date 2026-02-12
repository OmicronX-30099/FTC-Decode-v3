@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent
import kotlin.math.cos
import kotlin.math.sin

data object ROBOT {
    private const val FIELD_WIDTH: Double = 141.5
    private const val FIELD_LENGTH: Double = 141.5
    private const val TURRET_Y_OFFSET: Double = -1.774
    private const val FAR_ZONE_BUFFER: Double = 12.0
    private const val CLOSE_ZONE_BUFFER: Double = 11.0

    internal var currAlliance: Alliance = Alliance.BLUE
    internal var currStage: Stage = Stage.TELEOP

    internal fun shooterPose(): Pose {
        val a: Double = PedroComponent.Companion.follower.heading
        return PedroComponent.Companion.follower.pose + Pose(
            TURRET_Y_OFFSET * cos(a),
            TURRET_Y_OFFSET * sin(a)
        )
    }

    internal fun getDistanceFromGoal(): Double = shooterPose().distanceFrom(currAlliance.goalPoses.flywheelGoalPose)
    internal fun inBlueFarZone(): Boolean {
        val p: Pose = shooterPose()
        return ((p.y >= 0.0) && (p.x < (FIELD_WIDTH / 2.0)) && (p.y < (p.x - FIELD_LENGTH / 3.0 + FAR_ZONE_BUFFER)))
    }
    internal fun inRedFarZone(): Boolean {
        val p: Pose = shooterPose()
        return ((p.y >= 0.0) && (p.x >= (FIELD_WIDTH / 2.0)) && (p.y < (-p.x + (FIELD_LENGTH * (2 / 3.0)) + FAR_ZONE_BUFFER)))
    }
    internal fun inCloseZone(): Boolean {
        val p: Pose = shooterPose()
        return ((p.y <= FIELD_LENGTH) && (p.y > (-p.x + FIELD_LENGTH - CLOSE_ZONE_BUFFER)) && (p.y > (p.x - CLOSE_ZONE_BUFFER)))
    }
}

enum class Alliance {
    BLUE {
        override val resetPoses: ResetPoses
            get() = TODO("Not yet implemented")
        override val goalPoses: GoalPoses = GoalPoses(Pose(9.75,126.0),Pose(14.5,130.75),Pose(11.5,133.5),Pose(141.5,141.5))
    },
    RED{
        override val resetPoses: ResetPoses
            get() = TODO("Not yet implemented")
        override val goalPoses: GoalPoses = GoalPoses(Pose(130.0,140.0),Pose(130.0,140.0),Pose(134.0,140.0),Pose(131.0,140.0))
    };
    abstract val resetPoses: ResetPoses
    abstract val goalPoses: GoalPoses
}

enum class Motif {
    UNKNOWN { override val tagID: Int = 0 },
    PPG { override val tagID: Int = 23 },
    PGP { override val tagID: Int = 22 },
    GPP { override val tagID: Int = 21 };
    abstract val tagID: Int
}

enum class Stage {
    TELEOP,
    AUTONOMOUS;
    open var currMotif: Motif = Motif.UNKNOWN
    open var useFlywheelVel: Boolean = false
}

data class GoalPoses(val turretGoalPoseBlueFar: Pose, val turretGoalPoseRedFar: Pose, val turretGoalPoseClose: Pose, val flywheelGoalPose: Pose)

data class ResetPoses(val resetPose1: Pose, val resetPose2: Pose, val resetPose3: Pose, val resetPose4: Pose, val resetPose5: Pose)
