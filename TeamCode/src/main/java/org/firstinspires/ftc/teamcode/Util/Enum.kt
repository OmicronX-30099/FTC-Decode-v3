package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

data object ROBOT {
    private const val TURRET_Y_OFFSET: Double = -1.774

    var currAlliance: Alliance        = Alliance.BLUE
    var currStage: Stage              = Stage.TELEOP

    fun turretPose(): Pose = follower.pose.offset(TURRET_Y_OFFSET)
}

enum class Alliance {
    RED {
        override val goalPoses: GoalPoses      = GoalPoses(Pose(),Pose())
        override val resetPoses: ResetPoses    = ResetPoses(Pose())
    },
    BLUE {
        override val goalPoses: GoalPoses      = GoalPoses(Pose(),Pose())
        override val resetPoses: ResetPoses    = ResetPoses(Pose())
    };
    abstract val goalPoses: GoalPoses
    abstract val resetPoses: ResetPoses
}

data class GoalPoses(val turretGoalPose: Pose, val flywheelGoalPose: Pose)
data class ResetPoses(val resetPose1: Pose)

enum class Stage {
    TELEOP,
    AUTONOMOUS;
    open var currMotif: Motif = Motif.UNKNOWN
}
enum class Motif {
    PPG             { override val tagID: Int = 23 },
    PGP             { override val tagID: Int = 22 },
    GPP             { override val tagID: Int = 21 },
    UNKNOWN         { override val tagID: Int = 1  };
    abstract val tagID: Int
}