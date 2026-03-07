@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

data object ROBOT {
    private const val TURRET_Y_OFFSET: Double = -1.774

    var currAlliance: Alliance = Alliance.BLUE
    var currStage: Stage = Stage.TELEOP
    var teleopStartPose: Pose = Pose(86.2,7.5,Math.toRadians(90.0))

    fun shooterPose(): Pose {
        val a: Double = follower.heading
        return follower.pose + Pose(
            TURRET_Y_OFFSET * cos(a),
            TURRET_Y_OFFSET * sin(a)
        )
    }
}

enum class Alliance {
    BLUE {
        override val resetPoses: ResetPoses  = ResetPoses(Pose(9.5,8.9,-PI).mirror(), Pose(117.0,129.0,-2.448).mirror(), Pose(116.0,130.0,-0.939).mirror())
        override val turretGoalPose: Pose = Pose(3.62, 137.88)
        override val flywheelGoalPose: Pose = Pose(141.5,141.5).mirror()
    },
    RED{
        override val resetPoses: ResetPoses  = ResetPoses(Pose(9.5,8.9,-PI), Pose(117.0,129.0,-2.448), Pose(116.0,130.0,-0.939))
        override val turretGoalPose: Pose = Pose(3.62,137.8).mirror()
        override val flywheelGoalPose: Pose = Pose(141.5,141.5)
    };
    abstract val resetPoses: ResetPoses
    abstract val turretGoalPose: Pose
    abstract val flywheelGoalPose: Pose
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
}

data class ResetPoses(val resetPose1: Pose, val resetPose2: Pose, val resetPose3: Pose)