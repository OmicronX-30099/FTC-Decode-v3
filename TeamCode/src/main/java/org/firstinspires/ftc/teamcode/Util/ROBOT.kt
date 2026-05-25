@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

data object ROBOT {
    private const val TURRET_X_OFFSET: Double = -0.96
    private const val TURRET_Y_OFFSET: Double = 0.0

    var currAlliance: Alliance = Alliance.BLUE
    var currStage: Stage = Stage.TELEOP
    var teleopStartPose: Pose = Pose(112.000, 130.750,Math.toRadians(90.0))
        set(value) {
            field = value.copy()
        }

    fun shooterPose(): Pose {
        if (follower.pose.x == 0.0 && follower.pose.y == 0.0) return follower.pose
        val h: Double = follower.pose.heading
        return follower.pose + Pose(
            TURRET_X_OFFSET * cos(h) - TURRET_Y_OFFSET * sin(h),
            TURRET_X_OFFSET * sin(h) + TURRET_Y_OFFSET * cos(h)
        )
    }
}

enum class Alliance {
    BLUE {
        override val resetPoses: ResetPoses  = ResetPoses(Pose(124.9,76.0,Math.toRadians(0.0)).mirror(), Pose(117.0,129.0,-2.448).mirror(), Pose(116.0,130.0,-0.939).mirror())
        override val turretGoalPose: Pose = Pose(3.62, 137.88)
        //override val turretGoalPose: Pose = Pose(0.0,137.5)
        override val flywheelGoalPose: Pose = Pose(141.5,141.5).mirror()
    },
    RED{
        override val resetPoses: ResetPoses  = ResetPoses(Pose(124.9,76.0,Math.toRadians(0.0)), Pose(117.0,129.0,-2.448), Pose(116.0,130.0,-0.939))
        override val turretGoalPose: Pose = Pose(3.62,137.88).mirror()
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