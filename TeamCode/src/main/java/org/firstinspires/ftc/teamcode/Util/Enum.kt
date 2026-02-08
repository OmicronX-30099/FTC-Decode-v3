package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent

data object ROBOT {
    private val fieldWidth: Double = 141.5
    private val fieldLength: Double = 141.5
    private val turretYOffset: Double = -1.7748
    private val farZoneBuffer: Double = 2.0
    private val closeZoneBuffer: Double = 2.0

    internal var currAlliance: Alliance = Alliance.BLUE
    internal var currStage: Stage = Stage.TELEOP

    internal fun shooterPose(): Pose {
        val a: Double = PedroComponent.Companion.follower.heading
        return PedroComponent.Companion.follower.pose + Pose(
            turretYOffset * Math.cos(a),
            turretYOffset * Math.sin(a)
        )
    }

    internal fun getDistanceFromGoal(): Double = shooterPose().distanceFrom(currAlliance.goalPoses.flywheelGoalPose)
    internal fun inBlueFarZone(): Boolean {
        val p: Pose = shooterPose()
        if ((p.y >= 0.0) && (p.x < (fieldWidth / 2.0)) && (p.y < (p.x - fieldLength / 3.0 + farZoneBuffer))) {
            return true
        } else {
            return false
        }
    }
    internal fun inRedFarZone(): Boolean {
        val p: Pose = shooterPose()
        if ((p.y >= 0.0) && (p.x >= (fieldWidth / 2.0)) && (p.y < (-p.x + (fieldLength * (2 / 3.0)) + farZoneBuffer))) {
            return true
        }
        else {
            return false
        }
    }
    internal fun inCloseZone(): Boolean {
        val p: Pose = shooterPose()
        if ((p.y <= fieldLength) && (p.y > (-p.x + fieldLength - closeZoneBuffer)) && (p.y > (p.x - closeZoneBuffer))) {
            return true
        } else {
            return false
        }
    }
}

enum class Alliance {
    BLUE {
        override val resetPoses: ResetPoses
            get() = TODO("Not yet implemented")
        override val goalPoses: GoalPoses
            get() = TODO("Not yet implemented")
    },
    RED{
        override val resetPoses: ResetPoses
            get() = TODO("Not yet implemented")
        override val goalPoses: GoalPoses
            get() = TODO("Not yet implemented")
    };
    abstract val resetPoses: ResetPoses
    abstract val goalPoses: GoalPoses
}

enum class Stage {
    TELEOP,
    AUTONOMOUS
}

data class GoalPoses(val turretGoalPoseBlueFar: Pose, val turretGoalPoseRedFar: Pose, val turretGoalPoseClose: Pose, val flywheelGoalPose: Pose)

data class ResetPoses(val resetPose1: Pose, val resetPose2: Pose, val resetPose3: Pose, val resetPose4: Pose, val resetPose5: Pose)
