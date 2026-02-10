package org.firstinspires.ftc.teamcode.Util

import com.pedropathing.geometry.Pose
import dev.nextftc.extensions.pedro.PedroComponent

data object ROBOT {
    private const val FIELD_WIDTH: Double = 141.5
    private const val FIELD_LENGTH: Double = 141.5
    private const val TURRET_Y_OFFSET: Double = -1.774
    private const val FAR_ZONE_BUFFER: Double = 2.0
    private const val CLOSE_ZONE_BUFFER: Double = 2.0

    internal var currAlliance: Alliance = Alliance.BLUE
    internal var currStage: Stage = Stage.TELEOP

    internal fun shooterPose(): Pose {
        val a: Double = PedroComponent.Companion.follower.heading
        return PedroComponent.Companion.follower.pose + Pose(
            TURRET_Y_OFFSET * Math.cos(a),
            TURRET_Y_OFFSET * Math.sin(a)
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
        override val goalPoses: GoalPoses = GoalPoses(Pose(131.75,126.0),Pose(127.0,130.75),Pose(130.0,133.5),Pose(141.5,141.5))
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