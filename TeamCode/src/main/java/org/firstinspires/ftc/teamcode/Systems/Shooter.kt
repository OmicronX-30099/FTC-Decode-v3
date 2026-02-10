package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Turret
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.TurretState
import org.firstinspires.ftc.teamcode.Util.ROBOT

object Shooter: SubsystemGroup(Turret, Flywheel) {
    internal var turretState: TurretState = TurretState.AUTO_AIM
    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    fun update() {
        when (turretState) {
            TurretState.AUTO_AIM -> { updateTurret() }
            TurretState.MANUAL -> {  }
        }
        when (flywheelState) {
            FlywheelState.AUTO_AIM -> { updateFlywheel() }
            FlywheelState.IDLE -> { Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY }
        }
        Turret.update()
        Flywheel.update()
    }

    private fun updateFlywheel() {
        val d: Double = ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        Flywheel.flywheelTarget = if (ROBOT.currStage.useFlywheelVel) {
                0.0142645 * d * d + 1.26161 * d + 748.88095
            } else {
                0.97 * (0.0142645 * d * d + 1.26161 * d + 748.88095)
            }
    }
    private fun updateTurret() {
        Turret.targetTurretAngle =
            if (ROBOT.inBlueFarZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.turretGoalPoseBlueFar)
            } else if (ROBOT.inRedFarZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.turretGoalPoseRedFar)
            } else if (ROBOT.inCloseZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.turretGoalPoseClose)
            } else {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
            }
    }
    private fun calculateTurretAngle(botPose: Pose, targetPose: Pose): Double =
        Math.toDegrees(Math.atan2(botPose.y - targetPose.y, botPose.x - targetPose.x) - botPose.heading)
    internal fun moveTurretBy(deg: Double) {
        turretState = TurretState.MANUAL
        Turret.targetTurretAngle += deg
    }

    // region SOTM_CODE
    private fun getCorrectedVec(botPose: Pose, targetPose: Pose): Vector {
        val r: Vector = Vector(targetPose.x - botPose.x, targetPose.y - botPose.y)
        val d: Double = r.magnitude
        val T: Double = -4.64e-5 * d * d + 1.51e-2 * d - 3.48e-1
        val v: Vector = follower.velocity
        return (r.minus(v.times(T)))
    }
    private fun calculateTurretAngle(botPose: Pose, targetPose: Pose, useVel: Boolean): Double {
        val finalVec: Vector = getCorrectedVec(botPose, targetPose)
        return Math.toDegrees(Math.atan2(finalVec.yComponent, finalVec.xComponent) - botPose.heading)
    }
    private fun updateFlywheel(useVel: Boolean) {
        val finalVec: Vector = getCorrectedVec(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        val d: Double = finalVec.magnitude
        Flywheel.flywheelTarget =  0.0142645 * d * d + 1.26161 * d + 748.88095
    }
    // endregion
}
