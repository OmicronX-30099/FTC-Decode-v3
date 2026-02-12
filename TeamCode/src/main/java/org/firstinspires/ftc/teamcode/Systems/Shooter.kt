@file:Suppress("PackageName", "unused", "SameParameterValue")

package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Turret
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import kotlin.math.atan2

object Shooter: SubsystemGroup(Turret, Flywheel) {
    private const val VEL_SCALAR: Double = 0.9
    private const val ANG_SCALAR: Double = 0.0
    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    fun update() {
        updateTurret()
        when (flywheelState) {
            FlywheelState.AUTO_AIM -> { updateFlywheel() }
            FlywheelState.IDLE -> { Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY }
            FlywheelState.STOPPED -> { Flywheel.flywheelTarget = 0.0 }
        }
        Turret.update()
        Flywheel.update()
    }

    private fun updateFlywheel() {
        val d: Double = (ROBOT.correctedPose(VEL_SCALAR, ANG_SCALAR)).distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        Flywheel.flywheelTarget =
                if (ROBOT.inCloseZone()) {
                    (0.0142645 * d * d + 1.26161 * d + 748.88095)
                } else {
                    (0.0142645 * d * d + 1.26161 * d + 748.88095)
                }
    }
    private fun updateTurret() {
        Turret.targetTurretAngle =
            if (ROBOT.inCloseZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.turretGoalPoseClose)
            } else if (ROBOT.inFarZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.turretGoalPoseFar)
            }
            else {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
            }
    }
    private fun calculateTurretAngle(botPose: Pose, targetPose: Pose): Double =
        Math.toDegrees(atan2(botPose.y - targetPose.y, botPose.x - targetPose.x) - botPose.heading)
}
