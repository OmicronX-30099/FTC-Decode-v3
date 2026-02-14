@file:Suppress("PackageName", "unused", "SameParameterValue")

package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Flywheel
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.Turret
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import kotlin.math.atan2

object Shooter: SubsystemGroup(Turret, Flywheel) {
    private val frontRGBLight: ServoEx = ServoEx(ConfigConstants.FRONT_LIGHT)
    private val backRGBLight: ServoEx = ServoEx(ConfigConstants.BACK_LIGHT)
    
    private const val VEL_SCALAR: Double = 0.9
    private const val ANG_SCALAR: Double = 0.0
    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    fun update() {
        updateTurret()
        when (flywheelState) {
            FlywheelState.AUTO_AIM -> {
                updateFlywheel()
                backRGBLight.position = 0.47
                Flywheel.update(true)
            }
            FlywheelState.MANUAL -> {
                // Do NOT call updateFlywheel() here.
                // Flywheel.flywheelTarget was already set by setFlywheelManualVelocity()
                backRGBLight.position = 0.722
                Flywheel.update(false)
            }
            FlywheelState.IDLE -> {
                Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY
                backRGBLight.position = 0.28
                Flywheel.update(false)
            }
            FlywheelState.STOPPED -> { Flywheel.flywheelTarget = 0.0
                backRGBLight.position = 0.28
                Flywheel.stop()
            }
        }

        Turret.update()
        if (Flywheel.isAtTarget()) { frontRGBLight.position = 0.47 }
        else { frontRGBLight.position = 0.28 }
    }
    fun getVelocity(pose: Pose): Double {
        val distance = pose.distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        val flywheelVelocity = if (distance < 120.0) {0.0142645 * distance * distance + 1.26161 * distance + 748.88095}
            else { 0.0142645 * distance * distance + 1.26161 * distance + 748.88095 + 20.0}
        return flywheelVelocity
    }
    fun setFlywheelManualVelocity(velocity: Double) {
        flywheelState = FlywheelState.MANUAL
        Flywheel.flywheelTarget = velocity
    }

    private fun updateFlywheel() {
        val d: Double = (ROBOT.correctedPose(VEL_SCALAR, ANG_SCALAR)).distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        Flywheel.flywheelTarget =
                if (ROBOT.inCloseZone()) {
                    (0.0142645 * d * d + 1.26161 * d + 748.88095)
                } else if (ROBOT.currAlliance == Alliance.BLUE) {
                    (0.0142645 * d * d + 1.26161 * d + 748.88095)+40.0
                } else {
                    (0.0142645 * d * d + 1.26161 * d + 748.88095)+40.0
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
