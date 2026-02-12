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
    internal var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    internal var turretOffset: Double = 0.0

    fun update() {
        updateTurret()
        when (flywheelState) {
            FlywheelState.AUTO_AIM -> { updateFlywheel() }
            FlywheelState.IDLE -> { Flywheel.flywheelTarget = Flywheel.IDLE_VELOCITY }
        }
        Turret.update()
        Flywheel.update()
    }

    private fun updateFlywheel() {
        val d: Double = ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.goalPoses.flywheelGoalPose)
        Flywheel.flywheelTarget =
                if (ROBOT.inCloseZone()) {
                    if (ROBOT.currStage == Stage.AUTONOMOUS) {
                        (0.0142645 * d * d + 1.26161 * d + 748.88095) - 20.0
                    }
                    (0.0142645 * d * d + 1.26161 * d + 748.88095)
                } else {
                    (0.0142645 * d * d + 1.26161 * d + 748.88095) + 40.0
                }
    }
    private fun updateTurret() {
        Turret.targetTurretAngle =
            if (ROBOT.inBlueFarZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
            } else if (ROBOT.inRedFarZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
            } else if (ROBOT.inCloseZone()) {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
            } else {
                calculateTurretAngle(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose)
            } + turretOffset
    }
    private fun calculateTurretAngle(botPose: Pose, targetPose: Pose): Double =
        Math.toDegrees(atan2(botPose.y - targetPose.y, botPose.x - targetPose.x) - botPose.heading)

    internal fun offSetTurretBy(deg: Double) { turretOffset += deg }
}
