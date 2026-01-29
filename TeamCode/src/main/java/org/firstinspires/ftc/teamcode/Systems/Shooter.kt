package org.firstinspires.ftc.teamcode.Final.Subsystems

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.FlywheelSubsystem
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.TurretState
import org.firstinspires.ftc.teamcode.Systems.ShooterSubsystems.TurretSubsystem
import org.firstinspires.ftc.teamcode.Util.currAlliance
import kotlin.math.atan2

object ShooterSystem: SubsystemGroup(TurretSubsystem, FlywheelSubsystem) {
    internal var currTurretState: TurretState = TurretState.AUTO_AIM
    internal var currFlywheelState: FlywheelState = FlywheelState.AUTO_AIM

    internal fun updateShooter() {
        if (currTurretState == TurretState.AUTO_AIM) { TurretSubsystem.targetAngle = calculateTurretAngle(getCorrectedBotPose(TurretState.AUTO_AIM)) }
        if (currFlywheelState == FlywheelState.AUTO_AIM) { FlywheelSubsystem.flywheelTargetVel = calculateFlywheelVelocity(getCorrectedBotPose(FlywheelState.AUTO_AIM)) }
        TurretSubsystem.updateTurret()
        FlywheelSubsystem.updateFlywheel()
    }
    private fun calculateTurretAngle(botPose: Pose): Double {
        return Math.toDegrees(atan2(botPose.y - currAlliance.goalPose.y, botPose.x - currAlliance.goalPose.x) - botPose.heading)
    }
    private fun calculateFlywheelVelocity(botPose: Pose): Double {
        val d: Double = botPose.distanceFrom(currAlliance.goalPose)
        return 0.0142645 * d * d + 1.26161 * d + 748.88095
    }
    private fun getCorrectedBotPose(state: State): Pose {
        val velocity: Vector = follower.velocity.times(state.linVelScalar)
        return follower.pose + Pose(velocity.xComponent, velocity.yComponent, follower.angularVelocity * state.angVelScalar)
    }
    internal fun moveTurretBy(deg: Double) { if (currTurretState == TurretState.MANUAL) { TurretSubsystem.targetAngle += deg } }
    internal fun increaseFlywheelVelocityBy(vel: Double) { if (currFlywheelState == FlywheelState.MANUAL) { FlywheelSubsystem.flywheelTargetVel += vel } }
}

internal interface State {
    val angVelScalar: Double
    val linVelScalar: Double
}