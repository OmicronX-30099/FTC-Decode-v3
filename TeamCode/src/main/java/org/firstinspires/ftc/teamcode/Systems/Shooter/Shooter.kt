@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Util.ROBOT
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min

object Shooter: SubsystemGroup(Turret, Flywheel) {
    private const val ITERATIONS: Int = 10
    var flywheelState: FlywheelState = FlywheelState.AUTO_AIM
        private set

    fun update() {
        when (flywheelState) {
            FlywheelState.AUTO_AIM -> { updateFlywheel() }
            FlywheelState.MANUAL -> { Flywheel.update() }
        }
        updateTurret()
    }
    fun flywheelManual() {
        flywheelState = FlywheelState.MANUAL
        Flywheel.targetVelocity = Flywheel.IDLE_VELOCITY
    }
    fun flywheelAutoAim() { flywheelState = FlywheelState.AUTO_AIM }

    fun reset() { Flywheel.reset(); Turret.reset(); flywheelState == FlywheelState.AUTO_AIM }
    fun debug(): String = "Turret Data: \n${Turret.debug()} \nFlywheel Data: \n${Flywheel.debug()}"

    private fun updateTurret() {
        val futureVec = getCorrectedVecIterative(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.turretGoalPose, follower.velocity)
        Turret.targetAngle = calculateTurretAngle(futureVec)
        Turret.update()
    }
    private fun updateFlywheel() {
        val futureVec = getCorrectedVecIterative(ROBOT.shooterPose(), ROBOT.currAlliance.goalPoses.flywheelGoalPose, follower.velocity)
        Flywheel.targetVelocity = calculateFlywheelVelocity(futureVec.magnitude)
        Flywheel.update()
    }
    private fun calculateFlywheelVelocity(d: Double) = ((0.019454 * d * d) + (2.007 * d) + 1102.62509)
    private fun calculateTurretAngle(vec: Vector) = Math.toDegrees(atan2(vec.yComponent, vec.xComponent) - follower.pose.heading)

    private fun getFlyTime(d: Double): Double {
        val t = (-4.64e-5 * d * d) + (1.51e-2 * d) - 3.48e-1
        return max(0.05, min(t, 0.7))
    }
    private fun getCorrectedVecIterative(botPose: Pose, targetPose: Pose, velocity: Vector): Vector {
        val r = Vector(Pose(targetPose.x-botPose.x, targetPose.y-botPose.y))
        var c = r
        repeat(ITERATIONS) {
            val t = getFlyTime(c.magnitude)
            c = r.plus(velocity.times(t))
        }
        return c
    }
}