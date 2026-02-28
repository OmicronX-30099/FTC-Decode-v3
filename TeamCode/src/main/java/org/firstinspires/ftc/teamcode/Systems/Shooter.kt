package org.firstinspires.ftc.teamcode.Systems

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min

object Shooter: SubsystemGroup(Turret, Flywheel) {
    private const val ITERATIONS: Int = 10
    var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    private fun getFlyTime(d: Double): Double {
        val t = (-4.64e-5 * d * d) + (1.51e-2 * d) - 3.48e-1
        return max(0.05, min(t, 0.7))
    }
    private fun getCorrectedVecIterative(botPose: Pose, targetPose: Pose, velocity: Vector): Vector {
        val r = Vector(Pose(targetPose.x-botPose.x, targetPose.y-botPose.y))
        var c = r
        repeat(ITERATIONS) {
            val t = getFlyTime(c)
            c = r.plus(velocity.times(t))
        }
        return c
    }

    private fun updateTurret() {
        val targetPose: Pose = Pose(134.0,134.0)
        val futureVec = getCorrectedVecIterative(follower.pose, targetPose, follower.velocity)
        Turret.targetAngle = calculateTurretAngle(futureVec)
    }
    private fun updateFlywheel() {
        val targetPose: Pose = Pose(134.0,134.0)
        val futureVec = getCorrectedVecIterative(follower.pose, targetPose, follower.velocity)
        val d = futureVec.magnitude
        Flywheel.targetVelocity = calculateFlywheelVelocity(futureVec.magnitude)
    }
    private fun calculateFlywheelVelocity(d: Double) = ((0.019454 * d * d) + (2.007 * d) + 1102.62509)
    private fun calculateTurretAngle(vec: Vector) = Math.toDegrees(atan2(vec.yComponent, vec.xComponent) - follower.pose.heading)
}