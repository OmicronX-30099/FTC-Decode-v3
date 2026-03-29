@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.genVector
import kotlin.math.atan2
import kotlin.math.hypot

object Shooter: SubsystemGroup(Turret, Flywheel) {
    private val shooterFrontRGB: ServoEx = ServoEx("front_light",-0.1)
    private val shooterMiddleRGB: ServoEx = ServoEx("back_light", -0.1)
    private const val ITERATIONS: Int = 8
    var flywheelState: FlywheelState = FlywheelState.PREDICTIVE_AUTO_AIM
        private set

    fun update() {
        when (flywheelState) {
            FlywheelState.PREDICTIVE_AUTO_AIM -> { updateFlywheel(true); updateTurret(true); shooterMiddleRGB.position = 0.722 }
            FlywheelState.AUTO_AIM -> { updateFlywheel(); updateTurret(); shooterMiddleRGB.position = 0.611 }
            FlywheelState.MANUAL -> { Flywheel.update(); updateTurret(); shooterMiddleRGB.position = 0.0 }
        }
        shooterFrontRGB.position = if (Flywheel.atTarget()) { 0.47 } else { 0.28 }
    }
    fun flywheelManual() {
        flywheelState = FlywheelState.MANUAL
        Flywheel.targetVelocity = Flywheel.IDLE_VELOCITY
    }
    fun increaseFlywheelVel(to: Double) { if (flywheelState == FlywheelState.MANUAL) Flywheel.targetVelocity = to }
    fun enablePredictive() { flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM }
    fun enableAutoAim() { flywheelState = FlywheelState.AUTO_AIM }

    fun reset() { Flywheel.reset(); Turret.reset(); flywheelState == FlywheelState.AUTO_AIM }
    fun debug(): String = "Turret Data: \n${Turret.debug()} \nFlywheel Data: \n${Flywheel.debug()}"

    private fun updateTurret(predictive: Boolean = false) {
        val vec =
            if (predictive) {
                getCorrectedVecIterative(
                    ROBOT.shooterPose(),
                    ROBOT.currAlliance.turretGoalPose,
                    follower.velocity
                )
            } else {
                ROBOT.shooterPose().genVector(ROBOT.currAlliance.turretGoalPose)
            }
        Turret.targetAngle = calculateTurretAngle(vec)
        Turret.update()
    }
    private fun updateFlywheel(predictive: Boolean = false) {
        val d =
            if (predictive) {
                getCorrectedVecIterative(ROBOT.shooterPose(), ROBOT.currAlliance.flywheelGoalPose, follower.velocity).magnitude
            } else {
                ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.flywheelGoalPose)
            }
        Flywheel.targetVelocity = calculateFlywheelVelocity(d)
        Flywheel.update()
    }
    private fun calculateFlywheelVelocity(d: Double) = ((0.01632 * d * d) + (3.49146 * d) + 985.48178)
    //y=0.01632x^{2}+3.49146x+985.48178
    private fun calculateTurretAngle(vec: Vector) = Math.toDegrees(atan2(vec.yComponent, vec.xComponent) - follower.pose.heading)

    private fun getFlyTime(d: Double): Double {
        val t = (-0.0000251859 * d * d) + (0.00940245 * d) - 0.111539
        return t//max(0.05, min(t, 0.7))
    }
    private fun getCorrectedVecIterative(botPose: Pose, targetPose: Pose, velocity: Vector): Vector {
        val r = Vector(
            hypot(targetPose.x-botPose.x, targetPose.y-botPose.y),
            atan2( botPose.y-targetPose.y,botPose.x-targetPose.x)
        )
        var c = r
        repeat(ITERATIONS) {
            val t = getFlyTime(c.magnitude)
            c = r.plus(velocity.times(t))
        }
        return c
    }
}