@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.genVector
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

object Shooter: SubsystemGroup(Turret, Flywheel, Hood, ShooterLights) {
    private const val ITERATIONS: Int = 8
    
    var flywheelState: FlywheelState = FlywheelState.PREDICTIVE_AUTO_AIM
        private set
    
    var shooterMethod: ShooterMethod = ShooterMethod.REGRESSION

    override fun initialize() {
        PhysicsShooter.precomputeField()
    }

    fun update() {
        when (flywheelState) {
            FlywheelState.PREDICTIVE_AUTO_AIM -> { updateFlywheel(true); updateTurret(true); updateHood(true) }
            FlywheelState.AUTO_AIM -> { updateFlywheel(); updateTurret(); updateHood() }
            FlywheelState.MANUAL -> { Flywheel.update(); updateTurret(); updateHood() }
        }
        
        val error = abs(Flywheel.targetVelocity - Flywheel.currentVelocity)
        ShooterLights.shooterRGB.position = when {
            error <= 40.0 -> 0.5
            error <= 100.0 -> 0.388
            else -> 0.277
        }
    }
    fun flywheelManual() {
        flywheelState = FlywheelState.MANUAL
        Flywheel.targetVelocity = Flywheel.IDLE_VELOCITY
    }
    fun enablePredictive() { flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM }
    fun enableAutoAim() { flywheelState = FlywheelState.AUTO_AIM }

    fun reset() { Flywheel.reset(); Turret.reset(); Hood.reset(); flywheelState = FlywheelState.AUTO_AIM }
    fun debug(): String = "Turret Data: \n${Turret.debug()} \nFlywheel Data: \n${Flywheel.debug()} \nHood Data: \n${Hood.debug()}"

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
        
        Flywheel.targetVelocity = if (shooterMethod == ShooterMethod.PHYSICS) {
            PhysicsShooter.getTargetParams(d).second
        } else {
            calculateFlywheelVelocity(d)
        }
        Flywheel.update()
    }
    private fun updateHood(predictive: Boolean = false) {
        val d =
            if (predictive) {
                getCorrectedVecIterative(ROBOT.shooterPose(), ROBOT.currAlliance.flywheelGoalPose, follower.velocity).magnitude
            } else {
                ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.flywheelGoalPose)
            }
        
        val velocityError = Flywheel.targetVelocity - Flywheel.currentVelocity
        
        if (shooterMethod == ShooterMethod.PHYSICS) {
            val targetAngle = PhysicsShooter.getTargetParams(d).first
            Hood.updatePhysics(targetAngle, velocityError)
        } else {
            Hood.updateRegression(d, velocityError)
        }
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

enum class ShooterMethod {
    REGRESSION,
    PHYSICS
}