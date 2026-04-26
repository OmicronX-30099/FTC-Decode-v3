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
    
    var flywheelState: FlywheelState = FlywheelState.AUTO_AIM

    var shooterMethod: ShooterMethod = ShooterMethod.REGRESSION

    private var lastTurretUpdateTime = 0L

    override fun initialize() {
        //PhysicsShooter.precomputeField()
    }

    fun update() {
        if (follower.pose.x == 0.0 && follower.pose.y == 0.0) return
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
        
        val newTarget = calculateTurretAngle(vec)
        val normalized = normalizeAngle(newTarget + Turret.offset)

        if (abs(normalized) > 167.0) {
            if (System.currentTimeMillis() - lastTurretUpdateTime >= 200) {
                Turret.targetAngle = newTarget
                lastTurretUpdateTime = System.currentTimeMillis()
            }
        } else {
            Turret.targetAngle = newTarget
        }
        Turret.update()
    }
    private fun updateFlywheel(predictive: Boolean = false) {
        val d =
            if (predictive) {
                getCorrectedVecIterative(ROBOT.shooterPose(), ROBOT.currAlliance.flywheelGoalPose, follower.velocity).magnitude
            } else {
                ROBOT.shooterPose().distanceFrom(ROBOT.currAlliance.flywheelGoalPose)
            }
        
        var targetVel = if (shooterMethod == ShooterMethod.PHYSICS) {
            PhysicsShooter.getTargetParams(d).second
        } else {
            Flywheel.calculateVelocity(d)
        }

        // Farzone velocity lock: y < 32 inches -> minimum velocity 1660
        if (follower.pose.y < 35.0) {
            targetVel = targetVel.coerceAtLeast(1660.0)
        }

        Flywheel.targetVelocity = targetVel
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
    //y=0.0118933x^{2}+4.02429x+937.20105 old
    //y=0.0101171x^2+4.20298x+945.28294
    private fun calculateTurretAngle(vec: Vector): Double {
        val angleToGoal = Math.toDegrees(atan2(vec.yComponent, vec.xComponent))
        val headingDeg = Math.toDegrees(follower.pose.heading)
        return angleToGoal - headingDeg
    }

    private fun getFlyTime(d: Double): Double {
        return if (shooterMethod == ShooterMethod.PHYSICS) {
            PhysicsShooter.getFlyTime(d)
        } else {
            val tps = Flywheel.calculateVelocity(d)
            val angle = Hood.getAngle(d)
            PhysicsShooter.getFlyTime(d, angle, tps)
        }
    }
    private fun getCorrectedVecIterative(botPose: Pose, targetPose: Pose, velocity: Vector): Vector {
        val r = Vector(
            hypot(targetPose.x-botPose.x, targetPose.y-botPose.y),
            atan2(targetPose.y-botPose.y,targetPose.x-botPose.x)
        )
        var c = r
        repeat(ITERATIONS) {
            val t = getFlyTime(c.magnitude)
            c = r.minus(velocity.times(t))
        }
        return c
    }
}

enum class ShooterMethod {
    REGRESSION,
    PHYSICS
}