@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.genVector
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

@Configurable
object Shooter: SubsystemGroup(Turret, Flywheel, Hood, ShooterLights) {
    private const val ITERATIONS: Int = 8

    @JvmField var RESPONSE_LATENCY_SECONDS: Double = 0.035
    @JvmField var ACCELERATION_GAIN: Double = 0.4
    @JvmField var ACCELERATION_FILTER_ALPHA: Double = 0.1
    @JvmField var TURRET_ANGULAR_VELOCITY_COMPENSATION_SECONDS: Double = 0.1
    @JvmField var TRANSLATION_STILL_VELOCITY: Double = 15.0
    @JvmField var STILL_PREDICTION_SCALE: Double = 0.5
    
    var flywheelState: FlywheelState = FlywheelState.PREDICTIVE_AUTO_AIM

    var shooterMethod: ShooterMethod = ShooterMethod.REGRESSION

    private var lastTurretUpdateTime = 0L
    private var filteredAcceleration: Vector = Vector()

    override fun initialize() {
        //PhysicsShooter.precomputeField()
        reset()
    }

    fun update() {
        val robotPose = follower.pose
        if (robotPose.x == 0.0 && robotPose.y == 0.0) return

        val velocity = follower.velocity
        val currentFlywheelVelocity = Flywheel.refreshVelocity()
        updateFilteredAcceleration(follower.acceleration)

        val shooterPose = ROBOT.shooterPose()
        val flywheelVec = if (flywheelState == FlywheelState.PREDICTIVE_AUTO_AIM) {
            getCorrectedVecIterative(
                shooterPose,
                ROBOT.currAlliance.flywheelGoalPose,
                velocity,
                filteredAcceleration
            )
        } else {
            shooterPose.genVector(ROBOT.currAlliance.flywheelGoalPose)
        }
        val turretVec = if (flywheelState == FlywheelState.PREDICTIVE_AUTO_AIM) {
            getCorrectedVecIterative(
                shooterPose,
                ROBOT.currAlliance.turretGoalPose,
                velocity,
                filteredAcceleration
            )
        } else {
            shooterPose.genVector(ROBOT.currAlliance.turretGoalPose)
        }

        when (flywheelState) {
            FlywheelState.PREDICTIVE_AUTO_AIM,
            FlywheelState.AUTO_AIM -> {
                updateFlywheel(flywheelVec.magnitude)
                updateTurret(turretVec)
                updateHood(flywheelVec.magnitude, currentFlywheelVelocity)
            }
            FlywheelState.MANUAL -> {
                Flywheel.update(shouldRefreshVelocity = false)
                updateTurret(turretVec)
                updateHood(flywheelVec.magnitude, currentFlywheelVelocity)
            }
        }
        
        val error = abs(Flywheel.targetVelocity - currentFlywheelVelocity)
        ShooterLights.setPosition(when {
            error <= 40.0 -> 0.5
            error <= 100.0 -> 0.388
            else -> 0.277
        })
    }
    fun flywheelManual() {
        flywheelState = FlywheelState.MANUAL
        Flywheel.targetVelocity = Flywheel.IDLE_VELOCITY
    }
    fun enablePredictive() { flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM }
    fun enableAutoAim() { flywheelState = FlywheelState.AUTO_AIM }

    fun reset() { Flywheel.reset(); Turret.reset(); Hood.reset(); ShooterLights.resetCache(); filteredAcceleration = Vector(); flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM }
    fun debug(): String = "Turret Data: \n${Turret.debug()} \nFlywheel Data: \n${Flywheel.debug()} \nHood Data: \n${Hood.debug()}"

    private fun updateTurret(vec: Vector) {
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
    private fun updateFlywheel(d: Double) {
        var targetVel = if (shooterMethod == ShooterMethod.PHYSICS) {
            PhysicsShooter.getTargetParams(d).second
        } else {
            Flywheel.calculateVelocity(d)
        }

        // Farzone velocity lock: y < 32 inches -> minimum velocity 1660
        if (follower.pose.y < 35.0) {
            targetVel = targetVel.coerceAtLeast(1660.0)
        }

        targetVel += Flywheel.velocityOffset
        Flywheel.targetVelocity = targetVel
        Flywheel.update(shouldRefreshVelocity = false)
    }
    private fun updateHood(d: Double, currentFlywheelVelocity: Double) {
        val velocityError = Flywheel.targetVelocity - currentFlywheelVelocity
        
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
        val angularVelocityCompensation = Math.toDegrees(follower.angularVelocity) *
            TURRET_ANGULAR_VELOCITY_COMPENSATION_SECONDS
        return angleToGoal - headingDeg - angularVelocityCompensation
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

    private fun updateFilteredAcceleration(acceleration: Vector) {
        val alpha = ACCELERATION_FILTER_ALPHA.coerceIn(0.0, 1.0)
        filteredAcceleration = filteredAcceleration
            .times(1.0 - alpha)
            .plus(acceleration.times(alpha))
    }

    private fun getCorrectedVecIterative(botPose: Pose, targetPose: Pose, velocity: Vector, acceleration: Vector): Vector {
        val r = Vector(
            hypot(targetPose.x-botPose.x, targetPose.y-botPose.y),
            atan2(targetPose.y-botPose.y,targetPose.x-botPose.x)
        )
        var c = r
        repeat(ITERATIONS) {
            val t = getFlyTime(c.magnitude)
            val lookaheadTime = t + RESPONSE_LATENCY_SECONDS.coerceAtLeast(0.0)
            val accelerationDisplacement = acceleration.times(
                0.5 * lookaheadTime * lookaheadTime * ACCELERATION_GAIN
            )
            c = r
                .minus(velocity.times(lookaheadTime))
                .minus(accelerationDisplacement)
        }
        return if (isDriverTranslationIdle(velocity)) {
            val scale = STILL_PREDICTION_SCALE.coerceIn(0.0, 1.0)
            r.plus(c.minus(r).times(scale))
        } else {
            c
        }
    }

    private fun isDriverTranslationIdle(velocity: Vector): Boolean {
        return abs(velocity.magnitude) < TRANSLATION_STILL_VELOCITY
    }
}

enum class ShooterMethod {
    REGRESSION,
    PHYSICS
}
