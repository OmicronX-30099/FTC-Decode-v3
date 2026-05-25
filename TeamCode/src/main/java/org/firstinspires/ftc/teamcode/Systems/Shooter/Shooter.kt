@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.firstinspires.ftc.teamcode.Util.ROBOT
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@Configurable
object Shooter: SubsystemGroup(Turret, Flywheel, Hood, ShooterLights) {
    private const val ITERATIONS: Int = 3
    private const val TURRET_X_OFFSET: Double = -0.96
    private const val TURRET_Y_OFFSET: Double = 0.0

    @JvmField var RESPONSE_LATENCY_SECONDS: Double = 0.015
    @JvmField var TURRET_ANGULAR_VELOCITY_COMPENSATION_SECONDS: Double = 0.1
    @JvmField var FEED_READY_TOLERANCE_TPS: Double = 40.0
    
    var flywheelState: FlywheelState = FlywheelState.PREDICTIVE_AUTO_AIM

    var shooterMethod: ShooterMethod = ShooterMethod.REGRESSION

    private var lastTurretUpdateTime = 0L
    var flywheelVelocityError: Double = 0.0
        private set
    var flywheelReadyToFeed: Boolean = false
        private set

    override fun initialize() {
        reset()
    }

    fun update() {
        val robotPose = follower.pose
        if (robotPose.x == 0.0 && robotPose.y == 0.0) return

        val velocity = follower.velocity
        val currentFlywheelVelocity = Flywheel.refreshVelocity()

        val shooterPose = shooterPose(robotPose)
        val flywheelVec = if (flywheelState == FlywheelState.PREDICTIVE_AUTO_AIM) {
            getCorrectedVecIterative(
                shooterPose,
                ROBOT.currAlliance.flywheelGoalPose,
                velocity
            )
        } else {
            vectorTo(shooterPose, ROBOT.currAlliance.flywheelGoalPose)
        }
        val turretVec = if (flywheelState == FlywheelState.PREDICTIVE_AUTO_AIM) {
            getCorrectedVecIterative(
                shooterPose,
                ROBOT.currAlliance.turretGoalPose,
                velocity
            )
        } else {
            vectorTo(shooterPose, ROBOT.currAlliance.turretGoalPose)
        }
        val robotHeadingDeg = Math.toDegrees(robotPose.heading)
        val robotAngularVelocityDeg = Math.toDegrees(follower.angularVelocity)

        when (flywheelState) {
            FlywheelState.PREDICTIVE_AUTO_AIM,
            FlywheelState.AUTO_AIM -> {
                updateFlywheel(flywheelVec.magnitude, robotPose.y)
                updateTurret(turretVec, robotHeadingDeg, robotAngularVelocityDeg)
                updateHood(flywheelVec.magnitude, currentFlywheelVelocity)
            }
            FlywheelState.MANUAL -> {
                Flywheel.update(shouldRefreshVelocity = false)
                updateTurret(turretVec, robotHeadingDeg, robotAngularVelocityDeg)
                updateHood(flywheelVec.magnitude, currentFlywheelVelocity)
            }
        }
        
        flywheelVelocityError = Flywheel.targetVelocity - currentFlywheelVelocity
        flywheelReadyToFeed = Flywheel.targetVelocity > 500.0 &&
            abs(flywheelVelocityError) <= FEED_READY_TOLERANCE_TPS
        val error = abs(flywheelVelocityError)
        ShooterLights.setPosition(when {
            error <= 40.0 -> 0.5
            error <= 100.0 -> 0.388
            else -> 0.277
        })
    }
    fun flywheelManual(velocity: Double = Flywheel.MANUAL_VELOCITY) {
        flywheelState = FlywheelState.MANUAL
        Flywheel.targetVelocity = velocity
    }
    fun enablePredictive() { flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM }
    fun enableAutoAim() { flywheelState = FlywheelState.AUTO_AIM }
    fun cycleFlywheelMode() {
        when (flywheelState) {
            FlywheelState.PREDICTIVE_AUTO_AIM -> flywheelManual()
            FlywheelState.MANUAL -> enableAutoAim()
            FlywheelState.AUTO_AIM -> enablePredictive()
        }
    }
    fun resetOffsets() {
        Turret.resetOffset()
        Flywheel.resetVelocityOffset()
    }

    fun reset() {
        Flywheel.reset()
        Turret.reset()
        Hood.reset()
        ShooterLights.resetCache()
        flywheelState = FlywheelState.PREDICTIVE_AUTO_AIM
        flywheelVelocityError = 0.0
        flywheelReadyToFeed = false
    }
    fun debug(): String = "Turret Data: \n${Turret.debug()} \nFlywheel Data: \n${Flywheel.debug()} \nHood Data: \n${Hood.debug()}"

    private fun updateTurret(vec: Vector, robotHeadingDeg: Double, robotAngularVelocityDeg: Double) {
        val newTarget = calculateTurretAngle(vec, robotHeadingDeg, robotAngularVelocityDeg)
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
    private fun updateFlywheel(d: Double, robotY: Double) {
        var targetVel = if (shooterMethod == ShooterMethod.PHYSICS) {
            PhysicsShooter.getTargetParams(d).second
        } else {
            Flywheel.calculateVelocity(d)
        }

        // Farzone velocity lock: y < 32 inches -> minimum velocity 1660
        if (robotY < 35.0) {
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

    private fun calculateTurretAngle(
        vec: Vector,
        robotHeadingDeg: Double,
        robotAngularVelocityDeg: Double
    ): Double {
        val angleToGoal = Math.toDegrees(atan2(vec.yComponent, vec.xComponent))
        val angularVelocityCompensation = robotAngularVelocityDeg *
            TURRET_ANGULAR_VELOCITY_COMPENSATION_SECONDS
        return angleToGoal - robotHeadingDeg - angularVelocityCompensation
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
            val lookaheadTime = t + RESPONSE_LATENCY_SECONDS.coerceAtLeast(0.0)
            c = r
                .minus(velocity.times(lookaheadTime))
        }
        return c
    }

    private fun shooterPose(robotPose: Pose): Pose {
        val heading = robotPose.heading
        return robotPose + Pose(
            TURRET_X_OFFSET * cos(heading) - TURRET_Y_OFFSET * sin(heading),
            TURRET_X_OFFSET * sin(heading) + TURRET_Y_OFFSET * cos(heading)
        )
    }

    private fun vectorTo(from: Pose, to: Pose): Vector =
        Vector(hypot(to.x - from.x, to.y - from.y), atan2(to.y - from.y, to.x - from.x))
}

enum class ShooterMethod {
    REGRESSION,
    PHYSICS
}
