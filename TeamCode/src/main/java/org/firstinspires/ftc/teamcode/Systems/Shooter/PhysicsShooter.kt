package org.firstinspires.ftc.teamcode.Systems.Shooter

import com.bylazar.configurables.annotations.Configurable
import kotlin.math.*

/**
 * Simplified Physics Engine for Wiffle Ball Trajectory
 * Uses algebraic projectile motion with tuning factors for lift and drag.
 */
@Configurable
object PhysicsShooter {
    // Environment & Hardware Constants
    var G = 386.09                // Gravity (in/s^2)
    var SHOOTER_HEIGHT = 10.0     // Launch height (inches)
    var GOAL_HEIGHT = 42.0        // Target height (inches)
    var FLYWHEEL_RADIUS = 1.417   // 72mm / 2 in inches
    
    // Tuning Parameters - ADJUST THESE TO MATCH YOUR SHOTS
    var liftFactor = 0.1        // 0.0 to 1.0. Higher = less power needed (ball floats more)
    var dragCoefficient = 0.135     // Higher = more power needed at long distances
    var flywheelMultiplier = 1.3  // Single roller ratio. Usually 1.8 to 2.2 for this hardware

    
    // Motor Constants
    var MOTOR_TPR = 28.0         // Bare motor ticks per rev
    var GEAR_RATIO = 1.5          // 20t motor / 30t flywheel (6000rpm motor : 4000rpm flywheel)
    val EFFECTIVE_TPR: Double get() = MOTOR_TPR * GEAR_RATIO

    /**
     * No precomputation needed for this simplified model.
     */
    fun precomputeField() { }

    /**
     * Returns Pair(TargetHoodDegrees, TargetFlywheelTicksPerSec)
     */
    fun getTargetParams(distance: Double): Pair<Double, Double> {
        val x = distance.coerceIn(20.0, 170.0)
        val h = GOAL_HEIGHT - SHOOTER_HEIGHT
        
        // Robot degrees: Higher = Flatter launch. 
        // Near (60") -> 25.0 (65 deg launch)
        // Far (100"+) -> 35.0 (55 deg launch)
        val hoodDeg = when {
            x < 40.0 -> 14.0
            x < 50.0 -> 20.0
            x < 70.0 -> 25.0
            x < 110.0 -> 30.0
            else -> 35.0
        }
        
        // Transform robot hood deg to physical launch angle (relative to floor)
        val launchAngle = 90.0 - hoodDeg
        val angleRad = Math.toRadians(launchAngle)
        
        // Effective gravity after lift compensation
        val gEff = G * (1.0 - liftFactor)
        
        val cosA = cos(angleRad)
        val tanA = tan(angleRad)
        val denominator = 2 * cosA * cosA * (x * tanA - h)
        
        // Fallback for physically impossible shots
        if (denominator <= 0.1) return hoodDeg to 1450.0
        
        var v0 = sqrt((gEff * x * x) / denominator)
        
        // Apply linear drag compensation based on distance
        v0 *= (1.0 + dragCoefficient * (x / 100.0))
        
        // Convert ball velocity to flywheel TPS
        val vFlywheel = v0 * flywheelMultiplier
        val rps = vFlywheel / (2 * PI * FLYWHEEL_RADIUS)
        val tps = rps * EFFECTIVE_TPR
        
        return hoodDeg to tps
    }

    /**
     * Calculates the flight time for a given distance based on the physics model.
     */
    fun getFlyTime(distance: Double): Double {
        val x = distance.coerceIn(20.0, 170.0)
        val h = GOAL_HEIGHT - SHOOTER_HEIGHT
        
        // Use the same hood logic as getTargetParams to get the angle
        val hoodDeg = when {
            x < 40.0 -> 14.0
            x < 50.0 -> 20.0
            x < 70.0 -> 25.0
            x < 110.0 -> 30.0
            else -> 35.0
        }
        
        val launchAngle = 90.0 - hoodDeg
        return getFlyTime(x, launchAngle)
    }

    /**
     * Calculates flight time based on horizontal distance and launch angle.
     * Use the formula: t = x / (v0 * cos(theta))
     */
    fun getFlyTime(distance: Double, hoodAngle: Double): Double {
        val x = distance.coerceIn(20.0, 170.0)
        val h = GOAL_HEIGHT - SHOOTER_HEIGHT
        val launchAngle = 90.0 - hoodAngle
        val angleRad = Math.toRadians(launchAngle)

        // Get the v0 required for this distance and angle from physics
        val gEff = G * (1.0 - liftFactor)
        val cosA = cos(angleRad)
        val tanA = tan(angleRad)
        val denominator = 2 * cosA * cosA * (x * tanA - h)
        if (denominator <= 0.1) return 0.5

        var v0 = sqrt((gEff * x * x) / denominator)
        v0 *= (1.0 + dragCoefficient * (x / 100.0))

        return x / (v0 * cos(angleRad))
    }

    /**
     * Calculates flight time based on horizontal distance, launch angle, and flywheel TPS.
     */
    fun getFlyTime(distance: Double, hoodAngle: Double, tps: Double): Double {
        val x = distance.coerceIn(20.0, 170.0)
        val launchAngle = 90.0 - hoodAngle
        val angleRad = Math.toRadians(launchAngle)

        // Convert tps to v0
        val rps = tps / EFFECTIVE_TPR
        val vFlywheel = rps * (2 * PI * FLYWHEEL_RADIUS)
        val v0 = vFlywheel / flywheelMultiplier

        val vx = v0 * cos(angleRad)
        if (vx <= 1.0) return 0.5
        
        return x / vx
    }
}
