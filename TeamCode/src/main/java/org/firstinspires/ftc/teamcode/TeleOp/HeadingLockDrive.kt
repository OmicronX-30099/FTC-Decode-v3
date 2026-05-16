package org.firstinspires.ftc.teamcode.TeleOp

import com.pedropathing.control.PIDFController
import com.pedropathing.math.MathFunctions as mf
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower as f
import dev.nextftc.hardware.driving.DriverControlledCommand
import java.util.function.Supplier
import kotlin.math.abs

class HeadingLockDriveCommand
    @JvmOverloads
    constructor(
        drivePower: Supplier<Double>,
        strafePower: Supplier<Double>,
        turnPower: Supplier<Double>,
        val headingGoal: Supplier<Double>,
        private val robotCentric: Boolean = true
    ) : DriverControlledCommand(drivePower, strafePower, turnPower) {

    @JvmOverloads
    constructor(
        drivePower: Supplier<Double>,
        strafePower: Supplier<Double>,
        turnPower: Supplier<Double>,
        headingGoal: Double,
        robotCentric: Boolean = true
    ) : this(
            drivePower,
            strafePower,
            turnPower,
            { headingGoal },
            robotCentric
        )

    val controller: PIDFController = PIDFController(f.constants.coefficientsHeadingPIDF)
    var turnScalar: Double = 0.5
    var lockHeading: Boolean = false
    companion object {
        const val SECONDARY_THRESHOLD: Double = 25.0
        const val JOYSTICK_DEADBAND: Double = 0.0
    }

    override fun start() {
        f.startTeleopDrive()
    }

    override fun calculateAndSetPowers(powers: DoubleArray) {
        val (drive, strafe, turn) = powers

        if ((turn > JOYSTICK_DEADBAND) || (!lockHeading)) {
            f.setTeleOpDrive(drive, strafe, turn * turnScalar, robotCentric)
            lockHeading = false
        } else {
            f.setTeleOpDrive(drive, strafe, updateHeadingLock(), robotCentric)
        }
    }

    fun updateHeadingLock(): Double {
        val error: Double =
            mf.getTurnDirection(f.pose.heading, headingGoal.get()) * mf.getSmallestAngleDifference(f.pose.heading, headingGoal.get())

        controller.coefficients =
            if (abs(error) < SECONDARY_THRESHOLD) { f.constants.coefficientsHeadingPIDF }
            else { f.constants.coefficientsSecondaryHeadingPIDF }

        controller.updateError(error)
        return controller.run()
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) f.breakFollowing()
    }
}