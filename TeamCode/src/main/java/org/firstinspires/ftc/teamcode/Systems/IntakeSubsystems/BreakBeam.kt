package org.firstinspires.ftc.teamcode.Systems.IntakeSubsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object BreakBeamSubsystem: Subsystem {
    private val intakeBreakBeam: DigitalChannel by lazy { ActiveOpMode.hardwareMap.get(DigitalChannel::class.java, "bb") }
    internal var currIntakeState: Boolean = false
    internal var prevIntakeState: Boolean = false

    fun update() {
        prevIntakeState = currIntakeState
        currIntakeState = intakeBreakBeam.state
    }
}