package org.firstinspires.ftc.teamcode.Systems

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Constants.ConfigConstants
import org.firstinspires.ftc.teamcode.Util.Motif
import org.firstinspires.ftc.teamcode.Util.ROBOT

object Miscellaneous: Subsystem {
    private val leftTiltServo: ServoEx = ServoEx(ConfigConstants.LEFT_TILT_SERVO)
    private val rightTiltServo: ServoEx = ServoEx(ConfigConstants.RIGHT_TILT_SERVO)

    private lateinit var limelight: Limelight3A

    internal fun raiseBot() { leftTiltServo.position = 0.0 .also { rightTiltServo.position = 0.0 } }
    internal fun lowerBot() { leftTiltServo.position = 0.0 .also { rightTiltServo.position = 0.0 } }

    internal fun startLimelight() {
        limelight = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, ConfigConstants.LIMELIGHT)
        limelight.pipelineSwitch(0)
        limelight.start()
    }
    internal fun checkForMotif() {
        val result: LLResult = limelight.latestResult
        if ((result != null) && result.isValid) {
            val tagID = result.fiducialResults[0].fiducialId
            Motif.entries.forEach {
                if (it.tagID == tagID) { ROBOT.currStage.currMotif = it }
            }
        } else {
            ROBOT.currStage.currMotif = Motif.UNKNOWN
        }
    }
    internal fun shutDownLimelight() { limelight.stop() }
}