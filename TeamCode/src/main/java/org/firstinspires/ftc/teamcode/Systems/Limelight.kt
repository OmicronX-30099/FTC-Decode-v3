package org.firstinspires.ftc.teamcode.Systems

import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.Util.Motif
import org.firstinspires.ftc.teamcode.Util.ROBOT

object Limelight: Subsystem {
    val limelight: Limelight3A by lazy { ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight") }

    const val LIMELIGHT_HEIGHT: Double = 0.0

    fun detectMotif() {
        val currTagID: Int = limelight.latestResult.fiducialResults[0].fiducialId
        Motif.entries.forEach {
            if (it.tagID == currTagID) {
                ROBOT.currStage.currMotif = it
            }
        }
    }
    fun getBlobTurnAngle() {}
}