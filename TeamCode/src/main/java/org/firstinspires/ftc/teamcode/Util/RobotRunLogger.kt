@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.Util

import android.os.SystemClock
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.Systems.Load.BreakBeam
import org.firstinspires.ftc.teamcode.Systems.Load.Load
import org.firstinspires.ftc.teamcode.Systems.Load.Rollers
import org.firstinspires.ftc.teamcode.Systems.Shooter.Flywheel
import org.firstinspires.ftc.teamcode.Systems.Shooter.Hood
import org.firstinspires.ftc.teamcode.Systems.Shooter.Shooter
import org.firstinspires.ftc.teamcode.Systems.Shooter.Turret
import java.io.BufferedWriter
import java.io.File
import java.io.FileWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

object RobotRunLogger {
    private const val LOG_DIR_NAME = "robot_run_logs"
    private const val SAMPLE_PERIOD_MS = 50L

    private data class MotorEntry(val name: String, val motor: DcMotor)
    private data class ServoEntry(val name: String, val servo: Servo)
    private data class CRServoEntry(val name: String, val servo: CRServo)
    private data class DigitalEntry(val name: String, val channel: DigitalChannel)

    private val timestampFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)

    private var writer: BufferedWriter? = null
    private var logFile: File? = null
    private var startElapsedMs: Long = 0L
    private var lastSampleMs: Long = Long.MIN_VALUE
    private var lastUpdateCallMs: Long = Long.MIN_VALUE
    private var lastLoggedSampleMs: Long = Long.MIN_VALUE
    private var updateCallDeltaMs: Long = 0L
    private var rowCount: Long = 0L

    private var motors: List<MotorEntry> = emptyList()
    private var servos: List<ServoEntry> = emptyList()
    private var crServos: List<CRServoEntry> = emptyList()
    private var digitalChannels: List<DigitalEntry> = emptyList()
    private var voltageSensors: List<Pair<String, VoltageSensor>> = emptyList()
    private var motorEncoderCache: MutableList<String> = mutableListOf()
    private var motorTargetCache: MutableList<String> = mutableListOf()
    private var motorModeCache: MutableList<String> = mutableListOf()
    private var motorZeroPowerCache: MutableList<String> = mutableListOf()
    private var motorVelocityCache: MutableList<String> = mutableListOf()
    private var motorCurrentAmpsCache: MutableList<String> = mutableListOf()
    private var motorOverCurrentCache: MutableList<String> = mutableListOf()
    private var voltageCache: MutableList<String> = mutableListOf()
    private var digitalStateCache: MutableList<String> = mutableListOf()
    private var nextSlowMotorIndex: Int = 0
    private var nextSlowVoltageIndex: Int = 0
    private var nextSlowDigitalIndex: Int = 0

    val filePath: String
        get() = logFile?.absolutePath ?: "not logging"

    fun start(hardwareMap: HardwareMap, opModeName: String) {
        stop()

        motors = hardwareMap.dcMotor.entrySet()
            .map { MotorEntry(cleanName(it.key), it.value) }
            .sortedBy { it.name }
        servos = hardwareMap.servo.entrySet()
            .map { ServoEntry(cleanName(it.key), it.value) }
            .sortedBy { it.name }
        crServos = hardwareMap.crservo.entrySet()
            .map { CRServoEntry(cleanName(it.key), it.value) }
            .sortedBy { it.name }
        digitalChannels = hardwareMap.digitalChannel.entrySet()
            .map { DigitalEntry(cleanName(it.key), it.value) }
            .sortedBy { it.name }
        voltageSensors = hardwareMap.voltageSensor.entrySet()
            .map { cleanName(it.key) to it.value }
            .sortedBy { it.first }
        motorEncoderCache = MutableList(motors.size) { "" }
        motorTargetCache = MutableList(motors.size) { "" }
        motorModeCache = MutableList(motors.size) { "" }
        motorZeroPowerCache = MutableList(motors.size) { "" }
        motorVelocityCache = MutableList(motors.size) { "" }
        motorCurrentAmpsCache = MutableList(motors.size) { "" }
        motorOverCurrentCache = MutableList(motors.size) { "" }
        voltageCache = MutableList(voltageSensors.size) { "" }
        digitalStateCache = MutableList(digitalChannels.size) { "" }
        nextSlowMotorIndex = 0
        nextSlowVoltageIndex = 0
        nextSlowDigitalIndex = 0

        val directory = File(AppUtil.FIRST_FOLDER, LOG_DIR_NAME)
        if (!directory.exists()) directory.mkdirs()
        val fileName = "${cleanName(opModeName)}_${timestampFormat.format(Date())}.csv"
        val file = File(directory, fileName)

        writer = BufferedWriter(FileWriter(file))
        logFile = file
        startElapsedMs = SystemClock.elapsedRealtime()
        lastSampleMs = Long.MIN_VALUE
        lastUpdateCallMs = Long.MIN_VALUE
        lastLoggedSampleMs = Long.MIN_VALUE
        updateCallDeltaMs = 0L
        rowCount = 0L

        writeHeader()
    }

    fun sample(ballCount: Int, drivetrainScalar: Double = Double.NaN) {
        val activeWriter = writer ?: return
        val now = SystemClock.elapsedRealtime()
        updateCallDeltaMs = if (lastUpdateCallMs == Long.MIN_VALUE) 0L else now - lastUpdateCallMs
        lastUpdateCallMs = now
        if (lastSampleMs != Long.MIN_VALUE && now - lastSampleMs < SAMPLE_PERIOD_MS) return
        lastSampleMs = now
        val loggedSampleDeltaMs = if (lastLoggedSampleMs == Long.MIN_VALUE) 0L else now - lastLoggedSampleMs
        lastLoggedSampleMs = now
        refreshOneSlowHardwareRead()

        val pose = dev.nextftc.extensions.pedro.PedroComponent.follower.pose
        val velocity = dev.nextftc.extensions.pedro.PedroComponent.follower.velocity
        val angularVelocity = dev.nextftc.extensions.pedro.PedroComponent.follower.angularVelocity
        val shooterPose = ROBOT.shooterPose()

        val values = mutableListOf<String>()
        values += now.toString()
        values += (now - startElapsedMs).toString()
        values += rowCount.toString()
        values += updateCallDeltaMs.toString()
        values += loggedSampleDeltaMs.toString()
        values += finite(if (loggedSampleDeltaMs > 0L) 1000.0 / loggedSampleDeltaMs else 0.0)
        values += ROBOT.currAlliance.name
        values += ROBOT.currStage.name
        values += finite(pose.x)
        values += finite(pose.y)
        values += finite(pose.heading)
        values += finite(Math.toDegrees(pose.heading))
        values += vectorX(velocity)
        values += vectorY(velocity)
        values += finite(velocity.magnitude)
        values += finite(angularVelocity)
        values += finite(Math.toDegrees(angularVelocity))
        values += finite(shooterPose.x)
        values += finite(shooterPose.y)
        values += ballCount.toString()
        values += bool(BreakBeam.cachedPos1Occupied)
        values += bool(BreakBeam.cachedPos2Occupied)
        values += bool(BreakBeam.cachedPos3Occupied)
        values += bool(BreakBeam.cachedBb1State)
        values += bool(BreakBeam.cachedBb2State)
        values += bool(BreakBeam.cachedBb3State)
        values += bool(BreakBeam.cachedBb4State)
        values += bool(BreakBeam.cachedBb5State)
        values += bool(BreakBeam.cachedBb6State)
        values += bool(Rollers.isFeeding)
        values += bool(Load.isWaitingForShooter)
        values += Load.shotRequestCount.toString()
        values += Load.shotStartCount.toString()
        values += Load.lastShotWaitMs.toString()
        values += Shooter.flywheelState.name
        values += Shooter.shooterMethod.name
        values += finite(Flywheel.targetVelocity)
        values += finite(Flywheel.currentVelocity)
        values += finite(Shooter.flywheelVelocityError)
        values += bool(Shooter.flywheelReadyToFeed)
        values += finite(Flywheel.velocityOffset)
        values += finite(Flywheel.calculatePow(Flywheel.currentVelocity))
        values += finite(Turret.targetAngle)
        values += finite(Turret.offset)
        values += finite(Hood.targetPosition)
        values += finite(drivetrainScalar)
        values += finite(DrivePowerLimiter.currentCap)
        values += DrivePowerLimiter.currentReason
        values += gamepadValues(dev.nextftc.ftc.ActiveOpMode.gamepad1)
        values += gamepadValues(dev.nextftc.ftc.ActiveOpMode.gamepad2)

        voltageCache.forEach { values += it }
        motors.forEachIndexed { index, entry ->
            val motor = entry.motor
            values += safe { finite(motor.power) }
            values += motorEncoderCache.getOrElse(index) { "" }
            values += motorTargetCache.getOrElse(index) { "" }
            values += motorModeCache.getOrElse(index) { "" }
            values += motorZeroPowerCache.getOrElse(index) { "" }
            if (motor is DcMotorEx) {
                values += motorVelocityCache.getOrElse(index) { "" }
                values += motorCurrentAmpsCache.getOrElse(index) { "" }
                values += motorOverCurrentCache.getOrElse(index) { "" }
            } else {
                values += ""
                values += ""
                values += ""
            }
        }
        servos.forEach { entry -> values += safe { finite(entry.servo.position) } }
        crServos.forEach { entry -> values += safe { finite(entry.servo.power) } }
        digitalStateCache.forEach { values += it }

        activeWriter.write(values.joinToString(",") { csv(it) })
        activeWriter.newLine()
        rowCount++

        if (rowCount % 100L == 0L) {
            activeWriter.flush()
        }
    }

    fun stop() {
        writer?.runCatching {
            flush()
            close()
        }
        writer = null
    }

    private fun writeHeader() {
        val columns = mutableListOf(
            "wall_time_ms",
            "match_time_ms",
            "sample",
            "opmode_loop_delta_ms",
            "log_sample_delta_ms",
            "log_sample_rate_hz",
            "alliance",
            "stage",
            "pose_x_in",
            "pose_y_in",
            "pose_heading_rad",
            "pose_heading_deg",
            "robot_velocity_x_in_s",
            "robot_velocity_y_in_s",
            "robot_speed_in_s",
            "robot_angular_velocity_rad_s",
            "robot_angular_velocity_deg_s",
            "shooter_pose_x_in",
            "shooter_pose_y_in",
            "ball_count",
            "beam_pos1_occupied",
            "beam_pos2_occupied",
            "beam_pos3_occupied",
            "beam_bb1_state",
            "beam_bb2_state",
            "beam_bb3_state",
            "beam_bb4_state",
            "beam_bb5_state",
            "beam_bb6_state",
            "rollers_is_feeding",
            "load_waiting_for_shooter",
            "shot_request_count",
            "shot_start_count",
            "last_shot_wait_ms",
            "flywheel_state",
            "shooter_method",
            "flywheel_target_tps",
            "flywheel_current_tps",
            "flywheel_error_tps",
            "flywheel_ready_to_feed",
            "flywheel_velocity_offset_tps",
            "flywheel_commanded_power",
            "turret_target_angle_deg",
            "turret_offset_deg",
            "hood_target_position",
            "drivetrain_scalar",
            "drive_current_limit_cap",
            "drive_current_limit_reason"
        )

        columns += gamepadHeaders("g1")
        columns += gamepadHeaders("g2")
        voltageSensors.forEach { (name, _) -> columns += "voltage_${name}_v" }
        motors.forEach { entry ->
            columns += "motor_${entry.name}_power"
            columns += "motor_${entry.name}_encoder_ticks"
            columns += "motor_${entry.name}_target_ticks"
            columns += "motor_${entry.name}_mode"
            columns += "motor_${entry.name}_zero_power"
            columns += "motor_${entry.name}_velocity_tps"
            columns += "motor_${entry.name}_current_amps"
            columns += "motor_${entry.name}_over_current"
        }
        servos.forEach { entry -> columns += "servo_${entry.name}_position" }
        crServos.forEach { entry -> columns += "crservo_${entry.name}_power" }
        digitalChannels.forEach { entry -> columns += "digital_${entry.name}_state" }

        writer?.write(columns.joinToString(",") { csv(it) })
        writer?.newLine()
        writer?.flush()
    }

    private fun gamepadHeaders(prefix: String): List<String> = listOf(
        "${prefix}_left_stick_x",
        "${prefix}_left_stick_y",
        "${prefix}_right_stick_x",
        "${prefix}_right_stick_y",
        "${prefix}_left_trigger",
        "${prefix}_right_trigger",
        "${prefix}_a",
        "${prefix}_b",
        "${prefix}_x",
        "${prefix}_y",
        "${prefix}_left_bumper",
        "${prefix}_right_bumper",
        "${prefix}_dpad_up",
        "${prefix}_dpad_down",
        "${prefix}_dpad_left",
        "${prefix}_dpad_right",
        "${prefix}_start",
        "${prefix}_back",
        "${prefix}_left_stick_button",
        "${prefix}_right_stick_button"
    )

    private fun gamepadValues(gamepad: Gamepad): List<String> = listOf(
        finite(gamepad.left_stick_x.toDouble()),
        finite(gamepad.left_stick_y.toDouble()),
        finite(gamepad.right_stick_x.toDouble()),
        finite(gamepad.right_stick_y.toDouble()),
        finite(gamepad.left_trigger.toDouble()),
        finite(gamepad.right_trigger.toDouble()),
        bool(gamepad.a),
        bool(gamepad.b),
        bool(gamepad.x),
        bool(gamepad.y),
        bool(gamepad.left_bumper),
        bool(gamepad.right_bumper),
        bool(gamepad.dpad_up),
        bool(gamepad.dpad_down),
        bool(gamepad.dpad_left),
        bool(gamepad.dpad_right),
        bool(gamepad.start),
        bool(gamepad.back),
        bool(gamepad.left_stick_button),
        bool(gamepad.right_stick_button)
    )

    private fun vectorX(vector: Vector): String = finite(vector.xComponent)
    private fun vectorY(vector: Vector): String = finite(vector.yComponent)

    private fun refreshOneSlowHardwareRead() {
        if (motors.isNotEmpty()) {
            val index = nextSlowMotorIndex % motors.size
            val motor = motors[index].motor
            motorEncoderCache[index] = safe { motor.currentPosition.toString() }
            motorTargetCache[index] = safe { motor.targetPosition.toString() }
            motorModeCache[index] = safe { motor.mode.name }
            motorZeroPowerCache[index] = safe { motor.zeroPowerBehavior.name }
            if (motor is DcMotorEx) {
                motorVelocityCache[index] = safe { finite(motor.velocity) }
                motorCurrentAmpsCache[index] = safe { finite(motor.getCurrent(CurrentUnit.AMPS)) }
                motorOverCurrentCache[index] = safe { bool(motor.isOverCurrent) }
            }
            nextSlowMotorIndex = (index + 1) % motors.size
        }

        if (voltageSensors.isNotEmpty()) {
            val index = nextSlowVoltageIndex % voltageSensors.size
            voltageCache[index] = safe { finite(voltageSensors[index].second.voltage) }
            nextSlowVoltageIndex = (index + 1) % voltageSensors.size
        }

        if (digitalChannels.isNotEmpty()) {
            val index = nextSlowDigitalIndex % digitalChannels.size
            digitalStateCache[index] = safe { bool(digitalChannels[index].channel.state) }
            nextSlowDigitalIndex = (index + 1) % digitalChannels.size
        }
    }

    private fun cleanName(name: String): String =
        name.trim().replace(Regex("[^A-Za-z0-9_]+"), "_").trim('_').ifEmpty { "unnamed" }

    private fun finite(value: Double): String =
        if (value.isFinite()) String.format(Locale.US, "%.6f", value) else ""

    private fun bool(value: Boolean): String = if (value) "1" else "0"

    private fun csv(value: String): String {
        if (value.none { it == ',' || it == '"' || it == '\n' || it == '\r' }) return value
        return "\"" + value.replace("\"", "\"\"") + "\""
    }

    private inline fun safe(block: () -> String): String =
        runCatching(block).getOrDefault("")
}
