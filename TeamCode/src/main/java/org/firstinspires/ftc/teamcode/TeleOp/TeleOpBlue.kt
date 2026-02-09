package org.firstinspires.ftc.teamcode.TeleOp

class TeleOpBlue: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load)
        includePedro(PedroConstants::createFollower)
    }

    val drivetrain: DriverControlledCommand by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            true
        )
    }

    override fun onStartButtonPressed() {
        ROBOT.currAlliance = Alliance.BLUE
        ROBOT.currStage = Stage.TELEOP
        follower.setStartingPose(Pose(72.0,72.0,PI/2))
        drivetrain.schedule()

        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue { Rollers.run(0.35,1.0) }
            .whenBecomesFalse { Rollers.run(0.0,0.0) }
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0).and(Gamepads.gamepad1.rightTrigger.inRange(0.0..0.0))
            .whenBecomesTrue { Rollers.run(-1.0,-1.0) }
            .whenBecomesFalse { Rollers.run(0.0,0.0) }
            
    }
}
