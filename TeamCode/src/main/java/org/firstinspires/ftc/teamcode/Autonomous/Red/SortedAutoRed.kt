@file:Suppress("PackageName", "unused")

package org.firstinspires.ftc.teamcode.Autonomous.Red

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Constants.PedroConstants
import org.firstinspires.ftc.teamcode.Systems.Load
import org.firstinspires.ftc.teamcode.Systems.LoadSubsystems.BilinearIndexMachine
import org.firstinspires.ftc.teamcode.Systems.LoadSubsystems.Rollers
import org.firstinspires.ftc.teamcode.Systems.Miscellaneous
import org.firstinspires.ftc.teamcode.Systems.Shooter
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.Motif
import org.firstinspires.ftc.teamcode.Util.ROBOT
import org.firstinspires.ftc.teamcode.Util.Stage
import org.firstinspires.ftc.teamcode.Util.addSubsystems
import org.firstinspires.ftc.teamcode.Util.includePedro
import kotlin.math.PI

@Autonomous(name = "Sorted Red Auto", group = "Sorted Autos", preselectTeleOp = "Red TeleOp")
class SortedAutoRed: NextFTCOpMode() {
    init {
        addSubsystems(Shooter, Load, Miscellaneous)
        includePedro(PedroConstants::createFollower)
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        ROBOT.currAlliance = Alliance.RED
        ROBOT.currStage = Stage.AUTONOMOUS
        ROBOT.currStage.useFlywheelVel = true
        Miscellaneous.startLimelight()
    }

    override fun onWaitForStart() { Miscellaneous.checkForMotif() }

    override fun onStartButtonPressed() {
        Miscellaneous.shutDownLimelight()
        follower.setStartingPose(Pose(79.0,7.5,Math.toRadians(90.0)))
        when (ROBOT.currStage.currMotif) {
            Motif.PPG -> { ppg() }
            Motif.PGP -> { pgp() }
            Motif.GPP -> { gpp() }
            Motif.UNKNOWN -> { pgp() }
        }
    }

    override fun onUpdate() {
        Shooter.update()
        telemetry.update()
    }

    fun gpp() {
        buildGPPPaths()
        val gpp = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.6),
            SequentialGroup(
                InstantCommand { Rollers.unlockShooter() .also { Rollers.run(0.5,0.8) } },
                Delay(1.6),
                InstantCommand { Rollers.stop() .also { Rollers.lockShooter() } }

            ),
            ParallelGroup(
                FollowPath(paths[1]),
                SequentialGroup(
                    InstantCommand {
                        Rollers.run(0.2,1.0)
                        BilinearIndexMachine.lockTransfer()
                        Delay(0.5)
                        BilinearIndexMachine.transferLeft()
                    },
                    Delay(1.7),
                    InstantCommand { BilinearIndexMachine.transferRight() },
                    Delay(0.7),
                    InstantCommand { Rollers.stop() }
                )
            ),
            Delay(0.25),
            FollowPath(paths[2],true,1.0),
            Delay(0.4),
            SequentialGroup(
                InstantCommand { BilinearIndexMachine.unlockTransfer() .also { BilinearIndexMachine.transferRight() } },
                InstantCommand { Rollers.run(1.0,0.0) .also{ Rollers.unlockShooter() } },
                Delay(0.3),
                InstantCommand { BilinearIndexMachine.transferLeft() },
                Delay(0.4),
                InstantCommand { Rollers.run(1.0,1.0) },
                Delay(0.5),
                InstantCommand {
                    Rollers.stop()
                    Rollers.lockShooter()
                    BilinearIndexMachine.lockTransfer()
                }
            ),
            Delay(0.2),
            ParallelGroup(
                FollowPath(paths[3],true,0.35),
                SequentialGroup(
                    InstantCommand {
                        Rollers.run(0.2,1.0)
                        BilinearIndexMachine.lockTransfer()
                        Delay(0.5)
                        BilinearIndexMachine.transferLeft()
                    },
                    Delay(1.7),
                    InstantCommand { BilinearIndexMachine.transferRight() },
                    Delay(0.7),
                    InstantCommand { Rollers.stop() }
                )
            ),
            Delay(0.1),
            FollowPath(paths[4],true,1.0),
            Delay(0.2),
            SequentialGroup(
                InstantCommand { BilinearIndexMachine.unlockTransfer() .also{ BilinearIndexMachine.transferMiddle() } },
                Delay(0.3),
                InstantCommand { Rollers.run(1.0,1.0) .also{ Rollers.unlockShooter() } },
                Delay(0.8),
                InstantCommand { BilinearIndexMachine.transferLeft() },
                Delay(0.4),
                InstantCommand { BilinearIndexMachine.transferRight() },
                Delay(0.5),
                InstantCommand {
                    Rollers.stop()
                    Rollers.lockShooter()
                    BilinearIndexMachine.lockTransfer()
                }

            ),
            ParallelGroup(
                FollowPath(paths[5],true,1.0),
                SequentialGroup(
                    InstantCommand {
                        Rollers.run(0.3,1.0)
                        Rollers.lockShooter()
                        BilinearIndexMachine.unlockTransfer()
                        BilinearIndexMachine.transferMiddle()
                    }

                )
            ),
            InstantCommand { Rollers.stop() },
            Delay(0.15),
            FollowPath(paths[6]),
            Delay(0.2),
            SequentialGroup(
                InstantCommand { Rollers.unlockShooter() .also{ Rollers.run(0.5,0.8) } },
                Delay(1.5),
                InstantCommand { Rollers.stop() .also { Rollers.lockShooter() } }

            )
        )
        gpp.schedule()
    }
    fun pgp() {
        buildPGPPaths()
        val pgp = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.6),SequentialGroup(
                InstantCommand { Rollers.unlockShooter() .also { Rollers.run(0.5,0.8) } },
                Delay(1.6),
                InstantCommand { Rollers.stop() .also { Rollers.lockShooter() } }
            ),
            ParallelGroup(
                FollowPath(paths[1],true,0.75),
                SequentialGroup(
                    InstantCommand {
                        Rollers.run(0.3,1.0)
                        Rollers.lockShooter()
                        BilinearIndexMachine.unlockTransfer()
                        BilinearIndexMachine.transferMiddle()
                    }
                )
            ),
            Delay(0.5),
            InstantCommand { Rollers.stop() },
            Delay(0.6),
            FollowPath(paths[2]),
            Delay(0.4),
            SequentialGroup(
                InstantCommand { Rollers.unlockShooter() .also{ Rollers.run(0.5,0.8) } },
                Delay(1.5),
                InstantCommand { Rollers.stop() .also{ Rollers.lockShooter() } }
            ),
            ParallelGroup(
                FollowPath(paths[3],true,0.35),
                SequentialGroup(
                    InstantCommand { Rollers.run(0.2,1.0) .also { BilinearIndexMachine.lockTransfer() } },
                    Delay(0.3),
                    InstantCommand { BilinearIndexMachine.transferLeft() },
                    Delay(1.0),
                    InstantCommand { BilinearIndexMachine.transferRight() },
                )
            ),
            Delay(0.2),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[4],true,1.0),
            Delay(0.2),
            SequentialGroup(
                InstantCommand { BilinearIndexMachine.unlockTransfer() .also{ BilinearIndexMachine.transferRight() } },
                Delay(0.2),
                InstantCommand { Rollers.run(0.5,1.0) .also{ Rollers.unlockShooter() } },
                Delay(1.0),
                InstantCommand { BilinearIndexMachine.transferLeft() },
                Delay(0.8),
                InstantCommand {
                    Rollers.stop()
                    Rollers.lockShooter()
                    BilinearIndexMachine.lockTransfer()
                }
            ),
            ParallelGroup(
                FollowPath(paths[5]),
                SequentialGroup(
                    Delay(0.2),
                    InstantCommand { BilinearIndexMachine.transferLeft() }
                )
            ),
            ParallelGroup(
                FollowPath(paths[6],true,0.35),
                SequentialGroup(
                    InstantCommand { Rollers.run(0.2,1.0) .also{ BilinearIndexMachine.lockTransfer() } },
                    Delay(0.3),
                    InstantCommand { BilinearIndexMachine.transferLeft() },
                    Delay(1.3),
                    InstantCommand { BilinearIndexMachine.transferRight() },
                    Delay(0.7),
                    InstantCommand { Rollers.stop() }
                )
            ),
            Delay(0.4),
            FollowPath(paths[7],true,1.0),
            Delay(0.6),
            SequentialGroup(
                InstantCommand { BilinearIndexMachine.unlockTransfer() .also{ BilinearIndexMachine.transferRight() } },
                Delay(0.2),
                InstantCommand { Rollers.run(1.0,0.0) .also { Rollers.unlockShooter() } },
                Delay(0.4),
                InstantCommand { BilinearIndexMachine.transferLeft() },
                Delay(0.5),
                InstantCommand { Rollers.run(1.0,1.0) },
                Delay(0.7),
                InstantCommand {
                    Rollers.stop()
                    Rollers.lockShooter()
                    BilinearIndexMachine.lockTransfer()
                }
            )
        )
        pgp.schedule()
    }
    fun ppg() {
        buildPPGPaths()
        val ppg = SequentialGroup(
            FollowPath(paths[0]),
            Delay(0.6),
            SequentialGroup(
                InstantCommand { Rollers.unlockShooter() .also{ Rollers.run(0.5,0.8) } },
                Delay(1.6),
                InstantCommand { Rollers.stop() .also { Rollers.lockShooter() } }
            ),
            ParallelGroup(
                FollowPath(paths[1]),
                SequentialGroup(
                    InstantCommand {
                        Rollers.run(0.2,1.0)
                        BilinearIndexMachine.lockTransfer()
                        Delay(0.5)
                        BilinearIndexMachine.transferLeft()
                    },
                    Delay(1.6),
                    InstantCommand { BilinearIndexMachine.transferRight() },
                    Delay(0.7),
                    InstantCommand { Rollers.stop() .also { BilinearIndexMachine.transferLeft() } }
                )
            ),
            Delay(1.0),
            FollowPath(paths[2]),
            Delay(0.4),
            SequentialGroup(
                InstantCommand { BilinearIndexMachine.unlockTransfer() .also{ BilinearIndexMachine.transferLeft() } },
                InstantCommand { Rollers.run(1.0,0.0) .also { Rollers.unlockShooter() } },
                Delay(0.3),
                InstantCommand { Rollers.run(1.0,1.0) },
                Delay(0.5),
                InstantCommand { BilinearIndexMachine.transferRight() },
                Delay(0.5),
                InstantCommand {
                    Rollers.stop()
                    Rollers.lockShooter()
                    BilinearIndexMachine.unlockTransfer()
                }
            ),
            ParallelGroup(
                FollowPath(paths[3],true,1.0),
                SequentialGroup(
                    InstantCommand { Rollers.run(0.2,1.0) },
                )
            ),
            Delay(0.2),
            InstantCommand { Rollers.stop() },
            FollowPath(paths[4],true,1.0),
            Delay(0.4),
            SequentialGroup(
                InstantCommand{ Rollers.unlockShooter() .also { Rollers.run(0.5,0.8) } },
                Delay(1.6),
                InstantCommand {
                    Rollers.stop()
                    Rollers.lockShooter()
                    BilinearIndexMachine.lockTransfer()
                }
            ),
            ParallelGroup(
                FollowPath(paths[5]),
                SequentialGroup(
                    Delay(0.2),
                    InstantCommand { BilinearIndexMachine.transferLeft() }
                )
            ),
            ParallelGroup(
                FollowPath(paths[6], true, 0.35),
                SequentialGroup(
                    InstantCommand { Rollers.run(0.2, 1.0) .also { BilinearIndexMachine.lockTransfer() } },
                    Delay(0.3),
                    InstantCommand { BilinearIndexMachine.transferLeft() },
                    Delay(1.3),
                    InstantCommand { BilinearIndexMachine.transferRight() },
                    Delay(1.0),
                    InstantCommand { Rollers.run(0.0, 0.0) }
                )
            ),
            FollowPath(paths[7], true, 1.0),
            Delay(0.6),
            SequentialGroup(
                InstantCommand { BilinearIndexMachine.unlockTransfer() .also { BilinearIndexMachine.transferRight() } },
                Delay(0.2),
                InstantCommand { Rollers.run(1.0, 0.0) .also { Rollers.unlockShooter() } },
                Delay(0.3),
                InstantCommand { Rollers.run(1.0, 1.0) },
                Delay(0.5),
                InstantCommand { BilinearIndexMachine.transferLeft() },
                Delay(0.7),
                InstantCommand { Rollers.run(0.0, 0.0) }
            )
        )
        ppg.schedule()
    }
    fun buildGPPPaths() {
        paths = arrayOf()
        val test1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(79.0,7.5),Pose(89.0,7.5)))
            .setConstantHeadingInterpolation(PI/2)
            .addPath(BezierLine(Pose(89.0,7.5),Pose(89.0,77.5)))
            .setLinearHeadingInterpolation(PI/2, Math.toRadians(-25.0))
            .build()
        val test2 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(89.0,77.5),Pose(94.5,55.0),Pose(125.0,65.25)))
            .setLinearHeadingInterpolation(Math.toRadians(-20.0),0.0)
            .addParametricCallback(0.17) { follower.setMaxPower(0.35) }
            .addParametricCallback(0.94) { follower.setMaxPower(1.0) }
            .build()
        val test3 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(125.0,65.25),Pose(103.0,66.5),Pose(90.0,85.5)))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test4 = follower.pathBuilder()
            .addPath((BezierLine(Pose(90.0, 85.5), Pose(120.0, 85.5))))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test5 = follower.pathBuilder()
            .addPath(BezierLine(Pose(120.0, 85.5), Pose(90.0, 85.5)))
            .setLinearHeadingInterpolation(0.0,Math.toRadians(-90.0))
            .build()
        val test6 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(90.0,85.5), Pose(90.0,40.0),Pose(125.0,33.0)))
            .setLinearHeadingInterpolation(Math.toRadians(-90.0),Math.toRadians(-10.0))
            .build()
        val test7 = follower.pathBuilder()
            .addPath(BezierLine(Pose(125.0,33.0), Pose(87.5,104.0)))
            .setConstantHeadingInterpolation(Math.toRadians(-55.0))
            .build()
        paths += test1
        paths += test2
        paths += test3
        paths += test4
        paths += test5
        paths += test6
        paths += test7
    }
    fun buildPGPPaths() {
        paths = arrayOf()
        val test1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(79.0,7.5),Pose(89.0,7.5)))
            .setConstantHeadingInterpolation(PI/2)
            .addPath(BezierLine(Pose(89.0,7.5),Pose(89.0,77.5)))
            .setLinearHeadingInterpolation(PI/2, Math.toRadians(-25.0))
            .build()
        val test2 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(89.0,77.5),Pose(94.5,55.0),Pose(125.0,65.25)))
            .setLinearHeadingInterpolation(Math.toRadians(-20.0),0.0)
            .build()
        val test3 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(125.0,65.25),Pose(100.0,65.0),Pose(90.0,83.5)))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test4 = follower.pathBuilder()
            .addPath((BezierLine(Pose(90.0, 83.5), Pose(125.0, 83.5))))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test5 = follower.pathBuilder()
            .addPath(BezierLine(Pose(125.0, 83.5), Pose(90.0, 83.5)))
            .setLinearHeadingInterpolation(0.0,Math.toRadians(-90.0))
            .build()
        val test6 = follower.pathBuilder()
            .addPath(BezierLine(Pose(90.0,83.5), Pose(90.0,37.0)))
            .setLinearHeadingInterpolation(Math.toRadians(-90.0),Math.toRadians(0.0))
            .build()
        val test7 = follower.pathBuilder()
            .addPath((BezierLine(Pose(90.0,37.0),Pose(125.0,37.0))))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test8 = follower.pathBuilder()
            .addPath(BezierLine(Pose(125.0,37.0), Pose(84.5,108.0)))
            .setConstantHeadingInterpolation(Math.toRadians(-62.0))
            .build()
        paths += test1
        paths += test2
        paths += test3
        paths += test4
        paths += test5
        paths += test6
        paths += test7
        paths += test8
    }
    fun buildPPGPaths() {
        paths = arrayOf()
        val test1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(79.0,7.5),Pose(89.0,7.5)))
            .setConstantHeadingInterpolation(PI/2)
            .addPath(BezierLine(Pose(89.0,7.5),Pose(89.0,77.5)))
            .setLinearHeadingInterpolation(PI/2, Math.toRadians(-25.0))
            .build()
        val test2 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(89.0,77.5),Pose(94.5,55.0),Pose(125.0,65.25)))
            .setLinearHeadingInterpolation(Math.toRadians(-20.0),0.0)
            .addParametricCallback(0.17) { follower.setMaxPower(0.35) }
            .addParametricCallback(0.94) { follower.setMaxPower(1.0) }
            .build()
        val test3 = follower.pathBuilder()
            .addPath(BezierCurve(Pose(125.0, 65.25), Pose(100.0, 65.0), Pose(90.0, 83.5)))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test4 = follower.pathBuilder()
            .addPath((BezierLine(Pose(90.0, 83.5), Pose(125.0, 83.5))))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test5 = follower.pathBuilder()
            .addPath(BezierLine(Pose(125.0, 83.5), Pose(90.0, 83.5)))
            .setLinearHeadingInterpolation(0.0,Math.toRadians(-90.0))
            .build()
        val test6 = follower.pathBuilder()
            .addPath(BezierLine(Pose(90.0,83.5), Pose(90.0,37.0)))
            .setLinearHeadingInterpolation(Math.toRadians(-90.0),Math.toRadians(0.0))
            .build()
        val test7 = follower.pathBuilder()
            .addPath((BezierLine(Pose(90.0,37.0),Pose(125.0,37.0))))
            .setConstantHeadingInterpolation(0.0)
            .build()
        val test8 = follower.pathBuilder()
            .addPath(BezierLine(Pose(125.0, 37.0), Pose(84.5, 105.5)))
            .setConstantHeadingInterpolation(Math.toRadians(-62.0))
            .build()
        paths += test1 //push plus shoot first
        paths += test2 //intake second spike plus push gate
        paths += test3 //shoot second spike
        paths += test4 //intake first spike
        paths += test5 //shoot first spike
        paths += test6 // go in front of third spike mark
        paths += test7 //intake third spike
        paths += test8 //shoot third spike
    }
}
