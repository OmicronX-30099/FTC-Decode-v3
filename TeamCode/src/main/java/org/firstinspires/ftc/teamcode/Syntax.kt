@file:Suppress("UNCHECKED_CAST", "SameParameterValue", "PackageDirectoryMismatch")

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import org.yaml.snakeyaml.Yaml
import java.io.File
import java.io.FileInputStream
import kotlin.math.pow
import kotlin.math.round

fun main() {
    AutoManager.loadFile("format.yaml")
}

object AutoManager {
    val pathList: MutableMap<String, PathChain> = mutableMapOf()
    var startPoint: Pose = Pose()

    fun loadFile(fileName: String) {
        val data = readYAML(openFile(fileName))
        parsePathSet(data)
    }
    private fun parsePathSet(data: Map<String, Any>) {
        // Remove prev paths
        pathList.clear()
        // Get startpoint and save it
        startPoint = parsePose(data["startData"] as Map<String, Any>)
        // Get pathchains and for each one, parse it and save it
        val pathChains = data["pathChains"] as ArrayList<Map<String, Any>>
        for (pChain in pathChains) {
            val res = parsePathChain(pChain)
            pathList.put(res.first, res.second)
        }
    }
    private fun parsePathChain(data: Map<String, Any>): Pair<String, PathChain> {
        // Break down the data
        val paths = data["paths"] as ArrayList<Map<String, Any>>
        val pathName = data["name"] as String
        // Create the pathchain and add every path to it
        var pChain: PathBuilder = follower.pathBuilder()
        for (path in paths) {
            pChain = parsePath(path, pChain)
        }
        // Return the data as a pair
        return pathName to pChain.build()
    }
    private fun parsePath(data: Map<String, Any>, currPChain: PathBuilder): PathBuilder {
        // Get start, control, and end points and store in list while parsing them into Pose class
        val headingData = data["heading"] as Map<String, Any>
        val controlPoints = data["controlPoints"] as ArrayList<Map<String, Any>>
        val pathPoints: ArrayList<Pose> = arrayListOf(
            parsePose(data["startPoint"] as Map<String, Any>)
        )
        for (p in controlPoints) { pathPoints.add(parsePose(p)) }
        pathPoints.add(parsePose(data["endPoint"] as Map<String, Any>))

        when (controlPoints.size) {
            2 -> { currPChain.addPath(BezierLine(pathPoints[0], pathPoints[1])) }
            else -> { currPChain.addPath(BezierCurve(pathPoints)) }
        }

        when (headingData["type"]) {
            "tangential" -> {
                currPChain.setTangentHeadingInterpolation()
                if (headingData["reversed"] as Boolean) { currPChain.setReversed() }
            }
            "constant" -> {
                currPChain.setConstantHeadingInterpolation(
                    Math.toRadians((headingData["degrees"] as Number).toDouble())
                )
            }
            "linear" -> {
                currPChain
            }
        }

        return currPChain
    }
    private fun parsePose(data: Map<String, Any>)
        = Pose(
            roundTo((data["x"] as Number).toDouble(), 5),
            roundTo((data["y"] as Number).toDouble(), 5),
        )
    private fun roundTo(num: Double, digits: Int): Double
        = round(num * 10.0.pow(digits)) / 10.0.pow(digits)
    private fun openFile(fName: String): File
        = File("", fName)
    private fun readYAML(file: File): Map<String, Any> {
        val yamlReader = Yaml()
        return (FileInputStream(file).use {
            inputStream -> yamlReader.load(inputStream)
        })
    }
}