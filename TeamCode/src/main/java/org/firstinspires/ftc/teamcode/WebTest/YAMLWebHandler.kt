@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.WebTest

import android.content.Context
import android.content.res.AssetManager
import com.qualcomm.robotcore.util.WebHandlerManager
import fi.iki.elonen.NanoHTTPD
import fi.iki.elonen.NanoHTTPD.IHTTPSession
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
import java.io.File

object YAMLWebHandler {
    private val storageDir = File(AppUtil.FIRST_FOLDER, "yaml_files")

    @JvmStatic
    @WebHandlerRegistrar
    fun registerWebContent(context: Context, webManager: WebHandlerManager) {
        if (!storageDir.exists()) storageDir.mkdirs()

        webManager.register("/yaml") { session -> webIntf(session, context) }
        webManager.register("/yaml/api/fileList", WebHandler(::handleFileList))
        webManager.register("/yaml/api/load", WebHandler(::getFileContents))
        webManager.register("/yaml/api/save", WebHandler(::handleWrite))
        webManager.register("/yaml/api/delete", WebHandler(::handleDeletion))
    }

    private fun webIntf(session: IHTTPSession, c: Context): NanoHTTPD.Response {
        val androidAssetManager: AssetManager = c.assets
        val webHTML: String
            = androidAssetManager.open("web/index.html")
                .bufferedReader()
                .use { it.readText() }
                .trimIndent()

        return NanoHTTPD.newFixedLengthResponse(
            NanoHTTPD.Response.Status.OK,
            NanoHTTPD.MIME_HTML,
            webHTML
        )
    }

    private fun openFile(session: IHTTPSession): File? {
        val fileName: String? = session.parameters["file"]?.get(0) ?: return null
        return File(storageDir, fileName!!)
    }

    private fun handleFileList(session: IHTTPSession): NanoHTTPD.Response {
        val allFiles = storageDir.listFiles()?.map { it.name } ?: emptyList()
        val jsonArray =
            "[" +
            allFiles.joinToString(
                ", ",
            ) { "\"${it}\"" } +
            "]"

        return NanoHTTPD.newFixedLengthResponse(
            NanoHTTPD.Response.Status.OK,
            "application/json",
            jsonArray
        )
    }

    private fun getFileContents(session: IHTTPSession): NanoHTTPD.Response {
        val file = openFile(session) ?: return NanoHTTPD.newFixedLengthResponse("Bad Request")
        return if (file.exists()) {
            NanoHTTPD.newFixedLengthResponse(
                NanoHTTPD.Response.Status.OK,
                NanoHTTPD.MIME_PLAINTEXT,
                file.readText()
            )
        } else {
            NanoHTTPD.newFixedLengthResponse(
                NanoHTTPD.Response.Status.NOT_FOUND,
                NanoHTTPD.MIME_PLAINTEXT,
                ""
            )
        }
    }

    private fun handleWrite(session: IHTTPSession): NanoHTTPD.Response {
        val file = openFile(session) ?: return NanoHTTPD.newFixedLengthResponse("Bad Request")

        val dataMap = HashMap<String, String>()
        try {
            session.parseBody(dataMap)
        } catch (e: Exception) {
            e.printStackTrace()
        }
        val fileContents = dataMap["postData"] ?: ""

        file.writeText(fileContents)

        return NanoHTTPD.newFixedLengthResponse(
            NanoHTTPD.Response.Status.OK,
            NanoHTTPD.MIME_PLAINTEXT,
            "Success"
        )
    }

    private fun handleDeletion(session: IHTTPSession): NanoHTTPD.Response {
        val file = openFile(session) ?: return NanoHTTPD.newFixedLengthResponse("Bad Request")

        return if (file.exists()) {
            file.delete()
            NanoHTTPD.newFixedLengthResponse(
                NanoHTTPD.Response.Status.OK,
                NanoHTTPD.MIME_PLAINTEXT,
                "Success"
            )
        } else {
            NanoHTTPD.newFixedLengthResponse(
                NanoHTTPD.Response.Status.NOT_FOUND,
                NanoHTTPD.MIME_PLAINTEXT,
                ""
            )
        }
    }
    fun getRawData(fileName: String): String
        = File(storageDir, fileName).readText()
}
