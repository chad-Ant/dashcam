package com.dashcam.connector

import android.content.ContentValues
import android.content.Context
import android.net.Uri
import android.os.Environment
import android.provider.MediaStore
import androidx.annotation.RequiresApi
import java.io.File
import java.io.IOException
import java.io.OutputStream
import java.nio.file.Files
import java.nio.file.StandardCopyOption
import java.security.SecureRandom

// Saving a recording without ever losing the copy already saved (review 2026-11-27 #1). Each file is written
// under a temporary name first; only when every file of the segment is complete are they given their final
// names, replacing earlier copies. A dropped connection, a cancel or a full disk leaves the earlier copies as
// they were and removes the partial ones.

/** One file being written under a temporary name. */
interface StagedFile {
    /** Opened once; the caller closes it. */
    fun open(): OutputStream
    /** Gives the file its final name, replacing an earlier file of that name. */
    fun publish()
    /** Removes the temporary file; safe to call after a failure at any point. */
    fun discard()
}

interface DownloadTarget {
    /** Where the files end up, for the screen. */
    val where: String
    fun stage(name: String): StagedFile
}

/**
 * Writes each of [names] through [write] into a staged file, then publishes them all. Anything thrown
 * (cancellation included) discards every staged file and leaves the earlier copies untouched.
 */
fun saveStaged(target: DownloadTarget, names: List<String>, write: (String, OutputStream) -> Unit): String {
    val staged = ArrayList<StagedFile>()
    try {
        for (name in names) {
            val file = target.stage(name)
            staged += file
            file.open().use { write(name, it) }
        }
    } catch (e: Throwable) {
        staged.forEach { runCatching { it.discard() } }
        throw e
    }
    // Every file is complete. A failure now (rare: a rename) still never removes an old copy before its
    // replacement is ready; the files not yet published are discarded.
    for ((i, file) in staged.withIndex()) {
        try { file.publish() } catch (e: Throwable) {
            staged.drop(i).forEach { runCatching { it.discard() } }
            throw e
        }
    }
    return target.where
}

private val random = SecureRandom()

/** "x.mkv" -> "x.partial-3f9a1c.mkv": same extension, so the type and any MediaStore naming stay the same. */
internal fun stagingName(name: String): String {
    val dot = name.lastIndexOf('.').takeIf { it > 0 } ?: name.length
    val tag = (1..6).joinToString("") { "0123456789abcdef"[random.nextInt(16)].toString() }
    return name.substring(0, dot) + ".partial-" + tag + name.substring(dot)
}

/** A plain directory (Android 9: the app's own Download/Dashcam folder; also the JVM tests). */
class DirectoryTarget(private val dir: File) : DownloadTarget {
    override val where: String get() = dir.absolutePath

    override fun stage(name: String): StagedFile {
        dir.mkdirs()
        val temp = File(dir, stagingName(name))
        return object : StagedFile {
            override fun open(): OutputStream = temp.outputStream()
            override fun publish() {   // rename(2) within one directory: atomic, replaces the old file
                Files.move(temp.toPath(), File(dir, name).toPath(), StandardCopyOption.REPLACE_EXISTING,
                    StandardCopyOption.ATOMIC_MOVE)
            }
            override fun discard() { temp.delete() }
        }
    }
}

/**
 * Download/Dashcam through MediaStore (Android 10+). A staged file is a pending entry (IS_PENDING = 1, which
 * other apps do not see, and which the system removes if the app dies first) under a temporary name.
 * Publishing deletes this app's earlier entries of the final name, then renames the pending entry and clears
 * IS_PENDING. An earlier copy is therefore deleted only after its replacement is complete.
 */
@RequiresApi(29)
class MediaStoreTarget(private val context: Context) : DownloadTarget {
    private val folder = Environment.DIRECTORY_DOWNLOADS + "/Dashcam/"
    override val where = "Download/Dashcam"

    override fun stage(name: String): StagedFile {
        val resolver = context.contentResolver
        val values = ContentValues().apply {
            put(MediaStore.Downloads.DISPLAY_NAME, stagingName(name))
            // Only the video gets a type. MediaStore appends the extension of any type that does not match
            // the name ("text/plain" turned x.ass into x.ass.txt, which players do not pick up as captions);
            // without one it keeps the name exactly.
            if (name.endsWith(".mkv")) put(MediaStore.Downloads.MIME_TYPE, "video/x-matroska")
            put(MediaStore.Downloads.RELATIVE_PATH, folder)
            put(MediaStore.Downloads.IS_PENDING, 1)
        }
        val uri: Uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
            ?: throw IOException("Could not create the download")
        return object : StagedFile {
            override fun open(): OutputStream =
                resolver.openOutputStream(uri) ?: throw IOException("Could not write the download")

            override fun publish() {
                // A second download of a segment replaces this app's earlier copy instead of adding "name (1)".
                resolver.delete(MediaStore.Downloads.EXTERNAL_CONTENT_URI,
                    "${MediaStore.Downloads.RELATIVE_PATH} = ? AND ${MediaStore.Downloads.DISPLAY_NAME} = ? AND " +
                        "${MediaStore.Downloads._ID} != ?", arrayOf(folder, name, uri.lastPathSegment ?: ""))
                val done = ContentValues().apply {
                    put(MediaStore.Downloads.DISPLAY_NAME, name)
                    put(MediaStore.Downloads.IS_PENDING, 0)
                }
                if (resolver.update(uri, done, null, null) != 1) throw IOException("Could not finish the download")
            }

            override fun discard() { resolver.delete(uri, null, null) }
        }
    }
}
