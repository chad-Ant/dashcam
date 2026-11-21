package com.dashcam.connector

// Plain Kotlin, no Android types: unit-tested on the JVM.

/** Where a picture goes inside a surface, in surface pixels. */
data class FitRect(val left: Float, val top: Float, val right: Float, val bottom: Float)

/**
 * The largest rectangle with the source's aspect ratio that fits the destination, centred
 * (letterboxed or pillarboxed). The limiting side is exactly the destination's, so the picture
 * never spills past the surface. Null when any size is not positive.
 */
fun letterbox(srcWidth: Int, srcHeight: Int, dstWidth: Int, dstHeight: Int): FitRect? {
    if (srcWidth <= 0 || srcHeight <= 0 || dstWidth <= 0 || dstHeight <= 0) return null
    // Compare aspect ratios exactly: srcW/srcH >= dstW/dstH  <=>  srcW*dstH >= srcH*dstW.
    val widthLimited = srcWidth.toLong() * dstHeight >= srcHeight.toLong() * dstWidth
    val width = if (widthLimited) dstWidth.toDouble() else srcWidth.toDouble() * dstHeight / srcHeight
    val height = if (widthLimited) srcHeight.toDouble() * dstWidth / srcWidth else dstHeight.toDouble()
    val left = (dstWidth - width) / 2
    val top = (dstHeight - height) / 2
    return FitRect(left.toFloat(), top.toFloat(), (left + width).toFloat(), (top + height).toFloat())
}

/** The clock tick that carries frame metadata into VideoState (and the caption) between immediate publishes. */
const val VIDEO_METADATA_TICK_MS = 500L

/**
 * Whether a decoded frame's metadata goes into VideoState at once instead of with the next clock
 * tick: when VideoState holds no current frame, i.e. the first frame after a (re)connect (every
 * connection change clears it) or the first after a stall long enough to count as stale. The
 * picture and caption then leave the placeholder without waiting for the tick.
 * [publishedReceivedAtMs]: receipt time of the frame VideoState holds, null when it holds none.
 */
fun publishFrameNow(publishedReceivedAtMs: Long?, nowMs: Long) =
    publishedReceivedAtMs == null || nowMs - publishedReceivedAtMs > VIDEO_STALE_MS
