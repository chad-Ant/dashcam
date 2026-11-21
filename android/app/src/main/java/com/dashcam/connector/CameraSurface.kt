package com.dashcam.connector

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PixelFormat
import android.graphics.RectF
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import androidx.compose.runtime.Composable
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.viewinterop.AndroidView
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch

/**
 * The camera picture on its own SurfaceView layer: each new [pictures] value is drawn straight
 * into the surface on Dispatchers.Default, with no recomposition, layout or Compose draw.
 * The surface sits behind the window (default z-order), so the sticky header and anything else
 * above the card still covers it. It paints its own rounded corners in [background] (the card
 * colour) because a SurfaceView's hole is not reliably clipped to a rounded shape.
 * Composed only while a frame should be shown; null pictures draw the black box only.
 *
 * Hosted in a plain AndroidView rather than AndroidExternalSurface: in foundation 1.9.0
 * AndroidExternalSurface's view is reusable and wires its holder callback only in the factory, so
 * after the LazyColumn deactivates and reuses this item (a quick scroll away and back) the view
 * keeps calling the old, cancelled scope and the surface is never drawn again. With no onReset
 * this view is not reusable: a reused or reactivated slot discards it and runs the factory again
 * with the current composition's scope.
 */
@Composable
internal fun CameraSurface(pictures: StateFlow<Bitmap?>, background: Color, cornerRadius: Dp, modifier: Modifier = Modifier) {
    // Read in composition; the surface callbacks below run outside it.
    val backgroundArgb = background.toArgb()
    val radiusPx = with(LocalDensity.current) { cornerRadius.toPx() }
    val scope = rememberCoroutineScope()
    AndroidView(
        factory = { context ->
            CameraSurfaceView(context, CameraSurfaceCallback(pictures, scope, backgroundArgb, radiusPx)).apply {
                holder.setFormat(PixelFormat.OPAQUE)
                holder.addCallback(callback)
            }
        },
        modifier = modifier,
        onRelease = { view ->
            view.holder.removeCallback(view.callback)
            view.callback.stop()
        },
    )
}

private class CameraSurfaceView(context: Context, val callback: CameraSurfaceCallback) : SurfaceView(context)

/** Starts a renderer per surface. The holder callbacks and [stop] all run on the main thread. */
private class CameraSurfaceCallback(
    private val pictures: StateFlow<Bitmap?>,
    private val scope: CoroutineScope,
    private val backgroundArgb: Int,
    private val radiusPx: Float,
) : SurfaceHolder.Callback {
    private var renderer: CameraRenderer? = null
    private var job: Job? = null
    private var size: MutableStateFlow<IntSize>? = null

    override fun surfaceCreated(holder: SurfaceHolder) {
        stop()
        val created = CameraRenderer(holder.surface, backgroundArgb, radiusPx)
        val frame = holder.surfaceFrame
        val sizeFlow = MutableStateFlow(IntSize(frame.width(), frame.height()))
        renderer = created
        size = sizeFlow
        // One collector, so draws are sequential: one thread at a time touches the surface.
        // Both flows hold a value, so the newest picture is drawn as soon as the surface has a size.
        job = scope.launch(Dispatchers.Default) {
            try {
                combine(pictures, sizeFlow) { bitmap, current -> Pair(bitmap, current) }
                    .collect { pair -> created.draw(pair.first, pair.second) }
            } finally {
                created.destroy()
            }
        }
    }

    // A resize redraws the newest picture at the new size.
    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
        size?.value = IntSize(width, height)
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        stop()
    }

    // Destruction waits for a draw in progress, so the surface is never touched after it.
    fun stop() {
        renderer?.destroy()
        job?.cancel()
        renderer = null
        job = null
        size = null
    }
}

private class CameraRenderer(private val surface: Surface, private val backgroundArgb: Int, private val radiusPx: Float) {
    private val lock = Any()
    private var alive = true   // guarded by lock
    private val paint = Paint(Paint.FILTER_BITMAP_FLAG)
    private val clip = Path()
    private val bounds = RectF()
    private val target = RectF()
    private var clipWidth = -1
    private var clipHeight = -1

    fun destroy() {
        synchronized(lock) { alive = false }
    }

    // Fills the surface with the card colour, a black rounded box over it, and the picture
    // letterboxed inside that box.
    fun draw(bitmap: Bitmap?, size: IntSize) {
        synchronized(lock) {
            if (!alive || size.width <= 0 || size.height <= 0 || !surface.isValid) return
            // A surface released or locked elsewhere is skipped, never a crash.
            val canvas = try {
                surface.lockHardwareCanvas()
            } catch (_: IllegalArgumentException) {
                return
            } catch (_: IllegalStateException) {
                return
            } catch (_: Surface.OutOfResourcesException) {
                return
            }
            try {
                // The locked canvas has the surface buffer's real size.
                val width = canvas.width
                val height = canvas.height
                if (width != clipWidth || height != clipHeight) {
                    bounds.set(0f, 0f, width.toFloat(), height.toFloat())
                    clip.reset()
                    clip.addRoundRect(bounds, radiusPx, radiusPx, Path.Direction.CW)
                    clipWidth = width
                    clipHeight = height
                }
                canvas.drawColor(backgroundArgb)
                canvas.save()
                canvas.clipPath(clip)
                canvas.drawColor(android.graphics.Color.BLACK)
                if (bitmap != null) {
                    val fit = letterbox(bitmap.width, bitmap.height, width, height)
                    if (fit != null) {
                        target.set(fit.left, fit.top, fit.right, fit.bottom)
                        canvas.drawBitmap(bitmap, null, target, paint)
                    }
                }
                canvas.restore()
            } finally {
                try {
                    surface.unlockCanvasAndPost(canvas)
                } catch (_: IllegalArgumentException) {
                } catch (_: IllegalStateException) {
                }
            }
        }
    }
}
