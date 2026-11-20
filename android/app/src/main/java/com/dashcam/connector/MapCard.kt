package com.dashcam.connector

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.FilledTonalButton
import androidx.compose.material3.FilledTonalIconButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableDoubleStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.FilterQuality
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onSizeChanged
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalUriHandler
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.TextLayoutResult
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.text.style.TextDecoration
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.IntSize
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import kotlin.math.floor

private const val OSM_COPYRIGHT = "https://www.openstreetmap.org/copyright"
private val NoFixBackground = Color(0xFF22303D)
private val TilePlaceholder = Color(0xFFC9CED3)
private val LiveMarker = Color(0xFF1A73E8)
private val StaleMarker = Color(0xFF7D8790)

/**
 * Vehicle map view state, held by the ViewModel so it survives tab switches and rotation.
 * Client memory only: the last known position is never persisted.
 */
class MapViewState {
    /** The current fix: fresh vehicle data with a valid position, else null. */
    var live by mutableStateOf<GeoFix?>(null)
        private set
    /** The newest fix ever received from the selected Jetson. */
    var last by mutableStateOf<GeoFix?>(null)
        private set
    var zoom by mutableDoubleStateOf(TileMath.DEFAULT_ZOOM.toDouble())
        private set
    /** Follow mode: the map centres on the car. Any pan/pinch gesture leaves it. */
    var following by mutableStateOf(true)
        private set
    private var freeX by mutableDoubleStateOf(0.5)
    private var freeY by mutableDoubleStateOf(0.5)

    fun update(fix: GeoFix?) {
        live = fix
        if (fix != null) last = fix
    }
    /** Another Jetson was selected: its car is not the one remembered here. */
    fun forget() { live = null; last = null; following = true }
    /** Map centre in normalised Mercator, or null before any fix (no map, no tiles). */
    fun center(): Pair<Double, Double>? {
        val fix = live ?: last ?: return null
        return if (following) Pair(TileMath.mercatorX(fix.longitude), TileMath.mercatorY(fix.latitude))
        else Pair(freeX, freeY)
    }
    fun gesture(focusX: Double, focusY: Double, panX: Double, panY: Double, factor: Double, tilePx: Double) {
        val (x, y) = center() ?: return
        val view = TileMath.transform(x, y, zoom, focusX, focusY, panX, panY, factor, tilePx)
        freeX = view.x; freeY = view.y; zoom = view.zoom; following = false
    }
    /** The +/- buttons zoom about the map centre and keep follow mode as it is. */
    fun zoomBy(delta: Int) { zoom = TileMath.stepZoom(zoom, delta) }
    /** Back to follow mode; the zoom is kept. */
    fun recentre() { following = true }
}

/**
 * OSM slippy map above the GNSS position card. Tiles are requested only while this card is
 * composed (Vehicle page, scrolled into the list) and the app is in the foreground, and never
 * before the first fix.
 */
@Composable
internal fun MapCard(map: MapViewState, tiles: TileLoader) = SurfaceCard("Map") {
    val center = map.center()
    val live = map.live
    val fix = live ?: map.last
    val zoom = map.zoom
    val following = map.following
    val tilePx = TileMath.TILE_SIZE * LocalDensity.current.density.toDouble()
    var mapSize by remember { mutableStateOf(IntSize.Zero) }
    val placements = remember(center, zoom, mapSize, tilePx) {
        if (center == null) emptyList()
        else TileMath.visibleTiles(center.first, center.second, zoom, mapSize.width.toDouble(),
            mapSize.height.toDouble(), tilePx)
    }
    val keys = remember(placements) { placements.map { it.key }.toSet() }
    // Lifecycle events arrive synchronously (recomposition stops in the background), so they
    // gate the loader: ON_STOP pauses every download and retry, ON_START resumes.
    val owner = LocalLifecycleOwner.current
    val currentKeys by rememberUpdatedState(keys)
    DisposableEffect(owner, tiles) {
        val observer = LifecycleEventObserver { _, event ->
            when (event) {
                Lifecycle.Event.ON_START -> tiles.request(currentKeys)
                Lifecycle.Event.ON_STOP -> tiles.pause()
                else -> Unit
            }
        }
        owner.lifecycle.addObserver(observer)   // replays ON_START if already started
        onDispose { owner.lifecycle.removeObserver(observer); tiles.pause() }
    }
    LaunchedEffect(keys) {
        if (owner.lifecycle.currentState.isAtLeast(Lifecycle.State.STARTED)) tiles.request(keys)
    }

    Box(Modifier.fillMaxWidth().height(300.dp).clip(RoundedCornerShape(14.dp))
        .background(if (center == null) NoFixBackground else TilePlaceholder)
        .onSizeChanged { mapSize = it }) {
        if (center == null || fix == null) {
            Text("No GNSS fix yet", Modifier.align(Alignment.Center), color = Color(0xFFA2B3C3),
                fontWeight = FontWeight.SemiBold)
        } else {
            val measurer = rememberTextMeasurer()
            val label = remember(measurer) {
                measurer.measure("Last known position",
                    style = TextStyle(color = Color(0xFF26323D), fontSize = 12.sp, fontWeight = FontWeight.Medium))
            }
            Canvas(Modifier.fillMaxSize()
                .semantics { contentDescription = if (live == null) "Map, last known car position" else "Map, car position" }
                .pointerInput(map, tilePx) {
                detectTransformGestures { centroid: Offset, pan: Offset, gestureZoom: Float, _: Float ->
                    map.gesture(centroid.x - size.width / 2.0, centroid.y - size.height / 2.0,
                        pan.x.toDouble(), pan.y.toDouble(), gestureZoom.toDouble(), tilePx)
                }
            }) {
                drawTiles(tiles, placements, center, zoom, tilePx)
                drawMarker(fix, stale = live == null, center, zoom, tilePx, label)
            }
            if (tiles.failing) Text("Map tiles need internet",
                Modifier.align(Alignment.TopStart).padding(8.dp)
                    .background(Color(0xE615212D), RoundedCornerShape(8.dp))
                    .padding(horizontal = 8.dp, vertical = 4.dp),
                color = Color(0xFFFFCA80), fontSize = 12.sp)
            Column(Modifier.align(Alignment.TopEnd).padding(8.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                MapButton("+", "Zoom in", zoom < TileMath.MAX_ZOOM) { map.zoomBy(1) }
                MapButton("−", "Zoom out", zoom > TileMath.MIN_ZOOM) { map.zoomBy(-1) }
            }
        }
        // One bottom-end column: the attribution keeps the bottom edge, Recentre sits above it.
        Column(Modifier.align(Alignment.BottomEnd), horizontalAlignment = Alignment.End) {
            if (center != null && fix != null && !following)
                FilledTonalButton(onClick = { map.recentre() }, modifier = Modifier.padding(8.dp)) { Text("Recentre") }
            Attribution()
        }
    }
    Hint(when {
        center == null -> "The map appears once the car reports a GNSS position. Tiles come from OpenStreetMap and need internet."
        live == null -> if (following) "Vehicle data stale · last known position" else "Vehicle data stale · free view"
        following -> "Following the car"
        else -> "Free view · Recentre to follow the car"
    })
}

@Composable private fun MapButton(symbol: String, description: String, enabled: Boolean, onClick: () -> Unit) {
    FilledTonalIconButton(onClick = onClick, enabled = enabled,
        modifier = Modifier.semantics { contentDescription = description }) {
        Text(symbol, fontSize = 20.sp, fontWeight = FontWeight.SemiBold)
    }
}

@Composable private fun Attribution(modifier: Modifier = Modifier) {
    val uriHandler = LocalUriHandler.current
    Text("© OpenStreetMap contributors",
        modifier.clip(RoundedCornerShape(topStart = 8.dp)).background(Color(0xE6FFFFFF))
            .clickable(onClickLabel = "Open the OpenStreetMap copyright page", role = Role.Button) {
                // No browser installed: nothing to open, the map keeps working.
                try { uriHandler.openUri(OSM_COPYRIGHT) } catch (_: Exception) { }
            }
            .padding(horizontal = 8.dp, vertical = 4.dp),
        color = Color(0xFF1A4F8B), fontSize = 11.sp, textDecoration = TextDecoration.Underline)
}

private fun DrawScope.drawTiles(tiles: TileLoader, placements: List<TilePlacement>,
                                center: Pair<Double, Double>, zoom: Double, tilePx: Double) {
    val width = size.width.toDouble()
    val height = size.height.toDouble()
    val tileSize = TileMath.tileSizeOnScreen(zoom, tilePx)
    for (placement in placements) {
        val (left, top) = TileMath.tileOrigin(placement, center.first, center.second, zoom, width, height, tilePx)
        // Snap both edges to whole pixels so neighbouring tiles never leave a seam.
        val x0 = floor(left).toInt()
        val y0 = floor(top).toInt()
        val dstSize = IntSize(floor(left + tileSize).toInt() - x0, floor(top + tileSize).toInt() - y0)
        val image = tiles.bitmap(placement.key)
        if (image != null) {
            drawImage(image, dstOffset = IntOffset(x0, y0), dstSize = dstSize, filterQuality = FilterQuality.Low)
            continue
        }
        // While a tile loads, show the matching quarter of its parent if that is in memory
        // (never fetched for this); otherwise the grey placeholder background stays.
        val parent = placement.key.parent?.let { tiles.bitmap(it) } ?: continue
        val halfW = parent.width / 2
        val halfH = parent.height / 2
        drawImage(parent, srcOffset = IntOffset((placement.key.x % 2) * halfW, (placement.key.y % 2) * halfH),
            srcSize = IntSize(halfW, halfH), dstOffset = IntOffset(x0, y0), dstSize = dstSize,
            filterQuality = FilterQuality.Low)
    }
}

private fun DrawScope.drawMarker(fix: GeoFix, stale: Boolean, center: Pair<Double, Double>, zoom: Double,
                                 tilePx: Double, label: TextLayoutResult) {
    val (dx, dy) = TileMath.screenOffset(TileMath.mercatorX(fix.longitude), TileMath.mercatorY(fix.latitude),
        center.first, center.second, zoom, tilePx)
    val position = Offset((size.width / 2 + dx).toFloat(), (size.height / 2 + dy).toFloat())
    val radius = 7.dp.toPx()
    val heading = fix.headingDeg
    if (!stale && heading != null) rotate(heading.toFloat(), position) {
        val arrow = Path().apply {
            moveTo(position.x, position.y - radius * 2.6f)
            lineTo(position.x + radius * 0.95f, position.y - radius * 0.6f)
            lineTo(position.x - radius * 0.95f, position.y - radius * 0.6f)
            close()
        }
        drawPath(arrow, LiveMarker)
        drawPath(arrow, Color.White, style = Stroke(width = 2.dp.toPx(), join = StrokeJoin.Round))
    }
    drawCircle(Color.White, radius = radius + 2.5.dp.toPx(), center = position)
    drawCircle(if (stale) StaleMarker else LiveMarker, radius = radius, center = position)
    if (!stale) return
    val inside = position.x in 0f..size.width && position.y in 0f..size.height
    if (!inside) return
    val margin = 4.dp.toPx()
    val pad = 6.dp.toPx()
    val boxW = label.size.width + pad * 2
    val boxH = label.size.height + pad
    val x = minOf(maxOf(position.x - boxW / 2, margin), size.width - boxW - margin)
    val y = position.y + radius + 6.dp.toPx()
    drawRoundRect(Color(0xE6FFFFFF), topLeft = Offset(x, y), size = Size(boxW, boxH),
        cornerRadius = CornerRadius(6.dp.toPx()))
    drawText(label, topLeft = Offset(x + pad, y + pad / 2))
}
