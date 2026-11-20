package com.dashcam.connector

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.ln
import kotlin.math.log2
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sinh

// Pure slippy-map math for the Vehicle map (no Android types, unit tested).
// Positions are normalised Web Mercator: x = 0..1 west to east, y = 0..1 north
// to south, independent of zoom. A world at zoom z is tilePx * 2^z pixels wide.

/** A usable GNSS position; [headingDeg] is 0..360 clockwise from north, or null. */
data class GeoFix(val latitude: Double, val longitude: Double, val headingDeg: Double?)

/** One OSM tile; x is already wrapped into 0 until 2^zoom. */
data class TileKey(val zoom: Int, val x: Int, val y: Int) {
    val url: String get() = "https://tile.openstreetmap.org/$zoom/$x/$y.png"
    /** The tile one zoom level out that covers this one, or null at zoom 0. */
    val parent: TileKey? get() = if (zoom > 0) TileKey(zoom - 1, x / 2, y / 2) else null
}

/** A tile to draw at grid [column]/[row]; the column is not wrapped (world copies). */
data class TilePlacement(val key: TileKey, val column: Int, val row: Int)

data class MapView(val x: Double, val y: Double, val zoom: Double)

object TileMath {
    const val TILE_SIZE = 256
    const val MIN_ZOOM = 3
    const val MAX_ZOOM = 19
    const val DEFAULT_ZOOM = 16
    const val MAX_LATITUDE = 85.0511
    /** Hard bound on tiles per view, far above any phone or tablet viewport. */
    const val MAX_TILES = 48

    /** Null unless lat/lon are finite, inside Web Mercator, and not the 0,0 placeholder. */
    fun fix(latitude: Double?, longitude: Double?, headingDeg: Double?): GeoFix? {
        if (latitude == null || longitude == null || !latitude.isFinite() || !longitude.isFinite()) return null
        if (abs(latitude) > MAX_LATITUDE || abs(longitude) > 180.0) return null
        if (latitude == 0.0 && longitude == 0.0) return null
        val heading = headingDeg?.takeIf { it.isFinite() }?.let { ((it % 360.0) + 360.0) % 360.0 }
        return GeoFix(latitude, longitude, heading)
    }

    fun clampZoom(zoom: Double): Double =
        if (zoom.isNaN()) DEFAULT_ZOOM.toDouble() else zoom.coerceIn(MIN_ZOOM.toDouble(), MAX_ZOOM.toDouble())

    /** The integer tile level drawn for a (possibly fractional) zoom. */
    fun tileZoom(zoom: Double): Int = clampZoom(zoom).roundToInt()

    /** On-screen size of one tile of [tileZoom] at [zoom] (0.71..1.41 x tilePx). */
    fun tileSizeOnScreen(zoom: Double, tilePx: Double): Double =
        tilePx * 2.0.pow(clampZoom(zoom) - tileZoom(zoom))

    fun worldSize(zoom: Double, tilePx: Double): Double = tilePx * 2.0.pow(clampZoom(zoom))

    fun mercatorX(longitude: Double): Double = (longitude + 180.0) / 360.0

    fun mercatorY(latitude: Double): Double {
        val s = sin(latitude * PI / 180.0)
        return (0.5 - ln((1 + s) / (1 - s)) / (4 * PI)).coerceIn(0.0, 1.0)
    }

    fun longitude(x: Double): Double = x * 360.0 - 180.0

    fun latitude(y: Double): Double = atan(sinh(PI * (1 - 2 * y))) * 180.0 / PI

    fun wrapX(x: Double): Double = x - floor(x)

    fun clampY(y: Double): Double = y.coerceIn(0.0, 1.0)

    /**
     * Tiles intersecting a [width] x [height] px viewport centred on ([centerX], [centerY]),
     * plus [margin] tiles on each side, nearest to the centre first. Rows outside the world
     * are dropped; columns wrap around the antimeridian.
     */
    fun visibleTiles(centerX: Double, centerY: Double, zoom: Double, width: Double, height: Double,
                     tilePx: Double, margin: Int = 0): List<TilePlacement> {
        val z = tileZoom(zoom)
        val size = tileSizeOnScreen(zoom, tilePx)
        if (!(width > 0 && height > 0 && size > 0) || !centerX.isFinite() || !centerY.isFinite()) return emptyList()
        val n = 1 shl z
        val px = centerX * n
        val py = centerY * n
        val halfW = width / 2 / size
        val halfH = height / 2 / size
        val firstColumn = floor(px - halfW).toInt() - margin
        val lastColumn = ceil(px + halfW).toInt() - 1 + margin
        val firstRow = maxOf(0, floor(py - halfH).toInt() - margin)
        val lastRow = minOf(n - 1, ceil(py + halfH).toInt() - 1 + margin)
        val tiles = ArrayList<TilePlacement>()
        for (row in firstRow..lastRow) for (column in firstColumn..lastColumn) {
            tiles.add(TilePlacement(TileKey(z, Math.floorMod(column, n), row), column, row))
        }
        return tiles.sortedBy {
            val dx = it.column + 0.5 - px
            val dy = it.row + 0.5 - py
            dx * dx + dy * dy
        }.take(MAX_TILES)
    }

    /** Top-left pixel of a placed tile relative to the viewport's top-left corner. */
    fun tileOrigin(placement: TilePlacement, centerX: Double, centerY: Double, zoom: Double,
                   width: Double, height: Double, tilePx: Double): Pair<Double, Double> {
        val n = 1 shl placement.key.zoom
        val size = tileSizeOnScreen(zoom, tilePx)
        return Pair(width / 2 + (placement.column - centerX * n) * size,
                    height / 2 + (placement.row - centerY * n) * size)
    }

    /** Pixel offset of point ([x], [y]) from the viewport centre, using the nearest world copy. */
    fun screenOffset(x: Double, y: Double, centerX: Double, centerY: Double, zoom: Double,
                     tilePx: Double): Pair<Double, Double> {
        var dx = x - centerX
        dx -= floor(dx + 0.5)
        val world = worldSize(zoom, tilePx)
        return Pair(dx * world, (y - centerY) * world)
    }

    /**
     * One pinch/drag step: the world point under [focusX]/[focusY] (px from the viewport
     * centre, before the move) stays under the fingers while the zoom changes by [factor],
     * then the map follows the fingers by [panX]/[panY] px.
     */
    fun transform(centerX: Double, centerY: Double, zoom: Double, focusX: Double, focusY: Double,
                  panX: Double, panY: Double, factor: Double, tilePx: Double): MapView {
        val before = clampZoom(zoom)
        val after = if (factor > 0 && factor.isFinite()) clampZoom(before + log2(factor)) else before
        val scaleBefore = worldSize(before, tilePx)
        val scaleAfter = worldSize(after, tilePx)
        val worldX = centerX + focusX / scaleBefore
        val worldY = centerY + focusY / scaleBefore
        val x = worldX - (focusX + panX) / scaleAfter
        val y = worldY - (focusY + panY) / scaleAfter
        return MapView(wrapX(x), clampY(y), after)
    }

    /** A +/- button step: whole zoom levels, clamped. */
    fun stepZoom(zoom: Double, delta: Int): Double = clampZoom(clampZoom(zoom).roundToInt() + delta.toDouble())
}
