package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test
import kotlin.math.floor

class TileMathTest {
    private val tile = TileMath.TILE_SIZE.toDouble()

    @Test fun fixRejectsUnusablePositions() {
        assertNull(TileMath.fix(null, 10.0, null))
        assertNull(TileMath.fix(10.0, null, null))
        assertNull(TileMath.fix(Double.NaN, 10.0, null))
        assertNull(TileMath.fix(10.0, Double.POSITIVE_INFINITY, null))
        assertNull(TileMath.fix(85.06, 10.0, null))
        assertNull(TileMath.fix(-85.06, 10.0, null))
        assertNull(TileMath.fix(10.0, 180.01, null))
        assertNull(TileMath.fix(10.0, -180.01, null))
        assertNull(TileMath.fix(0.0, 0.0, 90.0))
        assertNotNull(TileMath.fix(0.0, 10.0, null))
        assertNotNull(TileMath.fix(85.0511, 180.0, null))
        assertNotNull(TileMath.fix(-85.0511, -180.0, null))
    }

    @Test fun headingIsNormalisedOrDropped() {
        assertEquals(350.0, TileMath.fix(1.0, 1.0, -10.0)!!.headingDeg!!, 1e-9)
        assertEquals(10.0, TileMath.fix(1.0, 1.0, 370.0)!!.headingDeg!!, 1e-9)
        assertNull(TileMath.fix(1.0, 1.0, Double.NaN)!!.headingDeg)
        assertNull(TileMath.fix(1.0, 1.0, null)!!.headingDeg)
    }

    @Test fun knownTilesMatchOsm() {
        // London (Trafalgar Square area) at z16 is OSM tile 16/32744/21792.
        val x = TileMath.mercatorX(-0.1278) * (1 shl 16)
        val y = TileMath.mercatorY(51.5074) * (1 shl 16)
        assertEquals(32744, floor(x).toInt())
        assertEquals(21792, floor(y).toInt())
        // Sydney at z12 is 12/3768/2457.
        assertEquals(3768, floor(TileMath.mercatorX(151.2093) * 4096).toInt())
        assertEquals(2457, floor(TileMath.mercatorY(-33.8688) * 4096).toInt())
        assertEquals(0.5, TileMath.mercatorX(0.0), 1e-12)
        assertEquals(0.5, TileMath.mercatorY(0.0), 1e-12)
        assertEquals("https://tile.openstreetmap.org/16/32744/21792.png", TileKey(16, 32744, 21792).url)
        assertEquals(TileKey(15, 16372, 10896), TileKey(16, 32744, 21792).parent)
    }

    @Test fun projectionRoundTrips() {
        for (lat in listOf(-85.0, -33.8688, 0.5, 51.5074, 85.0)) {
            assertEquals(lat, TileMath.latitude(TileMath.mercatorY(lat)), 1e-9)
        }
        for (lon in listOf(-180.0, -0.1278, 151.2093, 179.9)) {
            assertEquals(lon, TileMath.longitude(TileMath.mercatorX(lon)), 1e-9)
        }
    }

    @Test fun zoomIsClampedToOsmRange() {
        assertEquals(3.0, TileMath.clampZoom(0.0), 0.0)
        assertEquals(19.0, TileMath.clampZoom(25.0), 0.0)
        assertEquals(16.0, TileMath.clampZoom(Double.NaN), 0.0)
        assertEquals(12.4, TileMath.clampZoom(12.4), 0.0)
        assertEquals(12, TileMath.tileZoom(12.4))
        assertEquals(13, TileMath.tileZoom(12.6))
        assertEquals(19, TileMath.tileZoom(30.0))
        assertEquals(17.0, TileMath.stepZoom(16.3, 1), 0.0)
        assertEquals(19.0, TileMath.stepZoom(19.0, 1), 0.0)
        assertEquals(3.0, TileMath.stepZoom(3.2, -1), 0.0)
        // A fractional zoom draws tiles of the nearest level, scaled by at most sqrt(2).
        assertEquals(tile * Math.pow(2.0, 0.4), TileMath.tileSizeOnScreen(12.4, tile), 1e-9)
        assertEquals(tile * Math.pow(2.0, -0.4), TileMath.tileSizeOnScreen(12.6, tile), 1e-9)
    }

    @Test fun visibleTilesCoverOnlyTheViewport() {
        // Centre exactly on a tile corner at z16: a 512 x 512 viewport touches 2 x 2 tiles.
        val n = (1 shl 16).toDouble()
        val tiles = TileMath.visibleTiles(1000 / n, 2000 / n, 16.0, 512.0, 512.0, tile)
        assertEquals(setOf(TileKey(16, 999, 1999), TileKey(16, 1000, 1999), TileKey(16, 999, 2000),
            TileKey(16, 1000, 2000)), tiles.map { it.key }.toSet())
        // Off the corner by half a tile: 3 x 3, the centre tile first.
        val shifted = TileMath.visibleTiles(1000.5 / n, 2000.5 / n, 16.0, 512.0, 512.0, tile)
        assertEquals(9, shifted.size)
        assertEquals(TileKey(16, 1000, 2000), shifted.first().key)
        // A one-tile margin adds one ring.
        assertEquals(25, TileMath.visibleTiles(1000.5 / n, 2000.5 / n, 16.0, 512.0, 512.0, tile, margin = 1).size)
        // Nothing to show without a size.
        assertTrue(TileMath.visibleTiles(0.5, 0.5, 16.0, 0.0, 300.0, tile).isEmpty())
        assertTrue(TileMath.visibleTiles(Double.NaN, 0.5, 16.0, 300.0, 300.0, tile).isEmpty())
    }

    @Test fun visibleTilesWrapColumnsAndDropRowsOutsideTheWorld() {
        // Top-left corner of the world at z3: columns wrap to 7, rows above 0 are dropped.
        val tiles = TileMath.visibleTiles(0.0, 0.0, 3.0, 512.0, 512.0, tile)
        assertEquals(setOf(TileKey(3, 7, 0), TileKey(3, 0, 0)), tiles.map { it.key }.toSet())
        val wrapped = tiles.first { it.key.x == 7 }
        assertEquals(-1, wrapped.column)
        val (left, top) = TileMath.tileOrigin(wrapped, 0.0, 0.0, 3.0, 512.0, 512.0, tile)
        assertEquals(0.0, left, 1e-9)
        assertEquals(256.0, top, 1e-9)
        for (t in TileMath.visibleTiles(0.3, 0.7, 19.0, 4000.0, 4000.0, tile)) {
            assertTrue(t.key.x in 0 until (1 shl 19) && t.key.y in 0 until (1 shl 19))
        }
        assertTrue(TileMath.visibleTiles(0.3, 0.7, 19.0, 20000.0, 20000.0, tile).size <= TileMath.MAX_TILES)
    }

    @Test fun screenOffsetUsesTheNearestWorldCopy() {
        val world = TileMath.worldSize(3.0, tile)
        val (dx, dy) = TileMath.screenOffset(0.99, 0.5, 0.01, 0.5, 3.0, tile)
        assertEquals(-0.02 * world, dx, 1e-6)
        assertEquals(0.0, dy, 1e-9)
        val (ex, ey) = TileMath.screenOffset(0.6, 0.75, 0.5, 0.5, 3.0, tile)
        assertEquals(0.1 * world, ex, 1e-6)
        assertEquals(0.25 * world, ey, 1e-6)
    }

    @Test fun pinchKeepsThePointUnderTheFingers() {
        val cx = 0.512345
        val cy = 0.339876
        val zoom = 14.2
        val focusX = 120.0
        val focusY = -80.0
        val panX = 15.0
        val panY = -7.0
        val world = TileMath.worldSize(zoom, tile)
        val pointX = cx + focusX / world
        val pointY = cy + focusY / world
        val view = TileMath.transform(cx, cy, zoom, focusX, focusY, panX, panY, 1.5, tile)
        assertEquals(TileMath.clampZoom(zoom + Math.log(1.5) / Math.log(2.0)), view.zoom, 1e-9)
        val (dx, dy) = TileMath.screenOffset(pointX, pointY, view.x, view.y, view.zoom, tile)
        assertEquals(focusX + panX, dx, 1e-6)
        assertEquals(focusY + panY, dy, 1e-6)
    }

    @Test fun dragMovesTheMapAndClampsZoom() {
        val view = TileMath.transform(0.5, 0.5, 16.0, 0.0, 0.0, 256.0, 0.0, 1.0, tile)
        assertEquals(16.0, view.zoom, 0.0)
        // Dragging the map right by one tile moves the centre one tile west.
        assertEquals(0.5 - 1.0 / (1 shl 16), view.x, 1e-12)
        assertEquals(0.5, view.y, 0.0)
        assertEquals(19.0, TileMath.transform(0.5, 0.5, 18.8, 0.0, 0.0, 0.0, 0.0, 8.0, tile).zoom, 0.0)
        assertEquals(3.0, TileMath.transform(0.5, 0.5, 3.5, 0.0, 0.0, 0.0, 0.0, 0.1, tile).zoom, 0.0)
        assertEquals(10.0, TileMath.transform(0.5, 0.5, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, tile).zoom, 0.0)
        // The centre wraps across the antimeridian and stays inside the world vertically.
        val west = TileMath.transform(0.0001, 0.5, 3.0, 0.0, 0.0, 256.0, 0.0, 1.0, tile)
        assertTrue(west.x in 0.0..1.0 && west.x > 0.5)
        assertEquals(0.0, TileMath.transform(0.5, 0.001, 3.0, 0.0, 0.0, 0.0, 10000.0, 1.0, tile).y, 0.0)
    }
}
