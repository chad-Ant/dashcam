package com.dashcam.connector

import org.junit.Assert.*
import org.junit.Test

class VideoPolicyTest {
    private fun assertRect(expected: FitRect, actual: FitRect?) {
        assertNotNull(actual)
        val fit = actual!!
        assertEquals(expected.left, fit.left, 1e-3f)
        assertEquals(expected.top, fit.top, 1e-3f)
        assertEquals(expected.right, fit.right, 1e-3f)
        assertEquals(expected.bottom, fit.bottom, 1e-3f)
    }

    @Test fun sameAspectFillsTheSurface() {
        assertRect(FitRect(0f, 0f, 1280f, 720f), letterbox(640, 360, 1280, 720))
        assertRect(FitRect(0f, 0f, 640f, 360f), letterbox(640, 360, 640, 360))
        // Downscale too.
        assertRect(FitRect(0f, 0f, 320f, 180f), letterbox(1920, 1080, 320, 180))
    }

    @Test fun widerSourceIsLetterboxedTopAndBottom() {
        // 2:1 into 16:9 (1600x900): full width, 800 high, 50 px bars.
        assertRect(FitRect(0f, 50f, 1600f, 850f), letterbox(1000, 500, 1600, 900))
    }

    @Test fun tallerSourceIsPillarboxedLeftAndRight() {
        // 4:3 into 16:9 (1600x900): full height, 1200 wide, 200 px bars.
        assertRect(FitRect(200f, 0f, 1400f, 900f), letterbox(640, 480, 1600, 900))
        // Portrait 9:16 into 1600x900: 506.25 wide, centred.
        assertRect(FitRect(546.875f, 0f, 1053.125f, 900f), letterbox(360, 640, 1600, 900))
    }

    @Test fun nearlyMatchingAspectStaysInsideAndCentred() {
        // A 1080x608 surface (rounded 16:9) with a 640x360 picture: width-limited, 0.25 px bars.
        val fit = letterbox(640, 360, 1080, 608)!!
        assertEquals(0f, fit.left, 0f)
        assertEquals(1080f, fit.right, 0f)
        assertEquals(607.5f, fit.bottom - fit.top, 1e-3f)
        assertEquals(fit.top, 608f - fit.bottom, 1e-3f)
        assertTrue(fit.top >= 0f && fit.bottom <= 608f)
    }

    @Test fun limitingSideIsExactlyTheSurfaceEdge() {
        // Odd sizes that do not divide evenly never spill past the surface.
        for ((src, dst) in listOf(Pair(641 to 359, 1079 to 607), Pair(333 to 777, 1001 to 563),
                                  Pair(4096 to 3, 1000 to 1000), Pair(3 to 4096, 1000 to 1000))) {
            val fit = letterbox(src.first, src.second, dst.first, dst.second)!!
            assertTrue(fit.left >= 0f && fit.top >= 0f)
            assertTrue(fit.right <= dst.first.toFloat() && fit.bottom <= dst.second.toFloat())
            assertTrue(fit.right - fit.left > 0f && fit.bottom - fit.top > 0f)
            assertTrue((fit.left == 0f && fit.right == dst.first.toFloat()) ||
                       (fit.top == 0f && fit.bottom == dst.second.toFloat()))
        }
    }

    @Test fun invalidSizesGiveNoRectangle() {
        assertNull(letterbox(0, 360, 1280, 720))
        assertNull(letterbox(640, 0, 1280, 720))
        assertNull(letterbox(640, 360, 0, 720))
        assertNull(letterbox(640, 360, 1280, 0))
        assertNull(letterbox(-640, 360, 1280, 720))
        assertNull(letterbox(640, 360, 1280, -1))
    }

    @Test fun hugeSizesDoNotOverflow() {
        assertRect(FitRect(0f, 0f, 1f, 1f), letterbox(Int.MAX_VALUE, Int.MAX_VALUE, 1, 1))
        val fit = letterbox(Int.MAX_VALUE, 1, 1000, 1000)!!
        assertEquals(1000f, fit.right - fit.left, 0f)
        assertTrue(fit.bottom - fit.top < 1e-3f)
    }

    @Test fun firstFrameAfterAConnectIsPublishedAtOnce() {
        // Every connection change clears VideoState.frame.
        assertTrue(publishFrameNow(null, 0))
        assertTrue(publishFrameNow(null, 123_456))
    }

    @Test fun laterFramesWaitForTheTick() {
        assertFalse(publishFrameNow(10_000, 10_000))
        assertFalse(publishFrameNow(10_000, 10_100))
        assertFalse(publishFrameNow(10_000, 10_000 + VIDEO_STALE_MS))
    }

    @Test fun firstFrameAfterAStallIsPublishedAtOnce() {
        // VideoState shows "Video stale" for that frame: the new one replaces it without waiting.
        assertTrue(publishFrameNow(10_000, 10_000 + VIDEO_STALE_MS + 1))
        assertTrue(publishFrameNow(10_000, 60_000))
    }

    @Test fun hdFramesDecodeAtFullSizeAndLargerOnesAreSubsampled() {
        assertEquals(1, decodeSampleSize(640, 360))
        assertEquals(1, decodeSampleSize(1920, 1080))   // HD is not halved (the old 1280 limit did)
        assertEquals(1, decodeSampleSize(1080, 1920))
        assertEquals(2, decodeSampleSize(1921, 1080))
        assertEquals(2, decodeSampleSize(3840, 2160))
        assertEquals(4, decodeSampleSize(4096, 4096, maxPx = 1024))
    }

    @Test fun qualityQueryAndCloseCode() {
        assertNull(VideoQuality.SD.query)   // the unchanged /v1/video route, which older agents serve
        assertEquals("hd", VideoQuality.HD.query)
        assertEquals(4001, HD_UNAVAILABLE_CLOSE_CODE)
    }

    @Test fun tickKeepsStalenessWithinTwoAndAHalfSeconds() {
        // The tick period plus the stale limit bounds how long a picture outlives its last frame.
        assertTrue(VIDEO_METADATA_TICK_MS + VIDEO_STALE_MS <= 2500)
    }
}
