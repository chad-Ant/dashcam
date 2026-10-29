package com.dashcam.connector

import android.content.Context
import android.graphics.Bitmap
import android.os.Build
import android.os.SystemClock
import android.util.Base64
import androidx.test.core.app.ActivityScenario
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import kotlinx.coroutines.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import java.io.File

// Opt-in integration test against service.py --simulate on the development
// host. pairingBase64 is an ephemeral simulator link, never a car credential.
@RunWith(AndroidJUnit4::class)
class ConnectorInstrumentedTest {
    @Test fun pinnedTlsSimulatorAndAdaptiveDashboard() {
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val context = instrumentation.targetContext
        val encoded = InstrumentationRegistry.getArguments().getString("pairingBase64")
        org.junit.Assume.assumeTrue("Requires local TLS simulator", encoded != null)
        val profile = DeviceProfile.parse(String(Base64.decode(encoded, Base64.DEFAULT), Charsets.UTF_8))
        if (Build.VERSION.SDK_INT >= 37) {
            instrumentation.uiAutomation.executeShellCommand("pm grant ${context.packageName} android.permission.ACCESS_LOCAL_NETWORK").close()
            SystemClock.sleep(500)
        }
        val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main)
        lateinit var repository: MonitorRepository
        instrumentation.runOnMainSync {
            repository = MonitorRepository(context, scope)
            repository.select(profile)
            repository.setActive(true)
        }
        try {
            val until = SystemClock.elapsedRealtime() + 20000
            while (!repository.state.value.runtimeFresh && SystemClock.elapsedRealtime() < until) SystemClock.sleep(100)
            val state = repository.state.value
            assertTrue("Expected live pinned TLS data, got ${state.status}", state.runtimeFresh)
            assertEquals("SIMULATED", state.snapshot!!.vehicle!!.getString("speed_source"))
            assertEquals(profile.id, state.snapshot!!.json.getString("device_id"))
            instrumentation.runOnMainSync { repository.setActive(false) }
            assertFalse(repository.state.value.transportFresh)
            // A different pinned certificate must never receive a snapshot.
            instrumentation.runOnMainSync {
                repository.select(profile.copy(pin = "0".repeat(64)))
                repository.setActive(true)
            }
            val rejectionDeadline = SystemClock.elapsedRealtime() + 10000
            while (!repository.state.value.status.startsWith("Certificate") && SystemClock.elapsedRealtime() < rejectionDeadline) SystemClock.sleep(100)
            assertTrue(repository.state.value.status, repository.state.value.status.startsWith("Certificate"))
            assertFalse(repository.state.value.transportFresh)
        } finally {
            instrumentation.runOnMainSync { repository.close() }; scope.cancel()
        }
        ProfileStore(context).save(listOf(profile))
        ActivityScenario.launch(MainActivity::class.java).use {
            SystemClock.sleep(3000)
            val screenshot = instrumentation.uiAutomation.takeScreenshot()
            assertNotNull(screenshot)
            File(context.filesDir, "dashboard.png").outputStream().use { out -> screenshot.compress(Bitmap.CompressFormat.PNG, 100, out) }
            screenshot.recycle()
        }
    }
}
