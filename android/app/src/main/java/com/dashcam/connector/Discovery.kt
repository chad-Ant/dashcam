package com.dashcam.connector

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update

data class DiscoveredDevice(val id: String, val host: String, val port: Int)

/**
 * Results and status of the current scan. Every NSD callback carries the scan it belongs
 * to and changes nothing once a newer scan (or a stop) has replaced it: a late failure or
 * service-lost callback from an old scan must not stop or empty the newer one.
 */
internal class ScanResults {
    private val mutable = MutableStateFlow<List<DiscoveredDevice>>(emptyList())
    val devices: StateFlow<List<DiscoveredDevice>> = mutable.asStateFlow()
    private val message = MutableStateFlow("")
    val status: StateFlow<String> = message.asStateFlow()
    private var scan = 0

    /** Starts a new scan and returns its number. */
    @Synchronized fun begin(): Int {
        scan++
        mutable.value = emptyList()
        message.value = "Looking for Jetsons on Wi-Fi…"
        return scan
    }

    /** Ends the current scan, if any; its late callbacks are then ignored. */
    @Synchronized fun end(text: String?) {
        scan++
        if (text != null) message.value = text
    }

    @Synchronized fun isCurrent(of: Int) = of == scan

    /** Runs [change] only while [of] is the current scan; returns whether it ran. */
    @Synchronized fun ifCurrent(of: Int, change: () -> Unit): Boolean {
        if (of != scan) return false
        change()
        return true
    }

    fun found(of: Int, item: DiscoveredDevice) = ifCurrent(of) {
        mutable.update { entries -> (entries.filterNot { it.id == item.id } + item).take(32) }
        message.value = "${mutable.value.size} device(s) found. Pairing verifies device identity."
    }

    fun lost(of: Int, id: String) = ifCurrent(of) { mutable.update { entries -> entries.filterNot { it.id == id } } }

    fun say(of: Int, text: String) = ifCurrent(of) { message.value = text }
}

@Suppress("DEPRECATION")
class Discovery(context: Context) {
    private val nsd = context.getSystemService(NsdManager::class.java)
    private val results = ScanResults()
    val devices = results.devices
    val status = results.status
    private var listener: NsdManager.DiscoveryListener? = null

    @Synchronized fun start() {
        stop()
        val scan = results.begin()
        val next = object : NsdManager.DiscoveryListener {
            override fun onDiscoveryStarted(type: String) {}
            override fun onDiscoveryStopped(type: String) { results.say(scan, "Discovery stopped") }
            override fun onStartDiscoveryFailed(type: String, error: Int) {
                // NsdManager has already dropped a listener that failed to start; only this scan's ends.
                failed(scan, this)
            }
            override fun onStopDiscoveryFailed(type: String, error: Int) {}
            override fun onServiceLost(service: NsdServiceInfo) {
                results.lost(scan, service.serviceName.substringBefore("._dashcam"))
            }
            override fun onServiceFound(service: NsdServiceInfo) {
                if (service.serviceType != "_dashcam._tcp." || !results.isCurrent(scan)) return
                nsd.resolveService(service, object : NsdManager.ResolveListener {
                    override fun onResolveFailed(info: NsdServiceInfo, error: Int) {
                        results.say(scan, "A device could not be resolved. Manual pairing is available.")
                    }
                    override fun onServiceResolved(info: NsdServiceInfo) {
                        val id = info.attributes["id"]?.toString(Charsets.UTF_8) ?: return
                        val host = info.host?.hostAddress ?: return
                        results.found(scan, DiscoveredDevice(id, host, info.port))
                    }
                })
            }
        }
        listener = next
        try { nsd.discoverServices("_dashcam._tcp.", NsdManager.PROTOCOL_DNS_SD, next) }
        catch (_: SecurityException) { listener = null; results.end("Allow local network access in Android settings") }
        catch (_: IllegalArgumentException) { listener = null; results.end("Discovery unavailable. Use a pairing link.") }
    }

    @Synchronized private fun failed(scan: Int, from: NsdManager.DiscoveryListener) {
        if (listener !== from || !results.isCurrent(scan)) return
        listener = null
        results.end("Discovery unavailable. Use a pairing link.")
    }

    @Synchronized fun stop() {
        val active = listener ?: return
        listener = null
        results.end("Discovery stopped")
        try { nsd.stopServiceDiscovery(active) } catch (_: IllegalArgumentException) {}
    }
}
