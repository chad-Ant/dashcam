package com.dashcam.connector

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update

data class DiscoveredDevice(val id: String, val host: String, val port: Int)

@Suppress("DEPRECATION")
class Discovery(context: Context) {
    private val nsd = context.getSystemService(NsdManager::class.java)
    private val mutable = MutableStateFlow<List<DiscoveredDevice>>(emptyList())
    val devices = mutable.asStateFlow()
    private val message = MutableStateFlow("")
    val status = message.asStateFlow()
    private var listener: NsdManager.DiscoveryListener? = null
    @Volatile private var generation = 0
    @Synchronized fun start() {
        stop()
        val epoch = ++generation
        mutable.value = emptyList()
        message.value = "Looking for Jetsons on Wi-Fi…"
        val next = object : NsdManager.DiscoveryListener {
            override fun onDiscoveryStarted(type: String) {}
            override fun onDiscoveryStopped(type: String) { message.value = "Discovery stopped" }
            override fun onStartDiscoveryFailed(type: String, error: Int) { message.value = "Discovery unavailable. Use a pairing link."; stop() }
            override fun onStopDiscoveryFailed(type: String, error: Int) {}
            override fun onServiceLost(service: NsdServiceInfo) {
                val id = service.serviceName.substringBefore("._dashcam")
                mutable.update { entries -> entries.filterNot { it.id == id } }
            }
            override fun onServiceFound(service: NsdServiceInfo) {
                if (service.serviceType != "_dashcam._tcp.") return
                nsd.resolveService(service, object : NsdManager.ResolveListener {
                    override fun onResolveFailed(info: NsdServiceInfo, error: Int) { message.value = "A device could not be resolved. Manual pairing is available." }
                    override fun onServiceResolved(info: NsdServiceInfo) {
                        if (generation != epoch) return
                        val id = info.attributes["id"]?.toString(Charsets.UTF_8) ?: return
                        val host = info.host?.hostAddress ?: return
                        val item = DiscoveredDevice(id, host, info.port)
                        mutable.update { entries -> (entries.filterNot { it.id == id } + item).take(32) }
                        message.value = "${mutable.value.size} device(s) found. Pairing verifies device identity."
                    }
                })
            }
        }
        listener = next
        try { nsd.discoverServices("_dashcam._tcp.", NsdManager.PROTOCOL_DNS_SD, next) }
        catch (_: SecurityException) { message.value = "Allow local network access in Android settings"; listener = null }
        catch (_: IllegalArgumentException) { message.value = "Discovery unavailable. Use a pairing link."; listener = null }
    }
    @Synchronized fun stop() {
        generation++
        listener?.let { try { nsd.stopServiceDiscovery(it) } catch (_: IllegalArgumentException) {} }
        listener = null
    }
}
