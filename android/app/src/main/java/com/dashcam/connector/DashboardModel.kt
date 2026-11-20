package com.dashcam.connector

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class DashboardModel(app: Application) : AndroidViewModel(app) {
    private val store = ProfileStore(app)
    val error = MutableStateFlow("")
    private val saved = MutableStateFlow(try { store.load() } catch (_: Exception) {
        error.value = "Saved credentials could not be decrypted. Pair the Jetson again."
        emptyList()
    })
    val profiles = saved.asStateFlow()
    val selected = MutableStateFlow(saved.value.firstOrNull()?.id)
    val repository = MonitorRepository(app, viewModelScope)
    val discovery = Discovery(app)
    // Vehicle map: view state and last known position in memory only, OSM tiles.
    val map = MapViewState()
    val tiles = TileLoader(app, viewModelScope)
    init {
        repository.select(saved.value.firstOrNull())
        viewModelScope.launch {
            // Same freshness rule as the cards: vehicle values only while runtime
            // and bridge are fresh (Snapshot.vehicle), else no current fix.
            repository.state.collect { state ->
                val vehicle = state.snapshot?.vehicle?.takeIf { state.runtimeFresh }
                map.update(TileMath.fix(vehicle.finite("latitude"), vehicle.finite("longitude"),
                    vehicle.finite("heading_deg")))
            }
        }
    }
    fun pair(link: String): Boolean {
        val profile = try {
            DeviceProfile.parse(link)
        } catch (e: IllegalArgumentException) {
            // A bad link is not a storage problem: say what is wrong with it.
            error.value = "Pairing link not accepted: ${e.message ?: "invalid link"}."
            return false
        }
        return try {
            val next = (saved.value.filterNot { it.id == profile.id } + profile).takeLast(8)
            store.save(next); saved.value = next; select(profile.id); error.value = ""
            true
        } catch (_: Exception) {
            error.value = "Could not save the pairing on this device (storage or keystore error)."
            false
        }
    }
    fun select(id: String) {
        if (selected.value != id) map.forget()
        selected.value = id
        repository.select(saved.value.find { it.id == id })
    }
    fun forget(id: String) {
        try {
            val next = saved.value.filterNot { it.id == id }
            store.save(next); saved.value = next
            // Removing another device must not switch or reconnect the one being watched.
            if (selected.value != id) return
            map.forget()   // the last known position belonged to the removed device
            selected.value = next.firstOrNull()?.id
            repository.select(next.firstOrNull())
        } catch (_: Exception) { error.value = "Could not remove saved device" }
    }
    fun updateAddress(device: DiscoveredDevice) {
        val paired = saved.value.find { it.id == device.id } ?: return
        try {
            val next = saved.value.map { if (it.id == device.id) paired.copy(host = device.host, port = device.port) else it }
            store.save(next); saved.value = next; select(device.id)
        } catch (_: Exception) { error.value = "Could not update device address" }
    }
    override fun onCleared() { repository.close(); discovery.stop(); tiles.close() }
}
