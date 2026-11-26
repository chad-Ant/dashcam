package com.dashcam.connector

import android.app.Activity
import android.content.Context
import android.content.ContextWrapper
import android.content.pm.ActivityInfo
import android.view.ViewGroup
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import androidx.compose.ui.window.DialogProperties
import androidx.compose.ui.window.DialogWindowProvider
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat

/**
 * The camera on the whole screen: its own dialog window, landscape, system bars hidden (a swipe
 * from an edge shows them briefly). A tap on the picture shows or hides [controls]; Back or
 * "Exit fullscreen" calls [onExit]. [picture] fills the black area and letterboxes itself; [caption]
 * goes under the controls (its own composable, so a 2 Hz caption recomposes only itself).
 * Video gating is unchanged: the Live page and its camera card stay composed behind the dialog.
 */
@Composable
internal fun FullscreenCamera(onExit: () -> Unit, controls: @Composable RowScope.() -> Unit,
                              caption: @Composable ColumnScope.() -> Unit, picture: @Composable BoxScope.() -> Unit) {
    Dialog(onDismissRequest = onExit, properties = DialogProperties(
        usePlatformDefaultWidth = false, decorFitsSystemWindows = false, dismissOnClickOutside = false)) {
        val view = LocalView.current
        val activity = LocalContext.current.findActivity()
        DisposableEffect(view, activity) {
            val window = (view.parent as? DialogWindowProvider)?.window
            window?.setLayout(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT)
            window?.setDimAmount(0f)
            val insets = window?.let { WindowCompat.getInsetsController(it, it.decorView) }
            insets?.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            insets?.hide(WindowInsetsCompat.Type.systemBars())
            val orientation = activity?.requestedOrientation
            activity?.requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_SENSOR_LANDSCAPE
            onDispose {
                insets?.show(WindowInsetsCompat.Type.systemBars())
                if (activity != null && orientation != null) activity.requestedOrientation = orientation
            }
        }
        var showControls by remember { mutableStateOf(true) }
        Box(Modifier.fillMaxSize().background(Color.Black)
            .clickable(interactionSource = remember { MutableInteractionSource() }, indication = null) {
                showControls = !showControls
            }) {
            picture()
            if (showControls) Column(Modifier.align(Alignment.TopStart).fillMaxWidth()
                .background(Color.Black.copy(alpha = .55f)).safeDrawingPadding().padding(horizontal = 16.dp, vertical = 8.dp),
                verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                    controls()
                    Spacer(Modifier.weight(1f))
                    FilledTonalButton(onClick = onExit) { Text("Exit fullscreen") }
                }
                caption()
            }
        }
    }
}

/**
 * SD / HD. [hdAllowed] false (a cloud session) shows HD disabled; SD is then the only choice.
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
internal fun QualitySelector(quality: VideoQuality, hdAllowed: Boolean, onSelect: (VideoQuality) -> Unit) {
    SingleChoiceSegmentedButtonRow {
        VideoQuality.entries.forEachIndexed { index, option ->
            SegmentedButton(selected = quality == option, onClick = { onSelect(option) },
                enabled = option == VideoQuality.SD || hdAllowed,
                shape = SegmentedButtonDefaults.itemShape(index, VideoQuality.entries.size)) {
                Text(option.label)
            }
        }
    }
}

/** The fullscreen placeholder in place of the card's picture, so only one surface draws the frames. */
@Composable
internal fun FullscreenPlaceholder(modifier: Modifier = Modifier) {
    Box(modifier.widthIn(max = 720.dp).fillMaxWidth().aspectRatio(16f / 9f)
        .background(Color.Black, RoundedCornerShape(14.dp)), contentAlignment = Alignment.Center) {
        Text("Showing fullscreen", color = Color(0xFFA2B3C3))
    }
}

private tailrec fun Context.findActivity(): Activity? = when (this) {
    is Activity -> this
    is ContextWrapper -> baseContext.findActivity()
    else -> null
}
