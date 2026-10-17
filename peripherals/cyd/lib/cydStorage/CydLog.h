#pragma once
// UART0 belongs to the binary Jetson display link. Never mix diagnostic text into it.
template<class... Args> inline void cydLog(const char*, Args...) {}
