// Unity build of the nudge physics engine (sources in third_party/nudge).
// Compiled as C23 in its own TU (MSVC: /std:clatest); game code talks to it
// through NudgeWorld and the C API in nudge.h.
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#undef small
#undef near
#undef far
#endif

#define CKIT_IMPLEMENTATION
#include "ckit.h"

#include "nudge.h"
#include "nudge.c"
