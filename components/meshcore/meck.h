/*
 * meck.h — minimal public interface for Meck (MeshCore on T-Display P4).
 *
 * This is the ONLY header from the meshcore component that main.cpp should
 * include. It deliberately does not pull in MeshCore protocol headers
 * (Mesh.h, Dispatcher.h, etc.) because:
 *
 *   1. Those headers cascade -Wreorder errors into main's stricter compile
 *      flags.
 *   2. P4SX1262Radio.h declares extern globals (SX1262, etc.) that LilyGo's
 *      main.cpp also defines at file scope as `auto`, causing conflicting
 *      declarations.
 *
 * Internal meshcore code uses target.h (which does include the protocol
 * headers); main.cpp only ever sees this minimal interface.
 */

#pragma once

bool meck_radio_attach();
