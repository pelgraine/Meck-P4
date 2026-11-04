/*
 * MeckCli.h -- Companion CLI (MeshCore v1.18 companion command set)
 *
 * One command handler shared by:
 *   - Settings > Experimental Features > Companion CLI (on-device screen)
 *   - the MeshCore app's Command Line, command 66 (CMD_RUN_CLI_COMMAND),
 *     over the WiFi companion link
 *
 * A port of upstream MeshCore v1.18's companion MyMesh::handleCommand() and
 * CommonRadioPrefs::handleCommand(), by way of the Meck Watch port, with the
 * replies word for word, mapped onto P4NodePrefs.
 */
#pragma once

#include <stdint.h>

// Reply buffer size callers must provide (as upstream's reply_buf).
#define MECK_CLI_REPLY_SIZE 166

struct P4NodePrefs;

// Runs one command. Writes the reply into 'reply' (MECK_CLI_REPLY_SIZE
// bytes) and returns true if the command was recognised; false means
// "Unknown command" (the caller appends that text, as upstream does).
bool meck_cli_handle(const char* command, char* reply);

// Provided by meck_app.cpp: the saved settings and their save (NVS + SD).
P4NodePrefs* meck_cli_prefs();
void meck_cli_save_prefs();
