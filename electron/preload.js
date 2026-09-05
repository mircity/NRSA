// Intentionally minimal: the NRSA prototype UI is a self-contained
// browser-side application (HTML/CSS/JS + localStorage) and does not
// require any Node.js/Electron APIs to be exposed to it.
//
// This file exists as a safe extension point: if/when the C++ engine in
// src/ is wired up to the desktop UI (e.g. via a local HTTP/IPC bridge),
// expose that bridge here with contextBridge.exposeInMainWorld(...)
// rather than turning nodeIntegration on.

// const { contextBridge } = require('electron');
// contextBridge.exposeInMainWorld('nrsa', { /* ... */ });
