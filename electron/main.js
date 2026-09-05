// NRSA desktop shell (Electron)
// -----------------------------------------------------------------------
// This file only WRAPS the existing NRSA prototype UI
// ("prototype/NRSA_RCC.html") into a native Windows desktop window, with
// an ETABS/SAP2000/AutoCAD-style splash (logo) screen shown while the app
// starts. Nothing inside prototype/, src/, tests/, data/, docs/ is
// modified by this file.
// -----------------------------------------------------------------------

const { app, BrowserWindow, Menu, shell, dialog } = require('electron');
const path = require('path');

const APP_NAME = 'NRSA';
const APP_TAGLINE = 'Structural Design & Analysis Software';
const APP_VERSION = app.getVersion();

const ICON_ICO = path.join(__dirname, '..', 'build', 'icon.ico');
const ICON_PNG = path.join(__dirname, '..', 'build', 'icon.png');
const MAIN_HTML = path.join(__dirname, '..', 'prototype', 'NRSA_RCC.html');
const SPLASH_HTML = path.join(__dirname, 'splash.html');

// Minimum time the splash screen stays visible, so it never just
// "flashes" on fast machines (matches the feel of ETABS / SAP2000 /
// AutoCAD splash screens).
const MIN_SPLASH_MS = 2200;

let splashWindow = null;
let mainWindow = null;

// Keep a single instance of the app running.
const gotSingleInstanceLock = app.requestSingleInstanceLock();
if (!gotSingleInstanceLock) {
  app.quit();
} else {
  app.on('second-instance', () => {
    if (mainWindow) {
      if (mainWindow.isMinimized()) mainWindow.restore();
      mainWindow.focus();
    }
  });
}

function createSplashWindow() {
  splashWindow = new BrowserWindow({
    width: 560,
    height: 420,
    frame: false,
    resizable: false,
    movable: false,
    transparent: true,
    backgroundColor: '#00000000',
    alwaysOnTop: true,
    center: true,
    show: false,
    skipTaskbar: true,
    icon: ICON_ICO,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true
    }
  });

  splashWindow.loadFile(SPLASH_HTML, {
    query: { name: APP_NAME, tagline: APP_TAGLINE, version: APP_VERSION }
  });

  splashWindow.once('ready-to-show', () => {
    splashWindow.show();
  });

  splashWindow.on('closed', () => {
    splashWindow = null;
  });
}

function createMainWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1024,
    minHeight: 640,
    show: false,
    backgroundColor: '#0b1220',
    title: `${APP_NAME} \u2014 ${APP_TAGLINE}`,
    icon: ICON_ICO,
    autoHideMenuBar: true,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: false, // the prototype UI relies on browser-side scripts/localStorage
      spellcheck: false,
      preload: path.join(__dirname, 'preload.js')
    }
  });

  mainWindow.maximize();
  mainWindow.loadFile(MAIN_HTML);

  // The app generates its print previews, 3D-print sheets and combined
  // reports by calling window.open("", "_blank") and then writing HTML
  // into the returned window before calling .print() on it. That must
  // keep behaving exactly like a normal browser popup — i.e. Electron
  // creates a real BrowserWindow for it — or every print/report/3D
  // print feature silently breaks. Only genuine external http/https
  // links (if any exist) are hardened here, keeping this consistent
  // with normal browser popup-blocker behavior.
  mainWindow.webContents.setWindowOpenHandler(({ url }) => {
    if (url && /^https?:\/\//i.test(url)) {
      shell.openExternal(url);
      return { action: 'deny' };
    }
    // Blank ("about:blank" / "") popups used by the app's own print and
    // report windows: let Electron open a real window for them.
    return {
      action: 'allow',
      overrideBrowserWindowOptions: {
        autoHideMenuBar: true,
        icon: ICON_ICO
      }
    };
  });

  const splashStartedAt = Date.now();

  const revealMainWindow = () => {
    const elapsed = Date.now() - splashStartedAt;
    const remaining = Math.max(MIN_SPLASH_MS - elapsed, 0);
    setTimeout(() => {
      if (splashWindow && !splashWindow.isDestroyed()) {
        splashWindow.close();
      }
      if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.show();
        mainWindow.focus();
      }
    }, remaining);
  };

  mainWindow.webContents.once('did-finish-load', revealMainWindow);
  // Safety net: if did-finish-load never fires for some reason, don't
  // leave the user staring at the splash screen forever.
  mainWindow.webContents.once('did-fail-load', revealMainWindow);

  mainWindow.on('closed', () => {
    mainWindow = null;
  });
}

function buildApplicationMenu() {
  const template = [
    {
      label: 'File',
      submenu: [{ role: 'quit', label: 'Exit' }]
    },
    {
      label: 'View',
      submenu: [
        { role: 'reload' },
        { role: 'forceReload' },
        { type: 'separator' },
        { role: 'resetZoom' },
        { role: 'zoomIn' },
        { role: 'zoomOut' },
        { type: 'separator' },
        { role: 'togglefullscreen' },
        { role: 'toggleDevTools' }
      ]
    },
    {
      label: 'Help',
      submenu: [
        {
          label: `About ${APP_NAME}`,
          click: () => {
            dialog.showMessageBox(mainWindow, {
              type: 'info',
              icon: ICON_PNG,
              title: `About ${APP_NAME}`,
              message: APP_NAME,
              detail: `${APP_TAGLINE}\nVersion ${APP_VERSION}\n\nRCC \u2022 Steel \u2022 Foundation \u2022 Analysis \u2022 Drawings\nPowered by NRSA`
            });
          }
        }
      ]
    }
  ];

  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

app.whenReady().then(() => {
  buildApplicationMenu();
  createSplashWindow();
  createMainWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createSplashWindow();
      createMainWindow();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
