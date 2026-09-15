const { contextBridge, ipcRenderer } = require('electron');

// Safe native file bridge for NRSA project files.
contextBridge.exposeInMainWorld('nrsaFile', {
  saveProject: (data, suggestedName, saveAs) =>
    ipcRenderer.invoke('nrsa:save-project', { data, suggestedName, saveAs }),
  openProject: () => ipcRenderer.invoke('nrsa:open-project'),
  getRecentProjects: () => ipcRenderer.invoke('nrsa:get-recent-projects'),
  openRecentProject: (filePath) => ipcRenderer.invoke('nrsa:open-recent-project', filePath),
  onMenuOpenProject: (callback) => {
    if (typeof callback !== 'function') return () => {};
    const handler = () => callback();
    ipcRenderer.on('nrsa:menu-open-project', handler);
    return () => ipcRenderer.removeListener('nrsa:menu-open-project', handler);
  },
  onMenuSaveProject: (callback) => {
    if (typeof callback !== 'function') return () => {};
    const handler = () => callback();
    ipcRenderer.on('nrsa:menu-save-project', handler);
    return () => ipcRenderer.removeListener('nrsa:menu-save-project', handler);
  },
  onOpenProjectFile: (callback) => {
    if (typeof callback !== 'function') return () => {};
    const handler = (_event, result) => callback(result);
    ipcRenderer.on('nrsa:open-project-file', handler);
    return () => ipcRenderer.removeListener('nrsa:open-project-file', handler);
  }
});
