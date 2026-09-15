// NRSA desktop shell (Electron)
// -----------------------------------------------------------------------
// This file only WRAPS the existing NRSA prototype UI
// ("prototype/NRSA_RCC.html") into a native Windows desktop window, with
// an ETABS/SAP2000/AutoCAD-style splash (logo) screen shown while the app
// starts. Nothing inside prototype/, src/, tests/, data/, docs/ is
// modified by this file.
// -----------------------------------------------------------------------

const { app, BrowserWindow, Menu, shell, dialog, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs/promises');

const APP_NAME = 'NRSA';
const APP_TAGLINE = 'Nanometer RCC and Steel Analysis';
const APP_VERSION = app.getVersion();

let currentProjectFile = null;
let pendingProjectToOpen = null;
const RECENT_PROJECTS_FILE = () => path.join(app.getPath('userData'), 'recent-projects.json');

async function getRecentProjects() {
  try {
    const raw = await fs.readFile(RECENT_PROJECTS_FILE(), 'utf8');
    const items = JSON.parse(raw);
    return Array.isArray(items) ? items.slice(0, 20) : [];
  } catch (_) { return []; }
}

async function rememberRecentProject(filePath) {
  if (!filePath) return;
  let items = await getRecentProjects();
  let stat = null;
  try { stat = await fs.stat(filePath); } catch (_) {}
  const entry = { path:filePath, name:path.basename(filePath), openedAt:new Date().toISOString(), modifiedAt:stat ? stat.mtime.toISOString() : null };
  items = items.filter(x => x && x.path && path.resolve(x.path).toLowerCase() !== path.resolve(filePath).toLowerCase());
  items.unshift(entry); items = items.slice(0,20);
  try { await fs.writeFile(RECENT_PROJECTS_FILE(), JSON.stringify(items,null,2),'utf8'); } catch (_) {}
}

ipcMain.handle('nrsa:save-project', async (_event,{data,suggestedName,saveAs})=>{
  let target=currentProjectFile;
  if(saveAs || !target){
    const result=await dialog.showSaveDialog({title:'Save NRSA Project',defaultPath:`${(suggestedName||'project').replace(/[\\/:*?"<>|]+/g,'_')}.nrsa`,filters:[{name:'NRSA Project',extensions:['nrsa']}]});
    if(result.canceled||!result.filePath) return {canceled:true};
    target=result.filePath.toLowerCase().endsWith('.nrsa')?result.filePath:`${result.filePath}.nrsa`;
  }
  await fs.writeFile(target,data,'utf8'); currentProjectFile=target; await rememberRecentProject(target);
  return {canceled:false,path:target,name:path.basename(target)};
});

ipcMain.handle('nrsa:open-project',async()=>{
  const result=await dialog.showOpenDialog({title:'Open NRSA Project',properties:['openFile'],filters:[{name:'NRSA Project',extensions:['nrsa']}]});
  if(result.canceled||!result.filePaths[0]) return {canceled:true};
  const target=result.filePaths[0],data=await fs.readFile(target,'utf8');
  currentProjectFile=target; await rememberRecentProject(target);
  return {canceled:false,path:target,name:path.basename(target),data};
});

ipcMain.handle('nrsa:get-recent-projects',async()=>{
  const items=await getRecentProjects(),checked=[];
  for(const item of items){try{const stat=await fs.stat(item.path);checked.push({...item,exists:stat.isFile(),modifiedAt:stat.mtime.toISOString()});}catch(_){checked.push({...item,exists:false});}}
  return checked;
});

ipcMain.handle('nrsa:open-recent-project',async(_event,filePath)=>{
  if(!filePath) return {canceled:true};
  const target=String(filePath);
  if(path.extname(target).toLowerCase()!=='.nrsa') throw new Error('Only .nrsa project files can be opened.');
  const data=await fs.readFile(target,'utf8'); currentProjectFile=target; await rememberRecentProject(target);
  return {canceled:false,path:target,name:path.basename(target),data};
});

const ICON_ICO=path.join(__dirname,'..','build','icon.ico');
const ICON_PNG=path.join(__dirname,'..','build','icon.png');
const MAIN_HTML=path.join(__dirname,'..','prototype','NRSA_RCC.html');
const SPLASH_HTML=path.join(__dirname,'splash.html');
const MIN_SPLASH_MS=2200;

let splashWindow=null,mainWindow=null;

function findNrsaProjectArgument(args){
  if(!Array.isArray(args)) return null;
  const candidate=args.find(arg=>typeof arg==='string'&&/\.nrsa$/i.test(arg));
  return candidate?path.resolve(candidate):null;
}

async function openProjectFileFromWindows(filePath){
  if(!filePath||!mainWindow||mainWindow.isDestroyed()){pendingProjectToOpen=filePath||pendingProjectToOpen;return;}
  try{
    const target=path.resolve(filePath);
    if(path.extname(target).toLowerCase()!=='.nrsa') return;
    const data=await fs.readFile(target,'utf8'); currentProjectFile=target; await rememberRecentProject(target);
    mainWindow.webContents.send('nrsa:open-project-file',{canceled:false,path:target,name:path.basename(target),data});
  }catch(err){dialog.showErrorBox('NRSA Project',`Could not open the project file.\n\n${err.message||err}`);}
}

const gotSingleInstanceLock=app.requestSingleInstanceLock();
if(!gotSingleInstanceLock){app.quit();}else{
  app.on('second-instance',(_event,commandLine)=>{
    if(mainWindow){if(mainWindow.isMinimized()) mainWindow.restore();mainWindow.focus();}
    const filePath=findNrsaProjectArgument(commandLine); if(filePath) openProjectFileFromWindows(filePath);
  });
}

function createSplashWindow(){
  splashWindow=new BrowserWindow({width:560,height:420,frame:false,resizable:false,movable:false,transparent:true,backgroundColor:'#00000000',alwaysOnTop:true,center:true,show:false,skipTaskbar:true,icon:ICON_ICO,webPreferences:{contextIsolation:true,nodeIntegration:false,sandbox:true}});
  splashWindow.loadFile(SPLASH_HTML,{query:{name:APP_NAME,tagline:APP_TAGLINE,version:APP_VERSION}});
  splashWindow.once('ready-to-show',()=>splashWindow.show());
  splashWindow.on('closed',()=>{splashWindow=null;});
}

function createMainWindow(){
  mainWindow=new BrowserWindow({width:1440,height:900,minWidth:1024,minHeight:640,show:false,backgroundColor:'#0b1220',title:`${APP_NAME} — ${APP_TAGLINE}`,icon:ICON_ICO,autoHideMenuBar:true,webPreferences:{contextIsolation:true,nodeIntegration:false,sandbox:false,spellcheck:false,preload:path.join(__dirname,'preload.js')}});
  mainWindow.maximize(); mainWindow.loadFile(MAIN_HTML);
  mainWindow.webContents.setWindowOpenHandler(({url})=>{
    if(url&&/^https?:\/\//i.test(url)){shell.openExternal(url);return {action:'deny'};}
    return {action:'allow',overrideBrowserWindowOptions:{autoHideMenuBar:true,icon:ICON_ICO}};
  });
  const splashStartedAt=Date.now();
  const revealMainWindow=()=>{const elapsed=Date.now()-splashStartedAt,remaining=Math.max(MIN_SPLASH_MS-elapsed,0);setTimeout(()=>{if(splashWindow&&!splashWindow.isDestroyed()) splashWindow.close();if(mainWindow&&!mainWindow.isDestroyed()){mainWindow.show();mainWindow.focus();}},remaining);};
  mainWindow.webContents.once('did-finish-load',()=>{revealMainWindow();if(pendingProjectToOpen){const filePath=pendingProjectToOpen;pendingProjectToOpen=null;setTimeout(()=>openProjectFileFromWindows(filePath),50);}});
  mainWindow.webContents.once('did-fail-load',revealMainWindow);
  mainWindow.on('closed',()=>{mainWindow=null;});
}

function buildApplicationMenu(){
  const template=[{label:'File',submenu:[
    {label:'Open Project…',accelerator:'CommandOrControl+O',click:async()=>{if(!mainWindow||mainWindow.isDestroyed())return;try{const result=await dialog.showOpenDialog(mainWindow,{title:'Open NRSA Project',properties:['openFile'],filters:[{name:'NRSA Project',extensions:['nrsa']}]});if(result.canceled||!result.filePaths[0])return;await openProjectFileFromWindows(result.filePaths[0]);}catch(err){dialog.showErrorBox('NRSA Project',`Could not open the project file.\n\n${err.message||err}`);}}},
    {label:'Save Project',accelerator:'CommandOrControl+S',click:()=>{if(mainWindow&&!mainWindow.isDestroyed())mainWindow.webContents.send('nrsa:menu-save-project');}},
    {type:'separator'},{role:'quit',label:'Exit'}]},
    {label:'View',submenu:[{role:'reload'},{role:'forceReload'},{type:'separator'},{role:'resetZoom'},{role:'zoomIn'},{role:'zoomOut'},{type:'separator'},{role:'togglefullscreen'},{role:'toggleDevTools'}]},
    {label:'Help',submenu:[{label:`About ${APP_NAME}`,click:()=>{dialog.showMessageBox(mainWindow,{type:'info',icon:ICON_PNG,title:`About ${APP_NAME}`,message:APP_NAME,detail:`${APP_TAGLINE}\nVersion ${APP_VERSION}\n\nRCC • Steel • Foundation • Analysis • Drawings\nPowered by NRSA`});}}]}];
  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

app.whenReady().then(()=>{
  pendingProjectToOpen=findNrsaProjectArgument(process.argv);
  buildApplicationMenu(); createSplashWindow(); createMainWindow();
  app.on('activate',()=>{if(BrowserWindow.getAllWindows().length===0){createSplashWindow();createMainWindow();}});
});
app.on('window-all-closed',()=>{if(process.platform!=='darwin')app.quit();});
