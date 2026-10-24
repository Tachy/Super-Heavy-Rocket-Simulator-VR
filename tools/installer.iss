; Installer of Super Heavy Rocket Simulator VR (Inno Setup 6), built by tools/package.ps1:
;   ISCC /DAppVersion=1.2.3 /DFileVersion=1.2.3.0 /DSourceDir=<build\package> /DTerrainDir=<build\part_terrain>
;        /DImageryDir=<build\part_imagery> /DOutputDir=<build> installer.iss
; Per user, no admin rights: %LOCALAPPDATA%\Programs\Super Heavy Rocket Simulator VR.
; Everything is inside (the repository is private, so the installer cannot fetch the data from the release); the
; Earth terrain and the aerial imagery are optional components. A newer installer installs over an older one (same
; AppId); the two data parts are removed first, so a component deselected later does not stay half there.

#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
#ifndef FileVersion
  #define FileVersion "0.0.0.0"
#endif
#ifndef SourceDir
  #define SourceDir "..\build\package"
#endif
#ifndef TerrainDir
  #define TerrainDir "..\build\part_terrain"
#endif
#ifndef ImageryDir
  #define ImageryDir "..\build\part_imagery"
#endif
#ifndef OutputDir
  #define OutputDir "..\build"
#endif
#define AppName "Super Heavy Rocket Simulator VR"
#define AppExe "SuperHeavyRocketSimulatorVR.exe"

[Setup]
AppId={{E3D12D35-3755-4AB8-9250-C39341ED3268}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher={#AppName}
AppPublisherURL=https://github.com/Tachy/Super-Heavy-Rocket-Simulator-VR
AppSupportURL=https://github.com/Tachy/Super-Heavy-Rocket-Simulator-VR/issues
AppUpdatesURL=https://github.com/Tachy/Super-Heavy-Rocket-Simulator-VR/releases
VersionInfoVersion={#FileVersion}
DefaultDirName={localappdata}\Programs\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
OutputDir={#OutputDir}
OutputBaseFilename=SuperHeavyRocketSimulatorVR-{#AppVersion}-setup
UninstallDisplayIcon={app}\{#AppExe}
UninstallDisplayName={#AppName}
Compression=lzma2
SolidCompression=yes
LZMAUseSeparateProcess=yes
WizardStyle=modern
CloseApplications=yes

[Languages]
Name: "german"; MessagesFile: "compiler:Languages\German.isl"
Name: "english"; MessagesFile: "compiler:Default.isl"

[CustomMessages]
german.DesktopMode=Desktop-Modus (ohne VR)
english.DesktopMode=desktop mode (no VR)
german.TypeFull=Vollständig
english.TypeFull=Full installation
german.TypeCustom=Benutzerdefiniert
english.TypeCustom=Custom installation
german.CompMain=Simulator mit Basisdaten (Ephemeriden, Geoid, Mondgelände, Texturen)
english.CompMain=Simulator with base data (ephemerides, geoid, lunar terrain, textures)
german.CompTerrain=Erdgelände um die Startplätze (hochauflösend; ohne: flacher Boden)
english.CompTerrain=Earth terrain around the launch sites (high resolution; without: flat ground)
german.CompImagery=Luftbilder der Startplätze (NAIP / Sentinel-2)
english.CompImagery=Aerial imagery of the launch sites (NAIP / Sentinel-2)

[Types]
Name: "full"; Description: "{cm:TypeFull}"
Name: "custom"; Description: "{cm:TypeCustom}"; Flags: iscustom

[Components]
Name: "main"; Description: "{cm:CompMain}"; Types: full custom; Flags: fixed
Name: "terrain"; Description: "{cm:CompTerrain}"; Types: full
Name: "imagery"; Description: "{cm:CompImagery}"; Types: full

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[InstallDelete]
Type: filesandordirs; Name: "{app}\data\terrain\earth"
Type: filesandordirs; Name: "{app}\data\imagery"

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Components: main; Flags: ignoreversion recursesubdirs
Source: "{#TerrainDir}\*"; DestDir: "{app}"; Components: terrain; Flags: ignoreversion recursesubdirs
Source: "{#ImageryDir}\*"; DestDir: "{app}"; Components: imagery; Flags: ignoreversion recursesubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\Start-VR.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\{#AppExe}"; Flags: runminimized
Name: "{group}\{#AppName} ({cm:DesktopMode})"; Filename: "{app}\Start-Desktop.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\{#AppExe}"; Flags: runminimized
Name: "{group}\{cm:UninstallProgram,{#AppName}}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\Start-VR.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\{#AppExe}"; Flags: runminimized; Tasks: desktopicon

[Run]
Filename: "{app}\Start-Desktop.cmd"; WorkingDir: "{app}"; Description: "{cm:LaunchProgram,{#AppName} ({cm:DesktopMode})}"; Flags: nowait postinstall skipifsilent unchecked runminimized

