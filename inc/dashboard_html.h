#pragma once
#include <pgmspace.h>

const char dashboard_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang=en class=dark>
<head>
<meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>NEXUS-4 Robot Dashboard</title>
<script src=https://cdn.tailwindcss.com></script>
<script>tailwind.config={darkMode:"class",theme:{extend:{colors:{robot:{dark:"#0f172a",card:"#1e293b",accent:"#3b82f6",success:"#10b981",warning:"#f59e0b",danger:"#ef4444"}},animation:{"pulse-fast":"pulse 1s cubic-bezier(0.4, 0, 0.6, 1) infinite"}}}}</script>
<script src=https://cdn.jsdelivr.net/npm/chart.js></script>
<script src=https://cdn.jsdelivr.net/npm/hammerjs@2.0.8></script>
<script src=https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2.0.1></script>
<script src=https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3.0.0></script>
<script src=https://unpkg.com/lucide@latest></script>
<script src=https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js></script>
<style>.scrollbar-thin::-webkit-scrollbar{width:6px;height:6px}.scrollbar-thin::-webkit-scrollbar-track{background:0 0}.scrollbar-thin::-webkit-scrollbar-thumb{background:#cbd5e1;border-radius:3px}.dark .scrollbar-thin::-webkit-scrollbar-thumb{background:#475569}.sensor-led{transition:all .1s ease-out}.status-dot{transition:all .15s ease-out}.glass-card{background-color:rgba(255,255,255,.7);backdrop-filter:blur(12px);border:1px solid rgba(0,0,0,.05);box-shadow:0 4px 6px -1px rgb(0 0 0 / .05),0 2px 4px -2px rgb(0 0 0 / .05)}.dark .glass-card{background-color:rgba(30,41,59,.7);border:1px solid rgba(255,255,255,.05);box-shadow:0 10px 15px -3px rgb(0 0 0 / .1),0 4px 6px -4px rgb(0 0 0 / .1)}.draggable.dragging{opacity:.5;border:2px dashed #3b82f6}.drop-zone{min-height:150px;transition:background-color .2s;border-radius:.75rem}.drop-zone.drag-over{background-color:rgba(59,130,246,.1);border:2px dashed rgba(59,130,246,.3)}.widget-header{cursor:grab}.widget-header:active{cursor:grabbing}.tab-drag-handle{cursor:grab}.tab-drag-handle:active{cursor:grabbing}.tab-btn.dragging{opacity:.5;background:#3b82f6!important}.widget-content{transition:all .3s ease-in-out;overflow:hidden}.collapsed .widget-content{max-height:0!important;padding-top:0!important;padding-bottom:0!important;opacity:0}.collapsed .collapse-icon{transform:rotate(-90deg)}.gutter-col{cursor:col-resize;background-color:transparent;transition:background-color .2s;width:16px;margin:0 -8px;z-index:20;position:relative}.gutter-col::after{content:'';position:absolute;left:50%;top:0;bottom:0;width:2px;background-color:rgba(148,163,184,.2);transition:all .2s}.dark .gutter-col::after{background-color:rgba(51,65,85,.5)}.gutter-col.active::after,.gutter-col:hover::after{background-color:#3b82f6!important;width:4px}body.is-resizing{cursor:col-resize!important;user-select:none}body.is-resizing .draggable,body.is-resizing .tab-btn,body.is-resizing iframe{pointer-events:none!important}.vertical-slider-container{display:flex;align-items:center;justify-content:center;height:180px;width:40px}.vertical-slider{width:180px;height:20px;transform:rotate(-90deg)}.toggle-checkbox:checked{right:0;border-color:#10b981}.toggle-checkbox:checked+.toggle-label{background-color:#10b981}.toggle-checkbox:checked+.toggle-label:after{transform:translateX(100%)}</style>
</head>
<body class="bg-slate-100 dark:bg-[#0a0f1d] text-slate-800 dark:text-slate-200 font-sans min-h-screen selection:bg-blue-500/30 transition-colors duration-300">
<nav class="border-b border-slate-200 dark:border-slate-800 bg-white/50 dark:bg-slate-900/50 backdrop-blur-lg sticky top-0 z-50 transition-colors duration-300">
<div class="container mx-auto px-4 py-3 flex justify-between items-center">
<div class="flex items-center space-x-3">
<i data-lucide=bot class="w-8 h-8 text-blue-500 dark:text-blue-400"></i>
<h1 class="text-xl font-bold tracking-tight text-slate-900 dark:text-white">NEXUS-4 <span class="text-slate-500 dark:text-slate-400 font-normal">Dashboard</span></h1>
</div>
<div class="flex items-center space-x-4">
<button id=btn-sim-toggle class="px-3 py-1 bg-yellow-500/10 text-yellow-600 dark:text-yellow-500 text-xs font-medium rounded-full border border-yellow-500/20 flex items-center hover:bg-yellow-500/20 transition-all" title="Toggle Simulation Mode">
<span class="relative flex h-2 w-2 mr-2">
<span id=sim-ping class="animate-ping absolute inline-flex h-full w-full rounded-full bg-yellow-400 opacity-75"></span>
<span class="relative inline-flex rounded-full h-2 w-2 bg-yellow-500"></span>
</span>
SIMULATION
</button>
<button id=btn-connect-toggle class="flex items-center px-3 py-1 rounded-full bg-red-500/10 text-red-600 dark:text-red-500 text-sm font-medium border border-red-500/20 transition-all hover:bg-red-500/20" title="Click to Reconnect">
<i id=conn-icon data-lucide=wifi-off class="w-4 h-4 mr-2"></i>
<span id=conn-text>Disconnected</span>
</button>
<button id=theme-toggle class="p-2 rounded-full bg-slate-200 dark:bg-slate-800 text-slate-600 dark:text-slate-300 hover:bg-slate-300 dark:hover:bg-slate-700 transition-colors">
<i data-lucide=sun class="w-5 h-5 hidden dark:block"></i>
<i data-lucide=moon class="w-5 h-5 block dark:hidden"></i>
</button>
</div>
</div>
</nav>
<main class="container mx-auto p-4 lg:p-6 max-w-[100%] xl:max-w-[1800px]">
<div id=main-layout class="flex flex-col xl:flex-row gap-6 xl:gap-0 items-stretch">
<div id=zone-1 class="col-zone flex-1 min-w-[300px] space-y-6 drop-zone xl:pr-4">
<div id=card-status class="glass-card rounded-xl p-5 shadow-sm dark:shadow-xl draggable" draggable=false>
<div class="widget-header flex justify-between items-center mb-4 p-2 -m-2 rounded-t-xl transition-colors hover:bg-slate-200/50 dark:hover:bg-slate-800/50">
<h2 class="text-lg font-semibold flex items-center text-slate-900 dark:text-white">
<i data-lucide=activity class="w-5 h-5 mr-2 text-blue-500 dark:text-blue-400"></i> Summary
</h2>
<div class="flex items-center space-x-2 text-slate-500 dark:text-slate-600">
<button class="collapse-trigger hover:text-slate-700 dark:hover:text-slate-300 transition-colors"><i data-lucide=chevron-down class="w-5 h-5 collapse-icon transition-transform"></i></button>
<i data-lucide=grip-vertical class="w-5 h-5 opacity-50"></i>
</div>
</div>
<div class="widget-content max-h-[600px]">
<div class=space-y-3>
<div class="flex justify-between items-center text-xs text-slate-500 dark:text-slate-400 px-1">
<span>Last Update:</span>
<span id=telemetry-timestamp class=font-mono>--:--:--.---</span>
</div>
<div class="flex space-x-2">
<div class="flex-1 bg-slate-200 dark:bg-slate-800 rounded-xl px-3 py-2 flex items-center justify-between">
<span class="text-[10px] font-bold text-slate-500 uppercase tracking-wider">State</span>
<span id=telemetry-state-pill class="text-xs font-black text-blue-500">IDLE</span>
</div>
<div id=obstacle-pill class="flex-1 bg-emerald-500/10 text-emerald-600 dark:text-emerald-400 rounded-xl px-3 py-2 text-xs font-bold flex items-center justify-center transition-colors">
<i data-lucide=shield-check class="w-4 h-4 mr-2"></i> CLEAR
</div>
</div>
<div class="grid grid-cols-4 gap-2">
<div class="bg-white/50 dark:bg-slate-900/50 px-2 py-2 rounded-lg border border-slate-200/50 dark:border-slate-800/50 text-center">
<span class="text-[9px] text-slate-400 uppercase block mb-1">Speed</span>
<span id=telemetry-speed class="font-mono font-bold text-sm block">0.00</span>
</div>
<div class="bg-white/50 dark:bg-slate-900/50 px-2 py-2 rounded-lg border border-slate-200/50 dark:border-slate-800/50 text-center">
<span class="text-[9px] text-slate-400 uppercase block mb-1">PWM</span>
<span id=telemetry-pwm class="font-mono font-bold text-sm block">0</span>
</div>
<div class="bg-white/50 dark:bg-slate-900/50 px-2 py-2 rounded-lg border border-slate-200/50 dark:border-slate-800/50 text-center">
<span class="text-[9px] text-slate-400 uppercase block mb-1">Sonar</span>
<span id=telemetry-sonar class="font-mono font-bold text-sm block">0</span>
</div>
<div class="bg-white/50 dark:bg-slate-900/50 px-2 py-2 rounded-lg border border-slate-200/50 dark:border-slate-800/50 text-center">
<span class="text-[9px] text-slate-400 uppercase block mb-1">Servo</span>
<span id=telemetry-servo class="font-mono font-bold text-sm block">90</span>
</div>
</div>
<div class="flex space-x-2">
<div class="flex-1 bg-white/50 dark:bg-slate-900/50 rounded-xl px-3 py-2 flex items-center justify-between border border-red-500/10 dark:border-red-500/5">
<div class="text-[10px] font-bold text-slate-500 uppercase tracking-wider flex items-center"><i data-lucide=alert-triangle class="w-3 h-3 mr-1 opacity-50"></i> Errors</div>
<div class="flex space-x-2 text-xs font-mono">
<div class="flex items-center"><span class="w-1.5 h-1.5 rounded-full bg-red-500 mr-1"></span><span id=summary-e2 class="font-bold text-slate-700 dark:text-slate-300">0.0</span></div>
<div class="flex items-center"><span class="w-1.5 h-1.5 rounded-full bg-blue-500 mr-1"></span><span id=summary-e3 class="font-bold text-slate-700 dark:text-slate-300">0.0</span></div>
</div>
</div>
<div class="flex-1 bg-white/50 dark:bg-slate-900/50 rounded-xl px-3 py-2 flex items-center justify-between border border-purple-500/10 dark:border-purple-500/5">
<div class="text-[10px] font-bold text-slate-500 uppercase tracking-wider flex items-center"><i data-lucide=scan-eye class="w-3 h-3 mr-1 opacity-50"></i> IR</div>
<div class="flex items-center space-x-2">
<div id=status-ir-primary class="flex space-x-0.5"></div>
<div class="w-px h-3 bg-slate-300 dark:bg-slate-700"></div>
<div id=status-ir-secondary class="flex space-x-0.5"></div>
</div>
</div>
</div>
</div>
</div>
</div>
<div id=card-controls class="glass-card rounded-xl p-5 shadow-sm dark:shadow-xl relative overflow-hidden draggable" draggable=false>
<div class="absolute top-0 left-0 w-1 h-full bg-red-500"></div>
<div class="widget-header flex justify-between items-center mb-4 p-2 -m-2 rounded-t-xl transition-colors hover:bg-slate-200/50 dark:hover:bg-slate-800/50">
<h2 class="text-lg font-semibold flex items-center text-slate-900 dark:text-white">
<i data-lucide=zap class="w-5 h-5 mr-2 text-yellow-500 dark:text-yellow-400"></i> Controls
</h2>
<div class="flex items-center space-x-2 text-slate-500 dark:text-slate-600">
<button class="collapse-trigger hover:text-slate-700 dark:hover:text-slate-300 transition-colors"><i data-lucide=chevron-down class="w-5 h-5 collapse-icon transition-transform"></i></button>
<i data-lucide=grip-vertical class="w-5 h-5 opacity-50"></i>
</div>
</div>
<div class="widget-content max-h-[600px]">
<div class="flex items-stretch space-x-4 mb-6">
<button id=btn-emergency class="flex-1 group relative bg-red-600 hover:bg-red-500 text-white p-4 rounded-lg font-bold text-lg flex items-center justify-center transition-all duration-200 active:scale-[0.98] overflow-hidden">
<div class="absolute inset-0 bg-[url('data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNDAiIGhlaWdodD0iNDAiIHZpZXdCb3g9IjAgMCA0MCA0MCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48cGF0aCBkPSJNMCAwaDQwdjQwSDB6bTIwIDIwTDAgNDBtNDAgMEwyMCAyMCIgc3Ryb2tlPSIjZmZmIiBzdHJva2Utd2lkdGg9IjEuNSIgb3BhY2l0eT0iLjEiIGZpbGw9Im5vbmUiLz48L3N2Zz4=')] opacity-20"></div>
<i data-lucide=octagon-alert class="w-6 h-6 mr-3 animate-pulse-fast"></i>E-STOP
</button>
<div class="flex flex-col justify-center items-center bg-slate-100 dark:bg-slate-900/50 p-3 px-5 rounded-xl border border-slate-200 dark:border-slate-800">
<div class="flex items-center space-x-2 mb-1">
<span class="text-[10px] font-bold text-slate-500 dark:text-slate-400 uppercase">Manual</span>
<label for=mode-toggle class="relative flex items-center cursor-pointer">
<input type=checkbox id=mode-toggle class="sr-only peer">
<div class="w-11 h-6 bg-slate-300 peer-focus:outline-none rounded-full peer dark:bg-slate-700 peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all dark:border-gray-600 peer-checked:bg-emerald-500"></div>
</label>
<span class="text-[10px] font-bold text-slate-500 dark:text-slate-400 uppercase">Auto</span>
</div>
<span id=mode-status-text class="text-xs font-black text-slate-700 dark:text-slate-300">MANUAL MODE</span>
</div>
</div>
<div class="flex justify-between items-center px-1 h-[220px]">
<div class="flex flex-col items-center space-y-2 z-10">
<span class="text-[10px] font-mono text-slate-500 uppercase">Steer</span>
<span id=val-servo class="text-sm font-bold text-orange-500 font-mono mb-2">90°</span>
<div class="vertical-slider-container bg-slate-100 dark:bg-slate-900/50 rounded-full border border-slate-200 dark:border-slate-700 py-2">
<input id=slider-servo type=range min=0 max=180 value=90 class="vertical-slider accent-orange-500">
</div>
</div>
<div class="flex-1 h-full flex items-center justify-center relative mx-4">
<div id=joystick-zone class="w-full h-full max-w-[200px] max-h-[200px] relative rounded-full bg-slate-100/50 dark:bg-slate-900/30 border-2 border-dashed border-slate-300 dark:border-slate-700">
<div class="absolute inset-0 flex items-center justify-center pointer-events-none text-slate-300 dark:text-slate-700 opacity-50">
<i data-lucide=move class="w-12 h-12"></i>
</div>
</div>
</div>
<div class="flex flex-col items-center space-y-2 z-10">
<span class="text-[10px] font-mono text-slate-500 uppercase">Speed</span>
<span id=val-pwm class="text-sm font-bold text-blue-500 font-mono mb-2">0</span>
<div class="vertical-slider-container bg-slate-100 dark:bg-slate-900/50 rounded-full border border-slate-200 dark:border-slate-700 py-2">
<input id=slider-pwm type=range min=0 max=255 value=0 class="vertical-slider accent-blue-500">
</div>
</div>
</div>
</div>
</div>
</div>
<div class="gutter-col hidden xl:block" data-left=zone-1 data-right=zone-2></div>
<div id=zone-2 class="col-zone flex-1 min-w-[300px] space-y-6 drop-zone xl:px-4">
<div id=card-charts class="glass-card rounded-xl p-5 shadow-sm dark:shadow-xl draggable" draggable=false>
<div class="widget-header flex flex-col sm:flex-row justify-between items-center mb-4 border-b border-slate-200/50 dark:border-slate-800/50 pb-4 p-2 -m-2 rounded-t-xl transition-colors hover:bg-slate-200/50 dark:hover:bg-slate-800/50">
<div class="flex items-center mb-2 sm:mb-0 mr-auto space-x-4">
<h2 class="text-lg font-semibold flex items-center text-slate-900 dark:text-white">
<i data-lucide=activity class="w-5 h-5 mr-2 text-blue-500 dark:text-blue-400"></i> Telemetry
</h2>
<div class="flex items-center space-x-1">
<button id=btn-chart-hold class="p-1.5 rounded-md text-slate-500 hover:text-amber-500 hover:bg-amber-500/10 transition-colors" title="Hold/Pause Telemetry">
<i data-lucide=pause class="w-4 h-4"></i>
</button>
<button id=btn-chart-refresh class="p-1.5 rounded-md text-slate-500 hover:text-blue-500 hover:bg-blue-500/10 transition-colors" title="Clear Data & Refresh"><i data-lucide=refresh-cw class="w-4 h-4"></i></button>
<button id=btn-chart-reset class="p-1.5 rounded-md text-slate-500 hover:text-blue-500 hover:bg-blue-500/10 transition-colors" title="Reset Zoom"><i data-lucide=zoom-out class="w-4 h-4"></i></button>
</div>
</div>
<div id=tab-container class="flex space-x-1 bg-slate-200/50 dark:bg-slate-900/50 p-1 rounded-lg mr-4 transition-colors">
<button draggable=false data-tab=ultrasonic class="tab-btn active group px-3 py-1.5 text-xs font-medium rounded-md transition-all flex items-center space-x-1">
<span>Sonar</span>
</button>
<button draggable=false data-tab=servo class="tab-btn group px-3 py-1.5 text-xs font-medium rounded-md transition-all text-slate-500 dark:text-slate-400 hover:text-slate-700 dark:hover:text-slate-200 flex items-center space-x-1">
<span>Servo</span>
</button>
<button draggable=false data-tab=pwm class="tab-btn group px-3 py-1.5 text-xs font-medium rounded-md transition-all text-slate-500 dark:text-slate-400 hover:text-slate-700 dark:hover:text-slate-200 flex items-center space-x-1">
<span>Motors</span>
</button>
</div>
<div class="flex items-center space-x-2 text-slate-500 dark:text-slate-600">
<button class="collapse-trigger hover:text-slate-700 dark:hover:text-slate-300 transition-colors"><i data-lucide=chevron-down class="w-5 h-5 collapse-icon transition-transform"></i></button>
<i data-lucide=grip-vertical class="w-5 h-5 opacity-50"></i>
</div>
</div>
<div class=widget-content style=overflow:hidden;height:400px>
<div class="relative h-full w-full">
<div id=ultrasonic-container class="plot-container absolute inset-0 transition-opacity duration-300">
<canvas id=ultrasonic-plot></canvas>
<div class="absolute top-2 right-2 font-mono text-3xl font-bold text-blue-500/80 dark:text-blue-500/80 pointer-events-none"><span id=ultrasonic-live-val>0</span><span class="text-sm ml-1">mm</span></div>
</div>
<div id=servo-container class="plot-container absolute inset-0 transition-opacity duration-300 opacity-0 pointer-events-none">
<canvas id=servo-plot></canvas>
<div class="absolute top-2 right-2 font-mono text-3xl font-bold text-orange-500/80 pointer-events-none"><span id=servo-live-val>90</span><span class="text-sm ml-1">deg</span></div>
</div>
<div id=pwm-container class="plot-container absolute inset-0 transition-opacity duration-300 opacity-0 pointer-events-none">
<canvas id=pwm-plot></canvas>
<div class="absolute top-2 right-2 font-mono text-3xl font-bold text-emerald-500/80 pointer-events-none"><span id=pwm-live-val>0</span><span class="text-sm ml-1">pwm</span></div>
</div>
</div>
</div>
</div>
<div id=card-logs class="glass-card rounded-xl p-4 shadow-sm dark:shadow-xl draggable" draggable=false>
<div class="widget-header flex justify-between items-center mb-2 p-2 -m-2 rounded-t-xl transition-colors hover:bg-slate-200/50 dark:hover:bg-slate-800/50">
<h2 class="text-xs font-semibold text-slate-500 uppercase tracking-wider flex items-center"><i data-lucide=terminal class="w-4 h-4 mr-2"></i> Logs</h2>
<div class="flex items-center space-x-2 ml-auto mr-4">
<select id=log-duration class="text-xs px-2 py-1 rounded-md bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 text-slate-700 dark:text-slate-300 focus:outline-none focus:ring-2 focus:ring-blue-500">
<option value=1>Last 1 min</option>
<option value=2>Last 2 mins</option>
<option value=3>Last 3 mins</option>
<option value=4>Last 4 mins</option>
<option value=5>Last 5 mins</option>
</select>
<button id=btn-download-logs class="flex items-center px-2 py-1 bg-blue-500/10 text-blue-600 dark:text-blue-400 text-xs font-medium rounded-md border border-blue-500/20 hover:bg-blue-500/20 transition-colors" title="Download Logs">
<i data-lucide=download class="w-3 h-3 mr-1"></i> Export
</button>
</div>
<div class="flex items-center space-x-2 text-slate-500 dark:text-slate-600">
<button class="collapse-trigger hover:text-slate-700 dark:hover:text-slate-300 transition-colors"><i data-lucide=chevron-down class="w-4 h-4 collapse-icon transition-transform"></i></button>
<i data-lucide=grip-vertical class="w-4 h-4 opacity-50"></i>
</div>
</div>
<div class="widget-content max-h-[300px]">
<div id=console-log class="h-32 overflow-y-auto scrollbar-thin font-mono text-[11px] leading-relaxed p-2 bg-slate-100 dark:bg-slate-950/50 rounded-lg border border-slate-200 dark:border-slate-800/50 transition-colors">
<div class=text-slate-500>[SYSTEM] Dashboard initialized.</div>
</div>
</div>
</div>
</div>
<div class="gutter-col hidden xl:block" data-left=zone-2 data-right=zone-3></div>
<div id=zone-3 class="col-zone flex-1 min-w-[300px] space-y-6 drop-zone xl:pl-4">
<div id=card-sensors class="glass-card rounded-xl p-5 shadow-sm dark:shadow-xl draggable" draggable=false>
<div class="widget-header flex justify-between items-center mb-6 p-2 -m-2 rounded-t-xl transition-colors hover:bg-slate-200/50 dark:hover:bg-slate-800/50">
<h2 class="text-lg font-semibold flex items-center text-slate-900 dark:text-white"><i data-lucide=scan-eye class="w-5 h-5 mr-2 text-purple-500 dark:text-purple-400"></i> IR Sensors</h2>
<div class="flex items-center space-x-2 text-slate-500 dark:text-slate-600">
<button class="collapse-trigger hover:text-slate-700 dark:hover:text-slate-300 transition-colors"><i data-lucide=chevron-down class="w-5 h-5 collapse-icon transition-transform"></i></button>
<i data-lucide=grip-vertical class="w-5 h-5 opacity-50"></i>
</div>
</div>
<div class=widget-content style=overflow:hidden>
<div class="space-y-6 h-full pb-4">
<div>
<div class="flex justify-between items-center mb-2"><h3 class="text-xs font-medium text-slate-500 dark:text-slate-400 uppercase tracking-wider">Primary Array</h3></div>
<div class="bg-slate-100 dark:bg-slate-900/80 p-3 rounded-xl border border-slate-200 dark:border-slate-800 flex justify-center space-x-2 h-32 items-end transition-colors" id=sensor-array-container></div>
</div>
<div>
<div class="flex justify-between items-center mb-2"><h3 class="text-xs font-medium text-slate-500 dark:text-slate-400 uppercase tracking-wider">Secondary Array</h3></div>
<div class="bg-slate-100 dark:bg-slate-900/80 p-3 rounded-xl border border-slate-200 dark:border-slate-800 flex justify-center space-x-2 h-32 items-end transition-colors" id=mid-sensor-array-container></div>
</div>
</div>
</div>
</div>
<div id=card-errors class="glass-card rounded-xl p-5 shadow-sm dark:shadow-xl flex flex-col draggable" draggable=false>
<div class="widget-header w-full flex justify-between items-center border-b border-slate-200 dark:border-slate-800 pb-3 mb-4 transition-colors p-2 -m-2 rounded-t-xl hover:bg-slate-200/50 dark:hover:bg-slate-800/50">
<h2 class="text-sm font-semibold text-slate-500 uppercase tracking-wider flex items-center">
<i data-lucide=trending-up class="w-4 h-4 mr-2"></i> Error Monitor
</h2>
<div class="flex items-center">
<div class="flex items-center space-x-1 mr-2">
<button id=btn-error-refresh class="p-1.5 rounded-md text-slate-500 hover:text-blue-500 hover:bg-blue-500/10 transition-colors" title="Clear Data"><i data-lucide=refresh-cw class="w-4 h-4"></i></button>
<button id=btn-error-reset class="p-1.5 rounded-md text-slate-500 hover:text-blue-500 hover:bg-blue-500/10 transition-colors" title="Reset Zoom"><i data-lucide=zoom-out class="w-4 h-4"></i></button>
</div>
<div class="flex items-center space-x-2 text-slate-500 dark:text-slate-600 border-l border-slate-200 dark:border-slate-700 pl-2">
<button id=btn-error-toggle class="p-1.5 rounded-md hover:bg-slate-200 dark:hover:bg-slate-800 transition-colors" title="Toggle Split/Merged View">
<i data-lucide=layers class="w-4 h-4"></i>
</button>
<button class="collapse-trigger hover:text-slate-700 dark:hover:text-slate-300 transition-colors"><i data-lucide=chevron-down class="w-5 h-5 collapse-icon transition-transform"></i></button>
<i data-lucide=grip-vertical class="w-5 h-5 opacity-50"></i>
</div>
</div>
</div>
<div class=widget-content style=min-height:300px>
<div class="flex justify-center space-x-8 mb-2 text-sm font-mono">
<div class="flex items-center"><span class="w-2 h-2 rounded-full bg-red-500 mr-2"></span>E2: <span id=val-e2 class="font-bold ml-1 text-slate-700 dark:text-slate-300">0.00</span></div>
<div class="flex items-center"><span class="w-2 h-2 rounded-full bg-blue-500 mr-2"></span>E3: <span id=val-e3 class="font-bold ml-1 text-slate-700 dark:text-slate-300">0.00</span></div>
</div>
<div id=error-mode-merged class="h-[300px] w-full relative transition-all duration-300">
<canvas id=error-merged-plot></canvas>
</div>
<div id=error-mode-split class="h-[300px] w-full hidden flex-col space-y-2 transition-all duration-300">
<div class="flex-1 relative min-h-0">
<canvas id=error-e2-plot></canvas>
</div>
<div class="flex-1 relative min-h-0">
<canvas id=error-e3-plot></canvas>
</div>
</div>
</div>
</div>
</div>
</div>
</main>
<script>lucide.createIcons();class Splitter{constructor(t){this.dashboard=t,this.gutters=document.querySelectorAll(".gutter-col"),this.activeGutter=null,this.init()}init(){this.gutters.forEach((t=>t.addEventListener("mousedown",(e=>this.startDrag(e,t))))),document.addEventListener("mousemove",(t=>this.onDrag(t))),document.addEventListener("mouseup",(()=>this.stopDrag()))}startDrag(t,e){this.activeGutter=e,this.activeGutter.classList.add("active"),this.leftCol=document.getElementById(e.dataset.left),this.rightCol=document.getElementById(e.dataset.right),this.startX=t.clientX,this.startLeftWidth=this.leftCol.offsetWidth,this.startRightWidth=this.rightCol.offsetWidth,document.body.classList.add("is-resizing")}onDrag(t){if(!this.activeGutter)return;const e=t.clientX-this.startX,s=document.getElementById("main-layout").offsetWidth;let a=(this.startLeftWidth+e)/s*100,i=(this.startRightWidth-e)/s*100;a<15?(a=15,i=(this.startLeftWidth+this.startRightWidth)/s*100-15):i<15&&(i=15,a=(this.startLeftWidth+this.startRightWidth)/s*100-15),this.leftCol.style.flex=`${a} 1 0px`,this.rightCol.style.flex=`${i} 1 0px`,this.dashboard.resizeCharts()}stopDrag(){this.activeGutter&&(this.activeGutter.classList.remove("active"),this.activeGutter=null,document.body.classList.remove("is-resizing"),this.dashboard.resizeCharts())}}class RobotDashboard{constructor(){this.ws=null,this.isConnected=!1,this.isSimulation=!1,this.simInterval=null,this.maxDataPoints=100,this.updateInterval=50,this.telemetryHistory=[],this.maxHistoryRetention=6e5,this.errorViewMerged=!0,this.isTelemetryPaused=!1,this.joystick=null,this.init()}init(){this.bindThemeToggle(),this.initializeCharts(),this.bindEvents(),this.enableDragging(),this.enableCollapsibles(),this.splitter=new Splitter(this),this.initJoystick(),this.connect()}connect(){if(!this.isConnected){this.log("[SYSTEM] Attempting to connect...","info"),document.getElementById("conn-text").textContent="Connecting...";try{this.ws=new WebSocket(`ws://${window.location.hostname}/ws`),this.ws.onopen=()=>{this.isConnected=!0,this.stopSimulation(),this.updateConnectionUI(!0),this.log("[SYSTEM] Connection established.","success")},this.ws.onmessage=t=>{try{const e=JSON.parse(t.data);this.processIncomingData(e)}catch(t){console.error("Error parsing WS data",t)}},this.ws.onerror=t=>{console.log("WS Error",t)},this.ws.onclose=()=>{this.isConnected&&this.log("[SYSTEM] Connection lost.","error"),this.isConnected=!1,this.updateConnectionUI(!1),this.isSimulation||(this.log("[SYSTEM] Falling back to simulation.","warning"),this.startSimulation())}}catch(t){this.log("[SYSTEM] Connection failed.","error"),this.startSimulation()}}}initJoystick(){const t={zone:document.getElementById("joystick-zone"),mode:"static",position:{left:"50%",top:"50%"},color:"#3b82f6",size:120,restOpacity:.8};this.joystick=nipplejs.create(t),this.joystick.on("move",((t,e)=>{if(!e.vector)return;const s=e.vector.y;let a=Math.floor(255*Math.max(0,s));const i=e.vector.x;let n=Math.floor(90+90*i);n=Math.max(0,Math.min(180,n)),document.getElementById("slider-pwm").value=a,document.getElementById("val-pwm").textContent=a,document.getElementById("slider-servo").value=n,document.getElementById("val-servo").textContent=n+"°",document.getElementById("telemetry-pwm").textContent=a,document.getElementById("telemetry-servo").textContent=n,this.sendCommand("SET_PWM",{value:a}),this.sendCommand("SET_SERVO",{value:n})})),this.joystick.on("end",(()=>{document.getElementById("slider-pwm").value=0,document.getElementById("val-pwm").textContent="0",document.getElementById("slider-servo").value=90,document.getElementById("val-servo").textContent="90°",document.getElementById("telemetry-pwm").textContent="0",document.getElementById("telemetry-servo").textContent="90",this.sendCommand("SET_PWM",{value:0}),this.sendCommand("SET_SERVO",{value:90})}))}processIncomingData(t){t.timestamp=Date.now(),this.telemetryHistory.push(t);const e=Date.now()-this.maxHistoryRetention;if(this.telemetryHistory.length>0&&this.telemetryHistory[0].timestamp<e){const t=this.telemetryHistory.findIndex((t=>t.timestamp>=e));t>0&&(this.telemetryHistory=this.telemetryHistory.slice(t))}this.isTelemetryPaused||(this.updateCharts(t.timestamp,t),this.updateTelemetry(t),t.sensors&&(this.renderSensors("sensor-array-container",t.sensors,"bg-emerald-500"),this.renderStatusIR("status-ir-primary",t.sensors,"bg-emerald-500")),t.midSensors&&(this.renderSensors("mid-sensor-array-container",t.midSensors,"bg-blue-500"),this.renderStatusIR("status-ir-secondary",t.midSensors,"bg-blue-500")))}disconnect(){this.ws&&this.ws.close()}startSimulation(){this.isSimulation||(this.isSimulation=!0,this.updateSimulationUI(!0),this.simState={tick:0,state:"IDLE",speed:0,targetSpeed:0,obstacleDistance:2e3,servoPos:90,linePos:3.5},this.simInterval&&clearInterval(this.simInterval),this.simInterval=setInterval((()=>{this.simState.tick+=.1,"LINE_FOLLOW"===this.simState.state?(this.simState.targetSpeed=.8,this.simState.servoPos=90+30*Math.sin(this.simState.tick),this.simState.linePos=3.5+2.5*Math.sin(.7*this.simState.tick)):"IDLE"===this.simState.state&&(this.simState.targetSpeed=0),"MANUAL"===this.simState.state?this.simState.speed=this.simState.targetSpeed:this.simState.speed+=.1*(this.simState.targetSpeed-this.simState.speed);let t=50*(Math.random()-.5),e=10*Math.sin(.5*this.simState.tick)+2*(Math.random()-.5),s=15*Math.cos(.3*this.simState.tick)+5*(Math.random()-.5);this.simState.obstacleDistance=Math.random()>.98?150:this.simState.obstacleDistance+.05*(2e3-this.simState.obstacleDistance);const a=(t,e,s)=>Array.from({length:t},((t,a)=>Math.min(4095,Math.max(0,Math.floor(4e3*Math.exp(-(Math.abs(a-e)**2)/(2*s**2))+95*Math.random()))))),i={state:this.simState.obstacleDistance<300?"OBSTACLE_AVOID":this.simState.state,speed:this.simState.speed,obstacle:this.simState.obstacleDistance<300,ultrasonic:Math.max(0,this.simState.obstacleDistance+t),pwm:255*this.simState.speed,servoAngle:"MANUAL"===this.simState.state?this.simState.servoPos:this.simState.servoPos+5*(Math.random()-.5),sensors:a(7,this.simState.linePos,1.2),midSensors:a(7,this.simState.linePos,1.5),e2:e,e3:s};this.processIncomingData(i)}),this.updateInterval))}stopSimulation(){this.isSimulation=!1,this.simInterval&&clearInterval(this.simInterval),this.updateSimulationUI(!1)}updateConnectionUI(t){const e=document.getElementById("btn-connect-toggle"),s=document.getElementById("conn-icon"),a=document.getElementById("conn-text");e.className=t?"flex items-center px-3 py-1 rounded-full bg-emerald-500/10 text-emerald-600 dark:text-emerald-500 text-sm font-medium border border-emerald-500/20 transition-all hover:bg-emerald-500/20":"flex items-center px-3 py-1 rounded-full bg-red-500/10 text-red-600 dark:text-red-500 text-sm font-medium border border-red-500/20 transition-all hover:bg-red-500/20",a.textContent=t?"Connected":"Disconnected",s.setAttribute("data-lucide",t?"wifi":"wifi-off"),lucide.createIcons()}updateSimulationUI(t){document.getElementById("btn-sim-toggle").classList.toggle("opacity-50",!t),document.getElementById("btn-sim-toggle").classList.toggle("grayscale",!t),document.getElementById("sim-ping").classList.toggle("hidden",!t)}bindThemeToggle(){this.isDark=document.documentElement.classList.contains("dark"),this.updateChartTheme(this.isDark),document.getElementById("theme-toggle").addEventListener("click",(()=>{document.documentElement.classList.toggle("dark"),this.isDark=document.documentElement.classList.contains("dark"),this.updateChartTheme(this.isDark)}))}updateChartTheme(t){if(!this.charts)return;const e=t?"#94a3b8":"#64748b",s=t?"#334155":"#e2e8f0";Object.values(this.charts).forEach((t=>{t.options.scales.x.grid.color=s,t.options.scales.y.grid.color=s,t.options.scales.x.ticks.color=e,t.options.scales.y.ticks.color=e,t.update("none")}))}enableCollapsibles(){document.querySelectorAll(".collapse-trigger").forEach((t=>{t.addEventListener("click",(t=>{const e=t.target.closest(".glass-card");e&&(e.classList.toggle("collapsed"),"card-charts"!==e.id||e.classList.contains("collapsed")||setTimeout((()=>this.resizeCharts()),350))}))}))}enableDragging(){const t=(t,e)=>{t.forEach((t=>{const s=t.querySelector(e);s&&(s.addEventListener("mouseenter",(()=>t.setAttribute("draggable","true"))),s.addEventListener("mouseleave",(()=>t.setAttribute("draggable","false"))),t.addEventListener("dragstart",(e=>{"true"===t.getAttribute("draggable")?(t.classList.add("dragging"),e.stopPropagation()):e.preventDefault()})),t.addEventListener("dragend",(()=>{t.classList.remove("dragging"),this.resizeCharts()})))}))};t(document.querySelectorAll(".draggable"),".widget-header"),t(document.querySelectorAll(".tab-btn"),".tab-drag-handle");[...document.querySelectorAll(".drop-zone"),document.getElementById("tab-container")].forEach((t=>{t.addEventListener("dragover",(e=>{e.preventDefault();const s=document.querySelector(".tab-btn.dragging"),a=document.querySelector(".draggable.dragging");if(s&&"tab-container"===t.id){const a=this.getDragAfterElementHorizontal(t,e.clientX);null==a?t.appendChild(s):t.insertBefore(s,a)}else if(a&&t.classList.contains("drop-zone")){t.classList.add("drag-over");const s=this.getDragAfterElement(t,e.clientY);null==s?t.appendChild(a):t.insertBefore(a,s)}})),t.addEventListener("dragleave",(()=>t.classList.remove("drag-over"))),t.addEventListener("drop",(()=>t.classList.remove("drag-over")))}))}getDragAfterElement(t,e){return[...t.querySelectorAll(".draggable:not(.dragging)")].reduce(((t,s)=>{const a=s.getBoundingClientRect(),i=e-a.top-a.height/2;return i<0&&i>t.offset?{offset:i,element:s}:t}),{offset:Number.NEGATIVE_INFINITY}).element}getDragAfterElementHorizontal(t,e){return[...t.querySelectorAll(".tab-btn:not(.dragging)")].reduce(((t,s)=>{const a=s.getBoundingClientRect(),i=e-a.left-a.width/2;return i<0&&i>t.offset?{offset:i,element:s}:t}),{offset:Number.NEGATIVE_INFINITY}).element}resizeCharts(){this.charts&&Object.values(this.charts).forEach((t=>t.resize()))}bindEvents(){document.getElementById("btn-emergency").addEventListener("click",(()=>this.handleEmergencyStop())),document.getElementById("tab-container").addEventListener("click",(t=>{t.target.closest(".tab-btn")&&this.switchTab(t.target.closest(".tab-btn").dataset.tab)})),window.addEventListener("resize",(()=>this.resizeCharts())),document.getElementById("btn-sim-toggle").addEventListener("click",(()=>this.isSimulation?this.stopSimulation():(this.isConnected&&this.disconnect(),this.startSimulation()))),document.getElementById("btn-connect-toggle").addEventListener("click",(()=>this.isConnected?this.disconnect():this.connect())),document.getElementById("btn-chart-refresh").addEventListener("click",(()=>this.refreshCharts())),document.getElementById("btn-chart-reset").addEventListener("click",(()=>this.resetChartZoom())),document.getElementById("btn-download-logs").addEventListener("click",(()=>{const t=parseInt(document.getElementById("log-duration").value);this.downloadLogs(t)})),document.getElementById("btn-chart-hold").addEventListener("click",(t=>{this.isTelemetryPaused=!this.isTelemetryPaused;const e=t.currentTarget;this.isTelemetryPaused?(e.classList.add("text-amber-500","bg-amber-500/10"),e.querySelector("i").setAttribute("data-lucide","play"),this.log("[UI] Telemetry PAUSED","warning")):(e.classList.remove("text-amber-500","bg-amber-500/10"),e.querySelector("i").setAttribute("data-lucide","pause"),this.log("[UI] Telemetry RESUMED","info")),lucide.createIcons()}));document.getElementById("slider-pwm").addEventListener("input",(t=>{const e=parseInt(t.target.value);document.getElementById("val-pwm").textContent=e,document.getElementById("telemetry-pwm").textContent=e,this.isSimulation&&(this.simState.state="MANUAL"),this.sendCommand("SET_PWM",{value:e})}));document.getElementById("slider-servo").addEventListener("input",(t=>{const e=parseInt(t.target.value);document.getElementById("val-servo").textContent=e+"°",document.getElementById("telemetry-servo").textContent=e,this.isSimulation&&(this.simState.state="MANUAL"),this.sendCommand("SET_SERVO",{value:e})})),document.getElementById("mode-toggle").addEventListener("change",(t=>{const e=t.target.checked?"AUTO":"MANUAL";document.getElementById("mode-status-text").textContent=e+" MODE",this.setManualControls("MANUAL"===e),this.isSimulation&&"MANUAL"===e&&(this.simState.state="IDLE"),this.sendCommand("SET_MODE",{mode:e}),this.isSimulation&&"AUTO"===e&&(this.simState.state="LINE_FOLLOW")})),document.getElementById("btn-error-toggle").addEventListener("click",(()=>{this.errorViewMerged=!this.errorViewMerged;document.querySelector("#btn-error-toggle i").setAttribute("data-lucide",this.errorViewMerged?"layers":"grid"),lucide.createIcons(),document.getElementById("error-mode-merged").classList.toggle("hidden",!this.errorViewMerged),document.getElementById("error-mode-merged").classList.toggle("flex",this.errorViewMerged),document.getElementById("error-mode-split").classList.toggle("hidden",this.errorViewMerged),document.getElementById("error-mode-split").classList.toggle("flex",!this.errorViewMerged),this.resizeCharts()})),document.getElementById("btn-error-refresh").addEventListener("click",(()=>this.refreshErrorCharts())),document.getElementById("btn-error-reset").addEventListener("click",(()=>this.resetErrorChartZoom()))}setManualControls(t){[document.getElementById("slider-pwm"),document.getElementById("slider-servo")].forEach((e=>e.disabled=!t)),document.getElementById("joystick-zone").style.pointerEvents=t?"auto":"none",document.getElementById("joystick-zone").style.opacity=t?"1":"0.5",t||(document.getElementById("slider-pwm").value=0,document.getElementById("val-pwm").textContent="0",document.getElementById("slider-servo").value=90,document.getElementById("val-servo").textContent="90°")}handleEmergencyStop(){this.log("[EMERGENCY] STOP triggered!","error"),this.sendCommand("EMERGENCY_STOP")}sendCommand(t,e={}){"SET_PWM"!==t&&"SET_SERVO"!==t&&"JOYSTICK_MOVE"!==t&&this.log(`[CMD] Sending: ${t.toUpperCase()} ${JSON.stringify(e)}`),this.isConnected&&this.ws?this.ws.send(JSON.stringify({command:t,...e})):this.isSimulation&&("SET_MODE"===t&&("AUTO"===e.mode?this.simState.state="LINE_FOLLOW":"MANUAL"===e.mode&&(this.simState.state="IDLE")),"SET_PWM"===t&&(this.simState.targetSpeed=e.value/255),"SET_SERVO"===t&&(this.simState.servoPos=e.value))}downloadLogs(t){const e=Date.now()-60*t*1e3,s=this.telemetryHistory.filter((t=>t.timestamp>=e));if(0===s.length)return void this.log(`[SYSTEM] No logs found for last ${t} minute(s).`,"warning");const a="data:text/json;charset=utf-8,"+encodeURIComponent(JSON.stringify(s,null,2)),i=document.createElement("a");i.setAttribute("href",a),i.setAttribute("download",`telemetry_log_${(new Date).toISOString().replace(/[:.]/g,"-")}.json`),document.body.appendChild(i),i.click(),i.remove(),this.log(`[SYSTEM] Exported ${s.length} data points (${t}m).`,"success")}log(t,e="info"){const s=document.getElementById("console-log"),a=document.createElement("div");a.innerHTML=`<span class="text-slate-400 dark:text-slate-600">[${(new Date).toLocaleTimeString().split(" ")[0]}]</span> ${t}`,a.className="error"===e?"text-red-600 dark:text-red-400":"warning"===e?"text-yellow-600 dark:text-yellow-400":"success"===e?"text-emerald-600 dark:text-emerald-400":"text-slate-600 dark:text-slate-300",s.appendChild(a),s.scrollTop=s.scrollHeight}switchTab(t){document.querySelectorAll(".tab-btn").forEach((e=>{const s=e.dataset.tab===t;e.classList.toggle("active",s),e.classList.toggle("bg-blue-100",s&&!this.isDark),e.classList.toggle("text-blue-600",s&&!this.isDark),e.classList.toggle("bg-blue-500/20",s&&this.isDark),e.classList.toggle("text-blue-400",s&&this.isDark),s?e.classList.remove("text-slate-500","dark:text-slate-400"):(e.classList.remove("bg-blue-100","text-blue-600","bg-blue-500/20","text-blue-400"),e.classList.add("text-slate-500","dark:text-slate-400"))})),["ultrasonic","servo","pwm"].forEach((e=>{const s=document.getElementById(`${e}-container`);e===t?s.classList.remove("opacity-0","pointer-events-none"):s.classList.add("opacity-0","pointer-events-none")}))}initializeCharts(){const t="#334155",e="#94a3b8",s={responsive:!0,maintainAspectRatio:!1,animation:!1,parsing:!1,scales:{x:{type:"time",time:{unit:"second",displayFormats:{second:"HH:mm:ss"}},display:!0,grid:{color:t},ticks:{maxRotation:0,autoSkip:!0,color:e}},y:{grid:{color:t},beginAtZero:!0,ticks:{color:e}}},plugins:{legend:{display:!1},tooltip:{enabled:!0,mode:"nearest",intersect:!1},zoom:{zoom:{wheel:{enabled:!0},pinch:{enabled:!0},mode:"xy"},pan:{enabled:!0,mode:"xy"}}},elements:{point:{radius:0,hoverRadius:5},line:{tension:.4,borderWidth:2}}},a=(t,e,a,i)=>{const n=document.getElementById(t).getContext("2d"),r=n.createLinearGradient(0,0,0,350);return r.addColorStop(0,`${a}80`),r.addColorStop(1,`${a}00`),new Chart(n,{type:"line",data:{datasets:[{label:e,borderColor:a,backgroundColor:r,fill:!0,data:[]}]},options:{...s,scales:{...s.scales,y:{...s.scales.y,max:i}}}})};this.charts={ultrasonic:a("ultrasonic-plot","Distance","#3b82f6",2e3),servo:a("servo-plot","Angle","#f97316",180),pwm:a("pwm-plot","PWM","#10b981",255)},this.chartData={ultrasonic:[],servo:[],pwm:[],e2:[],e3:[]};const i=JSON.parse(JSON.stringify(s));delete i.scales.y.beginAtZero,this.charts.errorMerged=new Chart(document.getElementById("error-merged-plot").getContext("2d"),{type:"line",data:{datasets:[{label:"E2 Error",borderColor:"#ef4444",backgroundColor:"#ef444420",data:[]},{label:"E3 Error",borderColor:"#3b82f6",backgroundColor:"#3b82f620",data:[]}]},options:i}),this.charts.errorE2=new Chart(document.getElementById("error-e2-plot").getContext("2d"),{type:"line",data:{datasets:[{label:"E2 Error",borderColor:"#ef4444",backgroundColor:"#ef444420",fill:!0,data:[]}]},options:i}),this.charts.errorE3=new Chart(document.getElementById("error-e3-plot").getContext("2d"),{type:"line",data:{datasets:[{label:"E3 Error",borderColor:"#3b82f6",backgroundColor:"#3b82f620",fill:!0,data:[]}]},options:i})}refreshCharts(){Object.keys(this.chartData).forEach((t=>this.chartData[t]=[])),Object.values(this.charts).forEach((t=>{t.data&&t.data.datasets&&(t.data.datasets.forEach((t=>t.data=[])),t.update())})),this.log("Charts cleared.","info")}resetChartZoom(){Object.values(this.charts).forEach((t=>t.resetZoom()))}refreshErrorCharts(){this.chartData.e2=[],this.chartData.e3=[],[this.charts.errorMerged,this.charts.errorE2,this.charts.errorE3].forEach((t=>{t.data.datasets.forEach((t=>t.data=[])),t.update()})),this.log("Error charts cleared.","info")}resetErrorChartZoom(){[this.charts.errorMerged,this.charts.errorE2,this.charts.errorE3].forEach((t=>t.resetZoom()))}updateCharts(t,e){const s=(e,s,a,i=0)=>{this.chartData[e].push({x:t,y:s}),this.chartData[e].length>this.maxDataPoints&&this.chartData[e].shift(),a&&(a.data.datasets[i].data=this.chartData[e],a.update("none"))};void 0!==e.ultrasonic&&(s("ultrasonic",e.ultrasonic,this.charts.ultrasonic),document.getElementById("ultrasonic-live-val").textContent=e.ultrasonic.toFixed(0)),void 0!==e.servoAngle&&(s("servo",e.servoAngle,this.charts.servo),document.getElementById("servo-live-val").textContent=e.servoAngle.toFixed(0)),void 0!==e.pwm&&(s("pwm",e.pwm,this.charts.pwm),document.getElementById("pwm-live-val").textContent=e.pwm.toFixed(0)),void 0!==e.e2&&(this.chartData.e2.push({x:t,y:e.e2}),this.chartData.e2.length>this.maxDataPoints&&this.chartData.e2.shift(),this.charts.errorMerged.data.datasets[0].data=this.chartData.e2,this.charts.errorE2.data.datasets[0].data=this.chartData.e2),void 0!==e.e3&&(this.chartData.e3.push({x:t,y:e.e3}),this.chartData.e3.length>this.maxDataPoints&&this.chartData.e3.shift(),this.charts.errorMerged.data.datasets[1].data=this.chartData.e3,this.charts.errorE3.data.datasets[0].data=this.chartData.e3),this.errorViewMerged?this.charts.errorMerged.update("none"):(this.charts.errorE2.update("none"),this.charts.errorE3.update("none"))}renderSensors(t,e,s="bg-emerald-500"){const a=document.getElementById(t);a.children.length!==e.length&&(a.innerHTML="",e.forEach((()=>{const t=document.createElement("div");t.className="flex-1 bg-slate-200 dark:bg-slate-800 rounded-md relative overflow-hidden flex items-end group h-full transition-colors",t.innerHTML=`<div class="sensor-led w-full ${s} opacity-80 transition-all duration-150"></div><span class="absolute bottom-0.5 left-0 right-0 text-center text-[9px] font-bold text-slate-900/70 dark:text-white/80 mix-blend-overlay z-10"></span>`,a.appendChild(t)}))),Array.from(a.children).forEach(((t,s)=>{const a=e[s];t.firstElementChild.style.height=`${Math.min(100,a/4095*100)}%`,t.lastElementChild.textContent=a,t.firstElementChild.classList.toggle("shadow-[0_0_15px_rgba(16,185,129,0.6)]",a>2400),t.firstElementChild.classList.toggle("brightness-125",a>2400)}))}renderStatusIR(t,e,s){const a=document.getElementById(t);a.children.length!==e.length&&(a.innerHTML="",e.forEach((()=>{const t=document.createElement("div");t.className="w-2 h-2 rounded-full bg-slate-300 dark:bg-slate-700 status-dot",a.appendChild(t)}))),Array.from(a.children).forEach(((t,a)=>{const i=e[a]>2400;t.className="w-2 h-2 rounded-full status-dot "+(i?s+" shadow-sm":"bg-slate-300 dark:bg-slate-700")}))}updateTelemetry(t){if(t.timestamp){const e=new Date(t.timestamp),s=e.toLocaleTimeString("en-GB",{hour12:!1})+"."+String(e.getMilliseconds()).padStart(3,"0");document.getElementById("telemetry-timestamp").textContent=s}if(t.state){const e=document.getElementById("telemetry-state-pill");switch(e.textContent=t.state.replace("_"," "),e.className="text-xs font-black",t.state){case"IDLE":e.classList.add("text-blue-500");break;case"LINE_FOLLOW":e.classList.add("text-emerald-500");break;case"OBSTACLE_AVOID":e.classList.add("text-yellow-500");break;case"EMERGENCY":e.classList.add("text-red-500","animate-pulse");break;case"MANUAL":e.classList.add("text-purple-500");break;default:e.classList.add("text-slate-500")}}if(void 0!==t.speed&&(document.getElementById("telemetry-speed").textContent=t.speed.toFixed(2)),void 0!==t.pwm&&(document.getElementById("telemetry-pwm").textContent=t.pwm.toFixed(0)),void 0!==t.ultrasonic&&(document.getElementById("telemetry-sonar").textContent=t.ultrasonic.toFixed(0)),void 0!==t.servoAngle&&(document.getElementById("telemetry-servo").textContent=t.servoAngle.toFixed(0)),void 0!==t.e2&&(document.getElementById("val-e2").textContent=t.e2.toFixed(2),document.getElementById("summary-e2").textContent=t.e2.toFixed(1)),void 0!==t.e3&&(document.getElementById("val-e3").textContent=t.e3.toFixed(2),document.getElementById("summary-e3").textContent=t.e3.toFixed(1)),void 0!==t.obstacle){const e=document.getElementById("obstacle-pill");t.obstacle?(e.innerHTML='<i data-lucide="shield-alert" class="w-4 h-4 mr-2"></i> DETECTED',e.className="flex-1 bg-red-500/10 text-red-600 dark:text-red-400 rounded-xl px-3 py-2 text-xs font-bold flex items-center justify-center transition-colors animate-pulse"):(e.innerHTML='<i data-lucide="shield-check" class="w-4 h-4 mr-2"></i> CLEAR',e.className="flex-1 bg-emerald-500/10 text-emerald-600 dark:text-emerald-400 rounded-xl px-3 py-2 text-xs font-bold flex items-center justify-center transition-colors"),lucide.createIcons()}}}document.addEventListener("DOMContentLoaded",(()=>{window.dashboard=new RobotDashboard,window.dashboard.switchTab("ultrasonic")}))</script>
</body>
</html>
)rawliteral";
