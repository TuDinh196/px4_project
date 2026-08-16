// ==============================================================================
//  Quadplane Condor — Flight Mission & Visual Map Center Engine (app.js)
// ==============================================================================

const host = (window.location.hostname && window.location.hostname.length > 0) ? window.location.hostname : '127.0.0.1';
const WS_URL = 'ws://' + host + ':8765';
let ws = null;

// Map & Layer handles
let map = null;
let satLayer, osmLayer;
let currentLayer = 'sat';
let droneMarker = null;
let plannedPathPolyline = null;
let liveFlownPolyline = null;
let waypointMarkers = [];
let targetMarker = null;

// Hanoi Home Coordinates
const HOME_LAT = 21.028511;
const HOME_LON = 105.804817;

// Telemetry Trail History
let flownCoordinates = [];
let traj3D_X = [0];
let traj3D_Y = [0];
let traj3D_Z = [0];
let selectedScenario = 'hover';

// DOM Handles
const elWsBadge = document.getElementById('ws-badge');
const elWsText = document.getElementById('ws-text');
const elFlightMode = document.getElementById('flight-mode');
const elBattery = document.getElementById('battery-voltage');
const elAlt = document.getElementById('val-alt');
const elAirspeed = document.getElementById('val-airspeed');
const elGroundspeed = document.getElementById('val-groundspeed');
const elSavings = document.getElementById('val-savings');
const elHeading = document.getElementById('val-heading');
const horizonPitchRoll = document.getElementById('horizon-pitch-roll');

// Motor Elements
const barM0 = document.getElementById('bar-m0');
const barM1 = document.getElementById('bar-m1');
const barM2 = document.getElementById('bar-m2');
const barM3 = document.getElementById('bar-m3');
const barM4 = document.getElementById('bar-m4');
const rpmM0 = document.getElementById('rpm-m0');
const rpmM1 = document.getElementById('rpm-m1');
const rpmM2 = document.getElementById('rpm-m2');
const rpmM3 = document.getElementById('rpm-m3');
const rpmM4 = document.getElementById('rpm-m4');

// Lifecycle steps
const stepGround = document.getElementById('step-ground');
const stepTakeoff = document.getElementById('step-takeoff');
const stepMission = document.getElementById('step-mission');
const stepReturn = document.getElementById('step-return');
const stepLand = document.getElementById('step-land');

// Buttons
const btnStartScenario = document.getElementById('btn-start-scenario');
const btnRtl = document.getElementById('btn-rtl');
const btnClearPath = document.getElementById('btn-clear-path');
const btnLayerSat = document.getElementById('btn-layer-sat');
const btnLayerOsm = document.getElementById('btn-layer-osm');
const btnRecenter = document.getElementById('btn-recenter');
const scenarioCards = document.querySelectorAll('.scenario-card');

// ==============================================================================
//  1. Leaflet Map & Custom Drone Icon Setup
// ==============================================================================

function createDroneIcon(headingDeg = 0, isFW = false) {
    const color = isFW ? '#38bdf8' : '#10b981';
    const svgIcon = `
        <div style="position: relative; width: 64px; height: 64px; display: flex; align-items: center; justify-content: center;">
            <div style="position: absolute; width: 22px; height: 22px; background: rgba(56, 189, 248, 0.45); border-radius: 50%; box-shadow: 0 0 12px #38bdf8;"></div>
            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="56" height="56" style="transform: rotate(${headingDeg}deg); filter: drop-shadow(0 0 8px ${color}) drop-shadow(0 0 3px #ffffff);">
                <!-- Fuselage -->
                <path d="M 50 10 L 54 35 L 54 85 L 50 92 L 46 85 L 46 35 Z" fill="${color}" stroke="#ffffff" stroke-width="2" />
                <!-- Main Wing (2.4m Span) -->
                <path d="M 50 38 L 96 46 L 94 53 L 50 48 L 6 53 L 4 46 Z" fill="${color}" stroke="#ffffff" stroke-width="2" />
                <!-- 4 VTOL Booms -->
                <rect x="22" y="24" width="5" height="48" fill="#334155" rx="2" stroke="#ffffff" stroke-width="0.5" />
                <rect x="73" y="24" width="5" height="48" fill="#334155" rx="2" stroke="#ffffff" stroke-width="0.5" />
                <!-- 4 VTOL Propeller Disks -->
                <circle cx="24.5" cy="24" r="9" fill="rgba(34, 211, 238, 0.35)" stroke="#22d3ee" stroke-width="2" stroke-dasharray="3,3" />
                <circle cx="24.5" cy="72" r="9" fill="rgba(34, 211, 238, 0.35)" stroke="#22d3ee" stroke-width="2" stroke-dasharray="3,3" />
                <circle cx="75.5" cy="24" r="9" fill="rgba(34, 211, 238, 0.35)" stroke="#22d3ee" stroke-width="2" stroke-dasharray="3,3" />
                <circle cx="75.5" cy="72" r="9" fill="rgba(34, 211, 238, 0.35)" stroke="#22d3ee" stroke-width="2" stroke-dasharray="3,3" />
                <!-- Nose Tractor Spinner -->
                <circle cx="50" cy="10" r="6" fill="#f59e0b" stroke="#ffffff" stroke-width="1.5" />
                <!-- V-Tail -->
                <path d="M 50 82 L 70 95 L 65 98 L 50 88 L 35 98 L 30 95 Z" fill="${color}" stroke="#ffffff" stroke-width="1" />
            </svg>
        </div>
    `;
    return L.divIcon({
        className: 'custom-drone-icon',
        html: svgIcon,
        iconSize: [64, 64],
        iconAnchor: [32, 32]
    });
}

function initLeafletMap() {
    map = L.map('leaflet-map', {
        center: [HOME_LAT, HOME_LON],
        zoom: 18,
        zoomControl: true
    });

    // Satellite Imagery (Esri World Imagery)
    satLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: '&copy; Esri &mdash; Satellite Imagery',
        maxZoom: 20
    });

    // OpenStreetMap
    osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors',
        maxZoom: 19
    });

    satLayer.addTo(map);

    // Add Home Landing Pad Marker
    const homeIcon = L.divIcon({
        className: 'home-pad-icon',
        html: '<div style="background:#10b981; border:2px solid #fff; width:26px; height:26px; border-radius:50%; display:flex; align-items:center; justify-content:center; color:#000; font-weight:800; font-size:12px; box-shadow:0 0 10px #10b981;">H</div>',
        iconSize: [26, 26],
        iconAnchor: [13, 13]
    });
    L.marker([HOME_LAT, HOME_LON], { icon: homeIcon }).addTo(map).bindPopup('<b>BÃI ĐÁP CONDOR (HOME PAD)</b><br>21.028511°N, 105.804817°E');

    // Drone Marker
    droneMarker = L.marker([HOME_LAT, HOME_LON], { icon: createDroneIcon(0) }).addTo(map);
    droneMarker.bindTooltip('✈️ CONDOR HYBRID VTOL', { permanent: true, direction: 'top', className: 'drone-tooltip', offset: [0, -25] });

    // Live Trail
    liveFlownPolyline = L.polyline([], {
        color: '#06b6d4',
        weight: 3.5,
        opacity: 0.85,
        smoothFactor: 1.0
    }).addTo(map);

    // Planned Path Polyline
    plannedPathPolyline = L.polyline([], {
        color: '#f59e0b',
        weight: 2.5,
        dashArray: '6, 8',
        opacity: 0.8
    }).addTo(map);

    drawPlannedScenarioPath('hover');
}

function nedToGPS(xNorth, yEast) {
    const dLat = xNorth / 111320.0;
    const dLon = yEast / (111320.0 * Math.cos(HOME_LAT * Math.PI / 180.0));
    return [HOME_LAT + dLat, HOME_LON + dLon];
}

// ==============================================================================
//  2. Scenario Path Generator & Overlay
// ==============================================================================

function drawPlannedScenarioPath(scenarioName) {
    // Clear old waypoint markers
    waypointMarkers.forEach(m => map.removeLayer(m));
    waypointMarkers = [];

    let pathGPS = [];
    pathGPS.push([HOME_LAT, HOME_LON]); // Takeoff point

    if (scenarioName === 'hover') {
        pathGPS.push(nedToGPS(0, 0));
    } else if (scenarioName === 'square') {
        const side = 20.0;
        const wps = [[side, 0], [side, side], [0, side], [0, 0]];
        wps.forEach(wp => pathGPS.push(nedToGPS(wp[0], wp[1])));
    } else if (scenarioName === 'circle') {
        const radius = 15.0;
        for (let a = 0; a <= 360; a += 15) {
            const rad = a * Math.PI / 180.0;
            pathGPS.push(nedToGPS(radius * Math.cos(rad), radius * Math.sin(rad)));
        }
        pathGPS.push(nedToGPS(0, 0));
    } else if (scenarioName === 'figure8') {
        const X = 25.0;
        const Y = 45.0;
        for (let a = 0; a <= 360; a += 10) {
            const rad = a * Math.PI / 180.0;
            const x = X * Math.sin(2 * rad);
            const y = Y * Math.sin(rad);
            pathGPS.push(nedToGPS(x, y));
        }
        pathGPS.push(nedToGPS(0, 0));
    } else if (scenarioName === 'vtol_mission') {
        const wps = [
            [50, 0],       // Fwd transition
            [150, 100],    // FW Cruise WP1
            [250, 200],    // FW Cruise WP2
            [300, 50],     // FW Cruise WP3
            [150, -50],    // Back transition
            [0, 0]         // VTOL Land
        ];
        wps.forEach(wp => pathGPS.push(nedToGPS(wp[0], wp[1])));
    }

    plannedPathPolyline.setLatLngs(pathGPS);

    // Add milestone pins
    pathGPS.forEach((pt, idx) => {
        if (idx === 0 || idx === pathGPS.length - 1) return;
        const pin = L.circleMarker(pt, {
            radius: 4,
            color: '#f59e0b',
            fillColor: '#fbbf24',
            fillOpacity: 1.0
        }).addTo(map);
        waypointMarkers.push(pin);
    });
}

// ==============================================================================
//  3. Plotly 3D Trajectory Visualization
// ==============================================================================

function initPlotly3D() {
    const layout = {
        margin: { l: 0, r: 0, b: 0, t: 0 },
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        scene: {
            xaxis: { title: 'Bắc (X)', color: '#94a3b8', gridcolor: '#1e293b' },
            yaxis: { title: 'Đông (Y)', color: '#94a3b8', gridcolor: '#1e293b' },
            zaxis: { title: 'Độ cao (m)', color: '#94a3b8', gridcolor: '#1e293b' },
            camera: { eye: { x: 1.6, y: 1.6, z: 1.2 } }
        }
    };

    const tracePath = {
        x: [0], y: [0], z: [0],
        mode: 'lines',
        type: 'scatter3d',
        line: { width: 5, color: '#06b6d4' },
        name: 'Vết bay Condor'
    };

    const traceDrone = {
        x: [0], y: [0], z: [0],
        mode: 'markers',
        type: 'scatter3d',
        marker: { size: 8, color: '#f59e0b', symbol: 'diamond' },
        name: 'Vị trí máy bay'
    };

    Plotly.newPlot('plot-3d', [tracePath, traceDrone], layout, { responsive: true, displayModeBar: false });
}

function updatePlotly3D(x, y, z) {
    traj3D_X.push(x);
    traj3D_Y.push(y);
    traj3D_Z.push(z);

    if (traj3D_X.length > 250) {
        traj3D_X.shift();
        traj3D_Y.shift();
        traj3D_Z.shift();
    }

    Plotly.update('plot-3d', {
        x: [traj3D_X, [x]],
        y: [traj3D_Y, [y]],
        z: [traj3D_Z, [z]]
    }, {}, [0, 1]);
}

// ==============================================================================
//  4. WebSocket Telemetry Stream & Real-Time Sync
// ==============================================================================

function connectWebSocket() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        elWsBadge.style.borderColor = '#10b981';
        elWsText.innerText = 'WS: TRỰC TUYẾN';
        elWsText.style.color = '#10b981';
    };

    ws.onmessage = (evt) => {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'telemetry') {
            updateDashboard(msg);
        }
    };

    ws.onclose = () => {
        elWsBadge.style.borderColor = '#ef4444';
        elWsText.innerText = 'WS: MẤT KẾT NỐI (Thử lại...)';
        elWsText.style.color = '#ef4444';
        setTimeout(connectWebSocket, 2000);
    };
}

function updateLifecycleUI(lifecycleStr) {
    [stepGround, stepTakeoff, stepMission, stepReturn, stepLand].forEach(el => el.className = 'lifecycle-step');

    if (lifecycleStr === 'GROUND_IDLE') {
        stepGround.className = 'lifecycle-step active';
    } else if (lifecycleStr === 'VTOL_TAKEOFF' || lifecycleStr === 'PREFLIGHT_ARMING') {
        stepGround.className = 'lifecycle-step completed';
        stepTakeoff.className = 'lifecycle-step active';
    } else if (lifecycleStr.startsWith('EXECUTING_')) {
        stepGround.className = 'lifecycle-step completed';
        stepTakeoff.className = 'lifecycle-step completed';
        stepMission.className = 'lifecycle-step active';
    } else if (lifecycleStr === 'RETURN_APPROACH') {
        stepGround.className = 'lifecycle-step completed';
        stepTakeoff.className = 'lifecycle-step completed';
        stepMission.className = 'lifecycle-step completed';
        stepReturn.className = 'lifecycle-step active';
    } else if (lifecycleStr === 'PRECISION_LANDING' || lifecycleStr === 'COMPLETED_LANDED') {
        stepGround.className = 'lifecycle-step completed';
        stepTakeoff.className = 'lifecycle-step completed';
        stepMission.className = 'lifecycle-step completed';
        stepReturn.className = 'lifecycle-step completed';
        stepLand.className = 'lifecycle-step active';
    }
}

function updateDashboard(tel) {
    const lat = tel.gps[0];
    const lon = tel.gps[1];
    const alt = tel.altitude;
    const yaw = tel.attitude[2];
    const roll = tel.attitude[0];
    const pitch = tel.attitude[1];
    const isFW = tel.mode.includes('FIXED_WING') || tel.mode.includes('FW');

    // Update Telemetry Cards
    elFlightMode.innerText = tel.mode;
    elBattery.innerText = tel.battery + 'V';
    elAlt.innerText = alt.toFixed(1);
    elAirspeed.innerText = tel.airspeed.toFixed(1);
    elGroundspeed.innerText = tel.groundspeed.toFixed(1);
    elHeading.innerText = (yaw >= 0 ? yaw : 360 + yaw).toFixed(1) + '°';

    if (tel.energy) {
        elSavings.innerText = tel.energy.savings_percent.toFixed(1);
    }

    // Update Drone Marker & Map
    droneMarker.setLatLng([lat, lon]);
    droneMarker.setIcon(createDroneIcon(yaw, isFW));

    flownCoordinates.push([lat, lon]);
    if (flownCoordinates.length > 500) flownCoordinates.shift();
    liveFlownPolyline.setLatLngs(flownCoordinates);

    // Update 3D Plot
    updatePlotly3D(tel.position[0], tel.position[1], alt);

    // Update PFD Artificial Horizon
    const pitchOffset = Math.min(Math.max(pitch * 2, -40), 40);
    horizonPitchRoll.setAttribute('transform', `rotate(${-roll}, 80, 60) translate(0, ${pitchOffset})`);

    // Update Motor RPM & Thrust Bars
    if (tel.motors) {
        const m = tel.motors;
        const vtolPct = Math.min((m.m0_rpm / 12000) * 100, 100);
        const tracPct = Math.min((m.m4_rpm / 15000) * 100, 100);

        barM0.style.width = vtolPct + '%';
        barM1.style.width = vtolPct + '%';
        barM2.style.width = vtolPct + '%';
        barM3.style.width = vtolPct + '%';
        barM4.style.width = tracPct + '%';

        rpmM0.innerText = m.m0_rpm + ' RPM';
        rpmM1.innerText = m.m1_rpm + ' RPM';
        rpmM2.innerText = m.m2_rpm + ' RPM';
        rpmM3.innerText = m.m3_rpm + ' RPM';
        rpmM4.innerText = m.m4_rpm + ' RPM';
    }

    // Update Lifecycle Bar
    if (tel.lifecycle) {
        updateLifecycleUI(tel.lifecycle);
    }
}

// ==============================================================================
//  5. UI Event Listeners & Scenario Controls
// ==============================================================================

scenarioCards.forEach(card => {
    card.addEventListener('click', () => {
        scenarioCards.forEach(c => c.classList.remove('active'));
        card.classList.add('active');
        selectedScenario = card.getAttribute('data-scenario');
        drawPlannedScenarioPath(selectedScenario);
    });
});

btnStartScenario.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        console.log('Sending start scenario:', selectedScenario);
        ws.send(JSON.stringify({ action: 'scenario', name: selectedScenario }));
    }
});

btnRtl.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ action: 'rtl' }));
    }
});

btnClearPath.addEventListener('click', () => {
    flownCoordinates = [];
    liveFlownPolyline.setLatLngs([]);
    traj3D_X = [0];
    traj3D_Y = [0];
    traj3D_Z = [0];
});

btnLayerSat.addEventListener('click', () => {
    map.removeLayer(osmLayer);
    map.addLayer(satLayer);
    btnLayerSat.classList.add('active');
    btnLayerOsm.classList.remove('active');
});

btnLayerOsm.addEventListener('click', () => {
    map.removeLayer(satLayer);
    map.addLayer(osmLayer);
    btnLayerOsm.classList.add('active');
    btnLayerSat.classList.remove('active');
});

btnRecenter.addEventListener('click', () => {
    if (droneMarker) {
        map.setView(droneMarker.getLatLng(), 18);
    }
});

// Initialization
window.addEventListener('DOMContentLoaded', () => {
    initLeafletMap();
    initPlotly3D();
    connectWebSocket();
});
