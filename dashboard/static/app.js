'use strict';
const el = id => document.getElementById(id);
let token = '', current = null, commandBusy = false, cameraBusy = false, cameraDirty = false;
let addressesLoaded = false, showingVideo = false;
const rows = new Map();

async function post(path, body = {}) {
  if (!token) throw Error('Dashboard is still connecting. Try again.');
  const response = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json','X-Dashboard-Token':token},
    body:JSON.stringify(body), signal:AbortSignal.timeout(10000)});
  const result = await response.json();
  if (!response.ok) throw Error(result.error || 'Command failed');
  return result;
}
function badge(id, text, good = false, bad = false) {
  el(id).textContent = text; el(id).className = 'badge' + (good ? ' good' : bad ? ' bad' : '');
}
function message(text, error = false) {
  el('result').textContent = text; el('result').style.borderColor = error ? 'var(--danger)' : 'var(--line)';
}
async function servoCommand(name, values = {}) {
  if (commandBusy && name !== 'stop') return;
  commandBusy = true; renderControls();
  try { const result = await post('/api/servo/' + name, values); message(result.message || 'Command applied.'); }
  catch (error) { message(error.message + (name === 'stop' ? ' Stop is NOT confirmed; use the servo power switch if needed.' : ''), true); }
  finally { commandBusy = false; renderControls(); }
}
function renderControls() {
  const device = current?.servo, s = device?.data;
  const blocked = document.visibilityState !== 'visible' || !device?.online || !s?.servoReady || s?.faulted || s?.running || !current?.operator_alive || commandBusy;
  el('all').disabled = el('groups').disabled = blocked || !!s?.manualActive;
  rows.forEach(row => {
    row.enable.disabled = blocked || row.active;
    row.release.disabled = commandBusy || !device?.online || !row.active;
    row.slider.disabled = blocked || !row.active;
  });
}
function renderServos(s) {
  const channels = s.channels.split(',').map(Number).filter(n => Number.isInteger(n) && n >= 0 && n <= 15);
  channels.forEach((channel, index) => {
    let row = rows.get(channel);
    if (!row) {
      const box = document.createElement('div'); box.className = 'servo';
      box.innerHTML = '<div class="servo-top"><label></label><span class="pulse"></span></div><input type="range" step="1"><div class="servo-buttons"><button class="enable">Enable at center</button><button class="release">Release</button></div>';
      row = {box, slider:box.querySelector('input'), enable:box.querySelector('.enable'), release:box.querySelector('.release'), active:false};
      row.slider.id = 'channel-' + channel; box.querySelector('label').htmlFor = row.slider.id;
      box.querySelector('label').textContent = 'Channel ' + channel;
      row.slider.onchange = () => servoCommand('servo', {channel, pulse:Number(row.slider.value)});
      row.enable.onclick = () => servoCommand('servo', {channel, pulse:current.servo.data.center});
      row.release.onclick = () => servoCommand('release', {channel});
      rows.set(channel, row); el('servos').append(box);
    }
    row.active = s.manual[index] !== null;
    row.slider.min = s.manualMin; row.slider.max = s.manualMax;
    if (document.activeElement !== row.slider) row.slider.value = row.active ? s.manual[index] : s.center;
    row.box.querySelector('.pulse').textContent = row.active ? s.manual[index] + ' µs commanded' : 'Pulses off';
  });
}
function clearFeed() {
  el('feed').removeAttribute('src'); el('feed').hidden = true; el('placeholder').hidden = false; showingVideo = false;
}
function render(state) {
  current = state; token = state.token || token;
  if (!addressesLoaded) {
    el('servoAddress').value = state.config.servo; el('cameraAddress').value = state.config.camera; addressesLoaded = true;
  }
  badge('operator', state.operator_alive ? 'Operator heartbeat live' : 'Operator disconnected', state.operator_alive);
  badge('servoOnline', state.servo.online ? 'Connected' : 'Offline', state.servo.online);
  badge('cameraOnline', state.camera.online ? 'Connected' : 'Offline', state.camera.online);
  const s = state.servo.data;
  if (state.servo.online && s) {
    renderServos(s); el('controllerMessage').textContent = s.message;
    badge('servoState', s.faulted ? 'Fault' : s.running ? 'Sweep running' : s.manualActive ? 'Manual active' : 'Outputs off', !s.faulted, s.faulted);
    const live = s.imuReady && s.imuValid;
    badge('imuState', live ? 'Live' : 'Unavailable', live);
    el('accel').textContent = live ? s.accel.map(n => n.toFixed(3)).join(' / ') : '— / — / —';
    el('gyro').textContent = live ? s.gyro.map(n => n.toFixed(2)).join(' / ') : '— / — / —';
  } else {
    el('controllerMessage').textContent = state.servo.error;
    badge('servoState', 'Unavailable'); badge('imuState', 'No data');
    el('accel').textContent = el('gyro').textContent = '— / — / —';
  }
  renderControls();
  const camera = state.camera.data, ready = state.camera.online && camera?.ready;
  ['resolution','fps','quality','applyCamera'].forEach(id => el(id).disabled = !ready || cameraBusy);
  if (ready && !cameraDirty && !cameraBusy) {
    el('resolution').value = camera.resolution; el('fps').value = String(camera.targetFps); el('quality').value = String(camera.quality);
  }
  el('cameraStats').textContent = ready ? camera.width + ' × ' + camera.height + ', target ' + camera.targetFps + ' fps. Camera send rate: ' + camera.sentFps + ' fps. Settings reset on camera restart.' : state.camera.error || camera?.error || 'Waiting for camera.';
  const p = state.pipeline;
  el('videoToggle').textContent = p.enabled ? 'Stop video' : 'Start video'; el('videoToggle').disabled = cameraBusy;
  el('receiveFps').textContent = p.fresh ? p.receive_fps.toFixed(1) + ' fps' : '—';
  el('frameAge').textContent = p.fresh && p.receive_age_ms !== null ? p.receive_age_ms + ' ms' : '—';
  if (p.enabled && p.fresh) {
    if (!showingVideo) { el('feed').src = '/api/video.mjpeg?t=' + Date.now(); showingVideo = true; }
    el('feed').hidden = false; el('placeholder').hidden = true;
  } else {
    clearFeed(); el('videoMessage').textContent = p.error || 'Waiting for a fresh camera frame...';
  }

}
async function poll() {
  try {
    if (token && document.visibilityState === 'visible') await post('/api/heartbeat').catch(() => {});
    const response = await fetch('/api/state', {cache:'no-store', signal:AbortSignal.timeout(2000)});
    if (!response.ok) throw Error('Local server unavailable');
    render(await response.json());
  } catch (error) {
    current = null; renderControls(); clearFeed(); badge('operator', 'PC server unavailable', false, true);
    el('accel').textContent = el('gyro').textContent = '— / — / —';
    el('frameAge').textContent = el('receiveFps').textContent = '—';
  } finally { setTimeout(poll, 500); }
}
el('stop').onclick = () => servoCommand('stop');
el('all').onclick = () => servoCommand('start', {mode:'all'});
el('groups').onclick = () => servoCommand('start', {mode:'groups'});
el('connect').onclick = async () => {
  try {
    await post('/api/config', {servo:el('servoAddress').value, camera:el('cameraAddress').value, stream_port:81});
    message('Addresses saved. Reconnecting; servo motion stays stopped.');
  } catch (error) { message(error.message, true); }
};
el('videoToggle').onclick = async () => {
  try { await post('/api/video', {enabled:!current?.pipeline.enabled}); }
  catch (error) { message(error.message, true); }
};
el('feed').onerror = clearFeed;
['resolution','fps','quality'].forEach(id => el(id).onchange = () => { cameraDirty = true; });
el('applyCamera').onclick = async () => {
  cameraBusy = true; clearFeed(); if (current) render(current);
  try {
    await post('/api/camera/settings', {resolution:el('resolution').value, fps:Number(el('fps').value), quality:Number(el('quality').value)});
    cameraDirty = false; message('Camera settings applied. Video resumes if it was running.');
  } catch (error) { message('Camera settings not confirmed: ' + error.message, true); }
  finally { cameraBusy = false; }
};
function cancelMotion() {
  if (token) post('/api/servo/stop').catch(() => {});
}
document.addEventListener('visibilitychange', () => { renderControls(); if (document.visibilityState !== 'visible') cancelMotion(); });
window.addEventListener('pagehide', () => {
  if (token) fetch('/api/servo/stop', {method:'POST', headers:{'Content-Type':'application/json','X-Dashboard-Token':token}, body:'{}', keepalive:true}).catch(() => {});
});
renderControls(); poll();
