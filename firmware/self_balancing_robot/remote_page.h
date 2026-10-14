// Phone remote web page served in REMOTE_WIFI mode. Hold an arrow to drive
// (the command repeats every 150 ms, well inside the remote timeout); release
// sends L. STOP is the emergency stop, GO allows balancing again.
#pragma once

constexpr const char kRemotePage[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>BalanceBot</title>
<style>
body{font-family:sans-serif;text-align:center;margin:0;padding:16px;background:#111;color:#eee;
  user-select:none;-webkit-user-select:none;touch-action:none}
#status{font-size:16px;line-height:1.5;min-height:3em}
.pad{display:grid;grid-template-columns:repeat(3,88px);grid-template-rows:repeat(3,88px);
  gap:12px;justify-content:center;margin:24px auto}
button{font-size:30px;border:0;border-radius:16px;background:#333;color:#fff}
button.on{background:#2a7}
#stop{background:#c22;font-size:20px}
#go{background:#262;font-size:20px}
</style></head><body>
<h3>BalanceBot</h3>
<div id="status">connecting...</div>
<div class="pad">
<span></span><button data-c="W">&#9650;</button><span></span>
<button data-c="A">&#9664;</button><button id="stop">STOP</button><button data-c="D">&#9654;</button>
<span></span><button data-c="S">&#9660;</button><button id="go">GO</button>
</div>
<script>
let held = null, timer = null;
function send(c) { fetch('/cmd?c=' + c).catch(() => {}); }
function press(button) {
  held = button.dataset.c;
  button.classList.add('on');
  send(held);
  clearInterval(timer);
  timer = setInterval(() => send(held), 150);
}
function release(button) {
  button.classList.remove('on');
  if (held === null) return;
  held = null;
  clearInterval(timer);
  send('L');
}
document.querySelectorAll('[data-c]').forEach(button => {
  button.addEventListener('pointerdown', e => {
    e.preventDefault();
    button.setPointerCapture(e.pointerId);
    press(button);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(type =>
    button.addEventListener(type, () => release(button)));
});
document.getElementById('stop').onclick = () => send('X');
document.getElementById('go').onclick = () => send('G');
async function poll() {
  const status = document.getElementById('status');
  try {
    const s = await (await fetch('/status')).json();
    status.textContent = s.state + ' | angle ' + s.angle.toFixed(1) + '° | speed ' +
      s.speed.toFixed(2) + ' m/s | ' + (s.battery > 0 ? s.battery.toFixed(2) + ' V' : 'battery: n/a');
  } catch (e) {
    status.textContent = 'no connection to the robot';
  }
  setTimeout(poll, 500);
}
poll();
</script>
</body></html>
)HTML";
