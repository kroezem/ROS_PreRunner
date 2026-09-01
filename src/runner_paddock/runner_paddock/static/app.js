const statusNode = document.querySelector('#connection');
const debugNode = document.querySelector('#debug');
const latest = {};

function render() {
  debugNode.textContent = JSON.stringify(latest, null, 2);
}

function connect() {
  const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';
  const socket = new WebSocket(`${scheme}://${window.location.host}/ws`);

  socket.addEventListener('open', () => {
    statusNode.textContent = 'Connected';
    statusNode.className = 'status connected';
  });
  socket.addEventListener('message', (event) => {
    try {
      const frame = JSON.parse(event.data);
      latest[frame.type || 'unknown'] = frame;
      render();
    } catch (error) {
      latest.error = `Invalid server frame: ${error}`;
      render();
    }
  });
  socket.addEventListener('close', () => {
    statusNode.textContent = 'Disconnected — retrying';
    statusNode.className = 'status waiting';
    window.setTimeout(connect, 1000);
  });
}

connect();
if ('serviceWorker' in navigator) {
  navigator.serviceWorker.register('/static/service-worker.js');
}
