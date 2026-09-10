// Bump CACHE on every shell asset change: a same-named cache is never
// refreshed, so a stale precache would outlive a redeploy.
const CACHE = 'runner-paddock-shell-v7';
const ASSETS = [
  '/',
  '/static/app.js',
  '/static/map_geometry.js',
  '/static/joystick_geometry.js',
  '/static/style.css',
  '/static/manifest.webmanifest',
];

self.addEventListener('install', (event) => {
  // Precache from the network, not the HTTP cache, and activate immediately.
  self.skipWaiting();
  event.waitUntil(caches.open(CACHE).then((cache) => cache.addAll(
    ASSETS.map((url) => new Request(url, { cache: 'reload' })),
  )));
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) => Promise.all(
      keys.filter((key) => key !== CACHE).map((key) => caches.delete(key)),
    )).then(() => self.clients.claim()),
  );
});

self.addEventListener('fetch', (event) => {
  if (event.request.method === 'GET') {
    event.respondWith(
      fetch(event.request).catch(() => caches.match(event.request)),
    );
  }
});
