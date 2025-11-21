// Simple Bun static file server for the gzweb panel (no Vite build needed).
const port = Number(process.env.PORT || 3000);
const root = new URL('../src/static/gzweb/', import.meta.url);

Bun.serve({
  port,
  async fetch(req) {
    const url = new URL(req.url);
    const pathname = url.pathname === '/' ? '/index.html' : url.pathname;
    const fileUrl = new URL('.' + pathname, root);
    const file = Bun.file(fileUrl);

    if (!(await file.exists())) {
      return new Response('Not found', { status: 404 });
    }

    return new Response(file, {
      headers: { 'Content-Type': file.type || 'application/octet-stream' },
    });
  },
});

console.log(`gzweb panel serving on http://localhost:${port}`);
