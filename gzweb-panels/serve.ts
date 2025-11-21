// Simple Bun static file server for the standalone gzweb panel.
const port = Number(process.env.PORT || 3000);
const root = new URL('./', import.meta.url);

const server = Bun.serve({
  port,
  async fetch(req) {
    const url = new URL(req.url);
    const pathname = url.pathname === '/' ? '/index.html' : url.pathname;
    const fileUrl = new URL('.' + pathname, root);
    const file = Bun.file(fileUrl);

    if (!(await file.exists())) {
      return new Response('Not found', { status: 404 });
    }

    const contentType = file.type || 'application/octet-stream';
    return new Response(file, {
      headers: {
        'Content-Type': contentType,
      },
    });
  },
});
