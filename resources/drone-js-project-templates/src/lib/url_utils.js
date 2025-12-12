/**
 * Shared URL utilities for TensorFleet drone JS template.
 */

/**
 * Convert a VM Manager URL (http/https or ws/wss) to a proxy WebSocket URL.
 *
 * @param {string} vmManagerUrl - The base VM Manager URL
 * @returns {string} The derived proxy WebSocket URL, or empty string on failure
 */
function toProxyWebSocketUrl(vmManagerUrl) {
    if (!vmManagerUrl) return '';
    try {
        const url = new URL(vmManagerUrl);

        // Already a WebSocket URL - just ensure /ws path
        if (url.protocol === 'ws:' || url.protocol === 'wss:') {
            if (!url.pathname || url.pathname === '/') {
                url.pathname = '/ws';
            }
            return url.toString();
        }

        // Convert HTTP(S) to WS(S)
        const protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
        const basePath = url.pathname.replace(/\/$/, '');
        const pathName = basePath.endsWith('/ws') ? basePath : `${basePath}/ws`;

        return `${protocol}//${url.host}${pathName}`;
    } catch {
        return '';
    }
}

module.exports = { toProxyWebSocketUrl };
