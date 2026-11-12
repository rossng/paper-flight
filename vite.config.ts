import { defineConfig } from 'vite';
export default defineConfig({ base: './', build: { rollupOptions: { output: { manualChunks: { three: ['three'] } } } }, server: { host: '0.0.0.0', allowedHosts: true } });
