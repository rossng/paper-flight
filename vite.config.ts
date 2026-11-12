import { defineConfig } from 'vite';
export default defineConfig({ base: './', build: { rollupOptions: { input: { main: 'index.html', foldingStudy: 'folding-study.html' }, output: { manualChunks: { three: ['three'] } } } }, server: { host: '0.0.0.0', allowedHosts: true } });
