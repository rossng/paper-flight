import { defineConfig } from 'vite';
// The flight solver is instantiated with a top-level await (src/physics/solver.ts), which
// a worker can only do as an ES module.
export default defineConfig({ base: './', worker: { format: 'es' }, build: { rollupOptions: { output: { manualChunks: { three: ['three'] } } } }, server: { host: '0.0.0.0', allowedHosts: true } });
