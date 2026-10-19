import { defineConfig } from 'vite';
import { resolve } from 'path';

export default defineConfig({
  base: './',
  resolve: {
    alias: {
      '@core': resolve(__dirname, 'src/core'),
      '@c4isr': resolve(__dirname, 'src/c4isr'),
      '@fusion': resolve(__dirname, 'src/fusion'),
      '@threat': resolve(__dirname, 'src/threat'),
    },
  },
  build: {
    target: 'es2022',
    outDir: 'dist',
    sourcemap: true,
    minify: 'esbuild',
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
      },
    },
  },
  server: {
    port: 5173,
    strictPort: true,
    cors: true,
  },
});
