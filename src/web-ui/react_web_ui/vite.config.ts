import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// https://vite.dev/config/
export default defineConfig({
  plugins: [react()],
  server: {
    host: '0.0.0.0',
    port: 5183,
    proxy: {
      // forward all `/api/...` calls to ROS2 backend
      '/api': {
        target: 'http://0.0.0.0:8000',
        changeOrigin: true,
        // optional: rewrite if backend does not expect `/api`
        // rewrite: (path) => path.replace(/^\/api/, ''),
      },
      // forward WebSocket connections to ROS2 backend
      '/ws': {
        target: 'ws://0.0.0.0:8000',
        ws: true,
        changeOrigin: true,
      },
    },
  }
})
