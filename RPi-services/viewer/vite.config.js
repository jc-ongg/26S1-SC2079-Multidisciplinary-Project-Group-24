import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Pi hotspot/LAN address and the local image-api.
// Override without editing this file:  PI_HOST=... API_HOST=... npm run dev
const PI_HOST = process.env.PI_HOST || 'http://192.168.18.19:5000'
const API_HOST = process.env.API_HOST || 'http://localhost:8000'

export default defineConfig({
  plugins: [react()],
  server: {
    host: true,
    port: 5173,
    proxy: {
      // MJPEG stream from the Raspberry Pi
      '/stream': {
        target: PI_HOST,
        changeOrigin: true,
        rewrite: (p) => p.replace(/^\/stream/, ''),
      },
      // YOLO inference API running locally
      '/api': {
        target: API_HOST,
        changeOrigin: true,
        rewrite: (p) => p.replace(/^\/api/, ''),
      },
    },
  },
})
