import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { execFile } from 'child_process'
import path from 'path'
import { fileURLToPath } from 'url'

const __dirname  = path.dirname(fileURLToPath(import.meta.url))
const projectRoot = path.resolve(__dirname, '..')

export default defineConfig({
  plugins: [
    react(),
    {
      name: 'qm-solver',
      configureServer(server) {
        server.middlewares.use('/api/qm-solve', (req, res, next) => {
          if (req.method !== 'POST') { next(); return; }

          let body = '';
          req.on('data', chunk => { body += chunk.toString(); });
          req.on('error', () => {
            res.statusCode = 500;
            res.setHeader('Content-Type', 'application/json');
            res.end(JSON.stringify({ error: 'Request stream error' }));
          });
          req.on('end', () => {
            let parsed;
            try { parsed = JSON.parse(body); }
            catch (e) {
              res.statusCode = 400;
              res.setHeader('Content-Type', 'application/json');
              res.end(JSON.stringify({ error: 'Invalid JSON body' }));
              return;
            }

            const { numVars, minterms = [], dontcares = [], varNames = [] } = parsed;
            const inlineStr = [
              String(numVars),
              minterms.join(','),
              dontcares.join(','),
              varNames.join(','),
            ].join(';');

            const simulatorPath = path.join(projectRoot, 'simulator')
            execFile(
              simulatorPath,
              ['--qm-inline', inlineStr],
              { cwd: projectRoot, maxBuffer: 4 * 1024 * 1024 },
              (err, stdout, stderr) => {
                res.setHeader('Content-Type', 'application/json');
                if (err) {
                  res.statusCode = 500;
                  res.end(JSON.stringify({ error: (stderr || err.message).trim() }));
                  return;
                }
                res.end(stdout);
              }
            );
          });
        });
      },
    },
  ],
  server: {
    proxy: {
      // 透明轉發到 Google Gemini API，避免瀏覽器 CORS 限制。
      // API key 由前端以 query param 帶入，Vite 只做轉發，不儲存。
      '/api/gemini': {
        target:       'https://generativelanguage.googleapis.com',
        changeOrigin: true,
        rewrite: path => path.replace(/^\/api\/gemini/, ''),
      },
    },
  },
})
