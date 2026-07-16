'use strict';
/*
 * Verificacion headless del simulador (correr con: node verificacion-orbita.js).
 * Motor IDENTICO al de index.html. Resultado de referencia (2026-07-16):
 *   38.74 vueltas en 60.000 pasos y seguia orbitando; r entre 72 y 181 (roseta que precesa);
 *   torque de reposicion medio 34.5% de la aceleracion central (tope 40%).
 * Datos del camino, tambien medidos aca:
 *   - velocidad del sonido del mar ~0.5 px/paso -> el planeta orbita cerca de Mach 1,
 *     por eso radia ondas a lo bestia (analogo exagerado de la radiacion gravitacional);
 *   - SIN el servo de reposicion, el planeta cae en espiral en ~2.3 vueltas (robusto
 *     en todas las configuraciones barridas: acople, densidad del mar, rigidez);
 *   - un servo que multiplica toda la velocidad inyecta energia radial y lo hace ESCAPAR:
 *     tiene que ser torque puro tangencial, acotado, como aca.
 */

const W = 600, H = 600, CX = W / 2, CY = H / 2;
const KII = 14, KTSOL = 400, KTP = 40, MASA = 0.03, MIND2 = 36, PLAYA = 60;
const N = Math.round(W * H / 500);
const SERVO_K = 3;          /* ganancia sobre el deficit relativo de L */
const SERVO_TOPE = 0.4;     /* el empuje jamas supera 40% de la aceleracion central */

const px = new Float64Array(N), py = new Float64Array(N);
const pvx = new Float64Array(N), pvy = new Float64Array(N);

function initSea() {
  for (let i = 0; i < N; i++) {
    px[i] = Math.random() * W; py[i] = Math.random() * H;
    pvx[i] = 0; pvy[i] = 0;
  }
}

function step(pl, modo) {
  let plfx = 0, plfy = 0;
  const plx = pl ? pl.x : 0, ply = pl ? pl.y : 0;
  for (let i = 0; i < N; i++) {
    const xi = px[i], yi = py[i];
    let ax = 0, ay = 0;
    for (let j = i + 1; j < N; j++) {
      const dx = xi - px[j], dy = yi - py[j];
      let d2 = dx * dx + dy * dy; if (d2 < MIND2) d2 = MIND2;
      const fz = KII / (d2 * Math.sqrt(d2));
      ax += dx * fz; ay += dy * fz;
      pvx[j] -= dx * fz; pvy[j] -= dy * fz;
    }
    {
      const dx = xi - CX, dy = yi - CY;
      let d2 = dx * dx + dy * dy; if (d2 < MIND2) d2 = MIND2;
      const fz = KTSOL / (d2 * Math.sqrt(d2));
      ax += dx * fz; ay += dy * fz;
    }
    if (pl) {
      const dx = xi - plx, dy = yi - ply;
      let d2 = dx * dx + dy * dy; if (d2 < MIND2) d2 = MIND2;
      const fz = KTP / (d2 * Math.sqrt(d2));
      ax += dx * fz; ay += dy * fz;
      plfx -= dx * fz * MASA; plfy -= dy * fz * MASA;
    }
    pvx[i] += ax; pvy[i] += ay;
  }
  for (let i = 0; i < N; i++) {
    if (modo === 'frio') {
      pvx[i] *= 0.995; pvy[i] *= 0.995;
    } else {
      const dxb = Math.min(px[i], W - px[i]), dyb = Math.min(py[i], H - py[i]);
      const db = Math.min(dxb, dyb);
      if (db < PLAYA) {
        const s = 1 - db / PLAYA;
        const f = 1 - 0.05 * s * s;
        pvx[i] *= f; pvy[i] *= f;
      }
    }
    px[i] += pvx[i]; py[i] += pvy[i];
    if (px[i] < 0) { px[i] = 0; pvx[i] *= -.6; } else if (px[i] > W) { px[i] = W; pvx[i] *= -.6; }
    if (py[i] < 0) { py[i] = 0; pvy[i] *= -.6; } else if (py[i] > H) { py[i] = H; pvy[i] *= -.6; }
  }
  if (pl && !pl.fixed) { pl.vx += plfx; pl.vy += plfy; pl.x += pl.vx; pl.y += pl.vy; }
  return [plfx, plfy];
}

console.log(`N=${N}, KII=${KII}, KTSOL=${KTSOL}, KTP=${KTP}, servo K=${SERVO_K}`);
initSea();
for (let s = 0; s < 3500; s++) step(null, 'frio');

const R0 = 150;
const pl = { x: CX + R0, y: CY, vx: 0, vy: 0, fixed: true };
for (let s = 0; s < 700; s++) step(pl, 'frio');
let fx = 0, cnt = 0;
for (let s = 0; s < 300; s++) { const f = step(pl, 'frio'); fx += f[0]; cnt++; }
const aR = -fx / cnt, vc = Math.sqrt(aR * R0);
console.log(`a_r=${aR.toExponential(3)}  v_circ=${vc.toFixed(3)}  (periodo ~${Math.round(2 * Math.PI * R0 / vc)} pasos)`);

pl.fixed = false; pl.vx = 0; pl.vy = vc;
const L0 = R0 * vc;                       /* momento angular objetivo */
let th = 0, prev = Math.atan2(pl.y - CY, pl.x - CX);
let rmin = R0, rmax = R0, out = 'SIGUE ORBITANDO al cortar';
let boostAcum = 0, boostMax = 0;          /* tamano de la correccion, para declararla */
const MAXS = 60000;
let s = 0;
const t0 = Date.now();
for (; s < MAXS; s++) {
  step(pl, 'vivo');
  /* --- servo de momento angular v2: torque puro, tangencial, acotado --- */
  {
    const rx = pl.x - CX, ry = pl.y - CY, r = Math.hypot(rx, ry) || 1;
    const L = rx * pl.vy - ry * pl.vx;
    const deficit = (L0 - L) / L0;
    if (deficit > 0) {
      const g = aR * Math.min(SERVO_TOPE, SERVO_K * deficit);
      /* direccion tangencial del giro original (L0 > 0 = antihorario) */
      pl.vx += g * (-ry / r); pl.vy += g * (rx / r);
      const rel = g / aR;
      boostAcum += rel; if (rel > boostMax) boostMax = rel;
    }
  }
  const dx = pl.x - CX, dy = pl.y - CY, r = Math.hypot(dx, dy);
  if (r < rmin) rmin = r; if (r > rmax) rmax = r;
  const a = Math.atan2(dy, dx);
  let d = a - prev; if (d > Math.PI) d -= 2 * Math.PI; if (d < -Math.PI) d += 2 * Math.PI;
  th += d; prev = a;
  if (r < 30) { out = 'CAPTURADO'; break; }
  if (r > 260) { out = 'ESCAPO'; break; }
  if (s > 0 && s % 6000 === 0)
    console.log(`  paso ${s}: r=${r.toFixed(0)}, vueltas=${(Math.abs(th) / (2 * Math.PI)).toFixed(2)}, v=${Math.hypot(pl.vx, pl.vy).toFixed(3)}, torque medio=${(boostAcum / s * 100).toFixed(1)}% de a_central`);
}
const vueltas = Math.abs(th) / (2 * Math.PI);
console.log(`\n-> ${out} (paso ${s}); vueltas=${vueltas.toFixed(2)}; r en [${rmin.toFixed(0)}, ${rmax.toFixed(0)}]`);
console.log(`   torque del servo: medio ${(boostAcum / Math.max(1, s) * 100).toFixed(1)}% de a_central, maximo ${(boostMax * 100).toFixed(0)}%  (${((Date.now() - t0) / 1000).toFixed(0)}s)`);
console.log('listo.');
