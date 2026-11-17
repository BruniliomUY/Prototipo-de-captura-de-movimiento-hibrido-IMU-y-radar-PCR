#!/usr/bin/env python3
"""
Barrido de parámetros del radar, una configuración por vez, con el péndulo en movimiento.

Para cada configuración:
  1) péndulo QUIETO  -> Enter -> calibra el radar (igual que un ON de master.py)
  2) arrancás el péndulo -> Enter -> mide SEGUNDOS leyendo IMU + radar como el loop de master.py
  3) imprime las estadísticas de esa configuración y pasa a la siguiente

Usa directamente init_radar, leer_radar e Imu de master.py (misma conexión, mismos reintentos).
No necesita ningún otro archivo. Va en src/rpi_master/ y se corre con master.py DETENIDO:

    python3 barrido_paso_a_paso.py                        # todas las configuraciones
    python3 barrido_paso_a_paso.py --configs 0 3          # solo algunas (por número)
    python3 barrido_paso_a_paso.py --referencia 0.153     # + sesgo contra una medida a mano

Guarda un CSV crudo por configuración y barrido_<fecha>_resumen.csv con una fila por configuración.
"""

import argparse
import csv
import sys
import time
from datetime import datetime

import numpy as np

import master
from master import Imu, crear_reset_imu, init_radar, leer_radar
from master import a121, ReflectorShape

SEGUNDOS = 30             # medición por configuración (con ~3 Hz son ~90 lecturas, ~7 oscilaciones)

# --- Configuraciones: cada una es la base + los cambios indicados ---
BASE = dict(master.RADAR_CONFIG, reflector_shape=ReflectorShape.PLANAR)
CONFIGS = [
    ("base (PLANAR)",       {}),
    ("PROFILE_1",           {"max_profile": a121.Profile.PROFILE_1}),
    ("step 1",              {"max_step_length": 1}),
    ("signal_quality 15",   {"signal_quality": 15.0}),
    ("signal_quality 30",   {"signal_quality": 30.0}),
    ("sin close range",     {"close_range_leakage_cancellation": False, "start_m": 0.07}),
    ("end_m 0.30",          {"end_m": 0.30}),
]

# --- Análisis ---
DIST_MIN_M, DIST_MAX_M = 0.03, 0.40   # fuera de esto = eco fantasma (los ~0,58 m)
ERROR_GRUESO_M = 0.03                 # a más de 3 cm de la curva distancia-ángulo = eco equivocado
EXTREMO_DEG = 25.0                    # |pitch| mayor = extremo de la oscilación
UMBRAL_PASO_DEG = -10.0               # un paso empieza en cada mínimo de pitch por debajo de esto
SEPARACION_PASOS_S = 1.5


# ===========================================================================
def cerrar(client, detector):
    for obj, metodo in ((detector, "stop"), (client, "close")):
        if obj is not None:
            try:
                getattr(obj, metodo)()
            except Exception:
                pass


def medir_config(imu_dev, nombre, segundos):
    """Calibra, mide y devuelve (filas, reconexiones). filas = (t, pitch, imu_ok, dist o None)."""
    input("\n  Péndulo QUIETO en reposo. Enter para calibrar...")
    t = time.time()
    client, det, contexto = init_radar()
    cerrar(client, det)            # se cierra para que no se acumulen frames mientras arrancás
    print(f"  Calibrado en {time.time() - t:.0f} s")

    input("  ARRANCÁ el péndulo y, cuando oscile estable, Enter para medir...")
    client, det, contexto = init_radar(contexto_guardado=contexto)   # reusa la calibración
    filas, reconexiones, ultimo_aviso = [], 0, None
    t_fin = time.time() + segundos
    try:
        while time.time() < t_fin:
            ts = time.time()
            imu = imu_dev.leer()
            r = leer_radar(det)
            if r["desconectado"]:
                reconexiones += 1
                print("  [RADAR] Reconectando...", file=sys.stderr)
                cerrar(client, det)
                client = det = None
                client, det, contexto = init_radar(contexto_guardado=contexto)
                continue
            filas.append((ts, imu["euler"][1], imu["ok"], r["distancia_m"]))
            restante = int(max(t_fin - time.time(), 0))
            if restante != ultimo_aviso:
                ultimo_aviso = restante
                print(f"\r  midiendo... {len(filas)} lecturas, faltan {restante:3d} s ",
                      end="", flush=True)
    finally:
        cerrar(client, det)
    print()
    return filas, reconexiones


def estadisticas(filas, segundos, referencia):
    t = np.array([f[0] for f in filas])
    p = np.array([f[1] for f in filas], dtype=float)
    ok = np.array([bool(f[2]) for f in filas])
    d = np.array([np.nan if f[3] is None else f[3] for f in filas])
    n = len(filas)
    e = dict(lecturas=n, hz=n / segundos)
    if n < 10:
        return e

    detecto = ~np.isnan(d)
    en_rango = detecto & (d >= DIST_MIN_M) & (d <= DIST_MAX_M)
    e["deteccion_pct"] = 100 * detecto.mean()
    e["fantasmas_pct"] = 100 * (detecto & ~en_rango).sum() / max(detecto.sum(), 1)

    # Curva distancia = f(pitch): lo que se aparta de ella es ruido (o un eco equivocado)
    usar = en_rango & ok
    valida = np.zeros(n, bool)
    if usar.sum() >= 12:
        x, y = p[usar], d[usar]
        sigue = np.ones(len(x), bool)
        for _ in range(10):
            coef = np.polyfit(x[sigue], y[sigue], 3)
            r = y - np.polyval(coef, x)
            nuevo = np.abs(r) <= ERROR_GRUESO_M
            if (nuevo == sigue).all():
                break
            sigue = nuevo
        valida[np.where(usar)[0][sigue]] = True
        rb = r[sigue]
        e["ruido_mm"] = 1000 * 1.4826 * np.median(np.abs(rb - np.median(rb)))
        e["equivocados_pct"] = 100 * (~sigue).sum() / len(x)
        if referencia is not None:
            e["sesgo_mm"] = 1000 * (np.polyval(coef, 0.0) - referencia)
    else:
        valida = en_rango

    extremos = ok & (np.abs(p) > EXTREMO_DEG)
    if extremos.any():
        e["detec_extremos_pct"] = 100 * valida[extremos].mean()

    # Mínimo de distancia por paso (un paso = de un mínimo de pitch al siguiente)
    ps = np.convolve(p, np.ones(3) / 3, mode="same")
    inicios = []
    for i in range(1, n - 1):
        if ok[i] and ps[i] < UMBRAL_PASO_DEG and ps[i] <= ps[i - 1] and ps[i] <= ps[i + 1]:
            if inicios and t[i] - t[inicios[-1]] < SEPARACION_PASOS_S:
                if ps[i] < ps[inicios[-1]]:
                    inicios[-1] = i
            else:
                inicios.append(i)
    mins = [d[a:b][valida[a:b]].min() for a, b in zip(inicios[:-1], inicios[1:]) if valida[a:b].any()]
    e["pasos"] = len(mins)
    if len(mins) >= 2:
        e["min_paso_cm"] = 100 * np.mean(mins)
        e["std_min_paso_mm"] = 1000 * np.std(mins, ddof=1)
    return e


COLUMNAS = [  # (clave, título, formato)
    ("lecturas", "lect", "{:4d}"), ("hz", "Hz", "{:4.1f}"),
    ("deteccion_pct", "detec%", "{:6.1f}"), ("detec_extremos_pct", "extr%", "{:5.1f}"),
    ("fantasmas_pct", "fant%", "{:5.1f}"), ("equivocados_pct", "equiv%", "{:6.1f}"),
    ("ruido_mm", "ruido", "{:5.1f}"), ("pasos", "pasos", "{:5d}"),
    ("min_paso_cm", "min cm", "{:6.2f}"), ("std_min_paso_mm", "stdMin", "{:6.2f}"),
    ("sesgo_mm", "sesgo", "{:6.1f}"), ("reconexiones", "recon", "{:5d}"),
]


def fila_tabla(e):
    out = []
    for k, titulo, fmt in COLUMNAS:
        v = e.get(k)
        ancho = max(len(titulo), len(fmt.format(0)))
        out.append(fmt.format(v) if v is not None else "-".rjust(ancho))
    return " ".join(out)


def encabezado():
    return " ".join(t.rjust(max(len(t), len(f.format(0)))) for _, t, f in COLUMNAS)


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--configs", type=int, nargs="+", help="números de configuración a correr")
    ap.add_argument("--segundos", type=float, default=SEGUNDOS)
    ap.add_argument("--referencia", type=float,
                    help="distancia radar-placa (m) medida a mano con el péndulo quieto en el centro")
    a = ap.parse_args()

    elegidas = a.configs if a.configs else list(range(len(CONFIGS)))
    print("Configuraciones:")
    for i, (nombre, _) in enumerate(CONFIGS):
        print(f"  {i}: {nombre}{'' if i in elegidas else '   (no se corre)'}")

    sello = datetime.now().strftime("%Y%m%d_%H%M%S")
    ruta_res = f"barrido_{sello}_resumen.csv"
    claves = ["config", "nombre"] + [k for k, _, _ in COLUMNAS]
    with open(ruta_res, "w", newline="") as f:
        csv.writer(f).writerow(claves)

    imu_dev = Imu(crear_reset_imu())
    resultados = []
    for i in elegidas:
        nombre, cambios = CONFIGS[i]
        print(f"\n=== Config {i}: {nombre} ===")
        master.RADAR_CONFIG = dict(BASE, **cambios)
        try:
            filas, reconexiones = medir_config(imu_dev, nombre, a.segundos)
        except KeyboardInterrupt:
            print("\nCortado por el usuario.")
            break
        except Exception as e:
            print(f"  FALLÓ: {type(e).__name__}: {e}")
            continue

        with open(f"barrido_{sello}_cfg{i}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "pitch", "imu_ok", "distancia_m"])
            for t, p, ok, d in filas:
                w.writerow([f"{t:.4f}", f"{p:.3f}", int(ok), "" if d is None else f"{d:.5f}"])

        e = estadisticas(filas, a.segundos, a.referencia)
        e["reconexiones"] = reconexiones
        resultados.append((i, nombre, e))
        print("\n  " + encabezado() + "\n  " + fila_tabla(e))
        if reconexiones:
            print(f"  OJO: {reconexiones} reconexión(es): esta config perdió datos, no es comparable.")
        with open(ruta_res, "a", newline="") as f:
            csv.writer(f).writerow([i, nombre] + [e.get(k, "") for k in claves[2:]])

    if resultados:
        print("\n=== Resumen ===")
        print(f"{'#':>2} {'config':<18} " + encabezado())
        for i, nombre, e in resultados:
            print(f"{i:>2} {nombre:<18} " + fila_tabla(e))
        print("\nMenor es mejor: fant%, equiv%, ruido, stdMin, |sesgo|, recon.  Mayor es mejor: detec%, extr%.")
        print(f"-> {ruta_res}")


if __name__ == "__main__":
    main()
