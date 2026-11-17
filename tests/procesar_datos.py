#!/usr/bin/env python3
"""
Procesa un CSV de medicion_<sid>.csv (master + Pico unidos) de un ensayo con péndulo.

1. Detecta cada paso con la aceleración de la IMU de la master (accel_x):
   cada máximo de accel_x es un extremo de la oscilación. Un paso va de un máximo al
   siguiente (oscilación completa, MODO="ciclo") o de un extremo al opuesto (MODO="medio").
2. Depura la distancia del radar: descarta valores fuera de rango (ecos fantasma) y los que
   no son coherentes con el ángulo del péndulo (ajuste robusto distancia vs pitch).
3. Para cada paso busca la distancia mínima válida.
4. Agrega columnas al CSV de entrada:
     paso             número de paso en la fila donde se detecta (máximo de accel_x)
     dist_anomala     1 si esa lectura del radar se descartó por anómala
     dist_min_paso_m  distancia mínima del paso (+ offset), en la fila donde ocurre: su tiempo
                      es el t_rel_s / timestamp de esa fila
     distancia_piernas_m  distancia cruda del radar + OFFSET_TOTAL_M, en cada fila
   Todas las distancias de salida (dist_min_paso_m, distancia_piernas_m y lo que se imprime)
   ya incluyen OFFSET_TOTAL_M. distancia_m queda cruda, como vino.

Uso:
    python3 procesar_pasos.py medicion.csv [--modo ciclo|medio] [--grafico]
Salida: <entrada>_procesado.csv (y <entrada>_procesado.png con --grafico)
"""

# ===========================================================================
#  GEOMETRÍA: lo que se suma a la distancia del radar (editar acá)
# ===========================================================================
OFFSET_MASTER_M = 0.045    # cara del radar -> pierna de la master (el radar está al borde)
OFFSET_PICO_M = 0.0025     # cara del reflector -> pierna de la Pico
OFFSET_TOTAL_M = OFFSET_MASTER_M + OFFSET_PICO_M

import argparse
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import find_peaks

# --- Detección de pasos ---
UMBRAL_ACCEL = 2.0        # m/s²: un extremo tiene que superar esto (en reposo accel_x ≈ 0)
PROMINENCIA = 3.0         # m/s²: cuánto tiene que sobresalir el pico de lo que lo rodea
SEPARACION_MIN_S = 1.5    # s: dos extremos del mismo lado no pueden estar más cerca que esto
SUAVIZADO_MUESTRAS = 3    # mediana móvil sobre accel_x antes de buscar picos (1 = sin suavizar)

# --- Depuración de la distancia ---
DIST_MIN_M = 0.03         # rango válido del radar (start_m de RADAR_CONFIG)
DIST_MAX_M = 0.40         # por encima: ecos fantasma (en este ensayo aparecen ~0.58-0.60 m)
GRADO_AJUSTE = 3          # polinomio distancia = f(pitch) que describe la geometría del péndulo
K_RESIDUO = 4.0           # lecturas a más de K·MAD de esa curva son anómalas...
RESIDUO_MIN_M = 0.010     # ...pero nunca se descarta algo a menos de 1 cm de la curva
VALIDAS_MIN_POR_PASO = 3  # menos lecturas válidas que esto => el paso queda sin mínimo
K_ENTRE_PASOS = 4.0       # mínimo de un paso a más de K·MAD de la mediana de los mínimos => anómalo


def suavizar(x, n):
    if n <= 1:
        return x
    return pd.Series(x).rolling(n, center=True, min_periods=1).median().to_numpy()


def detectar_pasos(df, modo):
    """Devuelve los índices (filas) donde empieza cada paso."""
    t = df["t_rel_s"].to_numpy()
    ax = suavizar(df["accel_x"].to_numpy(), SUAVIZADO_MUESTRAS)
    dt = np.median(np.diff(t))
    sep = max(1, int(round(SEPARACION_MIN_S / dt)))

    pos, _ = find_peaks(ax, height=UMBRAL_ACCEL, prominence=PROMINENCIA, distance=sep)
    if modo == "ciclo":
        return pos
    neg, _ = find_peaks(-ax, height=UMBRAL_ACCEL, prominence=PROMINENCIA, distance=sep)
    # alternar positivo/negativo; si hay dos seguidos del mismo lado, quedarse con el mayor
    ext = sorted([(i, 1) for i in pos] + [(i, -1) for i in neg])
    limpio = []
    for i, s in ext:
        if limpio and limpio[-1][1] == s:
            if abs(ax[i]) > abs(ax[limpio[-1][0]]):
                limpio[-1] = (i, s)
        else:
            limpio.append((i, s))
    return np.array([i for i, _ in limpio], dtype=int)


def depurar_distancia(df):
    """Devuelve (distancia depurada con NaN en los descartes, máscara de anómalas).

    1) Fuera de rango del radar (ecos fantasma a 2x-4x de la distancia real).
    2) Incoherente con el ángulo: la distancia al reflector depende del pitch del péndulo,
       así que se ajusta (de forma robusta) distancia = f(pitch) y se descarta lo que se
       aparta de esa curva. Un filtro temporal no sirve acá: a ~3 Hz y con huecos del
       radar, las lecturas vecinas están en otra fase de la oscilación.
    """
    d = df["distancia_m"].astype(float)
    fuera = (d < DIST_MIN_M) | (d > DIST_MAX_M)
    lejos = pd.Series(False, index=d.index)

    usar = d.notna() & ~fuera & df["pitch"].notna()
    if "imu_ok" in df:
        usar &= df["imu_ok"] == 1
    x, y = df.loc[usar, "pitch"].to_numpy(), d[usar].to_numpy()
    if len(x) > 3 * (GRADO_AJUSTE + 1):
        sigue = np.ones(len(x), bool)
        for _ in range(10):
            coef = np.polyfit(x[sigue], y[sigue], GRADO_AJUSTE)
            r = y - np.polyval(coef, x)
            mad = 1.4826 * np.median(np.abs(r[sigue] - np.median(r[sigue])))
            nuevo = np.abs(r) <= max(K_RESIDUO * mad, RESIDUO_MIN_M)
            if (nuevo == sigue).all():
                break
            sigue = nuevo
        lejos[d[usar].index[~sigue]] = True

    anomala = (fuera | lejos) & d.notna()
    return d.where(~anomala), anomala


def procesar(df, modo):
    inicios = detectar_pasos(df, modo)
    dist, anomala = depurar_distancia(df)

    df = df.copy()
    df["paso"] = pd.array([pd.NA] * len(df), dtype="Int64")
    df["dist_anomala"] = anomala.astype(int)
    df["dist_min_paso_m"] = np.nan

    pasos = []
    for n, (a, b) in enumerate(zip(inicios[:-1], inicios[1:]), start=1):
        df.loc[a, "paso"] = n
        tramo = dist.iloc[a:b]
        validas = tramo.dropna()
        info = dict(paso=n, t_inicio=df.at[a, "t_rel_s"], t_fin=df.at[b, "t_rel_s"],
                    validas=len(validas), fila_min=None, dist_min=np.nan, nota="")
        if len(validas) < VALIDAS_MIN_POR_PASO:
            info["nota"] = f"solo {len(validas)} lecturas válidas"
        else:
            info["fila_min"] = validas.idxmin()
            info["dist_min"] = validas.min()
        pasos.append(info)

    # Mínimos raros comparados con el resto de los pasos
    mins = pd.Series([p["dist_min"] for p in pasos], dtype=float)
    if mins.notna().sum() >= 4:
        med = mins.median()
        mad = max((mins - med).abs().median() * 1.4826, 0.003)
        for p in pasos:
            if pd.notna(p["dist_min"]) and abs(p["dist_min"] - med) > K_ENTRE_PASOS * mad:
                p["nota"] = f"mínimo atípico ({p['dist_min']:.4f} m), descartado"
                p["dist_min"] = np.nan

    for p in pasos:
        if pd.notna(p["dist_min"]):
            df.loc[p["fila_min"], "dist_min_paso_m"] = round(p["dist_min"], 4)

    return df, pd.DataFrame(pasos), inicios


def graficar(df, inicios, ruta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = df["t_rel_s"]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    a1.plot(t, df["accel_x"], color="#2a6fdb", lw=1.4, label="accel_x (master)")
    a1.plot(t.iloc[inicios], df["accel_x"].iloc[inicios], "v", color="#d1495b", ms=8,
            label="paso detectado")
    for i in inicios:
        for a in (a1, a2):
            a.axvline(t.iloc[i], color="#999", lw=0.6, ls=":")
    a1.set_ylabel("m/s²")
    a1.legend(loc="upper right")
    a1.grid(alpha=0.3)

    ok = df["dist_anomala"] == 0
    dp = df["distancia_piernas_m"]
    tope = 0.25 + OFFSET_TOTAL_M
    a2.plot(t[ok], dp[ok] * 100, ".", color="#555", label="distancia válida (+offset)")
    a2.plot(t[~ok], dp[~ok].clip(upper=tope) * 100, "x", color="#e8a33d", ms=8,
            label=f"descartada (anómala, recortada a {tope * 100:.0f} cm)")
    m = df["dist_min_paso_m"].notna()
    a2.plot(t[m], df["dist_min_paso_m"][m] * 100, "o", color="#d1495b", ms=8,
            mfc="none", mew=2, label="mínimo del paso")
    a2.set_ylabel("cm (radar + offset)")
    a2.set_xlabel("t_rel_s (s)")
    a2.legend(loc="upper right")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(ruta, dpi=120)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--modo", choices=["ciclo", "medio"], default="ciclo",
                    help="ciclo: un paso = oscilación completa; medio: un paso = media oscilación")
    ap.add_argument("--grafico", action="store_true", help="guarda un PNG para corroborar")
    ap.add_argument("--salida", help="ruta del CSV de salida")
    a = ap.parse_args()

    df = pd.read_csv(a.csv)
    faltan = {"t_rel_s", "accel_x", "pitch", "distancia_m"} - set(df.columns)
    if faltan:
        sys.exit(f"Faltan columnas en el CSV: {', '.join(sorted(faltan))}")

    dt = df["t_rel_s"].diff().median()
    print(f"{len(df)} filas | {1/dt:.1f} Hz efectivos | modo={a.modo}")
    if 1 / dt < 10:
        print(f"  AVISO: {1/dt:.1f} Hz es poco para buscar mínimos; el mínimo real puede caer "
              "entre dos muestras.")

    out, pasos, inicios = procesar(df, a.modo)
    print(f"Offset sumado a las distancias: master {OFFSET_MASTER_M * 100:.2f} cm + Pico "
          f"{OFFSET_PICO_M * 100:.2f} cm = {OFFSET_TOTAL_M * 100:.2f} cm")

    # Los filtros se aplicaron sobre la distancia cruda del radar; ahora las distancias de
    # salida pasan a ser distancia cruda + OFFSET_TOTAL_M.
    out["distancia_piernas_m"] = (out["distancia_m"] + OFFSET_TOTAL_M).round(4)
    out["dist_min_paso_m"] = (out["dist_min_paso_m"] + OFFSET_TOTAL_M).round(4)
    pasos["dist_min"] = pasos["dist_min"] + OFFSET_TOTAL_M

    base = os.path.splitext(a.salida or a.csv)[0]
    ruta_csv = a.salida or f"{base}_procesado.csv"
    if not a.salida:
        ruta_csv = os.path.join(os.getcwd(), os.path.basename(ruta_csv))
    out.to_csv(ruta_csv, index=False)

    print(f"Pasos completos detectados: {len(pasos)}  |  lecturas de radar descartadas: "
          f"{int(out['dist_anomala'].sum())} de {int(df['distancia_m'].notna().sum())}")
    with pd.option_context("display.width", 140):
        print(pasos[["paso", "t_inicio", "t_fin", "validas", "dist_min", "nota"]]
              .to_string(index=False, float_format=lambda v: f"{v:.4f}"))
    v = pasos["dist_min"].dropna()
    if len(v):
        print(f"Mínimo por paso: media {v.mean()*100:.2f} cm | desvío {v.std()*100:.2f} cm | "
              f"rango {v.min()*100:.2f}-{v.max()*100:.2f} cm")
    print(f"-> {ruta_csv}")

    if a.grafico:
        ruta_png = os.path.splitext(ruta_csv)[0] + ".png"
        graficar(out, inicios, ruta_png)
        print(f"-> {ruta_png}")


if __name__ == "__main__":
    main()