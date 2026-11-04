"""
Controller Webots principale (basique.py)
Visione OpenCV (T.1/T.2) + NavigationController (T.3) + Localization (T.7) + Path Planning (T.8)

Griglia cartesiana 5x5 con nodi 'xy': x = colonna (0 sinistra -> 4 destra),
y = riga (0 in basso -> 4 in alto). Esempio: '00' angolo in basso a sinistra,
'44' angolo in alto a destra. Direzioni: 'E' = +x (destra), 'N' = +y (alto).
"""
import math
import numpy as np
import cv2
from controller import Robot

from grid_map import GridMap
from localization import RobotLocalizer
from navigation_controller import NavigationController

# ==========================================
# 1. CONFIGURAZIONE MISSIONE
# ==========================================
# Il robot e' posizionato in (-1.1, -1.1) = angolo in basso a sinistra = nodo '00',
# rotation ~0 => guarda verso +x = Est.
START_NODE = '00'
INITIAL_HEADING = 'N'
TARGET_NODE = '34'        # colonna 3, riga 4 (bordo alto, NON l'angolo): test di arrivo a un nodo intermedio
BLOCKED_EDGES = []        # es: [('21', '22')]
DEST_PAUSE_TIME = 5.0     # pausa (secondi) a ruote ferme sulla destinazione
RETURN_TO_START = True    # dopo la pausa torna al nodo di partenza
RETURN_NODE = START_NODE  # nodo di ritorno ('START' = base esterna collegata a '40', curva non ancora gestita)
BLOCKED_NODES = []        # nodi non attraversabili, es. ['22'] (= tutti i loro archi bloccati)
DELIVERY = {'edge': ('02', '03'), 'progress': 0.85}           # consegna a META' TRATTO, es. {'edge': ('14', '24'), 'progress': 0.5}
                          # None = vai a TARGET_NODE. progress tra 0.25 e 0.85, misurato dal PRIMO nodo dell'arco
                          # lungo il verso di marcia scelto dal planner. Richiede gli encoder.
EDGE_LENGTH_M = 0.55      # distanza nodo-nodo (controlla la riga [ODO]); serve per la consegna a meta' tratto
DELIVERY_RETURN = 'REVERSE'  # dopo una consegna a meta' tratto: 'REVERSE' = inversione di 180 gradi sulla linea e
                             # ritorno al nodo PRECEDENTE (piu' breve); 'FORWARD' = prosegue al nodo successivo

# ==========================================
# 2. PARAMETRI
# ==========================================
TARGET_TIME_STEP_MS = 64  # viene arrotondato a un multiplo del basicTimeStep del mondo
MAX_WHEEL_SPEED = 10.0    # limitato poi da getMaxVelocity() dei motori
BASE_SPEED = 4.8
TURN_SPEED = 4.2
KP_STEER = 0.022

# Configurazione motori (da verificare con il test descritto nel messaggio):
SWAP_SIDES = False        # True se Motor2/Motor4 sono in realta' a DESTRA
LEFT_SIGN = 1.0           # -1.0 se i motori sinistri girano al contrario
RIGHT_SIGN = 1.0          # -1.0 se i motori destri girano al contrario
DEBUG_LIST_DEVICES = False  # True: stampa tutti i device del robot e basta
DEBUG_SHOW_MASK = False     # True: mostra anche la maschera bianca binarizzata

NODE_STOP_PAUSE = 0.60        # pausa a ruote ferme su ogni nodo
# Allineamento sul nodo dopo il rilevamento (anello chiuso sulla barra orizzontale):
# avanza finche' la barra e' ancora visibile nella parte bassa dell'immagine, poi ancora ALIGN_EXTRA_TIME.
# Se il robot ruota PRIMA di essere sul nodo (linea troppo lontana) => aumenta ALIGN_EXTRA_TIME;
# se lo supera => diminuiscilo (puo' essere anche 0).
ALIGN_SPEED_FACTOR = 0.6      # velocita' di avvicinamento = BASE_SPEED * fattore
ALIGN_EXTRA_M = 0.19          # metri da percorrere DOPO che la barra e' sparita (misurati con gli encoder PS1..PS4)
ALIGN_EXTRA_TIME = 1.5        # fallback a tempo se gli encoder non sono disponibili
ALIGN_MAX_TIME = 4.0          # sicurezza: massimo tempo di allineamento
WHEEL_RADIUS_M = 0.0325       # raggio ruota: DA VERIFICARE (vedi riga [ODO] stampata ad ogni nodo)
TURN_90_TIME = 1.50           # rotazione a tempo (grossolana); poi TURN_CENTER ricentra sulla linea
TURN_180_TIME = 3.00
CENTER_TOL_PX = 10            # tolleranza (px) per considerare la linea centrata dopo una curva
KP_CENTER = 0.03              # guadagno della rotazione di ricentraggio
MIN_CENTER_SPEED = 1.2        # velocita' minima ruote durante il ricentraggio (attrito)
CENTER_SOFT_TIME = 1.0        # dopo questo tempo basta una tolleranza piu' larga (poi sterza il line-following)
CENTER_SOFT_TOL_FRAC = 0.12   # tolleranza larga = 12% della larghezza immagine (minimo CENTER_SOFT_TOL_MIN_PX)
CENTER_SOFT_TOL_MIN_PX = 25
CENTER_TIMEOUT = 4.0          # se non riesce a ricentrarsi entro questo tempo => STOP
BAR_ROW_FRAC = 0.6            # una riga del ramo laterale e' "barra" se bianca per almeno il 60%
BAR_MIN_ROWS_FRAC = 0.03      # servono almeno il 3% dell'altezza immagine di righe "barra"
DEBUG_LOG = True              # stampa ogni 0.5 s lo stato della visione
INTERSECTION_COOLDOWN = 1.7   # secondi dopo l'inizio di un segmento prima di accettare un nodo
INTERSECTION_CONFIRM_FRAMES = 2   # frame consecutivi con incrocio visto (anti-rumore)
SEGMENT_TIME_EST = 4.0        # tempo stimato per un segmento (solo per il progresso fittizio 0..0.95)
MIN_TRAVEL_FOR_LOST_ARRIVAL = 2.5  # "linea finita" vale come arrivo solo dopo almeno questi secondi
LOST_ARRIVAL_TIME = 0.25      # linea assente per almeno questi secondi => fine linea (nodo)
LINE_LOST_TIMEOUT = 1.5       # linea assente piu' di cosi' senza essere vicini a un nodo => STOP

# ==========================================
# 3. HARDWARE
# ==========================================
robot = Robot()

basic_step = robot.getBasicTimeStep()
TIME_STEP = int(basic_step * max(1, round(TARGET_TIME_STEP_MS / basic_step)))

if DEBUG_LIST_DEVICES:
    for i in range(robot.getNumberOfDevices()):
        d = robot.getDeviceByIndex(i)
        print(f"[DEVICE] {d.getName()}  (tipo {d.getNodeType()})")
    raise SystemExit("DEBUG_LIST_DEVICES: elenco stampato, controller terminato.")

camera = robot.getDevice("PiCam")
camera.enable(TIME_STEP)

motors_names = ["Motor2", "Motor1", "Motor4", "Motor3"]
motors = []
for name in motors_names:
    m = robot.getDevice(name)
    if m is None:
        raise SystemExit(f"Motore '{name}' non trovato: usa DEBUG_LIST_DEVICES=True per vedere i nomi.")
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
    max_v = m.getMaxVelocity()
    if max_v > 0:
        MAX_WHEEL_SPEED = min(MAX_WHEEL_SPEED, max_v)
    motors.append(m)

# Encoder delle ruote (PositionSensor PS1..PS4): odometria = media dei |delta angolo| in rad
encoders = []
for _n in ("PS1", "PS2", "PS3", "PS4"):
    _d = robot.getDevice(_n)
    if _d is not None:
        _d.enable(TIME_STEP)
        encoders.append(_d)
HAVE_ENCODERS = len(encoders) == 4
odo_rad = 0.0          # rotazione cumulata media delle ruote [rad]
_prev_enc = None


def update_odometry():
    global odo_rad, _prev_enc
    if not HAVE_ENCODERS:
        return
    cur = [e.getValue() for e in encoders]
    if any(math.isnan(v) for v in cur):
        return
    if _prev_enc is not None:
        odo_rad += sum(abs(c - p) for c, p in zip(cur, _prev_enc)) / len(cur)
    _prev_enc = cur


def set_tank_velocities(left_vel, right_vel):
    if SWAP_SIDES:
        left_vel, right_vel = right_vel, left_vel
    lv = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, LEFT_SIGN * float(left_vel)))
    rv = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, RIGHT_SIGN * float(right_vel)))
    motors[0].setVelocity(lv)
    motors[2].setVelocity(lv)
    motors[1].setVelocity(rv)
    motors[3].setVelocity(rv)


def stop_motors():
    set_tank_velocities(0.0, 0.0)


# ==========================================
# 4. T.7 / T.8 / T.3
# ==========================================
grid = GridMap()
for label, node in (('START_NODE', START_NODE), ('TARGET_NODE', TARGET_NODE), ('RETURN_NODE', RETURN_NODE)):
    if node not in grid.adjacency:
        raise SystemExit(f"{label}='{node}' non esiste in GridMap. Nodi validi: {sorted(grid.adjacency)}")
if INITIAL_HEADING not in ('N', 'E', 'S', 'W'):
    raise SystemExit("INITIAL_HEADING deve essere 'N', 'E', 'S' o 'W'")

for n in BLOCKED_NODES:
    if n not in grid.adjacency:
        raise SystemExit(f"BLOCKED_NODES: nodo '{n}' inesistente. Nodi validi: {sorted(grid.adjacency)}")
# Un nodo bloccato = tutti gli archi che lo toccano sono bloccati (il planner conosce solo archi bloccati)
ALL_BLOCKED = list(BLOCKED_EDGES) + [(n, m) for n in BLOCKED_NODES for m in grid.adjacency[n]]

if DELIVERY_RETURN not in ('REVERSE', 'FORWARD'):
    raise SystemExit("DELIVERY_RETURN deve essere 'REVERSE' o 'FORWARD'")
if DELIVERY is None:
    mission_target = {'type': 'node', 'node': TARGET_NODE}
    target_desc = f"'{TARGET_NODE}'"
    must_be_free = [START_NODE, TARGET_NODE, RETURN_NODE]
else:
    _u, _v = DELIVERY['edge']
    _p = float(DELIVERY['progress'])
    if _v not in grid.adjacency.get(_u, set()):
        raise SystemExit(f"DELIVERY: '{_u}'-'{_v}' non e' un arco della griglia")
    if not 0.25 <= _p <= 0.85:
        raise SystemExit("DELIVERY: 'progress' deve stare tra 0.25 e 0.85")
    if not HAVE_ENCODERS:
        raise SystemExit("DELIVERY richiede gli encoder PS1..PS4 (non trovati)")
    mission_target = {'type': 'edge', 'edge': (_u, _v), 'progress': _p}
    target_desc = f"meta' tratto {_u}-{_v} al {_p * 100:.0f}%"
    must_be_free = [START_NODE, _u, _v, RETURN_NODE]
for n in must_be_free:
    if n in BLOCKED_NODES:
        raise SystemExit(f"Il nodo '{n}' (partenza/destinazione/ritorno) non puo' essere in BLOCKED_NODES")

localizer = RobotLocalizer(grid=grid, current_node=START_NODE, orientation=INITIAL_HEADING)
nav = NavigationController(grid=grid, localizer=localizer)

mission_event = nav.load_mission(mission_target, blocked_edges=ALL_BLOCKED)
print("==========================================")
print(f"[MISSIONE CARICATA] Da {START_NODE} (dir: {INITIAL_HEADING}) a {target_desc}")
if ALL_BLOCKED:
    print(f"[BLOCCHI] nodi: {BLOCKED_NODES}  archi: {BLOCKED_EDGES}")
print(f"[PERCORSO DIJKSTRA] Nodi: {mission_event['path']}")
print(f"[COMANDI PREVISTI]  {nav.plan['commands']}")
print(f"[TIME_STEP] {TIME_STEP} ms, velocita' max ruote {MAX_WHEEL_SPEED:.2f} rad/s, "
      f"encoder: {'SI' if HAVE_ENCODERS else 'NO (uso i tempi)'}")
print("==========================================")


# ==========================================
# 5. VISIONE (incroci +, T e spigoli a L)
# ==========================================
def has_bar(region):
    """True se nella regione c'e' una BARRA ORIZZONTALE (righe quasi tutte bianche).
    Una linea verticale che scivola nel ramo laterale NON conta come incrocio."""
    row_frac = np.count_nonzero(region, axis=1) / float(region.shape[1])
    min_rows = max(2, int(BAR_MIN_ROWS_FRAC * region.shape[0]))
    return int(np.count_nonzero(row_frac > BAR_ROW_FRAC)) >= min_rows


def analyze_camera_image(camera_device):
    """Ritorna un dict con i risultati della visione."""
    raw = camera_device.getImage()
    if raw is None:
        return dict(line_ok=False, error=0.0, wide_ok=False, wide_error=0.0,
                    approaching=False, intersection=False, lr=0.0, rr=0.0, bar_any=False)

    w = camera_device.getWidth()
    h = camera_device.getHeight()
    img = np.frombuffer(raw, np.uint8).reshape((h, w, 4)).copy()

    gray = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
    _, mask = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

    # Rami laterali: incrocio = riempimento > 12% E presenza di una barra orizzontale
    inter_top = int(h * 0.58)
    inter_bottom = int(h * 0.82)
    left_wing = mask[inter_top:inter_bottom, 0: int(w * 0.32)]
    right_wing = mask[inter_top:inter_bottom, int(w * 0.68): w]
    left_ratio = np.count_nonzero(left_wing) / float(left_wing.size)
    right_ratio = np.count_nonzero(right_wing) / float(right_wing.size)
    left_hit = left_ratio > 0.12 and has_bar(left_wing)
    right_hit = right_ratio > 0.12 and has_bar(right_wing)
    intersection_detected = left_hit or right_hit

    # Pre-allarme (zona piu' alta): congela lo sterzo prima del nodo
    app_left = mask[int(h * 0.38):inter_bottom, 0: int(w * 0.32)]
    app_right = mask[int(h * 0.38):inter_bottom, int(w * 0.68): w]
    approaching_intersection = (
        (np.count_nonzero(app_left) / float(app_left.size) > 0.08 and has_bar(app_left)) or
        (np.count_nonzero(app_right) / float(app_right.size) > 0.08 and has_bar(app_right)))

    # Barra ancora visibile in tutta la parte bassa dell'immagine? (serve per fermarsi SUL nodo)
    low_top = int(h * 0.40)
    bar_any = (has_bar(mask[low_top:, 0: int(w * 0.32)]) or
               has_bar(mask[low_top:, int(w * 0.68): w]))

    # Line following sul corridoio centrale
    roi_top = int(h * 0.55)
    roi_bottom = int(h * 0.95)
    x_margin = int(w * 0.28)
    roi_center = mask[roi_top:roi_bottom, x_margin: w - x_margin]

    M = cv2.moments(roi_center)
    line_detected = M["m00"] > (255 * 12)
    line_error = 0.0
    if line_detected:
        cx_local = int(M["m10"] / M["m00"])
        cx_global = cx_local + x_margin
        cy_global = int(M["m01"] / M["m00"]) + roi_top
        line_error = cx_global - (w / 2.0)
        cv2.circle(img, (cx_global, cy_global), 5, (0, 255, 0, 255), -1)

    # Fallback a tutta larghezza: serve a recuperare quando la linea esce dal corridoio centrale
    Mw = cv2.moments(mask[roi_top:roi_bottom, :])
    wide_detected = Mw["m00"] > (255 * 12)
    wide_error = (Mw["m10"] / Mw["m00"] - w / 2.0) if wide_detected else 0.0

    color_l = (0, 0, 255, 255) if left_hit else (255, 180, 0, 255)
    color_r = (0, 0, 255, 255) if right_hit else (255, 180, 0, 255)
    cv2.rectangle(img, (0, inter_top), (int(w * 0.32), inter_bottom), color_l, 1)
    cv2.rectangle(img, (int(w * 0.68), inter_top), (w - 1, inter_bottom), color_r, 1)
    cv2.rectangle(img, (x_margin, roi_top), (w - x_margin, roi_bottom), (0, 255, 0, 255), 1)

    cv2.imshow("PiCam - Visione Robot", img)
    if DEBUG_SHOW_MASK:
        cv2.imshow("PiCam - Maschera", mask)
    cv2.waitKey(1)

    return dict(line_ok=line_detected, error=line_error, wide_ok=wide_detected,
                wide_error=wide_error, approaching=approaching_intersection,
                intersection=intersection_detected, lr=left_ratio, rr=right_ratio,
                bar_any=bar_any)


# ==========================================
# 6. MACCHINA A STATI
# ==========================================
# FOLLOW_LINE -> (nodo rilevato) ARRIVE_ALIGN -> NODE_PAUSE -> REQUEST_MANEUVER
#   -> STRAIGHT: FOLLOW_LINE   |   LEFT/RIGHT/UTURN: TURNING -> FOLLOW_LINE
fsm_state = 'REQUEST_MANEUVER'
current_action = None
state_start_time = robot.getTime()
segment_start_time = robot.getTime()
inter_frames = 0
lost_since = None
last_dbg = -1.0
leg = 'OUT'          # 'OUT' = andata verso TARGET_NODE, 'BACK' = ritorno
align_gone_since = None
align_odo_gone = None
align_odo_start = 0.0
seg_odo_start = 0.0
turn_odo_start = 0.0


def turn_wheels(action, speed):
    """(sx, dx) per ruotare SUL POSTO: LEFT = sx indietro / dx avanti."""
    if action == 'LEFT':
        return -speed, speed
    if action in ('RIGHT', 'UTURN'):
        return speed, -speed
    return -speed * 0.6, speed * 0.6   # ALIGN_REQUIRED


def begin_follow(now):
    """Inizia (o riprende) il seguimento di un segmento."""
    global segment_start_time, fsm_state, inter_frames, lost_since, seg_odo_start
    seg_odo_start = odo_rad
    segment_start_time = now
    inter_frames = 0
    lost_since = None
    fsm_state = 'FOLLOW_LINE'


def start_return():
    """Pianifica il ritorno a RETURN_NODE dal nodo corrente (equivale a plan_return_to_base)."""
    global leg, fsm_state
    if localizer.current_node == RETURN_NODE:
        print("[FINE MISSIONE]")
        fsm_state = 'DONE'
        return
    try:
        ev = nav.load_mission({'type': 'node', 'node': RETURN_NODE}, blocked_edges=ALL_BLOCKED)
    except (ValueError, RuntimeError) as exc:
        print(f"[RITORNO IMPOSSIBILE] {exc}")
        fsm_state = 'DONE'
        return
    leg = 'BACK'
    print("==========================================")
    print(f"[RITORNO] Da {localizer.current_node} (dir: {localizer.orientation}) a {RETURN_NODE}")
    print(f"[PERCORSO] Nodi: {ev['path']}")
    print(f"[COMANDI PREVISTI] {nav.plan['commands']}")
    print("==========================================")
    fsm_state = 'REQUEST_MANEUVER'


while robot.step(TIME_STEP) != -1:
    now = robot.getTime()
    update_odometry()
    vis = analyze_camera_image(camera)
    line_ok, error_px = vis['line_ok'], vis['error']
    wide_ok, wide_err = vis['wide_ok'], vis['wide_error']
    is_approaching, is_intersection = vis['approaching'], vis['intersection']

    if DEBUG_LOG and now - last_dbg >= 0.5:
        last_dbg = now
        print(f"[DBG {now:6.2f}s {fsm_state:15s}] line={int(line_ok)} err={error_px:6.1f} "
              f"wide={int(wide_ok)} werr={wide_err:6.1f} L={vis['lr']:.2f} R={vis['rr']:.2f} "
              f"appr={int(is_approaching)} inter={int(is_intersection)} bar={int(vis['bar_any'])}")

    # Emergenza del NavigationController: ferma tutto e spiega perche'
    if nav.phase == 'STOPPED' and fsm_state != 'DONE':
        stop_motors()
        print("==========================================")
        print(f"[STOP DI EMERGENZA] {nav.events[-1].get('reason')}")
        print(f"[STATO] nodo={localizer.current_node} snapshot={nav.snapshot()}")
        print("==========================================")
        fsm_state = 'DONE'
        continue

    # --- Richiesta manovra ---
    if fsm_state == 'REQUEST_MANEUVER':
        if nav.phase == 'COMPLETED':
            stop_motors()
            print("==========================================")
            if leg == 'OUT':
                print(f"[DESTINAZIONE RAGGIUNTA] Robot fermo a '{localizer.current_node}': "
                      f"pausa di {DEST_PAUSE_TIME:.1f} s")
                state_start_time = now
                fsm_state = 'DEST_PAUSE'
            elif leg == 'FINISH_EDGE':     # dopo la consegna a meta' tratto: arrivato al nodo, ora ritorna
                print(f"[NODO RAGGIUNTO DOPO LA CONSEGNA] '{localizer.current_node}' "
                      f"(dir: {localizer.orientation})")
                start_return()
            else:
                print(f"[RITORNO COMPLETATO] Robot fermo a '{localizer.current_node}'")
                fsm_state = 'DONE'
            print("==========================================")
            continue

        cmd_event = nav.next_command()
        current_action = cmd_event['action']
        print(f"\n[NODO {cmd_event['at_node']} -> {cmd_event['next_node']}] Azione: {current_action}")

        if current_action == 'STRAIGHT':
            nav.maneuver_result(success=True, line_reacquired=True)
            begin_follow(now)
        else:
            # Il robot e' gia' centrato sul nodo (ARRIVE_ALIGN) o parte dal nodo iniziale
            state_start_time = now
            turn_odo_start = odo_rad
            fsm_state = 'TURNING'

    # --- Rotazione a tempo (grossolana, sul posto) ---
    elif fsm_state == 'TURNING':
        elapsed = now - state_start_time
        target_duration = TURN_180_TIME if current_action == 'UTURN' else TURN_90_TIME
        centered_early = ((elapsed > target_duration * 0.65) and line_ok and (abs(error_px) < CENTER_TOL_PX)
                          and current_action != 'UTURN')   # nell'UTURN la linea passa sotto il robot a ~90 gradi

        if elapsed < target_duration and not centered_early:
            set_tank_velocities(*turn_wheels(current_action, TURN_SPEED))
        else:
            stop_motors()
            state_start_time = now
            fsm_state = 'TURN_CENTER'

    # --- Ricentraggio in anello chiuso sulla linea (sempre sul posto) ---
    elif fsm_state == 'TURN_CENTER':
        elapsed = now - state_start_time
        soft_tol = max(CENTER_SOFT_TOL_MIN_PX, CENTER_SOFT_TOL_FRAC * camera.getWidth())
        strict_ok = line_ok and abs(error_px) < CENTER_TOL_PX
        soft_ok = line_ok and elapsed > CENTER_SOFT_TIME and abs(error_px) < soft_tol
        if strict_ok or soft_ok:
            stop_motors()
            if leg == 'REVERSE_EDGE':     # inversione sul tratto: missione gia' completata
                localizer.reverse_direction()
                leg = 'FINISH_EDGE'
            else:
                nav.maneuver_result(success=True, line_reacquired=True)
            print(f"  -> Rotazione {current_action} completata, linea a {error_px:.0f} px "
                  f"({'tolleranza larga' if not strict_ok else 'centrata'}; "
                  f"ruote: {odo_rad - turn_odo_start:.2f} rad).")
            begin_follow(now)
            if leg == 'FINISH_EDGE':
                segment_start_time = now - INTERSECTION_COOLDOWN   # il nodo precedente e' gia' a distanza
        elif elapsed > CENTER_TIMEOUT:
            stop_motors()
            print(f"  -> [TURN FALLITA] {current_action}: linea={int(line_ok)} err={error_px:.0f} px "
                  f"wide={int(wide_ok)} werr={wide_err:.0f} px dopo {elapsed:.1f}s "
                  f"(ruote: {odo_rad - turn_odo_start:.2f} rad)")
            if leg == 'REVERSE_EDGE':
                nav.emergency_stop('Inversione sul tratto fallita')
            else:
                nav.maneuver_result(success=False, line_reacquired=False)
        else:
            err = error_px if line_ok else (wide_err if wide_ok else None)
            if err is None:   # nessuna linea visibile: continua a cercarla nel verso della curva
                set_tank_velocities(*turn_wheels(current_action, MIN_CENTER_SPEED * 1.5))
            else:             # err>0: linea a destra -> ruota a destra (ruote sx avanti, dx indietro)
                speed = max(MIN_CENTER_SPEED, min(TURN_SPEED * 0.6, abs(KP_CENTER * err)))
                speed = speed if err > 0 else -speed
                set_tank_velocities(speed, -speed)

    # --- Seguimento segmento ---
    elif fsm_state == 'FOLLOW_LINE':
        travel_time = now - segment_start_time
        partial_end = None
        if leg != 'FINISH_EDGE':
            step = nav.plan['detailed_steps'][nav.step_index]
            partial_end = step['end_progress'] if step['end_progress'] < 1.0 - 1e-8 else None
            if partial_end is None:
                nav.progress_update(min(0.95, travel_time / SEGMENT_TIME_EST))  # progresso FITTIZIO a tempo
            else:   # consegna a meta' tratto: progresso REALE da encoder
                frac = (odo_rad - seg_odo_start) * WHEEL_RADIUS_M / EDGE_LENGTH_M
                if frac >= partial_end:
                    stop_motors()
                    nav.progress_update(partial_end)
                    nav.reached_partial_target(stopped=True)
                    if nav.phase == 'STOPPED':
                        continue
                    print("==========================================")
                    print(f"[CONSEGNA A META' TRATTO] {step['segment'][0]}->{step['segment'][1]} "
                          f"al {partial_end * 100:.0f}% (percorso: "
                          f"{(odo_rad - seg_odo_start) * WHEEL_RADIUS_M:.3f} m): pausa di {DEST_PAUSE_TIME:.1f} s")
                    print("==========================================")
                    state_start_time = now
                    fsm_state = 'DEST_PAUSE'
                    continue
                nav.progress_update(frac)

        # Linea persa: tracciamento del tempo
        if line_ok or wide_ok:
            lost_since = None
        elif lost_since is None:
            lost_since = now
        lost_for = 0.0 if lost_since is None else now - lost_since

        arrived = False
        # Su un tratto con consegna a meta' la camera vede la barra del nodo molto prima del punto di consegna
        # (anche a ~27 cm dal nodo): l'arrivo qui lo decide SOLO l'odometria, non l'incrocio visto.
        if travel_time > INTERSECTION_COOLDOWN and partial_end is None:
            inter_frames = inter_frames + 1 if is_intersection else 0
            if inter_frames >= INTERSECTION_CONFIRM_FRAMES:
                arrived = True
            elif travel_time > MIN_TRAVEL_FOR_LOST_ARRIVAL and lost_for >= LOST_ARRIVAL_TIME:
                arrived = True  # fine linea (spigolo/bordo) dopo un tratto plausibile

        if arrived:
            stop_motors()
            if leg == 'FINISH_EDGE':    # missione gia' completata: aggiorna solo il localizzatore
                expected_node = localizer.to_node
                localizer.confirm_arrival(expected_node)
                res = {'phase': nav.phase}
            else:
                expected_node = nav.plan['detailed_steps'][nav.step_index]['segment'][1]
                res = nav.confirmed_intersection(expected_node)
                if nav.phase == 'STOPPED':
                    continue  # gestito all'inizio del ciclo
            print(f"  -> [NODO RILEVATO] Arrivo a '{expected_node}' (fase: {res.get('phase')})")
            state_start_time = now
            align_gone_since = None
            align_odo_gone = None
            align_odo_start = odo_rad
            fsm_state = 'ARRIVE_ALIGN'
            continue

        if lost_for > LINE_LOST_TIMEOUT:
            nav.emergency_stop('Linea persa lontano da un nodo')
            continue

        if is_approaching or is_intersection:
            set_tank_velocities(BASE_SPEED, BASE_SPEED)        # sterzo congelato vicino al nodo
        elif line_ok:
            correction = max(-2.2, min(2.2, KP_STEER * error_px))
            set_tank_velocities(BASE_SPEED + correction, BASE_SPEED - correction)
        elif wide_ok:   # linea fuori dal corridoio centrale: recupero rallentando e sterzando verso di essa
            correction = max(-2.0, min(2.0, KP_STEER * 1.5 * wide_err))
            set_tank_velocities(BASE_SPEED * 0.5 + correction, BASE_SPEED * 0.5 - correction)
        else:
            set_tank_velocities(BASE_SPEED * 0.5, BASE_SPEED * 0.5)  # linea persa: avanti piano

    # --- Avvicinamento al nodo in anello chiuso (dopo OGNI arrivo, anche prima di STRAIGHT) ---
    elif fsm_state == 'ARRIVE_ALIGN':
        elapsed = now - state_start_time
        if vis['bar_any']:
            align_gone_since = None
            align_odo_gone = None
        elif align_gone_since is None:
            align_gone_since = now
            align_odo_gone = odo_rad
        if HAVE_ENCODERS:
            extra_done = (align_odo_gone is not None and
                          (odo_rad - align_odo_gone) * WHEEL_RADIUS_M >= ALIGN_EXTRA_M)
        else:
            extra_done = align_gone_since is not None and now - align_gone_since >= ALIGN_EXTRA_TIME
        if elapsed < ALIGN_MAX_TIME and not extra_done:
            set_tank_velocities(BASE_SPEED * ALIGN_SPEED_FACTOR, BASE_SPEED * ALIGN_SPEED_FACTOR)
        else:
            stop_motors()
            gone = 'mai' if align_gone_since is None else f"{align_gone_since - state_start_time:.2f}s"
            print(f"  -> [ALIGN] {elapsed:.2f}s, barra sparita dopo: {gone}, "
                  f"avanzamento: {(odo_rad - align_odo_start) * WHEEL_RADIUS_M:.3f} m")
            if HAVE_ENCODERS:
                print(f"  -> [ODO] nodo-nodo: {(odo_rad - seg_odo_start) * WHEEL_RADIUS_M:.3f} m "
                      f"(cella attesa ~0.55 m; se diverso: WHEEL_RADIUS_M = "
                      f"{WHEEL_RADIUS_M:.4f} * 0.55 / misura)")
            state_start_time = now
            fsm_state = 'NODE_PAUSE'

    # --- Pausa sul nodo ---
    elif fsm_state == 'NODE_PAUSE':
        stop_motors()
        if now - state_start_time >= NODE_STOP_PAUSE:
            fsm_state = 'REQUEST_MANEUVER'

    # --- Pausa lunga sulla destinazione, poi (opzionale) ritorno al nodo di partenza ---
    elif fsm_state == 'DEST_PAUSE':
        stop_motors()
        if now - state_start_time >= DEST_PAUSE_TIME:
            if not RETURN_TO_START:
                print("[FINE MISSIONE]")
                fsm_state = 'DONE'
            elif localizer.from_node is not None and DELIVERY_RETURN == 'REVERSE':
                # Consegna a meta' tratto: inversione di 180 gradi sulla linea, poi torna al nodo precedente
                leg = 'REVERSE_EDGE'
                current_action = 'UTURN'
                print(f"[INVERSIONE] sul tratto {localizer.from_node}->{localizer.to_node}, "
                      f"ritorno al nodo '{localizer.from_node}'")
                state_start_time = now
                turn_odo_start = odo_rad
                fsm_state = 'TURNING'
            elif localizer.from_node is not None:
                # 'FORWARD': prosegui lungo la linea fino al nodo successivo, poi ritorna
                leg = 'FINISH_EDGE'
                print(f"[PROSEGUO] fino al nodo '{localizer.to_node}', poi ritorno a {RETURN_NODE}")
                begin_follow(now)
                segment_start_time = now - INTERSECTION_COOLDOWN   # siamo gia' a >=25% dal nodo precedente
            else:
                start_return()

    # --- Fine ---
    elif fsm_state == 'DONE':
        stop_motors()