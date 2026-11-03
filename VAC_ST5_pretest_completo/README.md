# VAC CentraleSupélec — T.7 + T.8 integrati

**Contenuto:** griglia topologica 5×5 `A..Y` + base `START` (visualizzata **S**); localizzazione event-based; pianificazione Dijkstra; consegne a nodi o a frazioni di segmento; ostacoli e ricalcolo; ritorno alla base.

## Avvio (Python 3.9+; solo libreria standard)

```bash
cd VAC_T7_T8_integrato
python simulation.py
python -m unittest discover -s tests -v
```

## File

- `grid_map.py`: **unica mappa** condivisa T.7/T.8. `START` non è `S`, perché **S è già l'incrocio S** nella griglia 5×5.
- `localization.py`: `RobotLocalizer` (eventi di movimento validati + odometria).
- `path_planning.py`: pianificazione, recupero ostacoli, rientro.
- `simulation.py`: esempio senza hardware.
- `tests/`: test unitari T.7 + test di integrazione.

## API minima

```python
from grid_map import GridMap, BASE
from localization import RobotLocalizer
from path_planning import plan_mission_from_state, handle_obstacle_and_reroute

grid = GridMap()
robot = RobotLocalizer(grid)
mission = {'type': 'edge', 'edge': ('A', 'B'), 'progress': 0.75}
plan = plan_mission_from_state(grid, robot.snapshot(), mission)
print(plan['path'], plan['detailed_steps'])

# Appena un ostacolo è VALIDATO durante un segmento:
# robot STOP immediato comandato dal controllore di sicurezza fisico
# proposal = handle_obstacle_and_reroute(grid, robot.snapshot(), mission)
# Eseguire la manovra di recupero SOLO dopo verifica dello spazio libero;
# poi aggiornare RobotLocalizer e ripianificare/riprendere.
```

`blocked_edges` accetta una lista di coppie di nodi: `plan_mission_from_state(grid, robot.snapshot(), mission, [('B','C')])`. Conservarla nell'applicazione durante una missione; rimuovere gli ostacoli solo dopo verifica.

### Stato e comandi

T.8 accetta **direttamente** il dizionario `robot.snapshot()` del T.7. `expected_end_state` è **previsto**, non va inserito direttamente nel localizzatore: `begin_segment`, `update_progress` e `confirm_arrival` seguono eventi reali confermati. Gli step contengono `segment`, `start_progress`, `end_progress`, `action`, `heading` e `cost`.

- `LEFT`, `RIGHT`, `STRAIGHT`, `UTURN` sono **richieste al controllore di navigazione**, non pilotano motori.
- `ALIGN_REQUIRED` appare per la curva `START`–`Y` o per heading sconosciuto: **non inventare un angolo cardinale**. Occorre una procedura specifica di orientamento/seguimento linea.
- Su un segmento, il calcolo può proporre un'inversione: eseguirla solo se fisicamente possibile e sicura.
- Se le lunghezze non sono state misurate, ogni arco vale **1 unità nominale**, non 1 metro. Con dati parziali `cost_unit='nominal_or_mixed'`; con **tutte** le lunghezze misurate `cost_unit='m'`. Calibrare prima di usare stop e distanze fisiche.

## Da verificare sul robot PRIMA dell'uso reale

1. Validare la topologia 5×5 dalla mappa reale e il collegamento curvo `START`–`Y`.
2. Misurare la lunghezza di tutti i segmenti (`grid.set_length(a,b,metri)`), compresa la curva.
3. Integrare camera/intersection detection (debouncing, riconoscimento dell'incrocio), odometria e sensore ostacoli.
4. Integrare il controllore di suivi de ligne e il movimento fisico nei nodi, con conferma manovra e arresto indipendente.
5. Validare il comportamento di UTURN/retromarcia e gli spazi liberi; non eseguire recuperi automatici senza test.
6. Aggiungere trasmissione PC–robot e dashboard se necessaria: **non incluse** in questo pacchetto.

**Questo è un pacchetto software T.7/T.8 integrato e testato, NON un sistema hardware pronto a guidare il robot senza ulteriori integrazioni.**

## Nuovo: T.3 `navigation_controller.py` (comandi ad alto livello)

```bash
python simulation_navigation.py
python -m unittest discover -s tests -v
```

Il controller è **solo software**: non invia PWM, non controlla Arduino, non legge la camera e non bypassa la supervisione di sicurezza. Da integrare con chi sviluppa le manovre fisiche. Il supervisore hardware deve poter fermare autonomamente i motori.

### Contratto per il collega che gestisce le manovre

```python
ctl = NavigationController(grid, localizer)
ctl.load_mission({'type': 'node', 'node': 'F'})
request = ctl.next_command()  # {'event':'MANEUVER_REQUEST', 'action':'LEFT', ...}
# Il codice motori esegue fisicamente la richiesta; la camera verifica la nuova linea.
ctl.maneuver_result(success=True, line_reacquired=True)  # SOLO dopo verifica reale
# Durante FOLLOWING: odometria assoluta della frazione di segmento:
ctl.progress_update(0.4)
# Quando è confermato l'arrivo fisico al nodo atteso (non prima):
ctl.confirmed_intersection(expected_node)
```

Il controller produce `FOLLOW_LINE_REQUEST` dopo l'allineamento e `STOP_REQUEST` in caso di errore. `STOP_REQUEST` **non disalimenta i motori da solo**; serve collegarlo a un supervisore che fermi immediatamente il robot. Per la destinazione al 75% di un segmento, chiamare `reached_partial_target(stopped=True)` *solo* dopo arresto fisico confermato e frazione odometrica coerente.

**Limiti intenzionali:** l'avvio di nuove missioni mentre il robot è già su un segmento e il recupero autonomo dagli ostacoli sono lasciati al futuro modulo di recovery/safety; la manovra sulla curva START–Y richiede un allineamento fisico specifico. Il sensore intersezioni deve produrre un unico evento filtrato/debounced per nodo ed evitare falsi positivi. La validità dell'esecuzione fisica non è stata verificata.

## Nuovo: integrazione seriale motori (SOLO test sorvegliati)

- `motor_bridge.py`: trasmette al firmware Arduino esistente il comando binario `C` con 4 PWM signed 16-bit; le 4 ruote sono FR, FL, RR, RL. Connessione 115200 baud `A12` e stop `C(0,0,0,0)`. Si evita `I0` perché nel firmware fornito la funzione non invia ACK.
- `motor_command_adapter.py`: accetta `MANEUVER_REQUEST` di T.3 e lo trasforma in un **breve impulso di prova**. Non conferma la manovra, e non pretende di realizzare una svolta di 90°.
- `firmware_originale/`: sketch completo ricevuto, **non modificato**, da mettere in una cartella chiamata `serial_link` nell'Arduino IDE mantenendo i nomi canonici dei file.
- `tests/test_motor_bridge.py`: prove del protocollo e dell'arresto tramite porta seriale fittizia, senza hardware.

Per il test (SOLO ruote sollevate, supervisione diretta e arresto fisico disponibile), usare da uno script interattivo su Raspberry Pi:

```python
from motor_bridge import MotorBridge
with MotorBridge(allow_hardware=True) as motors:
    print(motors.supervised_pulse('LEFT', pwm=50, duration_s=0.10))
```

**Avvertenze importanti:** questo frammento aziona motori reali; non usarlo col robot appoggiato sul pavimento. Il firmware originale non ha un watchdog funzionante (`task1()` contiene `A COMPLETER`, task disattivato): se il Raspberry/USB si blocca mentre i motori girano, il comando zero potrebbe non arrivare. Prima della guida autonoma serve un *watchdog hardware/firmware indipendente*, comando di stop di emergenza e validazione di segni e cablaggi. La rampa PWM della task 2 ha una temporizzazione da verificare; l'ACK seriale conferma la ricezione del comando, NON che la rotazione sia stata eseguita correttamente. `UTURN` invia solo un impulso dello stesso verso di LEFT: angolo effettivo e allineamento devono essere controllati con feedback. Non inviare ancora comandi autonomi su pavimento.

Per eseguire tutti i test: `python -m unittest discover -s tests -v`.

## Nuovo: odometria encoder + monitoraggio angolo delle svolte

File aggiunti:

- `encoder_odometry.py`: distanza firmata, rotazione relativa e controllo coerenza dei quattro encoder; `SegmentProgressEstimator` restituisce la frazione percorsa di un segmento con lunghezza misurata; `EncoderTurnMonitor` restituisce `TRACKING`, `ANGLE_REACHED` o `FAULT`.
- `odometry_adapter.py`: collega i progressi ricavati dagli encoder a `NavigationController.progress_update` e quindi al T.7. Va avviato **solo dopo** una manovra confermata e l'acquisizione della linea.
- `simulation_odometry.py`: demo con **parametri fittizi**, senza hardware.
- `motor_bridge.py`: aggiunta `get_encoder_counts()` tramite richieste binarie `N` (due ruote anteriori) e `n` (due posteriori), dati signed 32-bit, ordine FR/FL/RR/RL.
- Test automatici di motori, odometria, overflow, segni, errore seriale e collegamento T.7.

### Comandi di prova senza hardware

```bash
python simulation_odometry.py
python -m unittest discover -s tests -v
```

### Parametri da misurare sul vero robot

```python
from encoder_odometry import EncoderCalibration
cal = EncoderCalibration(
    ticks_per_wheel_rev=...,     # numero effettivo di conteggi per giro ruota
    wheel_diameter_m=...,        # diametro ruota in metri
    forward_signs=(..., ..., ..., ...), # +/-1, verificati per FR FL RR RL
    effective_track_width_m=..., # larghezza effettiva da calibrare con svolte reali
)
```

Non inserire numeri ricavati solo da supposizioni. Verificare i conteggi di ogni ruota con un giro manuale e misurare la distanza reale percorsa; per skid steering la larghezza effettiva varia con grip, superficie e velocità. Prima di validare il verso di rotazione controllare i segni relativi degli encoder. **`ANGLE_REACHED` è una stima odometrica, non una manovra fisica confermata**: la camera deve confermare `line_reacquired` prima di chiamare `maneuver_result(True, True)`.

### Sicurezza e limiti hardware

Non è stato attivato alcun ciclo autonomo che muova il robot fino a 90°: il firmware originale ha ancora il watchdog non implementato; deve essere aggiunto e validato con un arresto fisico prima di impiegare feedback encoder per azionamenti continui. Le routine di conteggio ISR e le letture di `long` vanno verificate per atomicità e coerenza sul microcontrollore. Anche la rampa della task a 500 ms va verificata fisicamente: può rendere inaffidabili impulsi PWM brevi. L'acquisizione seriale può fallire: in tal caso il software richiede stop, ma non garantisce che i motori si arrestino senza watchdog indipendente.

La curva tra `START` e `Y` e le svolte ai nodi non sono automaticamente risolte dall'odometria; vanno confermate e calibrate nel contesto del seguito di linea e della detection d'intersection.

## Pacchetto aggiuntivo — arresto a soglia encoder (pre-test fisico)

Leggere prima **`PROCEDURA_TEST_FISICI.md`**. Nuovi file:

- `closed_loop_turn.py`: ciclo feedback encoder per LEFT/RIGHT/UTURN, fast/slow PWM, soglia anticipata di stop, timeout, rilevamento blocco, discrepanze encoder, STOP in `finally`.
- `calibration_TEMPLATE.json` e `calibration_store.py`: nessun parametro reale presunto; la calibrazione mancante/non verificata impedisce l'avvio da CLI.
- `test_turn_controller.py`: simulazione/no hardware di default; per prova fisica servono flag espliciti, calibrazione verificata e supervisione.
- `firmware_watchdog/serial_link/`: **copia modificata** del firmware originale con watchdog di 350 ms, task motori 20 ms, arresto immediato a `C(0,0,0,0)`, lettura atomica delle coppie encoder. **Non verificato su Arduino vero**; l'originale rimane in `firmware_originale/`.
- `tests/test_closed_loop_turn.py`: test del ciclo incluse condizioni di fault e stop.

**Nota:** il comando encoder-angle non garantisce rotazione fisica esatta, perché le ruote slittano, e non sostituisce la verifica della camera. Non eseguire questa versione su robot a terra senza prima calibrare e verificare sicurezza/watchdog e con una supervisione ravvicinata.

```bash
python3 -m unittest discover -s tests -v
python3 test_turn_controller.py --simulate
```
