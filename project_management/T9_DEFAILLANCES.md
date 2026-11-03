# T.9 ? D?finition et gestion des d?faillances

**?quipe 9 ? RenoCS ; responsables : Georgios et Ioannis.**
?ch?ance : jeudi 5 novembre 2026 ? 12 h, heure de Paris.

| Situation | D?tection / mode | R?action et reprise |
|---|---|---|
| Cam?ra absente, invalide, p?rim?e ou traitement trop lent | D?fauts existants de `PhysicalIntersectionAdapter` / contr?le de fra?cheur ? `SAFE_STOP` | Arr?t demand? et verrouill?. Cam?ra r?par?e, session explicitement renouvel?e si verrouill?e, entr?es saines, op?rateur et immobilit? v?rifi?e pour reprendre. |
| Cam?ra en d?marrage / UNKNOWN bref | `DEGRADED` | Pause conservatrice ; conserver le tracker. Retour automatique sur images valides, sans lever un arr?t de navigation. |
| UNKNOWN persistant | `PERSISTENT_UNKNOWN` fourni par l'adaptateur apr?s 2 s ? `SAFE_STOP` | M?me proc?dure de reprise que pour la cam?ra ; aucun second temporisateur UNKNOWN. |
| Liaison perdue, ACK absent/inattendu ou ?tat s?rie p?rim? | `SERIAL_LOST`, `SERIAL_TIMEOUT`, `SERIAL_PROTOCOL_ERROR`, etc. ? `SAFE_STOP` | Inhiber le d?placement ; livraison du Stop non garantie. R?initialiser la liaison apr?s ?chec puis obtenir un ?change born? r?ussi, corriger tous les d?fauts, autorisation op?rateur et immobilit? v?rifi?e. |
| D?faut moteur fiable et r?cent | `REPORTED_MOTOR_FAILURE` ? `SAFE_STOP` | R?paration et nouveau retour fiable sans d?faut. La disparition du retour ne permet pas la reprise. Op?rateur et immobilit? v?rifi?e requis. |
| Retour moteur absent/non fiable | Diagnostic limit? ; si d?clar? obligatoire ? `SAFE_STOP` | Ne pas inventer de panne ni de preuve d'immobilit?. `GetSpeed()` fourni n'est pas une source fiable d?montr?e. |
| Intersection X/T confirm?e sans d?faut | `NORMAL`, arr?t de classe `NAVIGATION` | Conserver l'arr?t normal. T.9 ne choisit ni LEFT, ni RIGHT, ni STRAIGHT. |

`SAFE_STOP` ne peut ?tre lev? par une commande de navigation ou une simple image saine. `recover(...)` exige une autorisation explicite, une immobilit? v?rifi?e ind?pendamment et des entr?es saines r?centes. `DEGRADED` impose une pause, pas une vitesse arbitraire.

## Interface et limites

Un seul propri?taire cam?ra/s?rie/moteurs appelle `FaultManager.update(...)` ? chaque cycle avec le r?sultat de `adapter.process_frame(...)` ou `adapter.check_frame_timeout()`, un `SerialStatus` et, seulement si fiable, un `MotorFeedback`. M?me horloge monotone d?clar?e pour toutes les entr?es ; `LINUX_BOOTTIME` convient ? Picamera2. Ne pas r?utiliser ind?finiment une ancienne d?cision.

Toutes les sorties moteur respectent `motion_allowed`. Bloquer de nouvelles consignes ne supprime pas les anciennes : le contr?leur doit explicitement tenter `link.Stop()` lors d'une nouvelle demande T.9 et transmettre `link.status` en cas d'?chec. `adapter.apply_stop_request()` ne conna?t que son propre verrou T.2/T.5. Aucune activation mat?rielle automatique n'est ajout?e.

`BoundedBlunoStop` r?utilise `arduino.Stop()` : `C` + quatre z?ros int16, ACK d?taill? du mode `A22`. `probe()` utilise la commande vide `B` / r?ponse `OK` d?j? pr?sente dans `Commandes.cpp` ; le futur contr?leur doit ordonnancer ces sondes. Lectures, ?critures et attente globale sont born?es. ?criture partielle, r?ponse inattendue ou entr?e s?rie d?j? en attente : resynchronisation explicite, pas de nouvelle tentative aveugle. Un ACK re?u ne prouve ni l'immobilit? ni, sans identifiant de transaction, l'absence totale de r?ponse ancienne retard?e.

Budgets provisoires : cam?ra 250 ms (? aligner sur `adapter.max_age_ns`), liaison/retour moteur 500 ms, transaction 300 ms. ? valider sur le robot. `WaitACK()` d'origine peut boucler ind?finiment ; les autres appels h?rit?s, dont `connect()`, restent ? borner avant utilisation en mouvement. Les d?lais pySerial ne sont pas une garantie temps r?el du pilote/USB ([API officielle](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)).

**Watchdog ind?pendant Bluno n?cessaire :** `task1` est vide et la protection IR est comment?e. Un Pi bloqu? ou un c?ble retir? ne peut pas garantir la livraison d'un Stop s?rie. Aucun firmware modifi?/flash? ; aucun arr?t physique valid?.

## Essais physiques ult?rieurs, sous contr?le

1. Cam?ra seule : d?connexion, UNKNOWN, orientation, horodatage et traitement sous charge ; v?rifier modes et r?cup?ration.
2. Moteurs neutralis?s : retrait USB, ACK absent/incorrect, d?lais et resynchronisation, sans faux acquittement.
3. Apr?s r?alisation s?par?e du watchdog : Pi bloqu? / c?ble perdu ? v?rifier la coupure locale ind?pendante.
4. Essai moteur autoris? ? faible ?nergie : mesurer immobilit? et distance d'arr?t, priorit? d?faut/navigation et verrouillage ; qualifier s?par?ment le retour moteur.

Tests locaux sur mocks uniquement : `python -m unittest discover -s tests -p "test_fault_management.py" -v`.
