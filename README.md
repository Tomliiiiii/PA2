# PA2 – Navigation and Environmental Mapping for an Autonomous Mobile Robot

Projektarbeit 2, ZHAW (ADLS), Tim Badertscher. Betreuer: Christian Glahn, EDXI Lab.

Ziel: Steuerung eines Waveshare RaspRover mit RPLIDAR A2M12, sodass der Roboter mit ROS2 eine Karte erstellt und darin selbständig zu Zielen fährt.

**Inhalt:** [Befehle am Roboter](#befehle-am-roboter) · [Fehlerbehebung](#fehlerbehebung) · [Demo-Ablauf](#demo-ablauf) · [Bericht kompilieren](#bericht-kompilieren) · [Ordnerstruktur](#ordnerstruktur)

---

## Befehle am Roboter

### 1. Verbindung

| Netzwerk | JupyterLab | Foxglove |
|---|---|---|
| Bekanntes WLAN (zu Hause) | `http://192.168.1.123:8888` | `ws://192.168.1.123:8765` |
| Kein bekanntes WLAN → Pi öffnet Hotspot **„AccessPopup“** | `http://192.168.50.5:8888` | `ws://192.168.50.5:8765` |

Der Hotspot öffnet nur, wenn kein bekanntes WLAN in Reichweite ist. Laptop zuerst mit dem Hotspot verbinden.

Jedes neue Terminal in JupyterLab zuerst in den Container:

```bash
docker exec -it ros2 bash
```

### 2. Alles starten (Terminal 1)

```bash
ros2 launch rasprover_bridge bringup.launch.py
```

- Roboter **die ersten Sekunden stillhalten** (Gyro-Kalibrierung).
- Etwa **20 s warten**, bis Nav2 bereit ist (SLAM Toolbox startet nach 5 s, Nav2 nach 15 s).
- **Ctrl+C** beendet alles, die Motoren stoppen.

Varianten:

```bash
ros2 launch rasprover_bridge bringup.launch.py nav:=false        # nur Roboter + SLAM (Kartieren)
ros2 launch rasprover_bridge bringup.launch.py foxglove:=false   # ohne Foxglove-Bridge
ros2 launch rasprover_bridge robot.launch.py                     # nur Roboter (Bridge, Lidar, URDF, Foxglove)
ros2 launch rasprover_bridge bringup.launch.py 2>&1 | tee /ros2_ws/nav_log.txt   # mit Log-Datei
```

### 3. Foxglove (Laptop)

1. **Desktop-App** öffnen (die Web-Version kann sich nicht mit `ws://` verbinden).
2. Open connection → **Foxglove WebSocket** → Adresse aus der Tabelle oben.
3. Im 3D-Panel einblenden: `/map`, `/scan`, `/plan`, `/local_costmap/costmap`, `/explore/frontiers`.
4. **Ziel setzen:** Panel-Einstellungen → Publish → bei **„2D pose“** das Topic `/goal_pose` eintragen (nicht bei „2D pose estimate“). Dann Publish-Werkzeug wählen, auf die Karte klicken und in Zielrichtung ziehen.

### 4. Autonome Exploration (Terminal 2)

```bash
source /ros2_ws/install/setup.bash
ros2 launch explore_lite explore.launch.py
```

Der Roboter fährt selbständig zu den Rändern der bekannten Karte und hört von selbst auf, wenn nichts mehr zu erkunden ist.

⚠️ **Offene Türen und Absätze absperren**, mit etwas, das höher als 20 cm und nicht schwarz-matt ist. Der Lidar misst nur in einer Ebene auf 19 cm Höhe und sieht Türschwellen, Stufen und tiefe Hindernisse nicht.

### 5. Weitere Befehle

```bash
# Karte speichern (vor Ctrl+C, sonst ist sie weg)
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/<name>

# Manuell fahren: i vorwärts, , rückwärts, j/l drehen, k stoppen
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Konstante Drehung zum Testen (mit Ctrl+C stoppen)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.6}}"
```

### 6. Prüfen

```bash
ros2 daemon stop && ros2 node list           # laufende Nodes (Daemon neu starten, falls Liste unvollständig)
ros2 topic hz /scan                          # Lidar, sollte ca. 10 Hz sein
ros2 topic hz /odom                          # Odometrie, sollte ca. 20 Hz sein
ros2 run tf2_ros tf2_echo map base_link      # Lokalisierung bereit?
ros2 topic echo /battery_state --field voltage
```

Log auswerten:

```bash
grep -E "Begin navigating|Goal succeeded|Goal failed|Failed to make progress|Running|missed|ERROR" /ros2_ws/nav_log.txt
```

### 7. Einzeln starten (zur Fehlersuche, je ein Terminal)

```bash
ros2 launch rasprover_bridge robot.launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/config/slam.yaml
ros2 run tf2_ros tf2_echo map base_link        # warten bis eine Transformation erscheint, dann Ctrl+C
ros2 launch nav2_bringup navigation_launch.py params_file:=/ros2_ws/config/nav2.yaml
```

### 8. Container und Build

Auf dem Pi (nicht im Container):

```bash
docker start ros2        # Container nach Neustart starten
docker restart ros2      # neu starten, wenn etwas hängt
docker ps                # läuft der Container?
```

Nach Code-Änderungen (im Container):

```bash
cd /ros2_ws
colcon build --packages-select rasprover_bridge
source install/setup.bash
```

Änderungen in `/ros2_ws/config/slam.yaml` oder `nav2.yaml` brauchen keinen Build, nur einen Neustart.

Dateien hochladen: JupyterLab zeigt nur `~/ugv_rpi/`. Hochgeladene Dateien landen dort und müssen im Terminal (auf dem Pi) verschoben werden, z.B. `mv ~/ugv_rpi/<datei> ~/ros2_ws/...`. Zum Herunterladen umgekehrt zuerst nach `~/ugv_rpi/` kopieren.

---

## Fehlerbehebung

| Problem | Lösung |
|---|---|
| Foxglove verbindet nicht | IP-Adresse prüfen, läuft `bringup.launch.py`? Desktop-App statt Web-Version verwenden. |
| Nav2: „transform base_link to map not available“ | `robot_state_publisher` oder SLAM läuft nicht. Alles beenden, `docker restart ros2`, neu starten. |
| SLAM: „Message Filter dropping message“ | Transformation `base_link → laser` fehlt → `robot_state_publisher` läuft nicht. |
| Lidar-Fehler `80008002` / Lidar-Motor dreht nicht | USB-Stromlimit: `usb_max_current_enable=1` in `/boot/firmware/config.txt`, dann Neustart. |
| `ros2 node list` unvollständig | `ros2 daemon stop`, dann nochmals. |
| Roboter fährt nicht los, wenn ein Ziel gesetzt wird | In Foxglove muss das Topic bei „2D pose“ `/goal_pose` sein. |
| Roboter dreht hin und her / bleibt an Hindernissen hängen | Log nach `Failed to make progress` und `Control loop missed` durchsuchen. Keine schwarz-matten Hindernisse (Lidar sieht sie schlecht). |
| Scan dreht sich im Stillstand gegenüber der Karte | Gyro-Drift. Bridge-Node neu starten (Zero-Velocity-Update in Arbeit). |
| `docker: permission denied` | Benutzer in Gruppe `docker` aufnehmen (`sudo usermod -aG docker $USER`), dann Neustart. |
| Keine serielle Verbindung zum ESP32 | Die Waveshare-Web-App darf nicht laufen (Autostart in `crontab` deaktiviert). |
| Mehrzeilige Befehle nach `docker exec` gehen verloren | Zuerst nur `docker exec -it ros2 bash` ausführen, dann die Befehle. |

---

## Demo-Ablauf

1. **Hardware zeigen:** RaspRover, Lidar mit 3D-gedruckter Halterung, Pi und ESP32.
2. **Starten** mit `bringup.launch.py`, kurz erklären, was startet.
3. **Foxglove:** Scan und Koordinatensysteme (`map`, `odom`, `base_link`, `laser`) zeigen.
4. **Exploration starten:** Roboter erstellt die Karte selbst.
5. **Ziele setzen** in der fertigen Karte, evtl. mit Hindernis dazwischen.
6. **Grenzen ansprechen:** tiefe Kanten (Türschwelle), schwarz-matte Objekte, Rechenleistung des Pi.

Vorher: Akku laden, Raum prüfen (Türen absperren), Verbindung über den Hotspot testen.

---

## Bericht kompilieren

```bash
latexmk -pdf main.tex
```

In VS Code mit LaTeX Workshop: Speichern kompiliert automatisch. Benötigt TeX Live mit den Sammlungen `collection-latexrecommended`, `collection-latexextra`, `collection-fontsrecommended`, `collection-langenglish`, `collection-pictures`, `collection-mathscience`.

Falls `tlmgr` mit „checksums differ“ abbricht, ist der Download-Server fehlerhaft:

```
tlmgr option repository https://mirror.ctan.org/systems/texlive/tlnet
```

---

## Ordnerstruktur

```
main.tex                  Hauptdatei (Einstellungen, Reihenfolge der Kapitel)
references.bib            Literatur
frontmatter/              Titelseite, Impressum, Abstract, Abkürzungen
chapters/                 Kapitel 1–5 (Introduction, Theory, Methods, Results, Discussion)
appendix/                 Anhänge A–E (KI-Deklaration, Befehle, Rohdaten, Code, Selbständigkeitserklärung)
figures/                  Abbildungen (Bilder und TikZ)
code/                     Code für den Anhang (Bridge-Node, URDF, Launch-Files)
```

Der Roboter-Code liegt zusätzlich im Repo `farmbot-zhaw-upm/Autonomous-Agricultural-Robot`.
