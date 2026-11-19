Firewood Shelf Filler 1.3.0
==========================
Blender-Erweiterung für statische Holzstapel mit vorgelagerter Physiksimulation.
Der Paket-Mindeststand ist Blender 4.2, getestet wurde mit Blender 4.3.2.

Installation
------------
1. Eine ältere Version des Add-ons deaktivieren.
2. In Blender unter Bearbeiten > Einstellungen > Erweiterungen die Option
   „Von Datenträger installieren“ wählen und die ZIP-Datei öffnen.
3. Im 3D-Viewport mit N die Seitenleiste öffnen und „Firewood“ wählen.

Holzstapel erzeugen
------------------
1. Eine Collection mit einzelnen Holzscheit-Meshes als Quelle auswählen.
2. Einen Würfel für den freien Innenraum auswählen oder erstellen.
   Die lokalen Achsen stehen für Breite (X), Tiefe (Y) und Höhe (Z).
3. „Füllhöhe im Viewport einstellen“ öffnen. Den türkisfarbenen Griff direkt
   am Stapel ziehen. Beim Ziehen erscheinen oder verschwinden Holzscheite,
   und die markierte Fläche zeigt die neue Füllgrenze.
4. Mit dem Häkchen am Griff oder Enter übernehmen. Blender berechnet dann
   die Physik und ersetzt den bisherigen Stapel durch das statische Ergebnis.
   Esc verwirft die Vorschau und stellt den bisherigen Stapel wieder her.

Während des Ziehens zeigt Blender die Holzscheite an ihren Startpositionen.
Die endgültige Verteilung entsteht, wenn die Scheite in der Simulation zur
Ruhe kommen. Die „Rasterbelegung“ steuert, wie viele Startpositionen innerhalb
der gewählten Höhe genutzt werden. Wenn kein Scheit hineinpasst, bedeutet
Übernehmen einen leeren Stapel.

Die Simulation nutzt eine temporäre Szene und vereinfachte Konvexhüllen.
Modifier auf Quellobjekten vorab auf eine Kopie anwenden, wenn ihre Geometrie
im Ergebnis sichtbar sein soll. Ein erfolgreicher Lauf ersetzt den Stapel
für das gewählte Fach. Bei einem fehlgeschlagenen Lauf bleibt der alte Stapel
erhalten.

Entwicklung
-----------
Aus dem Repository-Verzeichnis:
  blender -b --factory-startup --python-exit-code 1 --python tests/smoke.py
  blender -b -c extension validate .
  blender -b -c extension build --source-dir . --output-dir /tmp

Grafischer Regressionstest (mit aktiver grafischer Sitzung):
  blender --factory-startup --enable-event-simulate --python tests/viewport_smoke.py
