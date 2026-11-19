Firewood Shelf Filler 1.3.3
==========================
Blender-Erweiterung für statische Holzstapel mit Produktbild- und Physikmodus.
Der Paket-Mindeststand ist Blender 4.2. Der Viewport wurde unter Blender
4.3.2 geprüft, der Geometrie- und Vorschauaufbau zusätzlich mit Blender 5.3.0.

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
3. Unter „Scheitlänge“ die Regaltiefe als Richtwert verwenden, eine exakte
   Länge eingeben oder die gemessenen Längen der Quellen übernehmen. Die
   Scheit-Dicke lässt sich unabhängig davon einstellen.
4. Als „Stapelstil“ Produktbild oder Physik wählen. Produktbild ist die
   Voreinstellung. Die Scheite liegen in versetzten Reihen, mit parallelen
   Längsachsen und leicht gestaffelten Stirnflächen. Physik lässt die Scheite
   frei fallen und sich setzen.
5. „Füllhöhe im Viewport einstellen“ öffnen. Den türkisfarbenen Griff direkt
   am Stapel ziehen. Beim Ziehen erscheinen oder verschwinden Holzscheite,
   und die markierte Fläche zeigt die neue Füllgrenze.
6. Mit dem Häkchen am Griff oder Enter übernehmen. Bei „Produktbild“ wird
   die gezeigte Anordnung übernommen, im Physikmodus folgt die Simulation.
   Esc verwirft die Vorschau und stellt den bisherigen Stapel wieder her.

Die „Rasterbelegung“ steuert, wie viele Positionen innerhalb der gewählten
Höhe genutzt werden. Im Physikmodus zeigt die Vorschau die Startpositionen.
Bei automatischer Anzahl und 100 % Rasterbelegung füllt dieser Modus nach
dem Setzen den freien Platz oberhalb des Stapels auf. Die automatische Anzahl
hat kein festes Limit. Wenn kein Scheit hineinpasst, übernimmt der Regler
einen leeren Stapel.

Der fertige Stapel folgt dem Füllwürfel beim Verschieben, Drehen und
Skalieren. Nach einer Größenänderung den Stapel neu erzeugen, damit die
eingegebenen Maße wieder exakt gelten. Zu lange Scheite meldet das Add-on
vor dem Erzeugen. Die sichtbaren Vorder- und Hinterkanten werden nach der
Simulation geprüft. Einen Stapel aus Version 1.3.0 einmal neu erzeugen, um ihn an den
Füllwürfel zu binden. Das Preset „Klein + viele (Physik)“ setzt eine ältere
gespeicherte Rasterbelegung wieder auf 100 %.

Die Simulation nutzt eine temporäre Szene und vereinfachte Konvexhüllen.
Modifier auf Quellobjekten vorab auf eine Kopie anwenden, wenn ihre Geometrie
im Ergebnis sichtbar sein soll. Ein erfolgreicher Lauf ersetzt den Stapel
für das gewählte Fach. Bei einem fehlgeschlagenen Lauf bleibt der alte Stapel
erhalten.

Entwicklung
-----------
Aus dem Repository-Verzeichnis:
  blender -b --factory-startup --python-exit-code 1 --python tests/smoke.py
  blender -b --factory-startup --python-exit-code 1 --python tests/product_smoke.py
  blender -b -c extension validate .
  blender -b -c extension build --source-dir . --output-dir /tmp

Grafischer Regressionstest (mit aktiver grafischer Sitzung):
  blender --factory-startup --enable-event-simulate --python tests/viewport_smoke.py
