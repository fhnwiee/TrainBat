Diego Mäder, 23.02.2023

A. Streckendaten:
Enthält verschiedene Streckendaten, welche im Tool verwendet werden können.

B. Fahrpläne:
Enthält Beispielfahrpläne, die im Tool verwendet werden können. Das Excel-File Fahrplan_ZH_HB_HB.xlsx gibt ein Beispiel vor, wie weitere
Fahrpläne erstellt werden müssen, um damit im Tool arbeiten zu können. Dieser Fahrplan kann als Standardfahrplan im Tool eingelesen und
simuliert werden.

C. Fahrzeugparameter:
Enthält Matlab-Skripte, in welchen die Fahrzeuge modelliert wurden. Diese Skripte können erweitert werden, um weitere Fahrzeuge zu
modellieren und die Strecken im Tool mit diesen Fahrzeugen zu simulieren. Bei erweiterung der Fahrzeugmodelle muss darauf geachtet
werden, dass dieselben Parameter in denselben Einheiten hinterlegt werden.

D. Messdaten Speisepunkt Museumstrasse:
Dieser Ordner enthält offizielle, von der SBB gemessene Werte von Strom, Spannung und Leistung im Speisepunkt Museumstrasse. Es handelt
sich dabei um über 8 Minuten gemittelte Werte.

E. Batteriemodul Bordline:
Enthält das Datenblatt des Batteriemoduls Borline von ABB. Dieses kann für die Bestimmung der Modulkonfiguration für die ausgelegten 
Traktionsbatterien verwendet werden. Das dazugehörige Matlab-Skript befindet sich in Ordner F.

F. Matlab Funktionen und Skripte:
Enthält alle Matlab-Skripte und Funktionen, um mit dem Tool arbeiten zu können. Alle Funktionen können vom Hauptskript
"MainScriptSimulationOfTrackData_Final.mlx" ausgeführt werden. 

G. Ablaufdiagramme:
Enthält die Ablaufdiagramme der verschiedenen Matlab-Funktionen. Kann für das einfachere Verständnis der Funktionsabläufe helfen.

H. Datenblatt KISS und weitere Fahrzeuge:
Enthält Datenblätter von verschiedenen Fahrzeugen, welche für die Simulation verwendet werden können.

K. Tabellen der Simulations- und Auslegungsresultate:
Enthält verschiedene Tabellen der durchgeführten Simulationen und Auslegungen der Traktionsbatterien mit den verschiedenen Methoden.
Wie diese Resultate erzeugt wurden ist im Dokument "Master Thesis_Projektbericht_Diego_Mäder_Final_V3.pdf" zu finden.

L. Modulkonfigurationen für die ausgelegten Traktionsbatterien:
Enthält eine Tabelle, in welcher die Resultate der Modulkonfigurationen mit dem Batteriemodul Bordline enthalten sind. Diese Resultate 
wurden jedoch mit einer älteren Version der Funktion "CalcModuleComposition.mlx" erstellt und sind fehlerbehaftet.

Master Thesis_Projektbericht_Diego_Mäder_Final_V3.pdf:
Dieses Dokument enthält den Projektbericht der von Diego Mäder durchgeführten Masterarbeit. Darin ist der Aufbau des Tools, sowie die
Resultate der Streckensimulationen und der Auslegung der Traktionsbatterien detailliert beschrieben.
