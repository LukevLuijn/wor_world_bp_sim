# Beroepsproduct simulatie

|Algemen informatie ||
|------------|------------------|
|**Student** | Luke van Luijn   |
|**Docent**  | Bram Knippenberg |
|**Vak**     | WoR - World      |
|**Opdracht**| Simulatie        |
|**Datum**   | 01-28-2021       |
|**Versie**  | 2                |

--- 
![esp8266 flowchart](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.github.com/anoff/plantbuddy/master/assets/esp8266.iuml)


---

## Installatie instructies

> **TODO** Beschrijf hoe de code gebouwd kan worden.

---

## Gebruikshandleiding

> **TODO** Beschrijf stap voor stap hoe de arm bewogen kan worden middels enkele voorbeelden.

---

## Ontwerpdocumentatie

In de onderstaande onderdelen staat de ontwerpdocumentatie van de simulatie opdracht verwerkt. Als eerste is de
structuur van het systeem uitgewerkt, gevolgd door de samenhang van de verschillende componenten en tot slot het gedrag
van het systeem. De verschillende onderdelen zijn verborgen achter inklapbare velden om de omvang van dit document te
minimaliseren. Deze onderdelen kunnen zichtbaar gemaakt worden door op de respectievelijke pijltjes te klikken.

<details>
    <summary style="font-size:17px"> Structuur </summary>

> **TODO**

</details>

<details>
    <summary style="font-size:17px"> Samenhang </summary>

> **TODO**

</details>

<details>
    <summary style="font-size:17px"> Gedrag </summary>

> **TODO**

</details>

---

## Behaalde eisen

In de onderstaande onderdelen staan lijsten weergegeven (druk op het pijltje). Deze lijsten bevatten de verschillende
eisen gesteld aan de opdracht. Door over de eis te 'hoveren' zal de tooltip de prioriteit en de beschrijving van de eis
weergeven. Voor elke eis staat vermeld of deze behaald is gevolgd door de reden waarom.

<details>
     <summary> Package </summary>

|Eis|Behaald|Toelichting|
|---|-------|-----------|
|[PA01](a "SHOULD &#124; Alle code is gepackaged volgens de ROS-directorystructuur.")                                                     |ja|De opzet en uitwerking zijn volgens de standaard ROS structuur opgezet.|
|[PA02](a "MUST &#124; Package is te bouwen met colcon op Foxy Fitzroy.")                                                                 |ja|De package is te bouwen in de foxy fitzroy architectuur.|
|[PA03](a "MUST &#124; De applicatie wordt gebouwd met C++ volgens de Object Orientedprincipes die je geleerd hebt bij eerdere courses.") |ja|De code is geschreven op basis van deze en eerder geleerde OO-principes.|
|[PA04](a "SHOULD &#124; Alle code voldoet aan de ROS C++ Style Guide.")                                                                  |ja|Alle code is geschreven op de manier beschreven in de [ROS2 styleguide](http://wiki.ros.org/CppStyleGuide).|

<br>
</details>
<details>
     <summary> Virtuele servo controller </summary>

|Eis|Behaald|Toelichting|
|---|-------|-----------|
|[VS01](a "MUST &#124; De virtuele controller luistert naar een topic waarop string messages in het formaat van de SSC-32U worden geplaatst. Van de interface moeten ten minste commando’s zijn opgenomen voor het verplaatsen van de servo’s met een ingestelde duur en het stoppen van de servo’s.")    |nee|n/a|
|[VS02](a "MUST &#124; De virtuele controller reageert op het topic (zie eis VS01) door bijbehorende joint_state messages te publiceren.") |nee|n/a|
|[VS03](a "MUST &#124; De virtuele robotarm wordt gevisualiseerd in Rviz (een URDF-modelvan de arm is beschikbaar op OnderwijsOnline).")   |nee|n/a|
|[VS04](a "MUST &#124; De virtuele robotarm gedraagt zich realistisch m.b.t. tijdgedrag (servo’s roteren kost tijd en gaat geleidelijk).") |nee|n/a|
|[VS05](a "SHOULD &#124; De virtuele robotarm kan op een willekeurige plaats in de virtuele wereld geplaatst worden.")                     |nee|n/a|

<br>
</details>
<details>
     <summary> Virtueel bekertje </summary>

|Eis|Behaald|Toelichting|
|---|-------|-----------|
|[VC01](a "SHOULD &#124; Er kan op een willekeurige plek in de virtuele wereld een bekertje geplaatst worden.") |ja|De cup_node node luistert naar een x en y waarde meegegeven aan de executable, deze x en y waarde zijn de locatie waar de beker geplaatst wordt.|
|[VC02](a "MUST &#124; Puliceert een 3D-visualisatie van het bekertje voor Rviz.")                              |ja|Door gebruik te maken van de MESH_RESOURCE van de marker kan er een .stl bestand ingeladen worden en weergegeven worden in RVIZ.|
|[VC03](a "SHOULD &#124; Detecteert de relevante punten van de gripper.")                                       |nee|n/a|
|[VC04](a "COULD &#124; Visualiseert de gedetecteerde punten van de gripper.")                                  |nee|n/a|
|[VC05](a "SHOULD &#124; Visualiseert wanneer de gripper het bekertje vastheeft.")                              |ja|Wanneer het bekertje beweegt zal het van kleur veranderen, wanneer het bekertje weer losgelaten wordt veranderd het ook van kleur. |
|[VC06](a "MUST &#124; Het bekertje beweegt mee met de gripper (als hij vastgehouden wordt).")                  |nee|n/a|
|[VC07](a "MUST &#124; Het bekertje is onderhevig aan zwaartekracht wanneer het losgelaten wordt.")             |nee|n/a|
|[VC08](a "MUST &#124; Het bekertje bepaalt en publiceert zijn positite.")                                      |ja|Het 'marker' bericht bevat een pose, deze pose wordt los van de marker naar een appart topic verstuurd (simulation/cup/pose).|
|[VC09](a "SHOULD &#124; Het bekertje bepaald en publiceert zijn snelheid.")                                    |nee|n/a|
|[VC10](a "COULD &#124; De snelheid wordt getoond met rqt_plot")                                                |nee|n/a|

<br>
</details>
<details>
    <summary> Demonstratie infrastructuur </summary>

|Eis|Behaald|Toelichting|
|---|-------|-----------|
|[DI01](a "MUST &#124; Een demoscript stuurt over de tijd een sequentie van commando’snaar de armcontroller.")    |nee|n/a|
|[DI02](a "COULD &#124; Locatie van het bekertje wordt in de roslaunch-configuratie bepaald.")                    |nee|n/a|
|[DI03](a "COULD &#124; Locatie van de arm in de wereld wordt in de roslaunch-configuratie bepaald.")              |nee|n/a| 

<br>
</details>
<details>
    <summary> Gebruikshandleiding </summary>

|Eis|Behaald|Toelichting|
|---|-------|-----------|
|[DM01](a "MUST &#124; Beschrijft hoe de code gebouwd kan worden.")                                            |nee|n/a|
|[DM02](a "MUST &#124; Beschrijft stap voor stap hoe de arm bewogen kan worden middels enkele voorbeelden.")   |nee|n/a|
|[DM03](a "MUST &#124; Beschrijft welke eisen gerealiseerd zijn. En geeft hierbij een (korte)toelichting.")    |ja|Zie het huidige onderdeel van dit document.|

<br>
</details>
<details>
    <summary> Ontwerpdocumentatie </summary>

|Eis|Behaald|Toelichting|
|---|-------|-----------|
|[DD01](a "MUST &#124; Beschrijft de structuur van de package (Nodes, topics, messages, etc. ).")                    |nee|n/a|
|[DD02](a "MUST &#124; Beschrijft de structuur en samenhang van de broncode(class-diagrams, beschrijving, etc.).")   |nee|n/a|
|[DD03](a "COULD &#124; Beschrijft hoe het gedrag van alle belangrijke componenten gerealiseerd is.")                |nee|n/a|
|[DD04](a "SHOULD &#124; Beschrijft de API van alle publieke interfaces.")                                           |ja|Alle code is voorzien van relevant doxygen commentaar.|

<br>
</details>