
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

## Installatie instructies

> **TODO** Beschrijf hoe de code gebouwd kan worden.

---

## Gebruikshandleiding

> **TODO** Beschrijf stap voor stap hoe de arm bewogen kan worden middels enkele voorbeelden.

---

## Behaalde eisen

In de onderstaande onderdelen staan lijsten weergegeven (druk op het pijltje). Deze lijsten bevatten de verschillende eisen gesteld aan de opdracht. Door over de eis te 'hoveren' zal de tooltip de prioriteit en de beschrijving van de eis weergeven. <br>
Voor elke eis staat vermeld of deze behaald is gevolgd door de reden waarom.

<details>
     <summary> Package </summary>
<br>

|Eis|Behaald|Reden|
|---|-------|-----|
|[PA01](a "SHOULD &#124; Alle code is gepackaged volgens de ROS-directorystructuur.")                                                     |nee|n/a|
|[PA02](a "MUST &#124; Package is te bouwen met colcon op Foxy Fitzroy.")                                                                 |nee|n/a|
|[PA03](a "MUST &#124; De applicatie wordt gebouwd met C++ volgens de Object Orientedprincipes die je geleerd hebt bij eerdere courses.") |nee|n/a|
|[PA04](a "SHOULD &#124; Alle code voldoet aan de ROS C++ Style Guide.")                                                                  |nee|n/a|

<br>
</details>
<details>
     <summary> Virtuele servo controller </summary>
<br>

|Eis|Behaald|Reden|
|---|-------|-----|
|[VS01](a "MUST &#124; De virtuele controller luistert naar een topic waarop string messages in het formaat van de SSC-32U worden geplaatst. Van de interface moeten ten minste commando’s zijn opgenomen voor het verplaatsen van de servo’s met een ingestelde duur en het stoppen van de servo’s.")    |nee|n/a|
|[VS02](a "MUST &#124; De virtuele controller reageert op het topic (zie eis VS01) door bijbehorende joint_state messages te publiceren.") |nee|n/a|
|[VS03](a "MUST &#124; De virtuele robotarm wordt gevisualiseerd in Rviz (een URDF-modelvan de arm is beschikbaar op OnderwijsOnline).")   |nee|n/a|
|[VS04](a "MUST &#124; De virtuele robotarm gedraagt zich realistisch m.b.t. tijdgedrag (servo’s roteren kost tijd en gaat geleidelijk).") |nee|n/a|
|[VS05](a "SHOULD &#124; De virtuele robotarm kan op een willekeurige plaats in de virtuele wereld geplaatst worden.")                     |nee|n/a|

<br>
</details>
<details>
     <summary> Virtueel bekertje </summary>
<br>

|Eis|Behaald|Reden|
|---|-------|-----|
|[VC01](a "SHOULD &#124; Er kan op een willekeurige plek in de virtuele wereld een bekertje geplaatst worden.") |nee|n/a|
|[VC02](a "MUST &#124; Puliceert een 3D-visualisatie van het bekertje voor Rviz.")                              |nee|n/a|
|[VC03](a "SHOULD &#124; Detecteert de relevante punten van de gripper.")                                       |nee|n/a|
|[VC04](a "COULD &#124; Visualiseert de gedetecteerde punten van de gripper.")                                  |nee|n/a|
|[VC05](a "SHOULD &#124; Visualiseert wanneer de gripper het bekertje vastheeft.")                              |nee|n/a|
|[VC06](a "MUST &#124; Het bekertje beweegt mee met de gripper (als hij vastgehouden wordt).")                  |nee|n/a|
|[VC07](a "MUST &#124; Het bekertje is onderhevig aan zwaartekracht wanneer het losgelaten wordt.")             |nee|n/a|
|[VC08](a "MUST &#124; Het bekertje bepaalt en publiceert zijn positite.")                                      |nee|n/a|
|[VC09](a "SHOULD &#124; Het bekertje bepaald en publiceert zijn snelheid.")                                    |nee|n/a|
|[VC10](a "COULD &#124; De snelheid wordt getoond met rqt_plot")                                                |nee|n/a|

<br>
</details>
<details>
    <summary> Demonstratie infrastructuur </summary>
<br>

|Eis|Behaald|Reden|
|---|-------|-----|
|[DI01](a "MUST &#124; Een demoscript stuurt over de tijd een sequentie van commando’snaar de armcontroller.")    |nee|n/a|
|[DI02](a "COULD &#124; Locatie van het bekertje wordt in de roslaunch-configuratie bepaald.")                    |nee|n/a|
|[DI03](a "COULD &#124; Locatie van de arm in de wereld wordt in de roslaunch-configuratiebepaald.")              |nee|n/a| 

<br>
</details>
<details>
    <summary> Gebruikshandleiding </summary>
<br>

|Eis|Behaald|Reden|
|---|-------|-----|
|[DM01](a "MUST &#124; Beschrijft hoe de code gebouwd kan worden.")                                            |nee|n/a|
|[DM02](a "MUST &#124; Beschrijft stap voor stap hoe de arm bewogen kan worden middels enkele voorbeelden.")   |nee|n/a|
|[DM03](a "MUST &#124; Beschrijft welke eisen gerealiseerd zijn. En geeft hierbij een (korte)toelichting.")    |nee|n/a|

<br>
</details>
<details>
    <summary> Ontwerpdocumentatie </summary>
<br>

|Eis|Behaald|Reden|
|---|-------|-----|
|[DD01](a "MUST &#124; Beschrijft de structuur van de package (Nodes, topics, messages, etc. ).")                    |nee|n/a|
|[DD02](a "MUST &#124; Beschrijft de structuur en samenhang van de broncode(class-diagrams, beschrijving, etc.).")   |nee|n/a|
|[DD03](a "COULD &#124; Beschrijft hoe het gedrag van alle belangrijke componenten gerealiseerd is.")                |nee|n/a|
|[DD04](a "SHOULD &#124; Beschrijft de API van alle publieke interfaces.")                                           |nee|n/a|

<br>
</details>