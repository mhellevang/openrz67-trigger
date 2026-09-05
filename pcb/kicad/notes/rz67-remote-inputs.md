# RZ67: S1/S2 og kontaktmotstand

Undersøkt 2026-09-05. Dokumentasjon, ikke produksjonsgodkjenning. Ingen komponenter endret.

## Konklusjon

Manualen gir funksjon, kontaktmerking og kameraets batteristrøm, men ingen funnet spesifikasjon for S1/S2-inngangsstrøm, terskelspenning eller maksimal kontaktmotstand. Nettundersøkelsen fant heller ingen bekreftede målinger av disse verdiene. Dette betyr ikke at opplysningene aldri er publisert.

Lav kontaktmotstand er en fornuftig måte å redusere usikkerheten på. TLP172AM er en bedre kandidat enn TLP172GM med hensyn til spenningsfall, men kameraets kompatibilitet er ikke dokumentert av dette alene. Endelig valg krever også kontroll av strøm, spenning/transienter, temperatur, lekkasje, timing og LED-driving.

## Hva originalmanualen faktisk sier

Kilde: [Mamiya RZ67 repair manual](/Users/mathiashellevang/Koofr/Manualer/Mamiya-RZ67-repair.pdf). Sidetall nedenfor er trykte sidetall. Hele dokumentet er OCR-søkt; relevante sider er også visuelt kontrollert. OCR kan overse tekst.

| Side | Opplysning | Betydning for utløseren |
|---|---|---|
| 95, fig. 114 | VS på søkerkontakt 2: 3,2 V ±2 mV. Kommer når S1 aktiveres med RZ-objektiv; justeres med VR1. | Referanseutgang til søkeren, ikke S1/S2-terskel eller tillatt spenningsfall over bryteren. |
| 102 | Winderens 9 V omformes til ca. 6 V, også til RC-kontakten. | Ikke anta at RC-kontaktens BW er kameraets ordinære batteriutgang. |
| 104, fig. 122 | RC-kontakt: 6 V/BW, Ground, S1, S2. | Identifiserer signalene, uten elektriske inngangsgrenser. Tegningsrekkefølgen er ikke en universell pinnummerering sett fra pluggfronten. |
| 106 | S1 aktiveres ved ca. 0,5 mm knappetrykk; S2 ved ca. 2 mm. | Støtter S1 før S2, men oppgir ikke et minimumsintervall i millisekunder. |
| 114, C-2 / fig. 131 | 3–7 mA med S1 på; 16–21 mA ved utløsning med 1 eller 4 sekunders lukkertid. | Målt i batteritilførselen med dummybatteri. Ikke målt gjennom RC-kontaktene og ikke en spesifisert grense for korte strømspisser. |
| 116, fig. 132-A | Frakoblet moving-coil-enhet: omtrent 6 Ω. | Gjelder aktuatoren, ikke RC-inngangens motstand. Kan ikke brukes til å beregne kontaktstrøm uten den mellomliggende kretsen. |

## Indirekte støtte fra originalt tilbehør

[Mamiya Electromagnetic Cable Release](https://ianbfoto.com/downloads/Mamiya%20645/Mamiya%20Electromagnetic%20Cable%20Release.pdf), én side, visuelt lest: Type B har omtrent 4 m rett kabel. Knappen har halvtrykk for måling og fulltrykk for utløsning. Kabeltverrsnitt, kontaktmotstand og inngangsgrenser oppgis ikke. Ledninger og kontakter har en endelig motstand, så kameraet krever ikke en ideell 0 Ω-bryter. Dette fastsetter likevel ingen numerisk motstandsgrense.

[Mamiya RS401](https://ianbfoto.com/downloads/Mamiya%20645/Mamiya%20645%20RZ67%20Remote%20Control%20RS401.pdf), fire sider, visuelt lest: beskriver original trådløs fjernutløser for blant annet RZ67-serien. Ingen S1/S2-strøm, kontaktmotstand eller terskelspenning funnet i denne bruksanvisningen.

[Jan Griffioens egen reparasjon](https://jantecnl.synology.me/en/mamiya-rz67-repair-shutter-release-electronics-problem/) beskriver at han hadde servicemanualen, men måtte arbeide med blokk- og ledningsskjema uten funksjonelt elektronikkskjema. Dette beskriver hans kildetilgang, ikke et bevis for at et slikt skjema ikke finnes.

## Komponentsammenligning

- [Omron G6K](https://components.omron.com/us-en/system/files/2026-05/datasheet_pdf/K106-E1.pdf): initial kontaktmotstand maks. 100 mΩ (0,1 Ω), målt ved 10 mA / 1 V. Spolemotstanden er ikke motstanden kameraet ser.
- [Toshiba TLP172GM](https://toshiba.semicon-storage.com/info/TLP172GM_datasheet_en_20230525.pdf?did=36716&prodName=TLP172GM): R_on maks. 35 Ω for kort måling under 1 s, 50 Ω kontinuerlig, ved databladets testbetingelser, 25 °C og LED-strøm 5 mA.
- [Toshiba TLP172AM](https://toshiba.semicon-storage.com/info/docget.jsp?did=36714&prodName=TLP172AM): R_on maks. 2 Ω ved databladets testbetingelser, 25 °C og LED-strøm 5 mA. Utgang klassifisert 60 V / 500 mA; grenser må vurderes med derating. Ikke en garanti for kamerakompatibilitet.

Illustrasjon, ikke målt kontaktstrøm: Dersom 7 mA gikk gjennom bryteren, ville 50 Ω gi 350 mV spenningsfall og 2 Ω gi 14 mV. Ved 21 mA ville tallene være 1,05 V og 42 mV. Det er ikke fastslått at kameraets batteristrøm går gjennom S1 eller S2. Tallene er derfor ikke en worst-case-beregning for kameraet.

## Avgrenset neste test

En utvendig adapter ved den allerede verifiserte RC-pinouten kan brukes til å sammenligne det fungerende mekaniske reléet med kjente seriemotstander i hver signalledning, én om gangen og sammen. Ikke bruk BW-pinnen og ikke tilfør ekstern spenning. Kontroller halvtrykk, utløsning, bulb og svakere batteri. Dette undersøker toleranse for kontaktmotstand på det aktuelle kameraet, men erstatter ikke kontroll av PhotoMOS-lekkasje, strømspisser og koblingstid eller testing av andre kameramodeller.
