# HM485_fwUpdate — Firmware-Update über den Bus, direkt aus FHEM

Flasht HBWired-Geräte mit [HBW-Booter](../README.md) **over-the-bus aus FHEM heraus** — so wie es
sonst nur die CCU kann. Praktisch ein natives `flash_tool.py` im FHEM-Modul, ohne externes Python.

**Transport-agnostisch:** Alle Frames gehen als `CMD_SEND` raus. Damit läuft es gleichermaßen mit
**HMW-Gateway**, echtem **eQ-3-LGW** und **HM485d**/USB-Adapter — die reichen die Frames alle
transparent auf den Bus. Es wird kein Sonderkommando im Transport gebraucht (anders als `discovery`,
das der Transport selbst fährt).

**Status: am echten Bus verifiziert** — mehrere vollständige Updates am Stück, inkl. `getConfig`
dazwischen und wiederholten Läufen mit geänderter `.hex`.

## Ablauf

Exakt die hs485d-/CCU-Choreografie:

```
z z  →  u (App: ACK, dann WDT-Reset)  →  ~1,2 s  →  u (Booter: ACK)
     →  p (Blockgröße)  →  w-Schleife  →  r (Verify)  →  g (CRC + App-Start)  →  Z Z
```

Details, auf die es ankommt:

* **Page 0 wird zuletzt geschrieben.** Bricht das Update ab, bleibt der Reset-Vektor ungültig —
  das Gerät startet also nie eine halb geflashte App, sondern bleibt im Booter.
* **Verify** liest den Flash per `r` zurück und vergleicht byte-genau gegen das Image.
* **`g`** überträgt Länge + CRC16 (Poly `0x1002`); der Booter startet die App nur bei Übereinstimmung.
* **`Z Z`** wird auch bei jedem Abbruch gesendet — der Bus bleibt nie im Zero-Communication stehen.

## Installation

1. Datei nach `/opt/fhem/FHEM/lib/HM485/HM485_fwUpdate.pm` kopieren.
2. In `10_HM485.pm` einbinden:
   ```perl
   use lib::HM485::HM485_fwUpdate;
   ```
3. **set-Befehl** ergänzen — in `%sets`/setList:
   ```perl
   'fwUpdate' => 'textField',
   ```
   und in `HM485_Set()`:
   ```perl
   } elsif ($cmd eq 'fwUpdate') {
       return HM485_fwu_Start($hash, $value);
   }
   ```
4. **Antwort-Hook** in `HM485_Parse($ioHash, $message)` — direkt nach
   `my $msgData = uc( unpack ('H*', substr($message, 4)));` und **vor**
   `if ($msgCmd == HM485::CMD_RESPONSE)`:
   ```perl
   foreach my $d (values %{$modules{HM485}{defptr}}) {
       next unless $d->{fwu} && $d->{IODev} && $d->{IODev} == $ioHash;
       HM485_fwu_OnResp($d, $msgData, $msgCmd, $msgId);
       return $ioHash->{NAME};
   }
   ```
   Bewusst in `HM485_Parse` und nicht in `HM485_ProcessResponse`: dort werden Antworten je nach
   `msgCmd` in `ProcessResponse` **oder** `ProcessEvent` verzweigt — der Hook muss beide sehen.
   `$msgCmd` wird für die NACK-Erkennung gebraucht, `$msgId` für die Zuordnung der Antwort zum
   gesendeten Frame (sonst kann das ACK eines **anderen** Geräts das Update verschieben).

## Benutzung

```
set <device> fwUpdate /pfad/zur/firmware.hex
```

Der Fortschritt steht im Reading `fwUpdateState` (`writing 42%` → `verifying 80%` → `done`
bzw. `error: …`).

## Voraussetzungen am Gerät

* **HBW-Booter** ist geflasht (einmalig per ISP) und die Fuses stimmen — siehe Haupt-[README](../README.md).
* Die **laufende App** muss den `u`-Handler haben, also mit einer HBWired-Version gebaut sein, die
  auf `START_BOOTER` mit `sendAck()` + `wdt_enable()` reagiert.
  ⚠️ Eine App **ohne** diesen Handler ist eine Einbahnstraße: Sie lässt sich per Bus flashen, danach
  kommt man aber nicht mehr in den Booter (Rettung dann nur per ISP).
* Ab **Booter v0.03** muss die App zusätzlich den RAM-Marker aus [`bootmagic.h`](../bootmagic.h)
  setzen — sonst springt der Booter nach dem `u` sofort in die App zurück.
* Das Image muss **komplett unterhalb der Boot-Section** liegen (328P/32A `0x7000`, 644P `0xF000`,
  1284P `0x1F000`). Der Booter lehnt Blöcke darüber ohnehin ab (Selbstschutz).

## Bekanntes Verhalten

* **Fehlt das ACK auf ein Frame, wiederholt das Gateway es selbst** (gleiches Control-Byte, nach
  ~200 ms) — meist bevor das Modul-Timeout (1,5 s) greift. „Kein Retry im Log" ist deshalb **kein**
  Beleg dafür, dass das ACK wirklich kam; im Zweifel am Bus mitschneiden.
* Das Modul-Timeout ist bewusst **größer als das feste 1-s-Fenster** von `00_HM485_LAN.pm`, damit
  sich Wiederholungen nicht stapeln.
* Ein Gerät im Booter-Modus beantwortet **keine Discovery** — die Zieladresse muss bekannt sein.
* Am Bus lässt sich Booter vs. App an der Announce unterscheiden: **devType `0x00` = Booter**.

## eQ-3-Originalgeräte

Das Wire-Protokoll ist dasselbe, und die Firmware der HMW-Module liegt als **normales Intel-HEX**
vor (verschlüsselt sind nur die `.eq3`-Container für Gateway/Coprozessor). Es fehlt nur ein Detail:

**hs485d spricht einen Bootloader mit einem anderen Control-Byte an als eine laufende App:**

| | Control | Senderadresse |
|---|---|---|
| App (`CTRL_IFRAME`) | `0x18` | ja |
| Bootloader (`CTRL_BOOT_IFRAME`) | `0x10` | **nein** |

Der eQ-3-Bootloader akzeptiert **nur `0x10`** und verwirft alles mit Senderadresse kommentarlos.
Das typische Fehlerbild: `u` **und** `p` werden quittiert, aber es kommt nie eine Blockgröße —
denn diese ACKs stammen noch von der laufenden **App**, die jedes adressierte Frame generisch
bestätigt. Ein ACK ist also **kein** Beleg dafür, dass der Bootloader läuft.

Umschalten (im FHEM-Eingabefeld, ohne Modul-Änderung):

```perl
{ $main::HM485_fwu_bootCtrl = '10' }
```

Das betrifft nur die Bootloader-Phase (2. `u`, `p`, `w`, `r`, `g`); `z z`, das erste `u` an die App
und `Z Z` bleiben unverändert. Die Blockgröße (**128 B** beim ATmega32A statt 64) kommt automatisch
aus der `p`-Antwort.

⚠️ **Nicht mit älteren HBW-Bootern kombinieren:** Unser Booter versteht `0x10` erst **ab FW `0x0005`**.
Frühere Versionen leiten „hat Sender" auch aus Bit 4 ab und würden in einem senderlosen Frame vier
Sender-Bytes suchen. Deshalb ist `'18'` der Default.

Weitere Punkte für eQ-3-Geräte:

* Der eQ-3-Bootloader **fällt nach ~4–5 s ohne Kommando in die App zurück** — zügig arbeiten.
* Am Sniffer erkennbar: Bootloader-Antworten gehen an Ziel **`00000000`** (senderloses Frame → das
  Gerät weiß nicht, wer gefragt hat), App-ACKs an `00000001`/CCU.
* Nicht brickbar: Der Bootloader liegt ab `0x7800`, das Image endet bei `0x77FF`.

Getestet wurde der Weg bisher **außerhalb** von FHEM (direkt am Bus, HMW-LC-Bl1-DR: `p` → `00 80`,
`r` → 128 Byte bit-genau). Der Weg über dieses Modul steht noch aus.

## Danksagung

Dieses Modul gäbe es ohne **[loetmeister](https://github.com/loetmeister)** nicht. Er hat es an 
eigener Hardware getestet — Lauf für Lauf, mit FHEM-Logs
**und** parallelen Bus-Mitschnitten — bis der komplette Update durchlief. Praktisch jeder Fehler
wurde erst durch seine Mitschnitte auffindbar.

Am Booter selbst hat er ebenfalls maßgeblich mitgewirkt. Vielen Dank dafür.
