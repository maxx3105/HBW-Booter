# ============================================================================
#  HM485_fwUpdate.pl  --  Over-the-Bus-Firmware-Update fuer HBWired-Geraete,
#  direkt aus FHEM heraus, "wie die CCU" (hs485d-Ablauf auf CMD_SEND-Ebene).
#
#  KOMPLETT:  Intel-HEX-Parser + Start + lockBus (z z) + u(App)/Reset/u(Booter) + p +
#             WRITE-Schleife (w) + VERIFY (r) + START_FW (g) + unlockBus (Z Z).
#
#  Transport-agnostisch: alle Frames gehen als CMD_SEND ('S') raus -> Gateway,
#  echtes eQ-3-LGW ODER HM485d reichen sie transparent auf den Bus. Kein Sonder-
#  kommando im Transport noetig (anders als 'discovery', das der Transport selbst faehrt).
#
#  Referenz-Ablauf (dreifach HW-verifiziert): flash_tool.py (HBW-Booter),
#  Homegear updateFirmware, Gateway busFlashRun. Zeilenverweise = flash_tool.py.
#
#  >>> INTEGRATION in 10_HM485.pm -- 2 Stellen (siehe Block ganz unten) <<<
# ============================================================================
package main;
use strict;
use warnings;

# --- State-Machine-Zustaende ---
use constant {
    FWU_LOCK      => 1,   # z z  (Broadcast, ohne ACK)
    FWU_APP       => 2,   # u an die laufende App -> ACK (App springt in Booter)
    FWU_RESETWAIT => 3,   # ~1s Reset-Pause (WDT-Reset der App)
    FWU_BOOT      => 4,   # u an den Booter    -> ACK (Booter bestaetigt)
    FWU_PSIZE     => 5,   # p -> ACK-Frame mit [00 Blockgroesse]
    FWU_WRITE     => 6,   # w-Schleife         (Teil 2)
    FWU_VERIFY    => 7,   # r-Schleife         (Teil 2)
    FWU_GO        => 8,   # g                  (Teil 2)
    FWU_UNLOCK    => 9,   # Z Z                (Teil 2)
};

use constant {
    # 1.5s: MUSS groesser sein als das Warte-Fenster des Transports. 00_HM485_LAN.pm
    # (SendQueueNextItem) sendet einmal, wartet fest 1 s und meldet dann per CMD_ALIVE '01'
    # einen NACK. Mit den urspruenglichen 0.5 s habe ich nachgelegt, BEVOR der Transport
    # ueberhaupt fertig gewartet hatte -> Frames stapelten sich.
    FWU_RESP_TIMEOUT => 1.5,
    # Nur 1 eigener Zusatzversuch. Das Gateway/LGW wiederholt jeden Bus-Frame bereits selbst
    # (im Mitschnitt 3x je Sendung); zusammen mit 3 eigenen Versuchen ergab das 9 identische
    # Frames auf dem Bus (von loetmeister gemessen). Bei echtem Ausfall greift ohnehin der NACK.
    FWU_MAX_RETRY    => 1,
    FWU_RESET_PAUSE  => 1.2,    # Reset-Pause zwischen u(App) und u(Booter); flash_tool: sleep(1.5)
};

# Control-Byte fuer die BOOTER-Kommandos (alles ab dem 2. 'u': u/p/w/r/g).
#   '18' = CTRL_IFRAME     -- mit Senderadresse (wie an eine laufende App)
#   '10' = CTRL_BOOT_IFRAME -- OHNE Senderadresse; so spricht hs485d/die CCU jeden Bootloader an
# Der **eq3-Original-Booter akzeptiert NUR 0x10** und verwirft alles mit Sender kommentarlos
# (Symptom: ACK auf 'u' und 'p', aber nie eine Blockgroesse -- das ACK kommt dann noch von der
# laufenden App, die jedes adressierte Frame generisch quittiert).
# Unser HBW-Booter versteht 0x10 erst **ab FW 0x0005** (davor: hasSender aus Bit 4 mitabgeleitet,
# er wuerde in einem senderlosen Frame vier Sender-Bytes suchen und es verwerfen).
# Default '18' = kompatibel zu allen bisher ausgelieferten HBW-Bootern.
# Umschalten zur Laufzeit ohne Modul-Aenderung, im FHEM-Eingabefeld:
#   { $main::HM485_fwu_bootCtrl = '10' }
our $HM485_fwu_bootCtrl = '18';

# ----------------------------------------------------------------------------
#  Intel-HEX -> zusammenhaengendes Byte-Array [0..maxAddr], Luecken = 0xFF.
#  1:1-Port von flash_tool.py::parse_hex (Rec-Typen 00=data, 04=ext-lin, 02=ext-seg).
# ----------------------------------------------------------------------------
sub HM485_fwu_parseHex {
    my ($path) = @_;
    open(my $fh, '<', $path) or return (undef, undef, "cannot open '$path': $!");
    my (%mem, $ext);
    $ext = 0;
    while (my $line = <$fh>) {
        $line =~ s/[\r\n\s]+$//;
        next unless substr($line, 0, 1) eq ':';
        my $ll = hex(substr($line, 1, 2));         # Byte-Anzahl
        my $a  = hex(substr($line, 3, 4));         # Offset (16 bit)
        my $tt = hex(substr($line, 7, 2));         # Record-Typ
        my $d  = substr($line, 9, $ll * 2);        # Datenteil (hex)
        if    ($tt == 0) { $mem{$ext + $a + $_} = hex(substr($d, $_ * 2, 2)) for (0 .. $ll - 1); }
        elsif ($tt == 4) { $ext = hex($d) << 16; } # Extended Linear Address
        elsif ($tt == 2) { $ext = hex($d) << 4;  } # Extended Segment Address
        # tt==01 (EOF) / 03/05 (Start) ignorieren
    }
    close($fh);
    return (undef, undef, "no data records in '$path'") unless %mem;
    my $maxA = (sort { $b <=> $a } keys %mem)[0];
    # WICHTIG: Fuer eigene HBW-Geraete MUSS das komplette Image < Boot-Section liegen
    # (328P/32A: 0x7000, 644P: 0xF000, 1284P: 0x1F000). Pruefung ist geraeteabhaengig ->
    # hier nur Info; der Booter lehnt Bytes >= BOOT_START ohnehin ab (Selbstschutz).
    my @img = map { defined($mem{$_}) ? $mem{$_} : 0xFF } (0 .. $maxA);
    return (\@img, $maxA, undef);
}

# ----------------------------------------------------------------------------
#  set <dev> fwUpdate <hexfile>   -> Einstieg
# ----------------------------------------------------------------------------
sub HM485_fwu_Start {
    my ($hash, $path) = @_;
    return 'fwUpdate already running for ' . $hash->{NAME} if $hash->{fwu};
    return 'usage: set <dev> fwUpdate <path-to.hex>'       unless $path;

    my ($img, $maxA, $err) = HM485_fwu_parseHex($path);
    return "fwUpdate: $err" if $err;

    $hash->{fwu} = {
        state => FWU_LOCK,
        img   => $img,        # Byte-Array [0..maxA]
        maxA  => $maxA,
        bs    => 64,          # Blockgroesse; aus der p-Antwort ueberschrieben
        pos   => 0,           # Schreib-/Verify-Cursor
        tries => 0,
    };
    readingsSingleUpdate($hash, 'fwUpdateState', 'start', 1);
    HM485::Util::Log3($hash, 2, sprintf(
        'fwUpdate: %s -- %d Bytes, 0x0000..0x%04X', $path, $maxA + 1, $maxA));
    HM485_fwu_Step($hash);
    return undef;
}

# ----------------------------------------------------------------------------
#  Sendet den Frame des AKTUELLEN States. Zuordnung der Antwort laeuft ueber den
#  State (Machine ist streng sequentiell -> immer nur EIN Frame offen), nicht ueber
#  requestId -- deshalb reicht IOWrite ohne die HM485_SendCommand-Queue.
# ----------------------------------------------------------------------------
sub HM485_fwu_Step {
    my ($hash) = @_;
    my $fu = $hash->{fwu} or return;
    my $dev = substr($hash->{DEF}, 0, 8);         # 8-hex Zieladresse (DEF kann Kanal-Suffix tragen)

    my $st = $fu->{state};
    if ($st == FWU_LOCK) {
        # START_ZERO_COMMUNICATION 2x als Broadcast, ohne ACK.
        # flash_tool.py:113-114  build(0xFFFFFFFF, .., [0x7A]) x2
        HM485_fwu_sendBroadcast($hash, '7A');
        HM485_fwu_sendBroadcast($hash, '7A');
        $fu->{state} = FWU_APP;
        InternalTimer(gettimeofday() + 0.1, 'HM485_fwu_Step', $hash);   # kurz Luft, dann u
    }
    elsif ($st == FWU_APP) {
        # u an die laufende App -> App bestaetigt (ACK) und macht WDT-Reset.
        # flash_tool.py:115  build(DEV, 0x18, CENTRAL, [0x75])
        HM485_fwu_sendAcked($hash, $dev, '75');
    }
    elsif ($st == FWU_RESETWAIT) {
        # ~1s Reset-Pause (App startet den Booter neu). flash_tool.py:116 sleep(1.5)
        $fu->{state} = FWU_BOOT;
        InternalTimer(gettimeofday() + FWU_RESET_PAUSE, 'HM485_fwu_Step', $hash);
    }
    elsif ($st == FWU_BOOT) {
        # 2. u -> jetzt antwortet der BOOTER (ACK + StartupReason). "wie hs485d" (u zweimal).
        # (flash_tool.py verzichtet auf das 2. u und lauscht nur passiv -- die CCU/hs485d
        #  schickt es aber, und unser Booter beantwortet es: CMD_START_BOOTER-Case.)
        HM485_fwu_sendAcked($hash, $dev, '75', $HM485_fwu_bootCtrl);
    }
    elsif ($st == FWU_PSIZE) {
        # p -> Booter meldet Blockgroesse als ACK-Frame [00 size]. flash_tool.py:123 [0x70]
        HM485_fwu_sendAcked($hash, $dev, '70', $HM485_fwu_bootCtrl);
    }
    elsif ($st == FWU_WRITE) {
        # w-Schleife: bs-Byte-Bloecke. Page 0 (base < 128) ZULETZT -- so bleibt der
        # Reset-Vektor bis zum Schluss ungeschrieben; ein Abbruch laesst die App garantiert
        # ungueltig (Booter sieht 0xFFFF-Reset-Vektor, bleibt drin). flash_tool.py:133-135.
        my $bs = $fu->{bs};
        unless ($fu->{blocks}) {                       # Reihenfolge einmalig festlegen
            my (@hi, @lo);
            for (my $b = 0; $b <= $fu->{maxA}; $b += $bs) {
                if ($b >= 128) { push @hi, $b } else { push @lo, $b }
            }
            $fu->{blocks} = [ @hi, @lo ];
            $fu->{blkIdx} = 0;
        }
        if ($fu->{blkIdx} >= scalar @{ $fu->{blocks} }) {   # alle Bloecke geschrieben -> Verify
            $fu->{state} = FWU_VERIFY;
            $fu->{pos}   = 0;
            readingsSingleUpdate($hash, 'fwUpdateState', 'verifying 0%', 1);
            HM485_fwu_Step($hash);
            return;
        }
        my $base = $fu->{blocks}[ $fu->{blkIdx} ];
        my $n    = ($base + $bs <= $fu->{maxA} + 1) ? $bs : ($fu->{maxA} + 1 - $base);
        # w:  77 baseHi baseLo n <n Datenbytes>
        my $pl = sprintf('77%02X%02X%02X', ($base >> 8) & 0xFF, $base & 0xFF, $n)
               . join('', map { sprintf('%02X', $fu->{img}[$base + $_]) } (0 .. $n - 1));
        HM485_fwu_sendAcked($hash, $dev, $pl, $HM485_fwu_bootCtrl);
    }
    elsif ($st == FWU_VERIFY) {
        # r-Schleife: Flash zuruecklesen + gegen das Image pruefen (normale Reihenfolge).
        # Der erste 'r' committet im Booter die letzte offene w-Page (flushPage). flash_tool.py:147-151.
        my $bs = $fu->{bs};
        if ($fu->{pos} > $fu->{maxA}) {                # alles verglichen -> App starten
            $fu->{state} = FWU_GO;
            HM485_fwu_Step($hash);
            return;
        }
        my $base = $fu->{pos};
        my $n    = ($base + $bs <= $fu->{maxA} + 1) ? $bs : ($fu->{maxA} + 1 - $base);
        # r:  72 baseHi baseLo n   -> Booter antwortet mit GENAU n Flash-Bytes (kein Echo)
        HM485_fwu_sendAcked($hash, $dev,
            sprintf('72%02X%02X%02X', ($base >> 8) & 0xFF, $base & 0xFF, $n),
            $HM485_fwu_bootCtrl);
    }
    elsif ($st == FWU_GO) {
        # g:  67 lenHi lenLo crcHi crcLo  -- Booter prueft appCrc ueber die ganze App und
        # startet sie nur bei Match; er ACKt g VOR dem Sprung (hbw_booter.c CMD_START_FW).
        # flash_tool.py:166-169.
        my $len = $fu->{maxA} + 1;
        my $crc = HM485_fwu_appcrc($fu->{img});
        HM485_fwu_sendAcked($hash, $dev, sprintf('67%02X%02X%02X%02X',
            ($len >> 8) & 0xFF, $len & 0xFF, ($crc >> 8) & 0xFF, $crc & 0xFF),
            $HM485_fwu_bootCtrl);
    }
    elsif ($st == FWU_UNLOCK) {
        # Z Z -- Zero-Communication aufheben (Broadcast, ohne ACK). flash_tool.py:179-181.
        HM485_fwu_sendBroadcast($hash, '5A');
        HM485_fwu_sendBroadcast($hash, '5A');
        readingsSingleUpdate($hash, 'fwUpdateState', 'done', 1);
        HM485::Util::Log3($hash, 2, "fwUpdate DONE ($hash->{NAME})");
        delete $hash->{fwu};
    }
}

# ----------------------------------------------------------------------------
#  Antwort kam (aus dem Hook in HM485_ProcessResponse). $respData = Payload-Hex.
# ----------------------------------------------------------------------------
sub HM485_fwu_OnResp {
    my ($hash, $respData, $msgCmd, $msgId) = @_;
    my $fu = $hash->{fwu} or return;
    $respData = '' unless defined $respData;
    my $target = uc(substr($hash->{DEF}, 0, 8));

    # NACK vom Transport: CMD_ALIVE (0x61) mit '01'<Adresse> = "Geraet hat nicht geantwortet"
    # (HM485_Parse -> HM485_SetStateNack). 00_HM485_LAN.pm hat den Frame da bereits 3x
    # wiederholt -> weiteres Warten ist sinnlos. Sofort abbrechen MIT klarer Ursache, statt
    # in Timeouts zu laufen und spaeter als "verify mismatch" zu enden.
    # WICHTIG: Die Adresse pruefen! Der Hook leitet JEDEN Frame hierher, auch NACKs, die einem
    # ANDEREN Geraet gelten. Waehrend des Updates ist das der Normalfall: 'z z' legt den ganzen
    # Bus stumm, andere FHEM-Automatiken (DOIF o.ae.) senden aber weiter und kassieren NACKs.
    # Ohne diese Pruefung brach das Update daran ab (von loetmeister am echten Bus beobachtet).
    if (defined($msgCmd) && $msgCmd == 0x61 && substr($respData, 0, 2) eq '01') {
        return if uc(substr($respData, 2, 8)) ne $target;   # NACK galt einem anderen Geraet
        HM485_fwu_Fail($hash, 'keine Antwort vom Geraet (NACK) bei state ' . $fu->{state}
            . ' -- Geraet im Booter? App mit u-Handler? Bus/Adresse ok?');
        return;
    }

    # Antwort-Zuordnung ueber die msgId: HM485_LAN_Write liefert sie beim Senden zurueck, die
    # Antwort traegt dieselbe. Damit koennen Antworten FREMDER Geraete (deren ACK ebenfalls
    # control 0x19 hat und sonst als unsere gelten wuerde) die State-Machine nicht verschieben.
    # Nur aktiv, wenn der Hook $msgId mitgibt -- sonst wie bisher (rein zustandsbasiert).
    return if (defined($msgId) && defined($fu->{reqId}) && $msgId != $fu->{reqId});
    # $respData = <Bus-Control-Byte><Nutzdaten>. NUR echte Bus-ACK-Frames gehoeren zur Anfrage:
    # deren Control ist 0x19|(seq<<5) -> untere 5 Bit == 0x19. Damit fallen BEIDE Stoerer raus,
    # die FHEM sonst dazwischenliefert und die die State-Zuordnung verschieben:
    #  - spontane Broadcasts StartupReason/Announce (control 0xF8, & 0x1F = 0x18)
    #  - Gateway-Keepalive/NACK (CMD_ALIVE, msgData '01<addr>' -> control 0x01). Ein reiner
    #    "& 0x07 == 1"-Test liesse diese '01'-Frames faelschlich als ACK durch (Log: got 42000077)!
    my $ctrl = hex(substr($respData, 0, 2) || '00');
    return unless ($ctrl & 0x1F) == 0x19;
    my $pl = substr($respData, 2);              # Nutzdaten nach dem Control-Byte
    RemoveInternalTimer($hash, 'HM485_fwu_Timeout');
    $fu->{tries} = 0;

    my $st = $fu->{state};
    if    ($st == FWU_APP)   { $fu->{state} = FWU_RESETWAIT; HM485_fwu_Step($hash); }  # App-ACK -> Reset-Pause
    elsif ($st == FWU_BOOT)  { $fu->{state} = FWU_PSIZE;     HM485_fwu_Step($hash); }  # Booter-ACK -> p
    elsif ($st == FWU_PSIZE) {                                                          # p-Antwort [00 size]
        $fu->{bs}  = hex(substr($pl, 2, 2)) if length($pl) >= 4;   # 2. Payload-Byte = Blockgroesse
        $fu->{bs}  = 64 unless $fu->{bs} && $fu->{bs} <= 128;
        $fu->{pos} = 0;
        $fu->{state} = FWU_WRITE;
        readingsSingleUpdate($hash, 'fwUpdateState', 'writing 0%', 1);
        HM485_fwu_Step($hash);   # -> w-Schleife
    }
    elsif ($st == FWU_WRITE) {                   # w-Block bestaetigt ([00 n])
        $fu->{blkIdx}++;
        my $tot = scalar @{ $fu->{blocks} };
        readingsSingleUpdate($hash, 'fwUpdateState',
            'writing ' . int(100 * $fu->{blkIdx} / $tot) . '%', 1)
            if ($fu->{blkIdx} % 16 == 0 || $fu->{blkIdx} == $tot);
        HM485_fwu_Step($hash);                   # naechster Block oder -> Verify
    }
    elsif ($st == FWU_VERIFY) {                  # r-Antwort = n Flash-Bytes -> vergleichen
        my $bs   = $fu->{bs};
        my $base = $fu->{pos};
        my $n    = ($base + $bs <= $fu->{maxA} + 1) ? $bs : ($fu->{maxA} + 1 - $base);
        my $exp  = join('', map { sprintf('%02X', $fu->{img}[$base + $_]) } (0 .. $n - 1));
        my $got  = uc(substr($pl, 0, $n * 2));      # Flash-Bytes ab Payload-Start (nach Control-Byte)
        if ($got ne $exp) {
            HM485_fwu_Fail($hash, sprintf('verify mismatch @0x%04X (exp %s.. got %s..)',
                $base, substr($exp, 0, 16), (length($got) ? substr($got, 0, 16) : '--')));
            return;
        }
        $fu->{pos} += $bs;
        my $vp = int(100 * $fu->{pos} / ($fu->{maxA} + 1));  $vp = 100 if $vp > 100;
        readingsSingleUpdate($hash, 'fwUpdateState', "verifying $vp%", 1)
            if (int($fu->{pos} / $bs) % 16 == 0 || $fu->{pos} > $fu->{maxA});
        HM485_fwu_Step($hash);                   # naechster Verify-Block oder -> g
    }
    elsif ($st == FWU_GO) {                       # g bestaetigt -> Bus freigeben
        $fu->{state} = FWU_UNLOCK;
        HM485_fwu_Step($hash);
    }
}

# ----------------------------------------------------------------------------
#  Kein ACK/Response in FWU_RESP_TIMEOUT -> Frame wiederholen, nach FWU_MAX_RETRY abbrechen.
# ----------------------------------------------------------------------------
sub HM485_fwu_Timeout {
    my ($hash) = @_;
    my $fu = $hash->{fwu} or return;
    if (++$fu->{tries} <= FWU_MAX_RETRY) {
        HM485::Util::Log3($hash, 3, 'fwUpdate: retry ' . $fu->{tries} . ' at state ' . $fu->{state});
        HM485_fwu_Step($hash);   # gleichen State erneut senden
    } else {
        HM485_fwu_Fail($hash, 'timeout at state ' . $fu->{state});
    }
}

sub HM485_fwu_Fail {
    my ($hash, $msg) = @_;
    RemoveInternalTimer($hash, 'HM485_fwu_Timeout');
    RemoveInternalTimer($hash, 'HM485_fwu_Step');
    # Bus IMMER wieder freigeben, auch im Fehlerfall (sonst bleiben andere Geraete stumm):
    HM485_fwu_sendBroadcast($hash, '5A');   # Z
    HM485_fwu_sendBroadcast($hash, '5A');   # Z
    readingsSingleUpdate($hash, 'fwUpdateState', "error: $msg", 1);
    HM485::Util::Log3($hash, 1, "fwUpdate FAILED ($hash->{NAME}): $msg");
    delete $hash->{fwu};
}

# ----------------------------------------------------------------------------
#  Sende-Helfer (auf CMD_SEND -- transport-agnostisch).
# ----------------------------------------------------------------------------
# Unicast + auf ACK/Response warten: IOWrite liefert die requestId; Response-Zuordnung
# aber ueber den State (s.o.). $payloadHex = Bus-Nutzdaten ab Kommando-Byte, z.B. '75'.
sub HM485_fwu_sendAcked {
    my ($hash, $target, $payloadHex, $ctrl) = @_;
    # $ctrl optional: Booter-Kommandos brauchen ggf. CTRL_BOOT_IFRAME (siehe $HM485_fwu_bootCtrl).
    my %p = (target => $target, data => $payloadHex);
    $p{ctrl} = $ctrl if defined $ctrl;
    # IOWrite MUSS das DEVICE-hash bekommen (FHEM findet ->{IODev} selbst) -- NICHT das
    # IO-hash: exakt wie HM485_DoSendCommand (IOWrite($hash, HM485::CMD_SEND, {target,data})).
    # Mit dem IO-hash sucht IOWrite dessen ->{IODev} (gibt es nicht) -> Frame wird verworfen.
    # Rueckgabe = msgId dieses Frames; die Antwort traegt dieselbe -> Zuordnung in OnResp.
    # Erst in eine Variable, dann zuweisen: ein direktes $hash->{fwu}{reqId} = IOWrite(...)
    # wuerde {fwu} per Autovivification NEU anlegen, falls der Lauf zwischendrin beendet wurde.
    my $rid = IOWrite($hash, HM485::CMD_SEND, \%p);
    return unless $hash->{fwu};
    $hash->{fwu}{reqId} = $rid;
    RemoveInternalTimer($hash, 'HM485_fwu_Timeout');
    InternalTimer(gettimeofday() + FWU_RESP_TIMEOUT, 'HM485_fwu_Timeout', $hash);
}

# Broadcast (z/Z): Zieladresse FFFFFFFF, KEINE ACK-Erwartung, kein Timer.
sub HM485_fwu_sendBroadcast {
    my ($hash, $payloadHex) = @_;
    IOWrite($hash, HM485::CMD_SEND, { target => 'FFFFFFFF', data => $payloadHex });
}

# ----------------------------------------------------------------------------
#  CRC16 (Poly 0x1002, MSB-first, Startwert 0xFFFF) ueber die App-Bytes. MUSS bit-genau
#  appCrc() im Booter (und flash_tool.py::appcrc) entsprechen -- der Booter startet die App
#  im 'g' nur bei Match. KEINE zwei Null-Anhaenge-Bytes (anders als die Frame-CRC crc16).
# ----------------------------------------------------------------------------
sub HM485_fwu_appcrc {
    my ($img) = @_;
    my $crc = 0xFFFF;
    foreach my $byte (@$img) {
        my $b = $byte;
        for (1 .. 8) {
            my $hi = $crc & 0x8000;
            $crc = ($crc << 1) & 0xFFFF;
            $crc |= 1 if ($b & 0x80);
            $crc ^= 0x1002 if $hi;
            $b = ($b << 1) & 0xFF;
        }
    }
    return $crc;
}

1;

# ============================================================================
#  HM485_fwUpdate.pm nach .../fhem/FHEM/lib/HM485 kopieren
#  >>> INTEGRATION in 10_HM485.pm  (kc-GitHub/FHEM-HM485) <<<
#
#  (0) weitere "use lib.." einbinden:
#  use lib::HM485::HM485_fwUpdate;
#
#  (1) set-Befehl:
#        %sets/setList:   nach "'getConfig' => 'noArg',":
#                         'fwUpdate' => 'textField',
#
#        in HM485_Set():  } elsif ($cmd eq 'fwUpdate') {
#                             return HM485_fwu_Start($hash, $value);
#              vor Zeile "}elsif ($cmd eq 'raw')... "
#      ($value = Dateipfad)
#
#  (2) Antwort-Hook -- in HM485_Parse($ioHash,$message), direkt NACH der Zeile
#         my $msgData = uc( unpack ('H*', substr($message, 4)));
#      und VOR  if ($msgCmd == HM485::CMD_RESPONSE)  einfuegen. Bewusst HIER (nicht in
#      HM485_ProcessResponse): faengt die Booter-Antwort ab, EGAL ob sie als CMD_RESPONSE
#      ODER CMD_EVENT reinkommt, und $msgData ist der VOLLE Payload:
#
#        foreach my $d (values %{$modules{HM485}{defptr}}) {
#            next unless $d->{fwu} && $d->{IODev} && $d->{IODev} == $ioHash;
#            HM485::Util::Log3($ioHash, 5, 'fwUpdate RX: msgCmd='.$msgCmd.' msgData='.$msgData);
#            HM485_fwu_OnResp($d, $msgData, $msgCmd, $msgId);   # $msgCmd = NACK-Erkennung,
#                                                               # $msgId  = Antwort-Zuordnung
#            return $ioHash->{NAME};
#        }
#
#  SENDE-SEITE ist gefixt: _sendAcked/_sendBroadcast rufen IOWrite($hash,...) mit dem
#  DEVICE-hash (wie HM485_DoSendCommand) statt dem IO-hash -> z/u/p/w/... gehen jetzt raus.
#  (Der Bug "timeout at state 2, nichts auf dem Bus" kam von IOWrite($ioHash,...).)
#
#  >>> DANACH NOCH AM LOG ZU JUSTIEREN (Payload-Offset, Punkt C) <<<
#  Der 'fwUpdate RX:'-Log zeigt beim ersten Durchlauf den echten $msgData-Aufbau. OnResp
#  liest aktuell: p-Antwort substr($resp,2,2)=Blockgroesse (erwartet '0040'); r-Antwort
#  substr($resp,0,n*2)=die n Flash-Bytes. Stehen im $msgData fuehrende Bytes (Sender-Adresse
#  o.ae.), die substr-Offsets in HM485_fwu_OnResp (FWU_PSIZE + FWU_VERIFY) entsprechend
#  verschieben. Sonst ist alles verifiziert (CRC bit-genau, Ablauf im Trockenlauf komplett).
# ============================================================================
