# Conduttimetro da campo
Questo è un conduttimetro da campo per l'analisi di acque superficiali e ipogee, sviluppato come Open Source dal Gruppo Speleologico Talpe del Carso.

## Hardware
Questo è l'attuale prototipo:

<img src="https://github.com/zorbaproject/Meatloaf/raw/master/conduttimetro-da-campo/menu.gif" width="600">

Lo schema hardware è:

<img src="https://github.com/zorbaproject/Meatloaf/raw/master/conduttimetro-da-campo/manuale/foto_conduttimetro/conduttimetro_da_campo_bb.png" width="600">

Sono necessari i seguenti componenti (prezzi medi su Aliexpress):
* Arduino MEGA R3 Euro 8
* LCD Keypad shield Euro 3 OPPURE LCD 1602 I2C https://it.aliexpress.com/item/32685612494.html
* Keypad 4*5 https://it.aliexpress.com/item/4000145872100.html Euro 2
* Data Logger Shield Euro 2,5
* DFRobot Gravity Electric Conductivity Kit K=1 (https://it.farnell.com/dfrobot/dfr0300/analog-electrical-conductivity/dp/2946108) (Versione precedente: Euro 34)  Euro 74
* Sensore di temperatura DS18B20 Euro 1,3
* Resistenza 10KOhm (Euro 1 pacco da 10 pezzi)
* Cavi Dupont
* Scatola derivazione IP55 125x125x75 (https://it.aliexpress.com/item/32985090338.html, 8 euro) oppure 155x115x60

L'assemblaggio costa quindi circa 90 euro (poco meno se si usa Arduino UNO) per i circuiti e la sonda, vanno poi aggiunti circa altri 20 euro per i cavi e il case.

Il datalogging shield richiede una pila CR1220 a 3V. Non è chiaro quanto duri, ma sicuramente alcuni anni.
Da notare che l'orologio viene reimpostato automaticamente aggiornando il firmware.

Affinché lo shield LCD possa funzionare con quello per il datalogging è necessario piegare o rimuovere i pin 10,11,12,13 dello shield LCD.
Nota: è possibile usare uno schermo LCD 1602 con I2C (https://win.adrirobot.it/rtc_module/immagini/test_rtc-DS1302_bb.jpg)

Nota: il circuito è in grado di funzionare con qualsiasi cella di conducibilità dotata di attacco BNC. Non è necessario modificare il circuito. (Fonte: https://www.dfrobot.com/product-1797.html#comment-4508952573)
Le celle più economiche sono prodotte da DJS:

<img src="http://web.archive.org/web/20200406124240im_/http://www.sinotester.com/Uploads/image/20180115/20180115084843_34419.jpg" width="600">

Va però considerato che queste celle sono progettate per un utilizzo di laboratorio, è sconsigliabile tenerle immerse più di 3 ore in acqua altrimenti perdono affidabilità nel corso del tempo. Per monitoraggi in continuo che devono durare mesi, conviene puntare su celle per utilizzo industriale.

## Requirements
Per poter compilare il codice è necessario avere installate queste librerie nel proprio Arduino IDE:
* EEPROM by Arduino
* Wire by Arduino  
* SD by Arduino 
* SPI by Arduino
* DFRobot_EC-master on https://github.com/DFRobot/DFRobot_EC
* DallasTemperature by Miles Burton et al
* OneWire by Jim Studt et al
* LiquidCrystal by Arduino, Adafruit
* RTClib by Adafruit


## La schermata principale
La schermata principale contiene questi dati:
```
T = 19.44^CnoLOG
EC = 463   uS/cm
```
Nell'angolo in alto a destra appare **noLOG** se il datalogging su scheda SD non è attivo, mentre appare **LOG** se il logging è attivo.

## Il menù
Attenzione: la lettura dei dati e il logging sono attivi solo mentre è visualizzata la schermata principale. Mentre si visualizza il menù, il datalogging è sospeso.
Per accedere al menù basta tenere premuto per 1 secondo il pulsante **Select**. Per scorrere le varie opzioni del menù si premono i tasti **Su** e **Giù**.
Esistono le seguenti opzioni:

* **Scrivi su SD**: mostra lo stato del datalogging, cioè la memorizzazione dei dati su scheda SD. Se appare un asterisco, il data logging è abilitato, altrimenti no. Per modificare l'impostazione (da disabilitato a abilitato e viceversa) basta premere il tasto **Select**.
* **Media su numero s**: mostra il numero di secondi entro i quali viene eseguita la media. Per ridurre l'effetto della variabilità casuale, soprattutto in acque con flusso rapido, il processore esegue una media sui dati degli ultimi 10 secondi. Per cambiare il numero di secondi considerati basta premere le frecce **Destra** e **Sinistra**.
* **HH:MM:SS DDMMYY**: mostra ora e data con due cifre per l'ora, due per i minuti, e due per i secondi, poi due per il giorno, due per il mese, e due per l'anno. Premendo il pulsante Select si può reimpostare data e ora, ma la funzione non è ancora implementata al momento.
* **Calibrazione**: premendo **Select** il dispositivo entra in modalità seriale (USB) e attende il comando per la calibrazione. È necessario che il dispositivo sia connesso a un computer tramite porta USB.
* **Reset memoria**: premendo **Select** si cancella la memoria EEPROM, eliminando i parametri di calibrazione del conduttimetro.
* **Esci**: premendo **Select** si esce dal menù e si ritorna alla schermata principale.

## Datalogging
Il datalogging avviene memorizzando su una scheda SD, una volta al secondo, i parametri rilevati. Questo è il tipo di output che si ottiene, nel file [DATALOG.TXT](./sample-DATALOG.txt):
```
0,2020-04-04T18:18:14,19.4375000000,398.2626600000
1,2020-04-04T18:18:15,19.3750000000,398.3140900000
2,2020-04-04T18:18:16,19.4375000000,394.9952100000
3,2020-04-04T18:18:17,19.3750000000,395.0466300000
```
I dati possono facilmente essere importati in LibreOffice Calc, indicando come lingua di origine l'inglese USA (i numeri sono scritti con il punto invece della virgola) e con la virgola come separatore dei campi. Si tratta infatti di una semplice tabella CSV.

<img src="https://github.com/zorbaproject/Meatloaf/raw/master/conduttimetro-da-campo/plot.png" width="600">

La prima colonna è un numero sequenziale, che inizia sempre da 0 ogni volta che si attiva il datalogging. In questo modo si possono facilmente riconoscere i vari campionamenti. Se si fa un giro di campionamenti in diversi punti, e si registra più volte la conducibilità, si possono separare i vari dati guardando gli zeri.

La seconda colonna è data e ora nel formato timestamp, come misurata dall'orologio RTC del dispositivo. Naturalmente, è importante tenere conto che se l'orologio è impostato con un'ora sbagliata, anche nel datalog si otterrà l'orario sbagliato.

Il terzo dato è la temperatura in gradi Celsius. Si considerano 3 cifre significative.

Il quarto dato è la condubilità in microSiemens per centimetro. Si considerano un massimo di 4 cifre significative, la precisione dipende dalla costante della cella utilizzata (K=0.1, K=1, oppure K=10).

Nel grafico si può notare che, utilizzando una cella non industriale con costante K=1 si ha una precisione di 10uS/cm. Utilizzando celle differenti si può ottenere una precisione maggiore (per conducibilità basse è consigliabile K=0.1). Il conduttimetro è comunque sensibile, basta inserire una perturbazione e viene registrata nel giro di pochi secondi.

## Calibrazione
Per la procedura di calibrazione sono necessarie due soluzioni standard: una da 12.88ms/cm e una da 1413us/cm.

La prima cosa da fare è collegare il dispositivo a un computer tramite la sua porta USB. È necessario un cavo USB tipo B. Il dispositivo si accenderà.

Poi si deve aprire un monitor seriale. Si può utilizzare l'Arduino IDE, oppure PuTTY.

A questo punto si preme il tasto **Select** per entrare nel menù e si scorre in basso fino a raggiungere la **Calibrazione**. Si preme **Select** per attivare la calibrazione. A questo punto compariranno dei dati sul monitor seriale.

Per iniziare la calibrazione, si deve dare dal monitor seriale il comando
```
enter
```
Si deve attendere che la lettura si stabilizzi intorno a un valore. A quel punto può essere eseguito il calcolo con il comando
```
cal
```
Bisogna poi seguire le istruzioni che appaiono sul terminale. Per salvare i dati sarà necessario dare il comando *exit*.

Terminata la calibrazione, il dispositivo può essere riavviato premendo il pulsante **Reset** oppure semplicemente scollegando il cavo USB e accendendolo tramite batterie.

## Known issues
Capita che la pulsantiera smetta di funzionare, e non sia possibile accedere al menù. Basta smontare lo shield LCD separandolo da quello di datalogging. Dopo qualche minuto si può ricomporre e dovrebbe funzionare.

## Credits
* Progettazione, programmazione e assemblaggio: Luca Tringali, G.S. Talpe del Carso - J.K. Kraski Krti
* Documentazione: Luca Tringali, G.S. Talpe del Carso - J.K.K.K.
* Finanziamento: contributi volontari dei soci G.S. Talpe del Carso - Jamarski Klub Kraski Krti, Regione Friuli Venezia Giulia (L.R. 15/2016)
