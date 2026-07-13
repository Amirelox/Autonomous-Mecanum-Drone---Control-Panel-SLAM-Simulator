# 📅 Plan Deweloperski: Rozbudowa Środowiska Symulacyjnego

Niniejszy dokument określa harmonogram i architekturę techniczną rozbudowy autonomicznego symulatora drona/łazika Mecanum. Celami głównymi są: eliminacja wiedzy *a priori* o wielkości świata, wprowadzenie zaawansowanego modelu dynamiki pojazdu elektrycznego (EV) oraz wdrożenie zaszumionej odometrii korygowanej przez pętlę SLAM.

---

## ✅ Wtorek (Zrealizowane): Ślepa Autonomia, Dynamiczne Bounding Box i Czysty BFS

**Cel:** Całkowite odcięcie algorytmu klienta od wiedzy „z góry”. Robot nie zna wymiarów labiryntu ($N, M$), nie dostaje współrzędnych mety z sieci i samodzielnie wylicza parametry środowiska po zakończeniu pierwszej próby. Zamiast potrójnego turnieju, ze względu na prostotę labiryntu, stosowany jest wyłącznie wydajny algorytm BFS.

### 1. Przebudowa Fazy DFS (Ślepe Mapowanie)
* **Ignorowanie komunikatów sukcesu:** W pliku `client.py` usuwany jest warunek przerywania eksploracji na podstawie flagi `is_at_meta` przesyłanej przez serwer.
* **Kryterium stopu:** Pierwsza faza (odkrywanie) kończy się tylko wtedy, gdy wewnętrzny stos `path_stack` jest pusty **oraz** robot fizycznie powróci na pozycję startową $(1, 1)$.
* **Wirtualne Bounding Box:** W konstruktorze klasy `DFSController` inicjalizowane są dwie zmienne: `max_logic_r = 1` oraz `max_logic_c = 1`. Przy każdym wejściu na nowy kafelek logiczny system aktualizuje je za pomocą funkcji `max()`, rejestrując najdalsze skrajne punkty pionowe i poziome osiągnięte przez pojazd.

### 2. Matematyczny Post-processing (Po Pierwszej Próbie)
W momencie powrotu na start i zatrzymania robota, uruchamia się jednorazowy skrypt analityczny:
* **Wyliczenie wymiarów labiryntu:** System wyznacza wysokość $N$ i szerokość $M$ na podstawie najwyższych odnalezionych indeksów logicznych:
  $$N = \frac{\text{max\_logic\_r} + 1}{2}$$
  $$M = \frac{\text{max\_logic\_c} + 1}{2}$$
* **Lokalizacja punktu STOP ($P_{\text{stop}}$):**
  * **Tryb Narożnik:** Z racji startu z pozycji $(1, 1)$, cel to z definicji przeciwległy narożnik oznaczający skrajne punkty mapy: 
    $$P_{\text{stop}} = (\text{max\_logic\_r}, \text{max\_logic\_c})$$
  * **Tryb Środek:** Cel zostaje wyliczony jako geometryczne centrum macierzy z użyciem dzielenia całkowitego:
    $$\text{Wiersz mety} = 2 \cdot \left( \left\lfloor \frac{N}{2} \right\rfloor \right) + 1$$
    $$\text{Kolumna mety} = 2 \cdot \left( \left\lfloor \frac{M}{2} \right\rfloor \right) + 1$$

### 3. Generowanie Ścieżki BFS i Maszyna Stanów
* Po zamrożeniu współrzędnych $P_{\text{stop}}$, robot uruchamia czysty algorytm **BFS (Breadth-First Search)**.
* Algorytm wyznacza najkrótszą trasę do celu i zapisuje ją w postaci tablicy `optimized_path`.
* Maszyna stanów (FSM) przełącza drona w tryb `STATE_SPEEDRUN`. W rundach 2–50 faza DFS jest pomijana – robot od pierwszej sekundy realizuje szybki bieg do zapamiętanego celu.

---

## 🏎️ Czwartek: Realistyczny Model Fizyczny EV i Ścinanie Zakrętów

**Cel:** Zastąpienie idealnego punktu matematycznego zaawansowanym modelem dynamiki pojazdu. Obliczenia pędu, masy oraz przyczepności opon zostają zaimplementowane bezpośrednio w silniku fizycznym serwera.

### 1. Implementacja Równań Dynamiki Newtona
* W pliku `main.py` wewnątrz pętli `physics_loop` wprowadzane są parametry: masa pojazdu ($m$) oraz siła oporów toczenia i powietrza ($F_{\text{oporu}}$).
* Komendy wejściowe z klienta (`cmd_vx`, `cmd_vy`) przestają bezpośrednio modyfikować pozycję. Reprezentują one siłę napędową generowaną przez silniki elektryczne ($F_{\text{silnik}}$).
* W każdej klatce symulacji obliczane jest przyspieszenie liniowe oraz nowa prędkość:
  $$a = \frac{F_{\text{silnik}} - F_{\text{oporu}}}{m}$$
  $$v = v + a \cdot dt$$
  Pojazd zyskuje naturalną bezwładność oraz drogę hamowania.

### 2. Limit Przyczepności Opon (Grip Limit i Poślizg)
* Wprowadzona zostaje kontrola siły odśrodkowej działającej na robota podczas gwałtownych manewrów skrętu:
  $$F_c = \frac{m \cdot v^2}{r}$$
* Jeśli siła $F_c$ przekroczy maksymalną siłę przyczepności statycznej opon ($F_{\text{max\_grip}}$), serwer aktywuje flagę poślizgu `is_skidding = True`. Pojazd zaczyna lecieć bezwładnie bokiem siłą pędu, ignorując rozkazy skrętu kół, dopóki tarcie poślizgowe go nie wyhamuje lub nie nastąpi kolizja ze ścianą.

### 3. Wyścigowe Ścinanie Zakrętów (Corner Cutting)
* W pliku `client.py` w trybie `STATE_SPEEDRUN` modyfikowana jest tolerancja osiągnięcia punktu kontrolnego.
* Zamiast oczekiwać na idealne wycentrowanie drona nad środkiem kafelka (margines $< 5$ pikseli), próg zostaje zwiększony do $15-20$ pikseli. W momencie wpadnięcia w tę strefę, układ sterowania natychmiast podaje współrzędne kolejnego celu, co pozwala na płynne, łukowe pokonywanie zakrętów bez gwałtownego hamowania.

---

## 📡 Piątek: Zaszumiona Odometria i Pętla Korekcji SLAM (ICP)

**Cel:** Przeniesienie symulacji w realia fizycznych zakłóceń sensorów. Wprowadzenie dryfu pozycji zmusza robota do ciągłego weryfikowania i prostowania trasy na podstawie wskazań laserów.

### 1. Generowanie Szumu i Dryfu Żyroskopowego
* W pliku `main.py` pozycja robota zostaje rozbita na dwie zmienne: `true_x, true_y` (idealne współrzędne serwera do sprawdzania kolizji) oraz `odometry_x, odometry_y` (wersja zanieczyszczona wysyłana do klienta).
* W pętli fizycznej do odometrii dodawany jest losowy szum Gaussa oraz stały błąd kumulacyjny imitujący uślizg rolek kół Mecanum. Im dłużej robot się porusza, tym bardziej jego wewnętrzne przekonanie o pozycji rozjeżdża się z rzeczywistością.
* Szum zostaje dodany również do kąta obrotu (`heading`), symulując dryf żyroskopu wirtualnej jednostki IMU.

### 2. Implementacja Pętli SLAM na Kliencie (Korekcja ICP)
* W pliku `client.py` robot traci bezgraniczne zaufanie do współrzędnych pozycji odbieranych z sieci. Wdrażana jest uproszczona pętla dopasowania geometrycznego (analogiczna do algorytmu *Iterative Closest Point* – ICP).
* **Logika działania:** Robot analizuje aktualną chmurę trafień z laserów LiDAR (względne odległości od ścian) i próbuje matematycznie dopasować ten kształt do znanej i pewnej siatki ścian zapisanej w `logic_map`.
* Jeśli odometria twierdzi, że robot znajduje się na pozycji $X = 120$, ale dopasowanie laserów do geometrii korytarza wykazuje, że jedyna pasująca fizycznie pozycja to $X = 114$, algorytm klienta nadpisuje odometrię wartością skorygowaną z lasera, trwale zerując nagromadzony dryf.

---

## 🏆 Podsumowanie Celów i Metryk

Po wdrożeniu powyższego planu, dashboard Streamlit będzie obrazował zaawansowane środowisko badawcze robotyki mobilnej:
1. **Wizualizacja Real World:** Pokaże płynną jazdę z uwzględnieniem masy, uślizgów kół i bezwładnego wchodzenia w zakręty.
2. **Wizualizacja SLAM Grid Map:** Udowodni skuteczność algorytmów filtracji, pokazując, jak wirtualna pozycja robota co chwilę koryguje się (skacze) do stanu rzeczywistego dzięki odbiciom laserów od ścian.
3. **Statystyki:** Wykażą bezbłędność wyznaczania trasy przez czysty algorytm BFS w warunkach braku jakiejkolwiek wiedzy startowej o wielkości i układzie labiryntu.