-Benchmark tego DFS
-zooptymalizować DFS aby był idealny
-generate new maze nie działa poprawnie

W pierwszym przejeździe robot szuka mety i buduje mapę (wtedy Path Efficiency jest niskie).

Gdy robot dotrze do mety i pozna cały układ, algorytm w pamięci procesora (np. za pomocą algorytmu Floorda-Warshalla, BFS, lub Dijkstry) wybiera ze zmapowanej siatki wyłącznie najkrótszą, idealną ścieżkę od startu do mety.

Robot wraca na start i rusza do Biegu Finałowego (tzw. Fast Run).