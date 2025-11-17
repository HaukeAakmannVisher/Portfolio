# Laptop (Planner + UI)

- **app.py** — Flask backend:
  - Loads the warehouse grid.
  - Runs **A\*** to plan paths.
  - Converts paths → **intersection actions** (left/right/straight/stop) per leg (Home→G1, G1→G2, G2→Home).
  - Exposes HTTP endpoints to **send tasks** to the Pi and **receive live logs** back.
  - Serves JSON used by the web page.

- **warehouse_map.html** — Browser UI:
  - Lets the user **pick Goal 1 & Goal 2** on the grid.
  - Draws the **planned route** and shows the **task list** per leg.
  - Streams and displays **live updates** (intersection events, obstacle detected).
