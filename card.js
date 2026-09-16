/**
 * Aqara FP2 Presence Sensor Card (ESPHome)
 *
 * Zone-mapper style card: draws the radar grid, zones, live targets with
 * trails, and lets you paint zones / change sensitivity directly on the card
 * (stored on the device through the ESPHome `set_zone` / `reset_zone`
 * api actions).
 *
 * Config:
 *   entity_prefix: sensor.fp2_living_room   (required; "<domain>.<entity id prefix>")
 *   device: fp2-living-room                  (ESPHome name; only if it differs from the entity prefix)
 *   title: Living room
 *   show_grid: true          show_fov: true          show_trails: true
 *   show_velocity: true      trail_length: 20        show_axes: true
 *   max_height: 520          cell_size: 0 (px, 0 = fit width)   debug: false
 *   view: [col, row, cols, rows]   crop of the 16x20 grid (default from the device)
 *   auto_crop: false               crop to the drawn zones/maps (+1 cell margin)
 */

const FP2_ZONE_COLORS = [
  [56, 132, 255], [255, 140, 40], [80, 200, 120], [200, 90, 255],
  [255, 80, 120], [40, 200, 220], [230, 200, 40], [150, 150, 150],
];

// Global map layers editable like zones (stored on the device via set_map)
const FP2_MAP_LAYERS = [
  { key: "interference", name: "Interference sources", color: [255, 80, 80], hint: "TVs, fans, curtains - ignored as presence" },
  { key: "edge", name: "Exclude area", color: [150, 150, 150], hint: "cells outside the room / behind walls" },
  { key: "exit", name: "Entry / exit", color: [80, 200, 120], hint: "doorways - used for enter/leave events" },
];

class AqaraFP2Card extends HTMLElement {
  constructor() {
    super();
    this.config = {};
    this._lastKey = null;
    this._trails = {};
    this.edit = null;
    this._painting = null;
    this.mapConfig = null;
  }

  log(...a) { if (this.config.debug) console.log("[FP2 Card]", ...a); }

  // ---------------------------------------------------------------- config
  setConfig(config) {
    if (!config.entity_prefix) throw new Error("entity_prefix is required, e.g. sensor.fp2_living_room");
    this.config = {
      show_grid: true, show_fov: true, show_trails: true, show_velocity: true,
      show_axes: true, trail_length: 20, max_height: 520, cell_size: 0, auto_crop: false, ...config,
    };
    this._lastKey = null;
    if (this.content) this.updateCard();
  }

  static getStubConfig() { return { entity_prefix: "sensor.fp2", title: "FP2" }; }
  getCardSize() { return 8; }
  getGridOptions() { return { columns: 6, rows: "auto", min_columns: 4 }; }

  set hass(hass) {
    this._hass = hass;
    if (!this.content) this.initializeCard();
    const key = this.stateKey();
    if (key !== this._lastKey) { this._lastKey = key; this.updateCard(); }
  }

  // ------------------------------------------------------------- entities
  deviceName() { return this.config.entity_prefix.replace(/^[^.]+\./, ""); }

  // ESPHome api actions are registered as esphome.<device_name>_<action>, where
  // device_name is the ESPHome `name` (dashes -> underscores). Entity ids use the
  // friendly name instead, so the two can differ (fp2-test2 vs "FP2 Test 2").
  serviceDevice() {
    if (this.config.device) return this.config.device.replace(/-/g, "_");
    const svc = (this._hass && this._hass.services && this._hass.services.esphome) || {};
    const candidates = Object.keys(svc).filter((k) => k.endsWith("_get_map_config")).map((k) => k.slice(0, -"_get_map_config".length));
    const dev = this.deviceName();
    if (candidates.includes(dev)) return dev;
    const squash = (x) => x.replace(/_/g, "");
    const m = candidates.filter((c) => squash(c) === squash(dev));
    if (m.length === 1) return m[0];
    if (candidates.length === 1) return candidates[0];
    return dev;
  }

  ent(domain, objectId) {
    if (!objectId) return null;
    const dev = this.deviceName();
    for (const id of [`${domain}.${dev}_${objectId}`, `${domain}.${objectId}`])
      if (this._hass && this._hass.states[id]) return id;
    return `${domain}.${dev}_${objectId}`;
  }

  st(id) { const s = id && this._hass && this._hass.states[id]; return s ? s.state : null; }
  num(id) { const v = parseFloat(this.st(id)); return Number.isFinite(v) ? v : null; }

  entityIds() {
    const mc = this.mapConfig || {};
    const ids = [
      this.ent("sensor", "targets"), this.ent("switch", "report_targets"),
      this.ent("binary_sensor", "presence"), this.ent("binary_sensor", "motion"),
      this.ent("sensor", mc.presence_event_object_id || "presence_event"),
      this.ent("sensor", mc.people_count_object_id || "people_count"),
      this.ent("sensor", "illuminance"), this.ent("sensor", mc.radar_temperature_object_id || "radar_temperature"),
      this.ent("sensor", "radar_software_version"), this.ent("select", "mounting_position"),
      this.ent("binary_sensor", mc.sleep_presence_object_id),
    ];
    (mc.zones || []).forEach((z) => {
      ids.push(this.ent("binary_sensor", z.presence_object_id));
      ids.push(this.ent("binary_sensor", z.motion_object_id));
      ids.push(this.ent("sensor", z.event_object_id));
    });
    return ids.filter(Boolean);
  }

  stateKey() {
    if (!this._hass) return null;
    return this.entityIds().map((id) => `${id}=${this.st(id)}`).join("|");
  }

  async fetchMapConfig() {
    const dev = this.serviceDevice();
    try {
      const r = await this._hass.callService("esphome", `${dev}_get_map_config`, {}, undefined, undefined, true);
      this.mapConfig = (r && r.response) || {};
      this.log("map config", this.mapConfig);
    } catch (e) {
      console.warn("[FP2 Card] get_map_config failed - is the api action in your ESPHome yaml?", e);
      this.mapConfig = {};
    }
    this._lastKey = null;
    this.updateCard();
  }

  // ------------------------------------------------------------------ DOM
  initializeCard() {
    this.innerHTML = `
      <ha-card>
        <div class="hdr">
          <div class="ttl"><span class="name"></span><span class="sub"></span></div>
          <div class="btns">
            <button class="b-live" title="Live target tracking"><ha-icon icon="mdi:eye"></ha-icon></button>
            <button class="b-edit" title="Edit zones"><ha-icon icon="mdi:pencil"></ha-icon></button>
          </div>
        </div>
        <div class="editor" hidden>
          <span class="e-title">Editing</span>
          <select class="e-zone"></select>
          <select class="e-sens"><option value="1">Low</option><option value="2">Medium</option><option value="3">High</option></select>
          <span class="e-cells"></span>
          <span class="sp"></span>
          <button class="e-clear">Clear</button>
          <button class="e-reset" title="Revert to the YAML grid">Reset</button>
          <button class="e-cancel">Cancel</button>
          <button class="e-save primary">Save</button>
        </div>
        <div class="wrap"><canvas class="cv"></canvas></div>
        <div class="chips"></div>
        <table class="zones"><tbody></tbody></table>
        <table class="targets"><thead><tr><th>ID</th><th>Position</th><th>Speed</th><th>Posture</th><th>SNR</th></tr></thead><tbody></tbody></table>
      </ha-card>
      <style>
        ha-card { padding: 12px 16px 14px; display: flex; flex-direction: column; gap: 10px; }
        .hdr { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
        .ttl { display: flex; flex-direction: column; }
        .ttl .name { font-size: 20px; font-weight: 500; }
        .ttl .sub { font-size: 12px; color: var(--secondary-text-color); }
        .btns { display: flex; gap: 6px; }
        .btns button, .editor button, .zones button {
          background: none; border: 1px solid var(--divider-color); border-radius: 6px;
          padding: 6px 8px; cursor: pointer; color: var(--primary-text-color); line-height: 0; }
        .btns button.active { background: var(--primary-color); color: var(--text-primary-color); border-color: var(--primary-color); }
        .editor { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; padding: 8px 10px; font-size: 13px;
          border: 1px solid var(--primary-color); border-radius: 6px; background: var(--secondary-background-color); }
        .editor[hidden] { display: none; }
        .editor .sp { flex: 1; }
        .editor select[hidden] { display: none; }
        .editor select, .zones select { border: 1px solid var(--divider-color); border-radius: 6px; padding: 4px 6px;
          background: var(--card-background-color); color: var(--primary-text-color); }
        .editor button, .zones button { line-height: normal; padding: 4px 10px; }
        .editor button.primary { background: var(--primary-color); color: var(--text-primary-color); border-color: var(--primary-color); }
        .editor .e-cells { color: var(--secondary-text-color); }
        .wrap { display: flex; justify-content: center; }
        .cv { display: block; max-width: 100%; border-radius: 6px; background: var(--card-background-color); touch-action: none; }
        .cv.editing { cursor: crosshair; }
        .chips { display: flex; flex-wrap: wrap; gap: 6px; font-size: 12px; }
        .chip { border: 1px solid var(--divider-color); border-radius: 12px; padding: 2px 9px; background: var(--secondary-background-color); }
        .chip .k { opacity: .65; margin-right: 5px; }
        .chip.on { border-color: rgba(56,132,255,.9); background: rgba(56,132,255,.15); }
        .chip.move { border-color: rgba(255,140,40,.9); background: rgba(255,140,40,.15); }
        table { border-collapse: collapse; width: 100%; font-size: 13px; }
        td, th { text-align: left; padding: 4px 6px; border-bottom: 1px solid var(--divider-color); }
        th { font-weight: 500; opacity: .7; }
        .zones .sw { display: inline-block; width: 12px; height: 12px; border-radius: 3px; margin-right: 8px; vertical-align: middle; }
        .zones td.act { text-align: right; white-space: nowrap; }
        .zones td.act button { padding: 2px 8px; margin-left: 4px; }
        .zones .state { font-size: 12px; color: var(--secondary-text-color); }
        .zones .hint { opacity: .6; font-size: 11px; margin-left: 6px; }
        .zones tr.on .state { color: rgb(56,132,255); }
        .zones tr.move .state { color: rgb(255,140,40); }
        .targets:empty, .targets tbody:empty { display: none; }
        .targets { font-variant-numeric: tabular-nums; }
      </style>`;
    this.content = this.querySelector("ha-card");
    this.canvas = this.querySelector(".cv");
    this.ctx = this.canvas.getContext("2d");
    this.editor = this.querySelector(".editor");

    this.querySelector(".b-live").addEventListener("click", () => this.toggleLive());
    this.querySelector(".b-edit").addEventListener("click", () => this.toggleEdit());
    this.querySelector(".e-zone").addEventListener("change", (e) => this.selectEditZone(e.target.value));
    this.querySelector(".e-sens").addEventListener("change", (e) => { if (this.edit) this.edit.sensitivity = Number(e.target.value); });
    this.querySelector(".e-clear").addEventListener("click", () => this.editClear());
    this.querySelector(".e-reset").addEventListener("click", () => this.editReset());
    this.querySelector(".e-cancel").addEventListener("click", () => this.toggleEdit(false));
    this.querySelector(".e-save").addEventListener("click", () => this.editSave());

    const c = this.canvas;
    c.addEventListener("pointerdown", (e) => { c.setPointerCapture(e.pointerId); this.pointerDown(e); });
    c.addEventListener("pointermove", (e) => this.pointerMove(e));
    c.addEventListener("pointerup", () => { this._painting = null; });
    c.addEventListener("pointerleave", () => { if (!this.edit) c.title = ""; });

    this.ro = new ResizeObserver(() => this.render());
    this.ro.observe(this.querySelector(".wrap"));
    this.fetchMapConfig();
  }

  disconnectedCallback() { if (this.ro) this.ro.disconnect(); }

  // --------------------------------------------------------------- update
  updateCard() {
    if (!this._hass || !this.canvas) return;
    this.data = this.gather();
    this.render();
    this.renderPanels();
  }

  gather() {
    const mc = this.mapConfig || {};
    const rows = mc.grid_rows || 20, cols = mc.grid_cols || 16;
    let view = mc.view || (mc.mounting_position && mc.mounting_position !== "wall" ? [2, 0, 14, 14] : [0, 0, 16, 20]);
    if (Array.isArray(this.config.view) && this.config.view.length === 4) {
      const [c, r, w, h] = this.config.view.map(Number);
      view = [Math.max(0, c), Math.max(0, r), Math.min(cols - c, w), Math.min(rows - r, h)];
    }
    const mounting = this.st(this.ent("select", "mounting_position")) || mc.mounting_position || "wall";
    const corner = /corner/.test(mounting);
    const parse = (hex) => this.parseGrid(hex, rows, cols);

    const zones = (mc.zones || []).map((z, i) => {
      const id = z.id || i + 1;
      const pres = this.ent("binary_sensor", z.presence_object_id);
      const mot = this.ent("binary_sensor", z.motion_object_id);
      const ev = this.ent("sensor", z.event_object_id);
      return {
        id, index: i,
        name: (z.name || `Zone ${id}`).replace(/\s*presence$/i, ""),
        grid: parse(z.grid), hex: z.grid, empty: !!z.empty || !/[1-9a-f]/i.test(z.grid || ""),
        sensitivity: z.sensitivity || 2, override: !!z.runtime_override,
        occupied: this.st(pres) === "on", moving: this.st(mot) === "on", event: this.st(ev),
        color: FP2_ZONE_COLORS[i % FP2_ZONE_COLORS.length],
      };
    });

    const targets = this.decodeTargets(this.st(this.ent("sensor", "targets")));
    if (this.config.show_trails) {
      const seen = new Set();
      targets.forEach((t) => {
        seen.add(t.id);
        const tr = (this._trails[t.id] = this._trails[t.id] || []);
        tr.push({ x: t.x, y: t.y });
        if (tr.length > this.config.trail_length) tr.shift();
      });
      Object.keys(this._trails).forEach((k) => { if (!seen.has(Number(k))) delete this._trails[k]; });
    }

    const layers = FP2_MAP_LAYERS.map((l) => ({
      ...l, id: `map:${l.key}`, grid: parse(mc[`${l.key}_grid`]), hex: mc[`${l.key}_grid`],
      empty: !/[1-9a-f]/i.test(mc[`${l.key}_grid`] || ""), override: !!mc[`${l.key}_override`],
    }));
    if (this.config.auto_crop && !this.edit) {
      let c0 = cols, r0 = rows, c1 = -1, r1 = -1;
      const grids = [...zones.map((z) => z.grid), ...layers.map((l) => l.grid)];
      grids.forEach((g) => g.forEach((row, r) => row.forEach((v, c) => { if (v) { c0 = Math.min(c0, c); c1 = Math.max(c1, c); r0 = Math.min(r0, r); r1 = Math.max(r1, r); } })));
      if (c1 >= 0) {
        c0 = Math.max(0, c0 - 1); r0 = Math.max(0, r0 - 1); c1 = Math.min(cols - 1, c1 + 1); r1 = Math.min(rows - 1, r1 + 1);
        view = [c0, r0, c1 - c0 + 1, r1 - r0 + 1];
      }
    }
    return {
      rows, cols, view, mounting, corner, layers,
      edge: parse(mc.edge_grid), interference: parse(mc.interference_grid), exit: parse(mc.exit_grid),
      zones, targets,
      presence: this.st(this.ent("binary_sensor", "presence")) === "on",
      motion: this.st(this.ent("binary_sensor", "motion")) === "on",
      event: this.st(this.ent("sensor", mc.presence_event_object_id || "presence_event")),
      people: this.num(this.ent("sensor", mc.people_count_object_id || "people_count")),
      lux: this.num(this.ent("sensor", "illuminance")),
      temp: this.num(this.ent("sensor", mc.radar_temperature_object_id || "radar_temperature")),
      fw: this.st(this.ent("sensor", "radar_software_version")),
      live: this.st(this.ent("switch", "report_targets")) === "on",
      bed: mc.sleep_presence_object_id ? this.st(this.ent("binary_sensor", mc.sleep_presence_object_id)) : null,
    };
  }

  parseGrid(hex, rows, cols) {
    const g = Array.from({ length: rows }, () => Array(cols).fill(0));
    if (!hex) return g;
    const n = Math.min(rows, Math.floor(hex.length / 4));
    for (let r = 0; r < n; r++) {
      const bits = parseInt(hex.substr(r * 4, 4), 16);
      for (let c = 0; c < cols; c++) g[r][c] = (bits >> (15 - c)) & 1;
    }
    return g;
  }

  gridToHex(g) {
    let hex = "";
    for (let r = 0; r < g.length; r++) {
      let bits = 0;
      for (let c = 0; c < g[r].length; c++) if (g[r][c]) bits |= 1 << (15 - c);
      hex += bits.toString(16).padStart(4, "0");
    }
    return hex;
  }

  decodeTargets(b64) {
    if (!b64 || b64 === "unknown" || b64 === "unavailable") return [];
    try {
      const bin = atob(b64);
      const b = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) b[i] = bin.charCodeAt(i);
      const s16 = (o) => { const v = (b[o] << 8) | b[o + 1]; return v > 0x7fff ? v - 0x10000 : v; };
      const out = [];
      for (let i = 0; i < b[0]; i++) {
        const o = 1 + i * 14;
        if (o + 14 > b.length) break;
        out.push({ id: b[o], x: s16(o + 1), y: s16(o + 3), z: s16(o + 5), velocity: s16(o + 7), snr: s16(o + 9),
          classifier: b[o + 11], posture: b[o + 12], active: b[o + 13] });
      }
      return out;
    } catch (e) { return []; }
  }

  // Raw radar coordinates -> full-grid cell coordinates (float). +X is the
  // viewer's left (sensor at the top looking down the canvas).
  toGrid(x, y) {
    if (this.data.corner) return { gc: 2 + ((-x + 400) / 800) * 14, gr: (y / 800) * 14 };
    return { gc: 8 - x / 50, gr: y / 50 };
  }
  sensorCell() { return this.data.corner ? (this.data.mounting === "right_corner" ? { gc: 16, gr: 0 } : { gc: 2, gr: 0 }) : { gc: 8, gr: 0 }; }
  toMeters(x, y) { return { x: x / 100, y: y / 100 }; }

  // ---------------------------------------------------------------- canvas
  render() {
    if (!this.data || !this.canvas) return;
    const d = this.data;
    const wrap = this.querySelector(".wrap");
    const W = wrap.clientWidth;
    if (!W) return;
    const [c0, r0, vc, vr] = d.view;
    const axes = this.config.show_axes ? 18 : 0;
    const cell = this.config.cell_size > 0
      ? this.config.cell_size
      : Math.max(8, Math.min((W - axes) / vc, (this.config.max_height - axes) / vr));
    const cw = Math.round(cell * vc + axes), ch = Math.round(cell * vr + axes);
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = cw * dpr; this.canvas.height = ch * dpr;
    this.canvas.style.width = `${cw}px`; this.canvas.style.height = `${ch}px`;
    const ctx = this.ctx;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cw, ch);
    this.geo = { c0, r0, vc, vr, cell, axes };
    const X = (gc) => axes + (gc - c0) * cell, Y = (gr) => axes + (gr - r0) * cell;
    const styles = getComputedStyle(this);
    const fg = styles.getPropertyValue("--primary-text-color") || "#333";
    const dim = styles.getPropertyValue("--secondary-text-color") || "#888";

    // field of view
    if (this.config.show_fov) {
      const s = this.sensorCell();
      ctx.save();
      ctx.beginPath();
      ctx.rect(axes, axes, vc * cell, vr * cell);
      ctx.clip();
      ctx.fillStyle = "rgba(56,132,255,0.06)";
      ctx.beginPath();
      ctx.moveTo(X(s.gc), Y(s.gr));
      if (d.corner) {
        const dir = d.mounting === "right_corner" ? Math.PI : 0;
        ctx.arc(X(s.gc), Y(s.gr), 14 * cell, dir, dir + Math.PI / 2, d.mounting === "right_corner");
      } else {
        ctx.arc(X(s.gc), Y(s.gr), 20 * cell, Math.PI / 2 - Math.PI / 3, Math.PI / 2 + Math.PI / 3);
      }
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    // grid
    if (this.config.show_grid) {
      ctx.strokeStyle = "rgba(128,128,128,0.25)";
      ctx.lineWidth = 1;
      for (let c = 0; c <= vc; c++) { ctx.beginPath(); ctx.moveTo(X(c0 + c), axes); ctx.lineTo(X(c0 + c), axes + vr * cell); ctx.stroke(); }
      for (let r = 0; r <= vr; r++) { ctx.beginPath(); ctx.moveTo(axes, Y(r0 + r)); ctx.lineTo(axes + vc * cell, Y(r0 + r)); ctx.stroke(); }
    }
    // axes (metres)
    if (axes) {
      ctx.fillStyle = dim; ctx.font = "10px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      const sc = this.sensorCell().gc;
      for (let c = 0; c <= vc; c += 2) {
        const m = ((c0 + c) - sc) * 0.5;
        ctx.fillText(`${m > 0 ? "+" : ""}${m}`, X(c0 + c), axes / 2);
      }
      ctx.textAlign = "right";
      for (let r = 0; r <= vr; r += 2) ctx.fillText(`${(r0 + r) * 0.5}`, axes - 3, Y(r0 + r));
    }

    const eachCell = (grid, fn) => {
      for (let r = r0; r < r0 + vr; r++) for (let c = c0; c < c0 + vc; c++) if (grid[r] && grid[r][c]) fn(X(c), Y(r));
    };
    const editingLayer = this.edit && String(this.edit.zoneId).startsWith("map:") ? this.edit.zoneId : null;
    const la = this.edit ? 0.35 : 1;
    // edge / exclude cells (hatched)
    if (editingLayer !== "map:edge") {
      ctx.fillStyle = `rgba(128,128,128,${0.35 * la})`;
      eachCell(d.edge, (x, y) => {
        ctx.fillRect(x, y, cell, cell);
        ctx.strokeStyle = `rgba(90,90,90,${0.5 * la})`; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + cell, y + cell); ctx.moveTo(x + cell, y); ctx.lineTo(x, y + cell); ctx.stroke();
      });
    }
    // interference
    if (editingLayer !== "map:interference") {
      ctx.fillStyle = `rgba(255,80,80,${0.3 * la})`;
      eachCell(d.interference, (x, y) => ctx.fillRect(x, y, cell, cell));
    }
    // entry / exit
    if (editingLayer !== "map:exit") {
      ctx.strokeStyle = `rgba(80,200,120,${0.9 * la})`; ctx.lineWidth = 2;
      eachCell(d.exit, (x, y) => ctx.strokeRect(x + 2, y + 2, cell - 4, cell - 4));
    }

    // zones
    d.zones.forEach((z) => {
      if (z.empty || (this.edit && this.edit.zoneId === z.id)) return;
      const [R, G, B] = z.color;
      const a = this.edit ? 0.12 : z.occupied ? (z.moving ? 0.6 : 0.45) : 0.16;
      ctx.fillStyle = `rgba(${R},${G},${B},${a})`;
      ctx.strokeStyle = `rgba(${R},${G},${B},${this.edit ? 0.3 : 0.9})`;
      ctx.lineWidth = 1.5;
      let minc = 99, maxc = -1, minr = 99, maxr = -1;
      eachCell(z.grid, (x, y) => { ctx.fillRect(x, y, cell, cell); ctx.strokeRect(x + 0.75, y + 0.75, cell - 1.5, cell - 1.5); });
      for (let r = 0; r < d.rows; r++) for (let c = 0; c < d.cols; c++) if (z.grid[r][c]) { minc = Math.min(minc, c); maxc = Math.max(maxc, c); minr = Math.min(minr, r); maxr = Math.max(maxr, r); }
      if (maxc >= 0 && !this.edit) {
        const lx = X((minc + maxc + 1) / 2), ly = Y((minr + maxr + 1) / 2);
        const sens = { 1: "L", 2: "M", 3: "H" }[z.sensitivity] || "";
        const label = `${z.name} · ${sens}`;
        ctx.font = `bold ${Math.min(cell * 0.55, 12)}px sans-serif`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
        const w = ctx.measureText(label).width + 8, h = Math.min(cell * 0.55, 12) + 6;
        ctx.fillStyle = "rgba(255,255,255,0.92)";
        ctx.fillRect(lx - w / 2, ly - h / 2, w, h);
        ctx.fillStyle = `rgb(${R},${G},${B})`;
        ctx.fillText(label, lx, ly);
      }
    });

    // trails + targets
    if (this.config.show_trails) {
      Object.values(this._trails).forEach((tr) => {
        if (tr.length < 2) return;
        ctx.strokeStyle = "rgba(255,170,0,0.35)"; ctx.lineWidth = 2; ctx.beginPath();
        tr.forEach((p, i) => { const g = this.toGrid(p.x, p.y); i ? ctx.lineTo(X(g.gc), Y(g.gr)) : ctx.moveTo(X(g.gc), Y(g.gr)); });
        ctx.stroke();
      });
    }
    d.targets.forEach((t) => {
      const g = this.toGrid(t.x, t.y);
      const px = X(g.gc), py = Y(g.gr);
      const col = t.posture >= 2 ? [150, 100, 255] : t.posture === 1 ? [80, 200, 120] : [255, 190, 0];
      if (this.config.show_velocity && t.velocity) {
        const s = this.sensorCell();
        const dx = g.gc - s.gc, dy = g.gr - s.gr, len = Math.hypot(dx, dy) || 1;
        const mag = Math.max(-3, Math.min(3, t.velocity / 60)) * cell;
        const ex = px + (dx / len) * mag, ey = py + (dy / len) * mag;
        ctx.strokeStyle = `rgb(${col})`; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(ex, ey); ctx.stroke();
      }
      const r = Math.min(cell * 0.32, 12);
      ctx.fillStyle = `rgba(${col},0.9)`; ctx.strokeStyle = "rgba(0,0,0,0.5)"; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(px, py, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      ctx.fillStyle = "#000"; ctx.font = `bold ${Math.min(cell * 0.4, 11)}px sans-serif`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(String(t.id), px, py);
    });

    // sensor marker
    {
      const s = this.sensorCell();
      const px = X(s.gc), py = Y(s.gr);
      ctx.strokeStyle = "rgba(150,90,255,0.9)"; ctx.lineWidth = 2;
      [4, 8, 12].forEach((rr) => { ctx.beginPath(); ctx.arc(px, py, rr, Math.PI * 0.15, Math.PI * 0.85); ctx.stroke(); });
      ctx.fillStyle = "rgb(150,90,255)"; ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2); ctx.fill();
    }

    // edit overlay
    if (this.edit) {
      const z = this.findEditable(this.edit.zoneId);
      const [R, G, B] = z ? z.color : [255, 190, 0];
      ctx.fillStyle = `rgba(${R},${G},${B},0.55)`; ctx.strokeStyle = `rgb(${R},${G},${B})`; ctx.lineWidth = 2;
      eachCell(this.edit.cells, (x, y) => { ctx.fillRect(x, y, cell, cell); ctx.strokeRect(x + 1, y + 1, cell - 2, cell - 2); });
    }
  }

  cellAt(e) {
    if (!this.geo) return null;
    const rect = this.canvas.getBoundingClientRect();
    const { c0, r0, vc, vr, cell, axes } = this.geo;
    const c = Math.floor((e.clientX - rect.left - axes) / cell) + c0;
    const r = Math.floor((e.clientY - rect.top - axes) / cell) + r0;
    if (c < c0 || c >= c0 + vc || r < r0 || r >= r0 + vr) return null;
    return { c, r };
  }

  // Drag paints a rectangle from the start cell (fill or erase, depending on
  // the start cell's state); a click toggles one cell.
  pointerDown(e) {
    if (!this.edit) return;
    const p = this.cellAt(e); if (!p) return;
    this._painting = { value: this.edit.cells[p.r][p.c] ? 0 : 1, start: p, base: this.edit.cells.map((r) => r.slice()) };
    this.paintRect(p);
  }
  pointerMove(e) {
    const p = this.cellAt(e);
    if (this.edit) { if (this._painting && p) this.paintRect(p); return; }
    if (!p) { this.canvas.title = ""; return; }
    const s = this.sensorCell();
    const zones = this.data.zones.filter((z) => z.grid[p.r] && z.grid[p.r][p.c]).map((z) => z.name);
    this.canvas.title = `${((p.c - s.gc) * 0.5).toFixed(1)} m, ${((p.r - s.gr) * 0.5).toFixed(1)} m` + (zones.length ? ` · ${zones.join(", ")}` : "");
  }
  paintRect(p) {
    const { value, start, base } = this._painting;
    const cells = base.map((r) => r.slice());
    for (let r = Math.min(start.r, p.r); r <= Math.max(start.r, p.r); r++)
      for (let c = Math.min(start.c, p.c); c <= Math.max(start.c, p.c); c++) cells[r][c] = value;
    this.edit.cells = cells;
    this.updateEditInfo(); this.render();
  }

  findEditable(id) {
    if (!this.data) return null;
    return this.data.zones.find((z) => z.id === id) || this.data.layers.find((l) => l.id === id) || null;
  }

  // ---------------------------------------------------------------- panels
  chip(k, v, cls = "") {
    if (v === null || v === undefined || v === "unknown" || v === "unavailable" || v === "") return "";
    return `<span class="chip ${cls}"><span class="k">${k}</span>${v}</span>`;
  }

  renderPanels() {
    const d = this.data;
    this.querySelector(".name").textContent = this.config.title || this.deviceName().replace(/_/g, " ");
    this.querySelector(".sub").textContent = `${d.mounting.replace("_", " ")} · ${d.corner ? "7 × 7 m" : "8 × 10 m"} · radar fw ${d.fw || "?"}`;
    this.querySelector(".b-live").classList.toggle("active", d.live);
    this.querySelector(".b-edit").classList.toggle("active", !!this.edit);

    this.querySelector(".chips").innerHTML =
      this.chip("Presence", d.presence ? "detected" : "clear", d.presence ? (d.motion ? "move" : "on") : "") +
      this.chip("Targets", d.targets.length, d.targets.length ? "move" : "") +
      this.chip("Event", d.event) + this.chip("People", d.people) +
      (d.bed !== null && d.bed !== undefined ? this.chip("Bed", d.bed === "on" ? "occupied" : "empty", d.bed === "on" ? "on" : "") : "") +
      this.chip("Light", d.lux !== null ? `${Math.round(d.lux)} lx` : null) +
      this.chip("Radar", d.temp !== null ? `${Math.round(d.temp)} °C` : null) +
      (d.live ? "" : `<span class="chip">live tracking off — press <ha-icon icon="mdi:eye" style="--mdc-icon-size:14px"></ha-icon></span>`);

    const tb = this.querySelector(".zones tbody");
    tb.innerHTML = d.zones.map((z) => {
      const [R, G, B] = z.color;
      const state = z.empty ? "not drawn" : z.occupied ? (z.moving ? "occupied · moving" : "occupied") : "empty";
      const ev = z.event && !z.empty && z.event !== "unknown" ? ` · ${z.event}` : "";
      return `<tr class="${z.occupied ? (z.moving ? "move" : "on") : ""}" data-id="${z.id}">
        <td><span class="sw" style="background:rgb(${R},${G},${B})"></span>${z.name}${z.override ? " *" : ""}</td>
        <td class="state">${state}${ev}</td>
        <td><select class="z-sens" data-id="${z.id}" ${z.empty ? "disabled" : ""}>
          <option value="1" ${z.sensitivity === 1 ? "selected" : ""}>Low</option>
          <option value="2" ${z.sensitivity === 2 ? "selected" : ""}>Medium</option>
          <option value="3" ${z.sensitivity === 3 ? "selected" : ""}>High</option></select></td>
        <td class="act"><button class="z-edit" data-id="${z.id}" title="Draw this zone">${z.empty ? "Draw" : "Edit"}</button></td>
      </tr>`;
    }).join("");
    tb.innerHTML += d.layers.map((l) => {
      const [R, G, B] = l.color;
      const n = l.grid.flat().filter(Boolean).length;
      return `<tr data-id="${l.id}">
        <td><span class="sw" style="background:rgb(${R},${G},${B})"></span>${l.name}${l.override ? " *" : ""}</td>
        <td class="state" colspan="2">${n ? `${n} cells · ${(n * 0.25).toFixed(1)} m²` : "not drawn"} <span class="hint">${l.hint}</span></td>
        <td class="act"><button class="z-edit" data-id="${l.id}">${n ? "Edit" : "Draw"}</button></td>
      </tr>`;
    }).join("");
    tb.querySelectorAll(".z-edit").forEach((b) => b.addEventListener("click", () => this.toggleEdit(true, b.dataset.id.startsWith("map:") ? b.dataset.id : Number(b.dataset.id))));
    tb.querySelectorAll(".z-sens").forEach((s) => s.addEventListener("change", () => this.setSensitivity(Number(s.dataset.id), Number(s.value))));

    const tt = this.querySelector(".targets tbody");
    tt.innerHTML = d.targets.map((t) => {
      const m = this.toMeters(t.x, t.y);
      const post = { 0: "moving", 1: "sitting", 2: "lying", 255: "-" }[t.posture] ?? t.posture;
      return `<tr><td>#${t.id}</td><td>${m.x >= 0 ? "+" : ""}${m.x.toFixed(2)}, ${m.y.toFixed(2)} m</td><td>${(t.velocity / 100).toFixed(2)} m/s</td><td>${post}</td><td>${t.snr}</td></tr>`;
    }).join("");
  }

  // --------------------------------------------------------------- actions
  toggleLive() {
    const id = this.ent("switch", "report_targets");
    if (!this._hass.states[id]) return;
    this._hass.callService("switch", this._hass.states[id].state === "on" ? "turn_off" : "turn_on", { entity_id: id });
  }

  async setSensitivity(zoneId, sens) {
    const z = this.data.zones.find((zz) => zz.id === zoneId);
    if (!z) return;
    await this.callSetZone(zoneId, z.hex, sens);
  }

  async callSetZone(zoneId, hex, sens) {
    const dev = this.serviceDevice();
    try {
      await this._hass.callService("esphome", `${dev}_set_zone`, { zone_id: zoneId, grid: hex, sensitivity: sens });
      await this.fetchMapConfig();
      return true;
    } catch (err) {
      console.error("[FP2 Card] set_zone failed", err);
      this.querySelector(".sub").textContent = `set_zone failed: ${err && err.message ? err.message : err}`;
      return false;
    }
  }

  toggleEdit(on, zoneId) {
    const enable = on === undefined ? !this.edit : on;
    if (enable) {
      const zones = this.data ? this.data.zones : [];
      if (!zones.length) return;
      const sel = this.querySelector(".e-zone");
      sel.innerHTML = zones.map((z) => `<option value="${z.id}">${z.name}${z.empty ? " (not drawn)" : ""}</option>`).join("") +
        this.data.layers.map((l) => `<option value="${l.id}">${l.name}</option>`).join("");
      sel.value = String(zoneId || zones[0].id);
      this.selectEditZone(sel.value);
      this.editor.hidden = false;
      this.canvas.classList.add("editing");
    } else {
      this.edit = null; this._painting = null;
      this.editor.hidden = true;
      this.canvas.classList.remove("editing");
    }
    this._lastKey = null;
    this.updateCard();
  }

  selectEditZone(raw) {
    const zoneId = String(raw).startsWith("map:") ? String(raw) : Number(raw);
    const z = this.findEditable(zoneId);
    if (!z) return;
    const isLayer = typeof zoneId === "string";
    this.edit = { zoneId, isLayer, kind: isLayer ? z.key : null, sensitivity: z.sensitivity || 2, cells: z.grid.map((r) => r.slice()) };
    this.querySelector(".e-title").textContent = `Editing ${z.name}`;
    this.querySelector(".e-sens").hidden = isLayer;
    this.querySelector(".e-sens").value = String(z.sensitivity || 2);
    this.updateEditInfo();
    this.render();
  }

  updateEditInfo() {
    if (!this.edit) return;
    const n = this.edit.cells.flat().filter(Boolean).length;
    this.querySelector(".e-cells").textContent = `${n} cells · ${(n * 0.25).toFixed(1)} m²`;
  }

  editClear() {
    if (!this.edit) return;
    this.edit.cells = this.edit.cells.map((r) => r.map(() => 0));
    this.updateEditInfo(); this.render();
  }

  async editSave() {
    if (!this.edit) return;
    const hex = this.gridToHex(this.edit.cells);
    if (this.edit.isLayer) {
      try {
        await this._hass.callService("esphome", `${this.serviceDevice()}_set_map`, { kind: this.edit.kind, grid: hex });
        await this.fetchMapConfig();
        this.toggleEdit(false);
      } catch (err) {
        console.error("[FP2 Card] set_map failed", err);
        this.querySelector(".sub").textContent = `set_map failed - add the set_map api action to your yaml`;
      }
      return;
    }
    if (await this.callSetZone(this.edit.zoneId, hex, this.edit.sensitivity)) this.toggleEdit(false);
  }

  async editReset() {
    if (!this.edit) return;
    try {
      if (this.edit.isLayer)
        await this._hass.callService("esphome", `${this.serviceDevice()}_reset_map`, { kind: this.edit.kind });
      else
        await this._hass.callService("esphome", `${this.serviceDevice()}_reset_zone`, { zone_id: this.edit.zoneId });
      await this.fetchMapConfig();
      this.toggleEdit(false);
    } catch (err) { console.error("[FP2 Card] reset_zone failed", err); }
  }
}

customElements.define("aqara-fp2-card", AqaraFP2Card);
window.customCards = window.customCards || [];
window.customCards.push({
  type: "aqara-fp2-card",
  name: "Aqara FP2 Presence Sensor Card",
  description: "Radar map with zones, live targets and on-card zone editing for the ESPHome FP2",
  preview: true,
});
