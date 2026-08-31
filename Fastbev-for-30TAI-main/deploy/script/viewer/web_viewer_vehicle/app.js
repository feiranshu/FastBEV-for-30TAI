const els = {
  frameSelect: document.getElementById("frameSelect"),
  prevFrame: document.getElementById("prevFrame"),
  nextFrame: document.getElementById("nextFrame"),
  playPause: document.getElementById("playPause"),
  liveSim: document.getElementById("liveSim"),
  fpsInput: document.getElementById("fpsInput"),
  refreshFrames: document.getElementById("refreshFrames"),
  scoreThreshold: document.getElementById("scoreThreshold"),
  thresholdValue: document.getElementById("thresholdValue"),
  status: document.getElementById("status"),
  canvas: document.getElementById("viewerCanvas"),
  summary: document.getElementById("summary"),
  classFilters: document.getElementById("classFilters"),
  selectAllClasses: document.getElementById("selectAllClasses"),
  clearAllClasses: document.getElementById("clearAllClasses"),
  detailTable: document.getElementById("detailTable"),
  rawLine: document.getElementById("rawLine"),
};

const classMap = {
  0: "car",
  1: "class_1",
  2: "class_2",
  3: "class_3",
  4: "class_4",
  5: "class_5",
  6: "class_6",
  7: "class_7",
  8: "class_8",
  9: "class_9",
};
const enabledClasses = new Set(Object.keys(classMap).map(Number));
const imageCache = new Map();
const nearestObjectClassIds = new Set([0, 1, 2, 3, 4, 5, 6, 7]);
const nearestObjectLimit = 3;
const canvasColors = {
  rangeRing: "#1e90ff",
  detection: "#fff200",
  selected: "#42c8ff",
  nearest: "#ff3030",
};

let frames = [];
let currentFrame = null;
let currentImages = new Map();
let hoverId = null;
let selectedId = null;
let playing = false;
let playTimer = null;
let liveMode = false;
let liveTimer = null;
let lastLiveTiming = null;
let lastLivePushSequence = null;
let lastPostToPaintMs = null;
let lastLiveImageSequence = null;
let thresholdInitialized = false;

function setStatus(text, isError = false) {
  els.status.textContent = text;
  els.status.style.color = isError ? "var(--danger)" : "var(--muted)";
}

async function getJson(url) {
  const res = await fetch(url, { cache: "no-store" });
  const data = await res.json();
  if (!res.ok) throw new Error(data.error || `${res.status} ${res.statusText}`);
  return data;
}

function frameFromUrl() {
  const params = new URLSearchParams(window.location.search);
  const value = params.get("frame");
  return /^\d{4}$/.test(value || "") ? value : null;
}

function liveFromUrl() {
  const params = new URLSearchParams(window.location.search);
  return params.get("live") === "1";
}

function setFrameUrl(frameId) {
  const url = new URL(window.location.href);
  url.searchParams.set("frame", frameId);
  window.history.replaceState(null, "", url);
}

async function loadFrameOptions() {
  const data = await getJson("/api/frames");
  frames = data.frames.filter((f) => f.complete);
  els.frameSelect.innerHTML = "";
  for (const frame of frames) {
    const option = document.createElement("option");
    option.value = frame.id;
    option.textContent = frame.id;
    els.frameSelect.appendChild(option);
  }

  if (!frames.length) {
    throw new Error("没有找到完整帧：需要 result 和 parameter 文件");
  }
}

async function loadFrames(preferredFrame = null) {
  await loadFrameOptions();

  const target = preferredFrame && frames.some((f) => f.id === preferredFrame)
    ? preferredFrame
    : frames[frames.length - 1].id;
  await loadFrame(target);
}

function loadImage(url) {
  if (imageCache.has(url)) return imageCache.get(url);
  const promise = new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = () => reject(new Error(`图片加载失败: ${url}`));
    img.src = url;
  });
  imageCache.set(url, promise);
  return promise;
}

async function loadFrame(frameId) {
  setStatus(`Loading frame ${frameId}...`);
  selectedId = null;
  hoverId = null;
  const data = await getJson(`/api/frame/${frameId}`);
  const images = new Map();
  await Promise.all(data.camera_images.map(async (item) => {
    images.set(item.view, await loadImage(item.url));
  }));

  currentFrame = data;
  currentImages = images;
  ensureFrameClasses(data);
  applyFrameThresholdDefault(data);
  els.frameSelect.value = frameId;
  setFrameUrl(frameId);
  resizeCanvas();
  drawScene();
  showDetection(null);
  setStatus(`Frame ${frameId}: ${data.detections.length} detections`);
  prefetchNextFrame();
}

async function loadLiveFrame() {
  const started = performance.now();
  const data = await getJson("/api/live");
  const apiDone = performance.now();
  if (["push_waiting", "carla_waiting", "vehicle_waiting"].includes(data.live?.mode)) {
    setStatus(data.live?.mode === "carla_waiting"
      ? "LIVE waiting for the first CARLA + board frame..."
      : data.live?.mode === "vehicle_waiting"
        ? "LIVE waiting for the first Vehicle frame..."
        : "LIVE waiting for the first board frame...");
    return;
  }
  const incomingSequence = Number(data.live?.sequence);
  const isLivePush = ["push", "carla", "vehicle"].includes(data.live?.mode);
  const isNewBoardFrame = isLivePush && Number.isFinite(incomingSequence) &&
    incomingSequence !== lastLiveImageSequence;
  if (isNewBoardFrame) {
    imageCache.clear();
    lastLiveImageSequence = incomingSequence;
  }
  const images = new Map();
  await Promise.all(data.camera_images.map(async (item) => {
    images.set(item.view, await loadImage(item.url));
  }));
  const imagesDone = performance.now();

  selectedId = null;
  hoverId = null;
  currentFrame = data;
  currentImages = images;
  ensureFrameClasses(data);
  applyFrameThresholdDefault(data);
  els.frameSelect.value = data.frame_id;
  resizeCanvas();
  const drawStarted = performance.now();
  drawScene();
  const drawDone = performance.now();
  await new Promise((resolve) => requestAnimationFrame(() => resolve()));
  const presented = performance.now();
  showDetection(null);
  const live = data.live || {};
  const receivedEpochMs = Number(live.push_received_epoch_ms);
  const pushSequence = Number(live.sequence);
  const isNewPush = ["push", "carla", "vehicle"].includes(live.mode) && Number.isFinite(pushSequence) &&
    pushSequence !== lastLivePushSequence;
  if (isNewPush && Number.isFinite(receivedEpochMs)) {
    lastLivePushSequence = pushSequence;
    lastPostToPaintMs = Math.max(0, Date.now() - receivedEpochMs);
  }
  const currentTiming = {
    api_ms: apiDone - started,
    images_ms: imagesDone - apiDone,
    draw_ms: drawDone - drawStarted,
    paint_ms: presented - drawDone,
    browser_total_ms: presented - started,
    post_to_paint_ms: isNewPush ? lastPostToPaintMs : null,
  };
  if (isNewBoardFrame || lastLiveTiming === null) {
    lastLiveTiming = currentTiming;
  }
  const postText = lastPostToPaintMs === null
    ? "waiting for push"
    : `${lastPostToPaintMs.toFixed(1)} ms${isNewPush ? "" : " (last push)"}`;
  const timingText = isNewBoardFrame ? "" : " (last board frame)";
  const sourceText = live.mode === "carla"
    ? `CARLA/${live.source ?? "edge"}, inference ${Number(live.inference_ms ?? 0).toFixed(1)} ms, ` +
      `gateway ${Number(live.gateway_roundtrip_ms ?? 0).toFixed(1)} ms | `
    : live.mode === "vehicle"
      ? `Vehicle/${live.source ?? "edge"}, inference ${Number(live.inference_ms ?? 0).toFixed(1)} ms, ` +
        `edge ${Number(live.edge_roundtrip_ms ?? 0).toFixed(1)} ms | `
    : "";
  setStatus(
    `LIVE seq ${live.sequence ?? "-"} frame ${data.frame_id}: ${data.detections.length} detections | ${sourceText}` +
    `POST->paint ${postText}, API ${lastLiveTiming.api_ms.toFixed(1)} ms, ` +
    `images ${lastLiveTiming.images_ms.toFixed(1)} ms, draw ${lastLiveTiming.draw_ms.toFixed(1)} ms${timingText}`
  );
}

function applyFrameThresholdDefault(frame) {
  if (thresholdInitialized) return;
  const threshold = Number(frame?.score_threshold_default);
  if (!Number.isFinite(threshold)) return;
  const clamped = Math.max(0, Math.min(1, threshold));
  els.scoreThreshold.value = clamped.toFixed(3);
  els.thresholdValue.textContent = clamped.toFixed(3);
  thresholdInitialized = true;
}

function ensureFrameClasses(frame) {
  let changed = false;
  for (const det of frame?.detections || []) {
    const id = Number(det.class_id);
    if (!Number.isInteger(id)) continue;
    if (!(id in classMap)) {
      classMap[id] = `class_${id}`;
      enabledClasses.add(id);
      changed = true;
    }
  }
  if (changed) buildClassFilters();
}

function resizeCanvas() {
  if (!currentFrame) return;
  const [w, h] = currentFrame.image_size;
  els.canvas.width = w;
  els.canvas.height = h;
}

function activeDetections() {
  if (!currentFrame) return [];
  const threshold = Number(els.scoreThreshold.value);
  return currentFrame.detections.filter((d) => d.score >= threshold && enabledClasses.has(Number(d.class_id)));
}

function buildClassFilters() {
  els.classFilters.innerHTML = "";
  for (const [classId, className] of Object.entries(classMap)) {
    const id = Number(classId);
    const label = document.createElement("label");
    label.className = "class-toggle";
    label.title = `${id} ${className}`;

    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = enabledClasses.has(id);
    input.dataset.classId = classId;
    input.addEventListener("change", () => {
      if (input.checked) enabledClasses.add(id);
      else enabledClasses.delete(id);
      syncClassFilterState();
      clearHiddenSelection();
      drawScene();
    });

    const text = document.createElement("span");
    text.textContent = `${id} ${className}`;
    label.appendChild(input);
    label.appendChild(text);
    els.classFilters.appendChild(label);
  }
}

function syncClassFilterState() {
  for (const input of els.classFilters.querySelectorAll("input[type='checkbox']")) {
    input.checked = enabledClasses.has(Number(input.dataset.classId));
  }
}

function clearHiddenSelection() {
  if (selectedId !== null && !activeDetections().some((d) => d.id === selectedId)) {
    selectedId = null;
    showDetection(null);
  }
}

function drawScene() {
  const ctx = els.canvas.getContext("2d");
  ctx.clearRect(0, 0, els.canvas.width, els.canvas.height);
  ctx.fillStyle = "#000";
  ctx.fillRect(0, 0, els.canvas.width, els.canvas.height);
  if (!currentFrame) return;

  drawCameraImages(ctx);
  drawBev(ctx);
  const detections = activeDetections();
  const nearest = nearestRoadUsers(detections);
  const nearestRanks = new Map(nearest.map((det, index) => [det.id, index + 1]));
  drawDetections(ctx, detections, nearestRanks);
  drawEgoMarker(ctx);
  drawNearestObjectsPanel(ctx, nearest);
}

function drawCameraImages(ctx) {
  const tileW = currentFrame.layout.camera_tile.width;
  const tileH = currentFrame.layout.camera_tile.height;
  const bottomY = currentFrame.layout.height - tileH;
  for (let i = 0; i < currentFrame.layout.views.length; i++) {
    const view = currentFrame.layout.views[i];
    const img = currentImages.get(view);
    if (!img) continue;
    const col = i % 3;
    const x = col * tileW;
    if (i < 3) {
      ctx.drawImage(img, x, 0, tileW, tileH);
    } else {
      ctx.save();
      ctx.translate(x + tileW, bottomY);
      ctx.scale(-1, 1);
      ctx.drawImage(img, 0, 0, tileW, tileH);
      ctx.restore();
    }
  }
}

function drawBev(ctx) {
  const bev = currentFrame.layout.bev;
  ctx.save();
  ctx.beginPath();
  ctx.rect(bev.x, bev.y, bev.size, bev.size);
  ctx.clip();
  ctx.fillStyle = "#121212";
  ctx.fillRect(bev.x, bev.y, bev.size, bev.size);
  const centre = [bev.x + bev.size * 0.5, bev.y + bev.size * 0.5];

  const worldScale = Number(bev.world_scale || 24);
  ctx.strokeStyle = "#262626";
  ctx.lineWidth = 1;
  ctx.globalAlpha = 1;
  for (let coordinate = -bev.range_m; coordinate <= bev.range_m + 0.1; coordinate += worldScale) {
    const p = bev.x + ((coordinate + bev.range_m) / (2.0 * bev.range_m)) * bev.size;
    const q = bev.y + bev.size - ((coordinate + bev.range_m) / (2.0 * bev.range_m)) * bev.size;
    drawLine(ctx, [p, bev.y], [p, bev.y + bev.size - 1]);
    drawLine(ctx, [bev.x, q], [bev.x + bev.size - 1, q]);
  }

  ctx.strokeStyle = canvasColors.rangeRing;
  ctx.fillStyle = canvasColors.rangeRing;
  ctx.lineWidth = 2;
  for (let physicalRadius = 1; physicalRadius <= 3; physicalRadius++) {
    const rc = physicalRadius * worldScale / (2.0 * bev.range_m) * bev.size;
    ctx.beginPath();
    ctx.arc(centre[0], centre[1], rc, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillText(`${physicalRadius}m`, centre[0] + rc + 4, centre[1] - 4);
  }

  drawArrow(ctx, centre, [centre[0], centre[1] - 70], "#4646ff", 2);
  drawArrow(ctx, centre, [centre[0] - 70, centre[1]], "#46ff46", 2);
  ctx.font = '500 15px "Segoe UI", Arial, sans-serif';
  ctx.fillStyle = "#4646ff";
  ctx.fillText("+X", centre[0] + 4, centre[1] - 74);
  ctx.fillStyle = "#46ff46";
  ctx.fillText("+Y", centre[0] - 94, centre[1] + 4);
  ctx.restore();
}

function nearestRoadUsers(detections) {
  return detections
    .filter((det) => nearestObjectClassIds.has(Number(det.class_id)) && Number.isFinite(det.distance))
    .sort((a, b) => (a.distance - b.distance) || (b.score - a.score) || (a.id - b.id))
    .slice(0, nearestObjectLimit);
}

function drawDetections(ctx, detections, nearestRanks) {
  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.shadowColor = "rgba(0, 0, 0, 0.95)";
  ctx.shadowBlur = 2.5;
  for (const det of detections) {
    const selected = det.id === selectedId;
    const hovered = det.id === hoverId;
    const nearestRank = nearestRanks.get(det.id);
    const alpha = nearestRank || selected ? 1 : hovered ? 0.98 : 0.9;
    const width = nearestRank ? 3.6 : selected ? 3.2 : hovered ? 3 : 2.25;
    ctx.strokeStyle = nearestRank
      ? canvasColors.nearest
      : selected
        ? canvasColors.selected
        : scoreColor(det.score);
    ctx.lineWidth = width;
    ctx.globalAlpha = alpha;

    drawPolygon(ctx, det.bev.polygon);
    drawArrow(ctx, det.bev.heading[0], det.bev.heading[1], ctx.strokeStyle, Math.max(2, width - 1));
    for (const view of det.camera_views) {
      drawCameraViewEdges(ctx, view);
    }

    if (nearestRank) {
      drawRankBadge(ctx, det.bev.center[0], det.bev.center[1], nearestRank);
      const bestView = bestCameraView(det.camera_views);
      if (bestView) {
        const bbox = bestView.clipped_bbox || bestView.bbox;
        drawRankBadge(ctx, bbox[0] + 11, bbox[1] + 11, nearestRank);
      }
    }
  }
  ctx.restore();
}

function cameraTileRect(viewName) {
  const views = currentFrame.layout.views || [];
  const index = views.indexOf(viewName);
  if (index < 0) return null;
  const tileW = currentFrame.layout.camera_tile.width;
  const tileH = currentFrame.layout.camera_tile.height;
  return {
    x: (index % 3) * tileW,
    y: index < 3 ? 0 : currentFrame.layout.height - tileH,
    w: tileW,
    h: tileH,
  };
}

function drawCameraViewEdges(ctx, view) {
  const rect = cameraTileRect(view.view);
  if (!rect) return;
  ctx.save();
  ctx.beginPath();
  ctx.rect(rect.x, rect.y, rect.w, rect.h);
  ctx.clip();
  for (const edge of view.edges) {
    drawLine(ctx, edge[0], edge[1]);
  }
  ctx.restore();
}

function bestCameraView(views) {
  let best = null;
  let bestArea = -1;
  for (const view of views || []) {
    const bbox = view.clipped_bbox || view.bbox;
    if (!bbox) continue;
    const area = Math.max(0, bbox[2] - bbox[0]) * Math.max(0, bbox[3] - bbox[1]);
    if (area > bestArea) {
      best = view;
      bestArea = area;
    }
  }
  return best;
}

function drawRankBadge(ctx, x, y, rank) {
  const radius = 10;
  const cx = Math.max(radius + 2, Math.min(els.canvas.width - radius - 2, x));
  const cy = Math.max(radius + 2, Math.min(els.canvas.height - radius - 2, y));
  ctx.save();
  ctx.globalAlpha = 1;
  ctx.shadowBlur = 2;
  ctx.fillStyle = canvasColors.nearest;
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.fillStyle = "#fff";
  ctx.font = '700 13px "Segoe UI", Arial, sans-serif';
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(String(rank), cx, cy + 0.5);
  ctx.restore();
}

function drawEgoMarker(ctx) {
  const bev = currentFrame.layout.bev;
  const cx = bev.x + bev.size * 0.5;
  const cy = bev.y + bev.size * 0.5;
  const halfWidth = 9;
  const halfHeight = 12;

  ctx.save();
  ctx.globalAlpha = 1;
  ctx.shadowColor = "rgba(0, 0, 0, 0.95)";
  ctx.shadowBlur = 4;
  ctx.beginPath();
  ctx.moveTo(cx, cy - halfHeight);
  ctx.lineTo(cx + halfWidth, cy + halfHeight);
  ctx.lineTo(cx, cy + halfHeight * 0.55);
  ctx.lineTo(cx - halfWidth, cy + halfHeight);
  ctx.closePath();
  ctx.fillStyle = "#ffffff";
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.strokeStyle = canvasColors.rangeRing;
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.restore();
}

function drawNearestObjectsPanel(ctx, nearest) {
  const bev = currentFrame.layout.bev;
  const panelWidth = 380;
  const headerHeight = 56;
  const rowHeight = 50;
  const panelHeight = headerHeight + nearestObjectLimit * rowHeight;
  const canvasRight = currentFrame.layout.width || els.canvas.width;
  const x = canvasRight - panelWidth;
  const y = bev.y + 6;
  const rankColumn = 58;
  const classColumn = 258;

  ctx.save();
  ctx.globalAlpha = 1;
  ctx.shadowColor = "rgba(0, 0, 0, 0.75)";
  ctx.shadowBlur = 8;
  ctx.fillStyle = "rgba(5, 9, 8, 0.9)";
  ctx.fillRect(x, y, panelWidth, panelHeight);
  ctx.shadowBlur = 0;
  ctx.strokeStyle = "rgba(232, 240, 247, 0.92)";
  ctx.lineWidth = 2;
  ctx.strokeRect(x, y, panelWidth, panelHeight);

  ctx.beginPath();
  ctx.moveTo(x, y + headerHeight);
  ctx.lineTo(x + panelWidth, y + headerHeight);
  ctx.moveTo(x + rankColumn, y);
  ctx.lineTo(x + rankColumn, y + panelHeight);
  ctx.moveTo(x + classColumn, y);
  ctx.lineTo(x + classColumn, y + panelHeight);
  for (let row = 1; row < nearestObjectLimit; row++) {
    const rowY = y + headerHeight + row * rowHeight;
    ctx.moveTo(x, rowY);
    ctx.lineTo(x + panelWidth, rowY);
  }
  ctx.stroke();

  ctx.font = '800 24px "Segoe UI", Arial, sans-serif';
  ctx.textBaseline = "middle";
  ctx.fillStyle = "#f4f8fc";
  ctx.textAlign = "center";
  ctx.fillText("#", x + rankColumn * 0.5, y + headerHeight * 0.5);
  ctx.fillText("CLASS", x + (rankColumn + classColumn) * 0.5, y + headerHeight * 0.5);
  ctx.fillText("DIST", x + (classColumn + panelWidth) * 0.5, y + headerHeight * 0.5);

  ctx.font = '700 22px "Segoe UI", Arial, sans-serif';
  for (let row = 0; row < nearestObjectLimit; row++) {
    const det = nearest[row];
    const cy = y + headerHeight + row * rowHeight + rowHeight * 0.5;
    if (!det) {
      ctx.fillStyle = "#788490";
      ctx.textAlign = "center";
      ctx.fillText("-", x + rankColumn * 0.5, cy);
      ctx.fillText("-", x + (rankColumn + classColumn) * 0.5, cy);
      ctx.fillText("-", x + (classColumn + panelWidth) * 0.5, cy);
      continue;
    }
    ctx.fillStyle = "#fff";
    ctx.textAlign = "center";
    ctx.fillText(String(row + 1), x + rankColumn * 0.5, cy);
    ctx.fillStyle = "#fff";
    ctx.textAlign = "left";
    ctx.fillText(classMap[det.class_id] || `class ${det.class_id}`, x + rankColumn + 12, cy);
    ctx.textAlign = "right";
    ctx.fillText(`${det.distance.toFixed(1)} cm`, x + panelWidth - 12, cy);
  }
  ctx.restore();
}

function drawPolygon(ctx, pts) {
  if (!pts.length) return;
  ctx.beginPath();
  ctx.moveTo(pts[0][0], pts[0][1]);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
  ctx.closePath();
  ctx.stroke();
}

function drawLine(ctx, a, b) {
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
}

function drawArrow(ctx, a, b, color, width) {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const angle = Math.atan2(dy, dx);
  const headLen = 10;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  drawLine(ctx, a, b);
  ctx.beginPath();
  ctx.moveTo(b[0], b[1]);
  ctx.lineTo(b[0] - headLen * Math.cos(angle - Math.PI / 6),
             b[1] - headLen * Math.sin(angle - Math.PI / 6));
  ctx.lineTo(b[0] - headLen * Math.cos(angle + Math.PI / 6),
             b[1] - headLen * Math.sin(angle + Math.PI / 6));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function scoreColor(score) {
  const strength = Math.max(0, Math.min(1, Number(score) || 0));
  const g = Math.round(180 + 75 * strength);
  const r = Math.round(80 + 175 * strength);
  return `rgb(${r}, ${g}, 0)`;
}

function imagePointFromEvent(event) {
  const rect = els.canvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * (els.canvas.width / rect.width),
    y: (event.clientY - rect.top) * (els.canvas.height / rect.height),
  };
}

function pointInPoly(point, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i][0], yi = poly[i][1];
    const xj = poly[j][0], yj = poly[j][1];
    const intersect = ((yi > point.y) !== (yj > point.y)) &&
      (point.x < ((xj - xi) * (point.y - yi)) / ((yj - yi) || 1e-9) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

function pointInBox(point, box) {
  return point.x >= box[0] && point.x <= box[2] && point.y >= box[1] && point.y <= box[3];
}

function boxCenterDistance(point, box) {
  const cx = (box[0] + box[2]) * 0.5;
  const cy = (box[1] + box[3]) * 0.5;
  return Math.hypot(point.x - cx, point.y - cy);
}

function hitTest(point) {
  const candidates = [];
  for (const det of activeDetections()) {
    if (pointInPoly(point, det.bev.polygon)) {
      candidates.push({ det, dist: boxCenterDistance(point, det.bev.bbox), scoreBias: det.score + 1 });
    }
    for (const view of det.camera_views) {
      if (pointInBox(point, view.bbox)) {
        candidates.push({ det, dist: boxCenterDistance(point, view.bbox), scoreBias: det.score });
      }
    }
  }
  candidates.sort((a, b) => (a.dist - b.dist) || (b.scoreBias - a.scoreBias));
  return candidates.length ? candidates[0].det : null;
}

function showDetection(det) {
  if (!det) {
    els.summary.textContent = "点击 BEV 或相机画面中的检测框。";
    els.detailTable.innerHTML = "";
    els.rawLine.textContent = "-";
    return;
  }

  const clsName = classMap[det.class_id] || `class ${det.class_id}`;
  els.summary.innerHTML = `<strong>#${det.id}</strong> ${clsName}, score ${det.score.toFixed(3)}`;
  const rows = [
    ["Frame", currentFrame.frame_id],
    ["Detection ID", det.id],
    ["Class", `${det.class_id}${classMap[det.class_id] ? ` (${classMap[det.class_id]})` : ""}`],
    ["Score", det.score.toFixed(6)],
    ["Distance", `${det.distance.toFixed(3)} cm`],
    ["Position", `x=${fmt(det.box.x)}, y=${fmt(det.box.y)}, z=${fmt(det.box.z)}`],
    ["Size", `w=${fmt(det.box.w)}, l=${fmt(det.box.l)}, h=${fmt(det.box.h)}`],
    ["Yaw", fmt(det.box.yaw)],
    ["Visible cameras", det.visible_cameras.length ? det.visible_cameras.join(", ") : "-"],
    ["BEV center", det.bev.center.map(fmt).join(", ")],
    ["BEV polygon", det.bev.polygon.map((p) => `[${p.map(fmt).join(", ")}]`).join(" ")],
  ];
  els.detailTable.innerHTML = rows
    .map(([k, v]) => `<dt>${escapeHtml(String(k))}</dt><dd>${escapeHtml(String(v))}</dd>`)
    .join("");
  els.rawLine.textContent = det.raw;
}

function fmt(value) {
  return Number(value).toFixed(3);
}

function escapeHtml(value) {
  return value.replace(/[&<>"']/g, (ch) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "\"": "&quot;",
    "'": "&#39;",
  }[ch]));
}

function currentFrameIndex() {
  if (!frames.length || !currentFrame) return -1;
  return frames.findIndex((f) => f.id === currentFrame.frame_id);
}

function stepFrame(delta) {
  const idx = currentFrameIndex();
  if (idx < 0) return Promise.resolve();
  const next = Math.max(0, Math.min(frames.length - 1, idx + delta));
  if (next === idx) {
    if (playing && next === frames.length - 1) stopPlayback();
    return Promise.resolve();
  }
  return loadFrame(frames[next].id).catch((err) => {
    setStatus(err.message, true);
    stopPlayback();
  });
}

function prefetchNextFrame() {
  const idx = currentFrameIndex();
  if (idx < 0 || idx + 1 >= frames.length) return;
  getJson(`/api/frame/${frames[idx + 1].id}`)
    .then((data) => Promise.all(data.camera_images.map((item) => loadImage(item.url))))
    .catch(() => {});
}

function playbackDelayMs() {
  const fps = Math.max(1, Math.min(30, Number(els.fpsInput.value) || 10));
  els.fpsInput.value = String(fps);
  return Math.round(1000 / fps);
}

function playbackTick() {
  if (!playing) return;
  stepFrame(1).finally(() => {
    if (playing) playTimer = window.setTimeout(playbackTick, playbackDelayMs());
  });
}

function startPlayback() {
  if (playing) return;
  stopLiveMode();
  playing = true;
  els.playPause.textContent = "暂停";
  playbackTick();
}

function stopPlayback() {
  playing = false;
  els.playPause.textContent = "播放";
  if (playTimer !== null) {
    window.clearTimeout(playTimer);
    playTimer = null;
  }
}

function liveTick() {
  if (!liveMode) return;
  loadLiveFrame()
    .catch((err) => {
      setStatus(err.message, true);
      stopLiveMode();
    })
    .finally(() => {
      if (liveMode) liveTimer = window.setTimeout(liveTick, playbackDelayMs());
    });
}

function startLiveMode() {
  if (liveMode) return;
  stopPlayback();
  liveMode = true;
  els.liveSim.textContent = "停止实时";
  liveTick();
}

function stopLiveMode() {
  liveMode = false;
  els.liveSim.textContent = "实时模拟";
  if (liveTimer !== null) {
    window.clearTimeout(liveTimer);
    liveTimer = null;
  }
}

els.canvas.addEventListener("mousemove", (event) => {
  const det = hitTest(imagePointFromEvent(event));
  const nextHover = det ? det.id : null;
  if (nextHover !== hoverId) {
    hoverId = nextHover;
    drawScene();
  }
});

els.canvas.addEventListener("mouseleave", () => {
  hoverId = null;
  drawScene();
});

els.canvas.addEventListener("click", (event) => {
  const det = hitTest(imagePointFromEvent(event));
  selectedId = det ? det.id : null;
  showDetection(det);
  drawScene();
});

els.frameSelect.addEventListener("change", () => {
  stopPlayback();
  stopLiveMode();
  loadFrame(els.frameSelect.value).catch((err) => setStatus(err.message, true));
});

els.prevFrame.addEventListener("click", () => {
  stopPlayback();
  stopLiveMode();
  stepFrame(-1);
});
els.nextFrame.addEventListener("click", () => {
  stopPlayback();
  stopLiveMode();
  stepFrame(1);
});
els.playPause.addEventListener("click", () => {
  if (playing) stopPlayback();
  else startPlayback();
});
els.liveSim.addEventListener("click", () => {
  if (liveMode) stopLiveMode();
  else startLiveMode();
});
els.refreshFrames.addEventListener("click", () => {
  loadFrames(currentFrame ? currentFrame.frame_id : frameFromUrl()).catch((err) => setStatus(err.message, true));
});

els.selectAllClasses.addEventListener("click", () => {
  for (const classId of Object.keys(classMap)) enabledClasses.add(Number(classId));
  syncClassFilterState();
  clearHiddenSelection();
  drawScene();
});

els.clearAllClasses.addEventListener("click", () => {
  enabledClasses.clear();
  syncClassFilterState();
  clearHiddenSelection();
  drawScene();
});

els.scoreThreshold.addEventListener("input", () => {
  els.thresholdValue.textContent = Number(els.scoreThreshold.value).toFixed(3);
  clearHiddenSelection();
  drawScene();
});

window.addEventListener("resize", drawScene);

buildClassFilters();
const initialLoad = liveFromUrl()
  ? Promise.resolve().then(() => startLiveMode())
  : loadFrames(frameFromUrl());
initialLoad
  .catch((err) => setStatus(err.message, true));
