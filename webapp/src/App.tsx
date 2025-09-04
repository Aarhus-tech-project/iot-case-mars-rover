import { useEffect, useRef, useState } from "react";
import * as signalR from "@microsoft/signalr";
import {
  Application,
  Assets,
  Container,
  Graphics,
  Sprite,
  Texture,
  TextureStyle,
  Text,
} from "pixi.js";

// --- crisp pixels everywhere ---
TextureStyle.defaultOptions.scaleMode = "nearest";

type GridMetaDto = {
  width: number;
  height: number;
  cellSizeM: number;
  seq: number;
  tMonoNs: number;
  url: string;
};
type PoseDto = { xM: number; yM: number; rotDeg: number; seq: number };
type LidarPointDto = { xM: number; yM: number };
type LidarDto = { points: LidarPointDto[]; seq: number };

const HUB_URL = "http://localhost:5080/telemetryHub";
const API_PREFIX = "http://localhost:5080";
const MAX_LIDAR_POINTS = 5000;

// visuals
const BALL_R = 1;
const BALL_FILL = 0x001aff;
const BALL_FILL_A = 0.98;
const BALL_OUT = 0x001aff;
const BALL_OUT_A = 0.9;
const BALL_OUT_W = 1.25;

// zoom/pan limits
const ZOOM_STEP = 1.2;
const MIN_SCALE = 0.05;
const MAX_SCALE = 20;

// grid-lines
const MAX_GRID_LINES = 300; // cap to avoid overdraw

export default function App() {
  const [status, setStatus] = useState("disconnected");
  const [grid, setGrid] = useState<GridMetaDto | null>(null);
  const [pose, setPose] = useState<PoseDto | null>(null);
  const [lidar, setLidar] = useState<LidarDto | null>(null);

  const [dimGrid, setDimGrid] = useState(false);
  const [showGridLines, setShowGridLines] = useState(true);
  const [measureMode, setMeasureMode] = useState(false);
  const [cellsPerLine, setCellsPerLine] = useState(20);

  const mountRef = useRef<HTMLDivElement | null>(null);

  const appRef = useRef<Application | null>(null);
  const worldRef = useRef<Container | null>(null);
  const gridSpriteRef = useRef<Sprite | null>(null);
  const gridLinesRef = useRef<Graphics | null>(null);
  const lidarDotsRef = useRef<Graphics | null>(null);
  const poseGfxRef = useRef<Graphics | null>(null);
  const measureGfxRef = useRef<Graphics | null>(null);
  const measureLabelRef = useRef<Text | null>(null);

  // camera + fit baseline
  const fitScaleRef = useRef(1);
  const prevGridSizeRef = useRef<{ w: number; h: number } | null>(null);
  const cameraRef = useRef<{ scale: number; cx: number; cy: number; userTouched: boolean }>({
    scale: 1,
    cx: 0,
    cy: 0,
    userTouched: false,
  });

  // measurement points in *world pixels* (not meters)
  const measureARef = useRef<{ x: number; y: number } | null>(null);
  const measureBRef = useRef<{ x: number; y: number } | null>(null);

  const toAbsolute = (u: string) =>
    u.startsWith("http://") || u.startsWith("https://") || u.startsWith("//")
      ? u
      : u.startsWith("/")
      ? `${API_PREFIX}${u}`
      : `${API_PREFIX}/${u}`;

  function installScrollGuards() {
    // Stop touch scroll bubbling to page (Safari rubber-band)
    const stopIfInside = (e: Event) => {
      const host = mountRef.current;
      if (host && host.contains(e.target as Node)) e.preventDefault();
    };
    window.addEventListener("touchmove", stopIfInside, { passive: false, capture: true });
    const canvas = appRef.current?.canvas;
    if (canvas) (canvas.style as any).touchAction = "none";
    return () => {
      window.removeEventListener("touchmove", stopIfInside, { capture: true } as any);
    };
  }

  // -----------------------------------------
  // PIXI mount
  // -----------------------------------------
  useEffect(() => {
    if (!mountRef.current) return;
    let destroyed = false;
    let removeGuards: (() => void) | undefined;
    let removeNav: (() => void) | undefined;

    (async () => {
      const app = new Application();
      await app.init({
        background: 0x000000,
        antialias: false,
        resizeTo: mountRef.current!,
        resolution: Math.max(1, window.devicePixelRatio || 1),
        autoDensity: true,
      });
      if (destroyed) {
        app.destroy(true);
        return;
      }

      const world = new Container();
      world.sortableChildren = true;
      app.stage.addChild(world);

      const gridSprite = new Sprite(); // texture we control (from canvas)
      gridSprite.zIndex = 0;
      world.addChild(gridSprite);

      const gridLines = new Graphics(); // optional cell lines overlay
      gridLines.zIndex = 0.5;
      world.addChild(gridLines);

      const lidarDots = new Graphics();
      lidarDots.zIndex = 1;
      world.addChild(lidarDots);

      const poseGfx = new Graphics();
      poseGfx.zIndex = 2;
      world.addChild(poseGfx);

      const measureGfx = new Graphics();
      measureGfx.zIndex = 3;
      world.addChild(measureGfx);

      const measureLabel = new Text({
        text: "",
        style: { fill: 0xffffff, fontSize: 12, fontFamily: "ui-monospace, Menlo, monospace" },
      });
      measureLabel.zIndex = 3.5;
      world.addChild(measureLabel);

      // DOM
      mountRef.current!.appendChild(app.canvas);
      app.canvas.style.cursor = "grab";
      app.canvas.oncontextmenu = (e) => e.preventDefault();

      appRef.current = app;
      worldRef.current = world;
      gridSpriteRef.current = gridSprite;
      gridLinesRef.current = gridLines;
      lidarDotsRef.current = lidarDots;
      poseGfxRef.current = poseGfx;
      measureGfxRef.current = measureGfx;
      measureLabelRef.current = measureLabel;

      removeGuards = installScrollGuards();
      removeNav = attachNavigation();

      // initial camera baseline (no fit yet; will happen after first texture)
      computeFitScale(); // so min zoom clamps right even before first texture
      drawAll();
    })();

    const onResize = () => {
      // preserve camera on resize if user touched; otherwise keep it fitted
      const cam = getCamera();
      computeFitScale();
      if (cameraRef.current.userTouched && cam) {
        applyCamera(cam.scale, cam.cx, cam.cy);
      } else {
        fitToView();
      }
      drawAll();
    };
    window.addEventListener("resize", onResize);

    return () => {
      destroyed = true;
      removeNav?.();
      removeGuards?.();
      window.removeEventListener("resize", onResize);
      if (appRef.current) appRef.current.destroy(true, { children: true, texture: true });
      appRef.current = null;
      worldRef.current = null;
      gridSpriteRef.current = null;
      gridLinesRef.current = null;
      lidarDotsRef.current = null;
      poseGfxRef.current = null;
      measureGfxRef.current = null;
      measureLabelRef.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // -----------------------------------------
  // SignalR
  // -----------------------------------------
  useEffect(() => {
    const conn = new signalR.HubConnectionBuilder().withUrl(HUB_URL).withAutomaticReconnect().build();

    conn.on("Grid", (dto: GridMetaDto) => setGrid(dto));
    conn.on("Pose", (dto: any) =>
      setPose({
        xM: dto.xM ?? dto.xm,
        yM: dto.yM ?? dto.ym,
        rotDeg: dto.rotdeg,
        seq: dto.seq,
      }),
    );
    conn.on("Lidar", (dto: any) => {
      const pts: LidarPointDto[] = [];
      for (const p of dto.points ?? []) {
        const x = Number(p.xM ?? p.xm);
        const y = Number(p.yM ?? p.ym);
        if (Number.isFinite(x) && Number.isFinite(y)) pts.push({ xM: x, yM: y });
      }
      setLidar({ points: pts.slice(-MAX_LIDAR_POINTS), seq: Number(dto.seq) || 0 });
    });

    conn
      .start()
      .then(() => setStatus("connected"))
      .catch((e) => {
        console.error(e);
        setStatus("error");
      });
    return () => {
      conn.stop();
    };
  }, []);

  // -----------------------------------------
  // Load PNG -> draw to canvas -> Texture.from(canvas)
  // Preserve camera unless grid size changed and user hasn’t interacted.
  // -----------------------------------------
  useEffect(() => {
    if (!grid?.url || !gridSpriteRef.current) return;
    const base = toAbsolute(grid.url);
    const url = base.includes("?") ? `${base}&seq=${grid.seq ?? 0}` : `${base}?seq=${grid.seq ?? 0}`;
    let cancelled = false;

    (async () => {
      try {
        const sprite = gridSpriteRef.current!;
        const app = appRef.current!;
        const world = worldRef.current!;

        // capture current camera before swapping texture
        const camBefore = getCamera();

        const tex = await makeTextureFromUrl(url, grid.width, grid.height);
        if (cancelled) return;

        // set new texture + dims
        sprite.texture = tex;
        sprite.width = grid.width;
        sprite.height = grid.height;
        sprite.position.set(0, 0);
        sprite.alpha = dimGrid ? 0.5 : 1.0;

        // recompute fit baseline for new content size
        computeFitScale();

        const prevSize = prevGridSizeRef.current;
        const sizeChanged = !prevSize || prevSize.w !== grid.width || prevSize.h !== grid.height;
        prevGridSizeRef.current = { w: grid.width, h: grid.height };

        // If the user already moved/zoomed OR size didn't change -> restore camera
        // Else (first time or size change) -> fit to view
        if (cameraRef.current.userTouched && camBefore) {
          // restore same center & scale; clamp to new bounds
          const { scale, cx, cy } = clampCameraToBounds(camBefore.scale, camBefore.cx, camBefore.cy);
          applyCamera(scale, cx, cy);
        } else if (!sizeChanged && camBefore) {
          const { scale, cx, cy } = clampCameraToBounds(camBefore.scale, camBefore.cx, camBefore.cy);
          applyCamera(scale, cx, cy);
        } else {
          fitToView();
        }

        drawAll();
      } catch (e) {
        console.error("grid texture build fail", e);
      }
    })();

    return () => {
      cancelled = true;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [grid?.url, grid?.seq, grid?.width, grid?.height, dimGrid]);

  // -----------------------------------------
  // redraw on data changes
  // -----------------------------------------
  useEffect(() => {
    drawAll();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pose, lidar, grid?.cellSizeM, grid?.width, grid?.height, showGridLines, measureMode]);

  // -----------------------------------------
  // Camera utilities
  // -----------------------------------------
  function clamp(n: number, a: number, b: number) {
    return Math.max(a, Math.min(b, n));
  }

  function computeFitScale() {
    const app = appRef.current;
    const sprite = gridSpriteRef.current;
    if (!app || !sprite) return;
    const gw = sprite.width || 1;
    const gh = sprite.height || 1;
    const rw = app.renderer.width;
    const rh = app.renderer.height;
    fitScaleRef.current = Math.min(rw / gw, rh / gh);
  }

  function fitToView() {
    const app = appRef.current,
      world = worldRef.current;
    const sprite = gridSpriteRef.current;
    if (!app || !world || !sprite) return;
    computeFitScale();
    const gw = sprite.width || 1,
      gh = sprite.height || 1;
    const rw = app.renderer.width,
      rh = app.renderer.height;

    world.pivot.set(gw / 2, gh / 2);
    world.scale.set(fitScaleRef.current);
    world.position.set(rw / 2, rh / 2);

    // Reset camera baseline (not "touched")
    cameraRef.current = {
      scale: fitScaleRef.current,
      cx: gw / 2,
      cy: gh / 2,
      userTouched: false,
    };
  }

  function viewportCenter() {
    const app = appRef.current!;
    return { sx: app.renderer.width / 2, sy: app.renderer.height / 2 };
  }

  function getCamera() {
    const world = worldRef.current;
    if (!world) return null;
    const { sx, sy } = viewportCenter();
    const local = world.toLocal({ x: sx, y: sy });
    return { scale: world.scale.x, cx: local.x, cy: local.y };
  }

  function applyCamera(scale: number, cx: number, cy: number) {
    const world = worldRef.current!;
    const { sx, sy } = viewportCenter();
    world.pivot.set(cx, cy);
    world.position.set(sx, sy);
    world.scale.set(scale);
    cameraRef.current.scale = scale;
    cameraRef.current.cx = cx;
    cameraRef.current.cy = cy;
  }

  function clampCameraToBounds(scale: number, cx: number, cy: number) {
    const sprite = gridSpriteRef.current!;
    const clampedCx = clamp(cx, 0, sprite.width);
    const clampedCy = clamp(cy, 0, sprite.height);
    // also clamp scale between min/max using current baseline
    const minScale = Math.min(MIN_SCALE * fitScaleRef.current, fitScaleRef.current * 0.05);
    const clampedScale = clamp(scale, minScale, MAX_SCALE);
    return { scale: clampedScale, cx: clampedCx, cy: clampedCy };
  }

  function markUserTouched() {
    cameraRef.current.userTouched = true;
  }

  function oneToOne() {
    const app = appRef.current,
      world = worldRef.current;
    const sprite = gridSpriteRef.current;
    if (!app || !world || !sprite) return;
    const gw = sprite.width || 1,
      gh = sprite.height || 1;
    const { sx, sy } = viewportCenter();
    world.pivot.set(gw / 2, gh / 2);
    world.scale.set(1);
    world.position.set(sx, sy);
    cameraRef.current = { scale: 1, cx: gw / 2, cy: gh / 2, userTouched: true };
  }

  function attachNavigation() {
    const app = appRef.current!;
    const world = worldRef.current!;
    const canvas = app.canvas;

    const min = () => Math.min(MIN_SCALE * fitScaleRef.current, fitScaleRef.current * 0.05);
    const max = () => MAX_SCALE;

    // wheel zoom
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      markUserTouched();

      const rect = canvas.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;

      const local = world.toLocal({ x: sx, y: sy });
      world.pivot.set(local.x, local.y);
      world.position.set(sx, sy);

      const dir = e.deltaY < 0 ? 1 : -1;
      const base = world.scale.x;
      const next = clamp(base * (dir > 0 ? ZOOM_STEP : 1 / ZOOM_STEP), min(), max());
      world.scale.set(next);

      // store camera
      const cam = getCamera();
      if (cam) applyCamera(next, cam.cx, cam.cy);
    };
    canvas.addEventListener("wheel", onWheel, { passive: false });

    // pinch zoom (Safari gesture*)
    let pinchStartScale = 1;
    const onGestureStart = (e: any) => {
      e.preventDefault();
      markUserTouched();
      pinchStartScale = world.scale.x;

      const rect = canvas.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const local = world.toLocal({ x: sx, y: sy });
      world.pivot.set(local.x, local.y);
      world.position.set(sx, sy);
    };
    const onGestureChange = (e: any) => {
      e.preventDefault();
      const target = clamp(pinchStartScale * e.scale, min(), max());
      world.scale.set(target);
      const cam = getCamera();
      if (cam) applyCamera(target, cam.cx, cam.cy);
    };
    const onGestureEnd = (e: any) => {
      e.preventDefault();
    };
    canvas.addEventListener("gesturestart", onGestureStart as any, { passive: false });
    canvas.addEventListener("gesturechange", onGestureChange as any, { passive: false });
    canvas.addEventListener("gestureend", onGestureEnd as any, { passive: false });

    // right/middle drag to pan; left-click used for measure
    let dragging = false;
    let startX = 0,
      startY = 0;
    let wStartX = 0,
      wStartY = 0;

    const onDown = (e: PointerEvent) => {
      if (e.button === 0 && measureMode) {
        handleMeasureClick(e);
        return;
      }
      if (e.button !== 2 && e.button !== 1) return;
      markUserTouched();
      dragging = true;
      startX = e.clientX;
      startY = e.clientY;
      wStartX = world.position.x;
      wStartY = world.position.y;
      canvas.style.cursor = "grabbing";
      (e.target as HTMLElement).setPointerCapture?.(e.pointerId);
    };
    const onMove = (e: PointerEvent) => {
      if (!dragging) return;
      const dx = e.clientX - startX;
      const dy = e.clientY - startY;
      world.position.set(wStartX + dx, wStartY + dy);
      const cam = getCamera();
      if (cam) applyCamera(cam.scale, cam.cx, cam.cy);
    };
    const onUp = () => {
      if (!dragging) return;
      dragging = false;
      canvas.style.cursor = "grab";
    };

    canvas.addEventListener("pointerdown", onDown);
    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);

    // cleanup
    return () => {
      canvas.removeEventListener("wheel", onWheel);
      canvas.removeEventListener("gesturestart", onGestureStart as any);
      canvas.removeEventListener("gesturechange", onGestureChange as any);
      canvas.removeEventListener("gestureend", onGestureEnd as any);
      canvas.removeEventListener("pointerdown", onDown);
      window.removeEventListener("pointermove", onMove);
      window.removeEventListener("pointerup", onUp);
    };
  }

  // -----------------------------------------
  // world-space helpers (bottom-left origin)
  // -----------------------------------------
  function toPx(xm: number) {
    return xm / (grid?.cellSizeM || 1e-6);
  }
  function toPy(ym: number) {
    return (grid?.height || 0) - ym / (grid?.cellSizeM || 1e-6);
  }

  // -----------------------------------------
  // measure tool
  // -----------------------------------------
  function handleMeasureClick(e: PointerEvent) {
    const world = worldRef.current!;
    const rect = (appRef.current!.canvas as HTMLCanvasElement).getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const local = world.toLocal({ x: sx, y: sy }); // local in *grid pixel* space

    if (!measureARef.current) {
      measureARef.current = { x: local.x, y: local.y };
      measureBRef.current = null;
    } else if (!measureBRef.current) {
      measureBRef.current = { x: local.x, y: local.y };
    } else {
      // third click resets
      measureARef.current = { x: local.x, y: local.y };
      measureBRef.current = null;
    }
    drawAll();
  }

  // -----------------------------------------
  // draw
  // -----------------------------------------
  function drawAll() {
    const world = worldRef.current;
    const gridSprite = gridSpriteRef.current;
    const gridLines = gridLinesRef.current;
    const dots = lidarDotsRef.current;
    const poseG = poseGfxRef.current;
    const mG = measureGfxRef.current;
    const mText = measureLabelRef.current;

    if (!gridSprite || !gridLines || !dots || !poseG || !world || !mG || !mText || !grid) return;

    // ensure z-order
    gridSprite.zIndex = 0;
    gridLines.zIndex = 0.5;
    dots.zIndex = 1;
    poseG.zIndex = 2;
    mG.zIndex = 3;
    mText.zIndex = 3.5;
    world.sortChildren();
    setCellsPerLine(1 / grid.cellSizeM);

    // grid lines
    gridLines.clear();
    if (showGridLines) {
      const app = appRef.current!;
      const world = worldRef.current!;
      const w = gridSprite.width | 0, h = gridSprite.height | 0;
    
      // keep strokes 1px on screen
      const strokeWorld = 1 / world.scale.x;
      gridLines.setStrokeStyle({ width: strokeWorld, color: 0x222222, alpha: 0.9 });
    
      // spacing in world pixels (map cells). Each pixel = 1 cell = cellSizeM meters.
      const stepPx = Math.max(1, Math.floor(cellsPerLine));
    
      // figure out visible world-rect (convert screen -> world)
      const tl = world.toLocal({ x: 0, y: 0 });
      const br = world.toLocal({ x: app.renderer.width, y: app.renderer.height });
      const minX = clamp(Math.floor(Math.min(tl.x, br.x)), 0, w);
      const maxX = clamp(Math.ceil(Math.max(tl.x, br.x)), 0, w);
      const minY = clamp(Math.floor(Math.min(tl.y, br.y)), 0, h);
      const maxY = clamp(Math.ceil(Math.max(tl.y, br.y)), 0, h);
    
      // start on a multiple of step
      let x = Math.ceil(minX / stepPx) * stepPx;
      for (; x <= maxX; x += stepPx) {
        gridLines.moveTo(x, minY);
        gridLines.lineTo(x, maxY);
      }
    
      let y = Math.ceil(minY / stepPx) * stepPx;
      for (; y <= maxY; y += stepPx) {
        gridLines.moveTo(minX, y);
        gridLines.lineTo(maxX, y);
      }
    
      gridLines.stroke();
    }

    // lidar dots
    dots.clear();
    const pts = (lidar?.points ?? []).slice(-MAX_LIDAR_POINTS);
    if (pts.length) {
      if (BALL_OUT_W > 0) dots.setStrokeStyle({ width: BALL_OUT_W, color: BALL_OUT, alpha: BALL_OUT_A });
      else dots.setStrokeStyle({ width: 0, color: 0x000000, alpha: 0 });
      dots.setFillStyle({ color: BALL_FILL, alpha: BALL_FILL_A });

      for (const p of pts) {
        if (!Number.isFinite(p.xM) || !Number.isFinite(p.yM)) continue;
        dots.circle(toPx(p.xM), toPy(p.yM), BALL_R);
      }
      dots.fill();
      if (BALL_OUT_W > 0) dots.stroke();
    }

    // pose + frame
    poseG.clear();
    poseG.setStrokeStyle({ width: 1, color: 0x44ff44, alpha: 0.4 });
    poseG.rect(0, 0, grid.width, grid.height);
    poseG.stroke();

    if (pose && Number.isFinite(pose.xM) && Number.isFinite(pose.yM) && Number.isFinite(pose.rotDeg)) {
      const rx = toPx(pose.xM),
        ry = toPy(pose.yM);

      // crosshair
      poseG.setStrokeStyle({ width: 2, color: 0xffffff, alpha: 0.9 });
      poseG.moveTo(rx - 10, ry);
      poseG.lineTo(rx + 10, ry);
      poseG.moveTo(rx, ry - 10);
      poseG.lineTo(rx, ry + 10);
      poseG.stroke();

      // arrow
      poseG.setStrokeStyle({ width: 3, color: 0xffcc00, alpha: 1 });
      poseG.moveTo(rx, ry);
      poseG.stroke();

      poseG.setFillStyle({ color: 0xffcc00, alpha: 1 });
      poseG.fill();

      poseG.circle(rx, ry, 4).fill();
    }

    // measure overlay
    mG.clear();
    mText.text = "";
    if (measureMode && (measureARef.current || measureBRef.current)) {
      const A = measureARef.current!;
      const B = measureBRef.current;
      mG.setStrokeStyle({ width: 2, color: 0xff66ff, alpha: 1 });
      mG.setFillStyle({ color: 0xff66ff, alpha: 1 });

      if (A && !B) {
        mG.circle(A.x, A.y, 3);
        mG.fill();
        mText.text = "Pick second point…";
        mText.position.set(A.x + 8, A.y - 8);
      } else if (A && B) {
        mG.circle(A.x, A.y, 3);
        mG.circle(B.x, B.y, 3);
        mG.fill();
        mG.moveTo(A.x, A.y);
        mG.lineTo(B.x, B.y);
        mG.stroke();

        const dxPix = B.x - A.x;
        const dyPix = B.y - A.y;
        const pixDist = Math.hypot(dxPix, dyPix);
        const meters = pixDist * (grid.cellSizeM || 0);
        mText.text = `${meters.toFixed(3)} m`;
        mText.position.set((A.x + B.x) / 2 + 8, (A.y + B.y) / 2 - 8);
      }
    }
  }

  // -----------------------------------------
  // Build a Texture from URL by drawing onto a canvas (nearest-neighbor)
  // -----------------------------------------
  async function makeTextureFromUrl(url: string, w: number, h: number): Promise<Texture> {
    // Use Pixi loader so caching / security model matches the rest
    const tex = await Assets.load(url);
    // Try to get the underlying source (ImageBitmap/HTMLImageElement/etc.)
    const src: any = tex?.source?.resource?.source ?? tex?.source?.resource ?? tex?.source;

    // Paint into a canvas at target size
    const canvas = document.createElement("canvas");
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext("2d", { willReadFrequently: true })!;
    ctx.imageSmoothingEnabled = false;

    if (src instanceof ImageBitmap) {
      ctx.drawImage(src, 0, 0, w, h);
    } else if (
      src instanceof HTMLImageElement ||
      src instanceof HTMLCanvasElement ||
      src instanceof HTMLVideoElement
    ) {
      ctx.drawImage(src as any, 0, 0, w, h);
    } else {
      // fallback fetch→bitmap
      const res = await fetch(url);
      const blob = await res.blob();
      const bmp = await createImageBitmap(blob);
      ctx.drawImage(bmp, 0, 0, w, h);
    }

    const out = Texture.from(canvas);
    // paranoia: enforce nearest on this texture too
    // @ts-ignore Pixi v8: .source.style exists
    if ((out as any).source?.style) (out as any).source.style.scaleMode = "nearest";
    return out;
  }

  return (
    <div
      style={{
        height: "100vh",
        width: "100vw",
        background: "#0b0b0b",
        color: "#ddd",
        display: "flex",
        flexDirection: "column",
        overscrollBehavior: "none",
      }}
    >
      <div
        style={{
          padding: 8,
          fontFamily: "ui-monospace, Menlo, monospace",
          fontSize: 12,
          display: "flex",
          gap: 12,
          alignItems: "center",
          flexWrap: "wrap",
        }}
      >
        <span>Status: {status}</span>
        <span>Grid: {grid ? `${grid.width}×${grid.height} @ ${grid.cellSizeM}m` : "—"}</span>
        <span>
          Pose: {pose ? `x=${pose.xM.toFixed(2)} y=${pose.yM.toFixed(2)}` : "—"}
        </span>
        <span>Lidar: {lidar?.points?.length ?? 0}</span>

        <label style={{ display: "inline-flex", alignItems: "center", gap: 6, cursor: "pointer" }}>
          <input type="checkbox" checked={dimGrid} onChange={(e) => setDimGrid(e.target.checked)} />
          Dim grid
        </label>

        <label style={{ display: "inline-flex", alignItems: "center", gap: 6, cursor: "pointer" }}>
          <input
            type="checkbox"
            checked={showGridLines}
            onChange={(e) => {
              setShowGridLines(e.target.checked);
            }}
          />
          Grid lines
        </label>

        <label style={{ display: "inline-flex", alignItems: "center", gap: 6, cursor: "pointer" }}>
          <input
            type="checkbox"
            checked={measureMode}
            onChange={(e) => {
              measureARef.current = null;
              measureBRef.current = null;
              setMeasureMode(e.target.checked);
            }}
          />
          Measure
        </label>

        <button
          onClick={() => {
            measureARef.current = null;
            measureBRef.current = null;
            drawAll();
          }}
          style={{
            padding: "4px 8px",
            background: "#202020",
            color: "#ddd",
            border: "1px solid #444",
            borderRadius: 4,
          }}
        >
          Clear measure
        </button>

        <button
          onClick={fitToView}
          style={{ padding: "4px 8px", background: "#202020", color: "#ddd", border: "1px solid #444", borderRadius: 4 }}
        >
          Fit
        </button>
        <button
          onClick={oneToOne}
          style={{ padding: "4px 8px", background: "#202020", color: "#ddd", border: "1px solid #444", borderRadius: 4 }}
        >
          1:1
        </button>
      </div>
      <div ref={mountRef} style={{ flex: 1, overflow: "hidden" }} />
    </div>
  );
}