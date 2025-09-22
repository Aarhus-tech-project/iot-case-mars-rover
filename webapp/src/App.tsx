import { useEffect, useRef, useState } from "react";
import * as signalR from "@microsoft/signalr";
import { Application, Container, Graphics, Sprite, Assets } from "pixi.js";

type GridMetaDto = {
  width: number;
  height: number;
  cellSizeM: number;
  seq: number;
  url: string;
};
type LidarPointDto = { xM: number; yM: number };
type LidarDto = { points: LidarPointDto[]; seq: number };
type ImuDto = {
  roverTimeNs: number;
  quaternionW: number; quaternionX: number; quaternionY: number; quaternionZ: number;
  accelX: number; accelY: number; accelZ: number;
  gyroX: number; gyroY: number; gyroZ: number;
  magX: number; magY: number; magZ: number;
  heading: number; roll: number; pitch: number;
  linearAccelX: number; linearAccelY: number; linearAccelZ: number;
  temperatureC: number;
  calibSys: number; calibGyro: number; calibAccel: number; calibMag: number;
};


const HUB_URL = "http://localhost:5080/telemetryHub";
const COMMAND_HUB_URL = "http://localhost:5080/commandHub";
const API_PREFIX = "http://localhost:5080";
const MAX_LIDAR_POINTS = 5000;
const HELP_MESSAGE = [
        "Available commands:",
        "- clear: Clear terminal",
        "- help: Show command list",
        "- forward <cm>",
        "- reverse <cm>",
        "- rot <angle>: (-) right, (+) left"
      ].join("\n");

type CommandEntry = {
  type: "user" | "server";
  text: string;
};

export default function App() {
  const [status, setStatus] = useState("disconnected");
  const [commandHubStatus, setCommandHubStatus] = useState<"disconnected" | "connecting" | "connected" | "error">("disconnected");
  const [grid, setGrid] = useState<GridMetaDto | null>(null);
  const [lidar, setLidar] = useState<LidarDto | null>(null);
  const [gridUrl, setGridUrl] = useState("");
  const [gridBust, setGridBust] = useState(0);
  const prevGridKeyRef = useRef<string | null>(null);
  const [imu, setImu] = useState<ImuDto | null>(null);

  const [command, setCommand] = useState("");
  const [history, setHistory] = useState<CommandEntry[]>(["Welcome to the Mars Rover Terminal! Type 'help' for a list of commands."].map(text => ({ type: "server", text })));

  const mountRef = useRef<HTMLDivElement | null>(null);
  const appRef = useRef<Application | null>(null);
  const gridSpriteRef = useRef<Sprite | null>(null);
  const lidarDotsRef = useRef<Graphics | null>(null);
  const commandConnRef = useRef<signalR.HubConnection | null>(null);
  

  // Setup Pixi
  useEffect(() => {
    if (!mountRef.current) return;
    const app = new Application();
    (async () => {
      await app.init({
        background: 0x000000,
        resizeTo: mountRef.current!,
      });
      mountRef.current!.appendChild(app.canvas);

      const world = new Container();
      app.stage.addChild(world);

      const gridSprite = new Sprite();
      world.addChild(gridSprite);

      const lidarDots = new Graphics();
      world.addChild(lidarDots);

      appRef.current = app;
      gridSpriteRef.current = gridSprite;
      lidarDotsRef.current = lidarDots;
    })(); 

    return () => {
      appRef.current?.destroy(true);
      appRef.current = null;
    };
  }, []);

  // SignalR connection
  useEffect(() => {
    const conn = new signalR.HubConnectionBuilder()
      .withUrl(HUB_URL)
      .withAutomaticReconnect()
      .build();

    conn.on("Grid", (dto: GridMetaDto) => {
      setGrid(dto);
      setGridBust(b => b + 1);
    });
    conn.on("Lidar", (dto: any) => {
      const pts: LidarPointDto[] = [];
      for (const p of dto.points ?? []) {
        const x = Number(p.xM ?? p.xm);
        const y = Number(p.yM ?? p.ym);
        if (Number.isFinite(x) && Number.isFinite(y)) pts.push({ xM: x, yM: y });
      }
      setLidar({ points: pts.slice(-MAX_LIDAR_POINTS), seq: Number(dto.seq) || 0 });
    });

    conn.on("Imu", (dto: ImuDto) => {
      setImu(dto);
    });

    conn.start().then(() => setStatus("connected")).catch(() => setStatus("error"));

    return () => {
      conn.stop();
    };
  }, []);

  // Command SignalR connection
  useEffect(() => {
    const conn = new signalR.HubConnectionBuilder()
      .withUrl(COMMAND_HUB_URL)
      .withAutomaticReconnect()
      .build();

    conn.on("CommandReply", (status: string, message: string) => {
      setHistory((prev) => [
        ...prev,
        { type: "server", text: `[${status}] ${message}` }
      ]);
    });

    conn.onclose(() => setCommandHubStatus("disconnected"));
    conn.onreconnecting(() => setCommandHubStatus("connecting"));
    conn.onreconnected(() => setCommandHubStatus("connected"));

    setCommandHubStatus("connecting");

    conn.start()
      .then(() => {
        setCommandHubStatus("connected");
        console.log("CommandHub connected");
      })
      .catch((err) => {
        console.error("CommandHub error", err);
        setCommandHubStatus("error");
      });

    commandConnRef.current = conn;

    return () => { conn.stop(); };
  }, []);

  function makeGridUrl(dto: GridMetaDto, bust: number): string {
    const base = dto.url.startsWith("http") ? dto.url : `${API_PREFIX}${dto.url}`;
    const seq = Number.isFinite(dto.seq) ? dto.seq : 0;
    // both seq (server version) and bust (message count) → unique per update
    return `${base}?seq=${seq}&bust=${bust}`;
  }

  // Load grid image
  useEffect(() => {
    if (!grid || !gridSpriteRef.current) return;
    setGridUrl(makeGridUrl(grid, gridBust));
    (async () => {
      const tex = await Assets.load(gridUrl);
      const sprite = gridSpriteRef.current!;
      sprite.texture = tex;
      sprite.width = grid.width;
      sprite.height = grid.height;
    })();
  }, [grid]);

  // Draw lidar dots
  useEffect(() => {
    if (!lidar || !grid || !lidarDotsRef.current) return;
    const dots = lidarDotsRef.current;
    dots.clear();
    dots.fill({ color: 0x00aaff, alpha: 0.9 });
    for (const p of lidar.points) {
      const x = p.xM / grid.cellSizeM;
      const y = grid.height - p.yM / grid.cellSizeM;
      dots.circle(x, y, 1);
    }
    dots.fill();
  }, [lidar, grid]);

  // --- Handle command submit ---
  const sendCommand = () => {
    const trimmedCommand = command.trim();

    if (!trimmedCommand) return;

    // Handle special "clear" command
    if (trimmedCommand.toLowerCase() === "clear") {
      setHistory([]);
      setCommand("");
      return;
    }

    if (trimmedCommand === "help") {
      setHistory((prev) => [
        ...prev,
        { type: "user", text: command.trim() },
        { type: "server", text: HELP_MESSAGE }
      ]);

      setCommand("");
      return;
    }

    // Add command to history
    setHistory((prev) => [...prev, { type: "user", text: trimmedCommand }]);

    // Handle disconnected state
    if (commandHubStatus !== "connected" || !commandConnRef.current) {
      setHistory((prev) => [
        ...prev,
        { type: "server", text: "[error] CommandHub not connected" }
      ]);
      setCommand("");
      return;
    }

    // Send command to server
    commandConnRef.current
      .invoke("SendCommand", trimmedCommand)
      .catch((err) =>
        setHistory((prev) => [
          ...prev,
          { type: "server", text: `[error] ${err.message}` }
        ])
      );

    setCommand("");
  };


  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "Enter") sendCommand();
  };

  return (
    <div className="app">
      <div className="main">
        <div className="status-bar">
          Hub Status: {status} | CommandHub: {commandHubStatus}
        </div>

        <div className="imu-panel">
          {imu ? (
            <pre>
              Time: {imu.roverTimeNs}
              {"\n"}Quaternion: [{imu.quaternionW.toFixed(2)}, {imu.quaternionX.toFixed(2)}, {imu.quaternionY.toFixed(2)}, {imu.quaternionZ.toFixed(2)}]
              {"\n"}Accel: [{imu.accelX.toFixed(2)}, {imu.accelY.toFixed(2)}, {imu.accelZ.toFixed(2)}] m/s²
              {"\n"}Gyro: [{imu.gyroX.toFixed(2)}, {imu.gyroY.toFixed(2)}, {imu.gyroZ.toFixed(2)}] °/s
              {"\n"}Mag: [{imu.magX.toFixed(2)}, {imu.magY.toFixed(2)}, {imu.magZ.toFixed(2)}] µT
              {"\n"}Euler: heading={imu.heading.toFixed(1)}°, roll={imu.roll.toFixed(1)}°, pitch={imu.pitch.toFixed(1)}°
              {"\n"}Linear Accel: [{imu.linearAccelX.toFixed(2)}, {imu.linearAccelY.toFixed(2)}, {imu.linearAccelZ.toFixed(2)}] m/s²
              {"\n"}Temperature: {imu.temperatureC.toFixed(1)} °C
              {"\n"}Calibration: sys={imu.calibSys}, gyro={imu.calibGyro}, accel={imu.calibAccel}, mag={imu.calibMag}
            </pre>
          ) : (
            <div>No IMU data yet</div>
          )}
        </div>

        <div ref={mountRef} className="canvas-container" />
      </div>

      <div className="sidebar">
        <div className="sidebar-icon">
          <img src="/Mars-Icon.png" alt="Mars Icon" />
        </div>
        <div className="sidebar-header">Rover Terminal</div>
        <div className="sidebar-content">
          {history.map((entry, i) => (
            <div key={i} className={entry.type}>
              {entry.text.includes('\n') ? (
                <pre>{entry.text}</pre>
              ) : (
                entry.text
              )}
            </div>
          ))}
        </div>
        <div className="sidebar-footer">
          <input
            type="text"
            placeholder="Enter command..."
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            onKeyDown={handleKeyPress}
          />
          <button onClick={sendCommand}>Send</button>
        </div>
      </div>
    </div>
  );
}