const canvas = document.querySelector("#drive-map");
const ctx = canvas.getContext("2d");

let width = 0;
let height = 0;
let pixelRatio = 1;
let pointerX = 0.62;
let pointerY = 0.42;
let t = 0;

function resize() {
  const rect = canvas.getBoundingClientRect();
  pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  width = Math.max(1, Math.floor(rect.width));
  height = Math.max(1, Math.floor(rect.height));
  canvas.width = Math.floor(width * pixelRatio);
  canvas.height = Math.floor(height * pixelRatio);
  ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
}

function lerp(a, b, amount) {
  return a + (b - a) * amount;
}

function roadX(y) {
  const wave = Math.sin(y * 0.006 + t * 0.72) * width * 0.035;
  const curve = Math.pow(y / Math.max(height, 1), 1.8) * width * 0.12;
  return width * lerp(0.58, 0.52, pointerX) + wave - curve;
}

function drawRoad() {
  const topY = -40;
  const bottomY = height + 90;
  const topWidth = width * 0.16;
  const bottomWidth = width * 0.72;

  ctx.beginPath();
  ctx.moveTo(roadX(topY) - topWidth, topY);
  ctx.bezierCurveTo(width * 0.32, height * 0.28, width * 0.18, height * 0.7, roadX(bottomY) - bottomWidth, bottomY);
  ctx.lineTo(roadX(bottomY) + bottomWidth * 0.42, bottomY);
  ctx.bezierCurveTo(width * 0.75, height * 0.66, width * 0.76, height * 0.22, roadX(topY) + topWidth, topY);
  ctx.closePath();
  ctx.fillStyle = "#252c31";
  ctx.fill();

  ctx.lineWidth = 2;
  ctx.strokeStyle = "rgba(255,255,255,0.18)";
  for (let lane = -1; lane <= 1; lane += 1) {
    ctx.setLineDash([20, 22]);
    ctx.lineDashOffset = -t * 36;
    ctx.beginPath();
    for (let y = topY; y <= bottomY; y += 22) {
      const progress = y / Math.max(height, 1);
      const laneOffset = lane * lerp(width * 0.035, width * 0.14, progress);
      const x = roadX(y) + laneOffset;
      if (y === topY) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.setLineDash([]);

  ctx.strokeStyle = "#f3c84b";
  ctx.lineWidth = 3;
  ctx.shadowColor = "rgba(243, 200, 75, 0.55)";
  ctx.shadowBlur = 18;
  ctx.beginPath();
  for (let y = topY; y <= bottomY; y += 18) {
    const progress = y / Math.max(height, 1);
    const x = roadX(y) - lerp(width * 0.06, width * 0.2, progress);
    if (y === topY) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function drawGrid() {
  ctx.strokeStyle = "rgba(36, 198, 220, 0.13)";
  ctx.lineWidth = 1;
  const gap = Math.max(54, width * 0.055);
  const offset = (t * 18) % gap;

  for (let x = -gap; x < width + gap; x += gap) {
    ctx.beginPath();
    ctx.moveTo(x + offset * 0.35, 0);
    ctx.lineTo(x - offset, height);
    ctx.stroke();
  }

  for (let y = -gap; y < height + gap; y += gap) {
    ctx.beginPath();
    ctx.moveTo(0, y + offset);
    ctx.lineTo(width, y + offset);
    ctx.stroke();
  }
}

function drawRoute() {
  ctx.strokeStyle = "#24c6dc";
  ctx.lineWidth = 5;
  ctx.shadowColor = "rgba(36, 198, 220, 0.9)";
  ctx.shadowBlur = 18;
  ctx.beginPath();
  for (let y = height * 0.02; y <= height * 1.02; y += 16) {
    const progress = y / Math.max(height, 1);
    const x = roadX(y) + Math.sin(progress * 5 + t) * width * 0.018;
    if (y === height * 0.02) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.shadowBlur = 0;
}

function drawVehicle() {
  const carX = roadX(height * 0.68) + (pointerX - 0.5) * 18;
  const carY = height * 0.68 + (pointerY - 0.5) * 18;
  const carW = Math.max(42, width * 0.05);
  const carH = carW * 1.62;

  ctx.save();
  ctx.translate(carX, carY);
  ctx.rotate(-0.12 + Math.sin(t * 1.3) * 0.025);

  for (let ring = 1; ring <= 3; ring += 1) {
    ctx.strokeStyle = `rgba(36, 198, 220, ${0.28 / ring})`;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.ellipse(0, -8, carW * ring * 1.45, carH * ring * 0.82, 0, 0, Math.PI * 2);
    ctx.stroke();
  }

  ctx.fillStyle = "#f6f8f5";
  ctx.strokeStyle = "rgba(255,255,255,0.7)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(-carW / 2, -carH / 2, carW, carH, 8);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#172029";
  ctx.fillRect(-carW * 0.3, -carH * 0.18, carW * 0.6, carH * 0.36);

  ctx.fillStyle = "#1e6bff";
  ctx.fillRect(-carW * 0.4, -carH * 0.43, carW * 0.8, carH * 0.07);

  ctx.fillStyle = "#e5524b";
  ctx.fillRect(-carW * 0.4, carH * 0.36, carW * 0.8, carH * 0.06);

  ctx.restore();
}

function drawSignals() {
  const signals = [
    [0.72, 0.2, "#41b883"],
    [0.8, 0.44, "#f3c84b"],
    [0.68, 0.66, "#e5524b"],
  ];

  signals.forEach(([xRatio, yRatio, color], index) => {
    const pulse = 0.5 + Math.sin(t * 2 + index) * 0.5;
    const x = width * xRatio + Math.sin(t + index) * 8;
    const y = height * yRatio;
    ctx.fillStyle = color;
    ctx.shadowColor = color;
    ctx.shadowBlur = 10 + pulse * 16;
    ctx.beginPath();
    ctx.arc(x, y, 6 + pulse * 2, 0, Math.PI * 2);
    ctx.fill();
    ctx.shadowBlur = 0;
  });
}

function render() {
  t += 0.012;
  ctx.clearRect(0, 0, width, height);

  const gradient = ctx.createLinearGradient(0, 0, width, height);
  gradient.addColorStop(0, "#10151a");
  gradient.addColorStop(0.55, "#18232a");
  gradient.addColorStop(1, "#0f151a");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, width, height);

  drawGrid();
  drawRoad();
  drawRoute();
  drawSignals();
  drawVehicle();

  requestAnimationFrame(render);
}

window.addEventListener("resize", resize);
window.addEventListener("pointermove", (event) => {
  pointerX = event.clientX / Math.max(window.innerWidth, 1);
  pointerY = event.clientY / Math.max(window.innerHeight, 1);
});

if (!CanvasRenderingContext2D.prototype.roundRect) {
  CanvasRenderingContext2D.prototype.roundRect = function roundRect(x, y, w, h, r) {
    const radius = Math.min(r, Math.abs(w) / 2, Math.abs(h) / 2);
    this.beginPath();
    this.moveTo(x + radius, y);
    this.arcTo(x + w, y, x + w, y + h, radius);
    this.arcTo(x + w, y + h, x, y + h, radius);
    this.arcTo(x, y + h, x, y, radius);
    this.arcTo(x, y, x + w, y, radius);
    this.closePath();
    return this;
  };
}

resize();
render();
