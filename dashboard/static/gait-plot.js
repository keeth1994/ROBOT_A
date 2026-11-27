'use strict';

class GaitPlot {
  constructor(canvas) {
    this.canvas = canvas;
    this.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#ff0000'];
    this.legs = ['FL', 'RL', 'FR', 'RR'];
    this.reset();
  }

  reset() {
    this.samples = [];
    this.bounds = {yaw:[-40, 40], pitch:[-60, 0]};
    this.draw();
  }

  append(samples) {
    this.samples.push(...samples);
    const latest = this.samples[this.samples.length - 1].time_s;
    this.samples = this.samples.filter(sample => sample.time_s >= latest - 10);
    for (const sample of samples) {
      for (const axis of ['yaw', 'pitch']) {
        this.bounds[axis][0] = Math.min(this.bounds[axis][0], Math.floor(Math.min(...sample[axis]) / 10) * 10);
        this.bounds[axis][1] = Math.max(this.bounds[axis][1], Math.ceil(Math.max(...sample[axis]) / 10) * 10);
      }
    }
    this.draw();
  }

  draw() {
    const width = this.canvas.clientWidth;
    if (!width) return;
    const height = width * 650 / 1100;
    const scale = window.devicePixelRatio || 1;
    this.canvas.width = Math.round(width * scale);
    this.canvas.height = Math.round(height * scale);
    const ctx = this.canvas.getContext('2d');
    ctx.setTransform(scale, 0, 0, scale, 0, 0);
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, width, height);
    ctx.fillStyle = '#111';
    ctx.textAlign = 'center';
    ctx.font = '16px system-ui, sans-serif';
    ctx.fillText('Rocker Gait Joint Targets', width / 2, 24);

    const left = 70, right = width - 20, top = 43, gap = 20;
    const plotHeight = (height - top - 47 - gap) / 2;
    const last = this.samples[this.samples.length - 1];
    const end = Math.max(10, last?.time_s || 0), start = end - 10;
    const x = time => left + (time - start) / 10 * (right - left);
    ctx.font = '12px system-ui, sans-serif';

    for (const [index, axis] of ['yaw', 'pitch'].entries()) {
      const yTop = top + index * (plotHeight + gap);
      const yBottom = yTop + plotHeight;
      const [minimum, maximum] = this.bounds[axis];
      const y = angle => yBottom - (angle - minimum) / (maximum - minimum) * plotHeight;
      ctx.lineWidth = 1;
      ctx.strokeStyle = '#b0b0b0';
      for (let value = minimum; value <= maximum; value += 10) {
        const row = y(value);
        ctx.beginPath(); ctx.moveTo(left, row); ctx.lineTo(right, row); ctx.stroke();
        ctx.fillStyle = '#111'; ctx.textAlign = 'right';
        ctx.fillText(value.toFixed(0), left - 8, row + 4);
      }
      for (let tick = 0; tick <= 5; tick++) {
        const column = x(start + tick * 2);
        ctx.beginPath(); ctx.moveTo(column, yTop); ctx.lineTo(column, yBottom); ctx.stroke();
        if (index === 1) {
          ctx.textAlign = 'center';
          ctx.fillText((start + tick * 2).toFixed(1), column, yBottom + 19);
        }
      }
      ctx.strokeStyle = '#111';
      ctx.strokeRect(left, yTop, right - left, plotHeight);
      ctx.save();
      ctx.translate(17, (yTop + yBottom) / 2); ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillText((axis === 'yaw' ? 'Yaw' : 'Pitch') + ' target (degrees)', 0, 0);
      ctx.restore();

      ctx.save();
      ctx.beginPath(); ctx.rect(left, yTop, right - left, plotHeight); ctx.clip();
      for (let leg = 0; leg < 4; leg++) {
        ctx.strokeStyle = this.colors[leg]; ctx.lineWidth = 1.7; ctx.beginPath();
        let previous = null;
        for (const sample of this.samples) {
          // Leave gaps when no fresh controller samples were received.
          if (previous === null || sample.time_s - previous > .5) ctx.moveTo(x(sample.time_s), y(sample[axis][leg]));
          else ctx.lineTo(x(sample.time_s), y(sample[axis][leg]));
          previous = sample.time_s;
        }
        ctx.stroke();
      }
      ctx.restore();

      const legendY = index === 0 ? yTop + 17 : yBottom - 13;
      ctx.fillStyle = 'rgba(255,255,255,.9)';
      ctx.fillRect(left + 6, legendY - 12, 256, 21);
      for (let leg = 0; leg < 4; leg++) {
        const legendX = left + 12 + leg * 63;
        ctx.strokeStyle = this.colors[leg]; ctx.lineWidth = 1.7;
        ctx.beginPath(); ctx.moveTo(legendX, legendY - 2); ctx.lineTo(legendX + 24, legendY - 2); ctx.stroke();
        ctx.fillStyle = '#111'; ctx.textAlign = 'left';
        ctx.fillText(this.legs[leg], legendX + 29, legendY + 2);
      }
    }
    ctx.fillStyle = '#111'; ctx.textAlign = 'center';
    ctx.fillText('Time (seconds)', (left + right) / 2, height - 8);
    if (!last) ctx.fillText('Waiting for live walking targets', (left + right) / 2, top + plotHeight / 2);
  }
}
