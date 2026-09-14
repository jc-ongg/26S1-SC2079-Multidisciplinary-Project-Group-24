export function rotateDirection(direction) {
  return (Number(direction) + 2) % 8;
}

// The API supplies the start and one endpoint per movement, including compressed moves.
export function snapshotVisits(commands) {
  let step = 0;
  const visits = [];
  for (const command of commands) {
    const snapshot = /^SNAP(\d+)(?:_[LCR])?$/.exec(command);
    if (snapshot) visits.push({ id: Number(snapshot[1]), step });
    else if (/^(FW|BW|FL|FR|BL|BR)/.test(command)) step++;
  }
  return visits;
}

export function routeSegments(path) {
  return path.slice(1).map((end, i) => {
    const start = path[i];
    const x = start.x + .5, y = 19.5 - start.y;
    const ex = end.x + .5, ey = 19.5 - end.y;
    if (Number(start.d) === Number(end.d)) return `M ${x} ${y} L ${ex} ${ey}`;
    // Approximate a quarter ellipse tangent to the start/end headings.
    const k = .5522847498;
    const vertical = Number(start.d) === 0 || Number(start.d) === 4;
    const cx1 = vertical ? x : x + (ex - x) * k;
    const cy1 = vertical ? y + (ey - y) * k : y;
    const cx2 = vertical ? ex - (ex - x) * k : ex;
    const cy2 = vertical ? ey : ey - (ey - y) * k;
    return `M ${x} ${y} C ${cx1} ${cy1}, ${cx2} ${cy2}, ${ex} ${ey}`;
  });
}
