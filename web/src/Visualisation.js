import React from "react";
import Plot from "react-plotly.js";

function hemisphere(r, c, a) {
  let x = [];
  let y = [];
  let z = [];
  let n = 100;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      let u = (i / (n - 1)) * 2 * Math.PI;
      let v = ((j / (n - 1)) * Math.PI) / 2;

      let xi = r * Math.cos(u) * Math.sin(v);
      let yi = r * Math.sin(u) * Math.sin(v);
      let zi = r * Math.cos(v);

      x.push(xi + c[0]);
      y.push(yi + c[1]);
      z.push(a * zi + c[2]);
    }
  }

  return [x, y, z];
}

class PlotlyFig extends React.Component {
  // only render when the trajectory data changes
  shouldComponentUpdate(nextProps, nextState) {
    if (this.props.tdata === nextProps.tdata) {
      return false;
    } else {
      return true;
    }
  }

  render() {
    if (!this.props.tdata) {
      return null;
    }

    let colours = [
      "#000000",
      "#E69F00",
      "#56B4E9",
      "#009E73",
      "#F0E442",
      "#0072B2",
      "#D55E00",
      "#CC79A7",
    ];
    let t = this.props.target;
    let r = this.props.r;

    let p = hemisphere(r, t, 1);
    let p2 = hemisphere(r, t, -1);

    // path opacity
    let lo = 0.2;

    // index of last data point
    let n = this.props.tdata[0][0].length - 1;

    let layout = {
      autosize: true,
      margin: {
        l: 0,
        r: 0,
        b: 0,
        t: 0,
        pad: 10,
      },
      legend: {
        orientation: "h",
      },
    };
    let data = [
      {
        x: p[0],
        y: p[1],
        z: p[2],
        type: "mesh3d",
        color: colours[4],
        name: "Target",
        opacity: 0.5,
      },
      {
        x: p2[0],
        y: p2[1],
        z: p2[2],
        type: "mesh3d",
        color: colours[4],
        opacity: 0.5,
      },
      // starting position uav 1
      {
        x: [this.props.tdata[0][0][0], this.props.tdata[0][0][n]],
        y: [this.props.tdata[0][1][0], this.props.tdata[0][1][n]],
        z: [this.props.tdata[0][2][0], this.props.tdata[0][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[0],
        },
        name: "Start/End UAV 1",
      },
      // starting position uav 2
      {
        x: [this.props.tdata[1][0][0], this.props.tdata[1][0][n]],
        y: [this.props.tdata[1][1][0], this.props.tdata[1][1][n]],
        z: [this.props.tdata[1][2][0], this.props.tdata[1][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[1],
        },
        name: "Start/End UAV 2",
      },
      // starting position uav 3
      {
        x: [this.props.tdata[2][0][0], this.props.tdata[2][0][n]],
        y: [this.props.tdata[2][1][0], this.props.tdata[2][1][n]],
        z: [this.props.tdata[2][2][0], this.props.tdata[2][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[2],
        },
        name: "Start/End UAV 3",
      },
      // starting position uav 4
      {
        x: [this.props.tdata[3][0][0], this.props.tdata[3][0][n]],
        y: [this.props.tdata[3][1][0], this.props.tdata[3][1][n]],
        z: [this.props.tdata[3][2][0], this.props.tdata[3][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[3],
        },
        name: "Start/End UAV 4",
      },
      {
        x: [this.props.tdata[4][0][0], this.props.tdata[4][0][n]],
        y: [this.props.tdata[4][1][0], this.props.tdata[4][1][n]],
        z: [this.props.tdata[4][2][0], this.props.tdata[4][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[4],
        },
        name: "Start/End UAV 5",
      },
      {
        x: [this.props.tdata[5][0][0], this.props.tdata[5][0][n]],
        y: [this.props.tdata[5][1][0], this.props.tdata[5][1][n]],
        z: [this.props.tdata[5][2][0], this.props.tdata[5][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[5],
        },
        name: "Start/End UAV 6",
      },
      {
        x: [this.props.tdata[6][0][0], this.props.tdata[6][0][n]],
        y: [this.props.tdata[6][1][0], this.props.tdata[6][1][n]],
        z: [this.props.tdata[6][2][0], this.props.tdata[6][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[6],
        },
        name: "Start/End UAV 7",
      },
      {
        x: [this.props.tdata[7][0][0], this.props.tdata[7][0][n]],
        y: [this.props.tdata[7][1][0], this.props.tdata[7][1][n]],
        z: [this.props.tdata[7][2][0], this.props.tdata[7][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[7],
        },
        name: "Start/End UAV 8",
      },
      {
        x: [this.props.tdata[8][0][0], this.props.tdata[8][0][n]],
        y: [this.props.tdata[8][1][0], this.props.tdata[8][1][n]],
        z: [this.props.tdata[8][2][0], this.props.tdata[8][2][n]],
        type: "scatter3d",
        mode: "markers",
        marker: {
          size: 5,
          color: colours[8],
        },
        name: "Start/End UAV 9",
      },
      {
        x: this.props.tdata[0][0],
        y: this.props.tdata[0][1],
        z: this.props.tdata[0][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 1",
        line: {
          width: 5,
          color: colours[0],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[1][0],
        y: this.props.tdata[1][1],
        z: this.props.tdata[1][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 2",
        line: {
          width: 5,
          color: colours[1],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[2][0],
        y: this.props.tdata[2][1],
        z: this.props.tdata[2][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 3",
        line: {
          width: 5,
          color: colours[2],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[3][0],
        y: this.props.tdata[3][1],
        z: this.props.tdata[3][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 4",
        line: {
          width: 5,
          color: colours[3],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[4][0],
        y: this.props.tdata[4][1],
        z: this.props.tdata[4][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 5",
        line: {
          width: 5,
          color: colours[4],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[5][0],
        y: this.props.tdata[5][1],
        z: this.props.tdata[5][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 6",
        line: {
          width: 5,
          color: colours[5],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[6][0],
        y: this.props.tdata[6][1],
        z: this.props.tdata[6][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 7",
        line: {
          width: 5,
          color: colours[6],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[7][0],
        y: this.props.tdata[7][1],
        z: this.props.tdata[7][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 8",
        line: {
          width: 5,
          color: colours[7],
        },
        opacity: lo,
      },
      {
        x: this.props.tdata[8][0],
        y: this.props.tdata[8][1],
        z: this.props.tdata[8][2],
        type: "scatter3d",
        mode: "lines",
        name: "UAV 9",
        line: {
          width: 5,
          color: colours[8],
        },
        opacity: lo,
      },
    ];
    return (
      <div style={{ minHeight: 450 }}>
        <Plot
          onUpdate={() => this.props.onChange()}
          data={data}
          layout={layout}
          useResizeHandler={true}
          config={{ responsive: true }}
          style={{ width: "100%", height: "100%" }}
        />
      </div>
    );
  }
}

export default PlotlyFig;
