import { SplitStream } from '/split-stream.js';

const TEST_SELECTED = 'selected';

const connectionStatus = document.getElementById('connection-status')
const connectButton = document.getElementById('connect')

// Set up event listeners
document.getElementById('collapse-history').addEventListener('click', function() {
  const collapsible = document.getElementById('history-bar');
  collapsible.classList.toggle('collapsed');
});

document.querySelectorAll('.test-list > button').forEach(button => button.addEventListener('click', ontestbuttonclick))
connectButton.addEventListener('click', onconnect);

navigator.serial.addEventListener("connect", (e) => {
  // Connect to `e.target` or add it to a list of available ports.
});

// Event listeners
function setConnected(version) {
  connectionStatus.innerHTML = `Connected to Pocket Power Prowler v${version}`
  connectButton.classList.add('hide')
}

function ondisconnnect() {
  connectionStatus.innerHTML = 'Disconnected'
  connectButton.classList.remove('hide')
}

async function onconnect() {
  const port = await navigator.serial.requestPort();
  await port.open({ baudRate: 115200 })

  const textDecoder = new TextDecoderStream();
  const lineSplitter = new SplitStream('\n')
  const reader = port.readable.pipeThrough(textDecoder).pipeThrough(lineSplitter).getReader()

  const textEncoder = new TextEncoderStream();
  const writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
  const writer = textEncoder.writable.getWriter();

  writer.write("v\n")

  const { value, done } = await reader.read()
  if (!done) setConnected(value)
  else ondisconnect()
}

function ontestbuttonclick(e) {
  document.querySelectorAll('.test > section.selected').forEach(node => node.classList.remove(TEST_SELECTED));
  document.querySelectorAll('.test-list > button.selected').forEach(node => node.classList.remove(TEST_SELECTED));

  document.getElementById(e.srcElement.dataset.testId).classList.add(TEST_SELECTED);
  e.srcElement.classList.add(TEST_SELECTED)
}

// Test functions
function loadTestGraph(maxLoadCurrent, minLoadCurrent) {
  // Set the dimensions and margins of the graph
  const margin = {top: 10, right: 30, bottom: 30, left: 60},
    width = 460 - margin.left - margin.right,
    height = 400 - margin.top - margin.bottom;

  // Append the svg object to the body of the page
  const svg = d3.select("#dataviz")
    .append("svg")
    .attr("width", width + margin.left + margin.right)
    .attr("height", height + margin.top + margin.bottom)
    .append("g")
    .attr("transform", `translate(${margin.left}, ${margin.top})`);

  // Add X axis
  const x = d3.scaleLinear()
    .domain([0, maxLoadCurrent])
    .range([ 0, width ]);
  svg.append("g")
    .attr("transform", `translate(0, ${height})`)
    .call(d3.axisBottom(x));

  // Add Y axis
  const y = d3.scaleLinear()
    .domain([0, minLoadVoltage])
    .range([ height, 0]);

  svg.append("g")
    .call(d3.axisLeft(y));

  // Add dots
  svg.append('g')
    .selectAll("dot")
    .data(data)
    .join("circle")
    .attr("cx", function (d) { return x(d.GrLivArea); } )
    .attr("cy", function (d) { return y(d.SalePrice); } )
    .attr("r", 1.5)
    .style("fill", "#69b3a2")
}
