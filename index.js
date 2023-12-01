
function generateSquareWave(frequency, numPoints) {
    const period = 1 / frequency;
    const halfPeriod = period / 2;

    const xValues = [];
    const yValues = [];

    for (let i = 0; i < numPoints; i++) {
        const t = (i / (numPoints - 1)) * period;
        const y = t % period < halfPeriod ? 1 : -1;

        xValues.push(t);
        yValues.push(y);
    }

    return { x: xValues, y: yValues, type: 'scatter', mode: 'lines', name: 'Square Wave' };
}

function generatePulseAndSine(frequency, delayStart, duration, numPoints) {
    var period = 1 / frequency;
    var halfPeriod = period / 2;
    var maxT = duration + delayStart + 10*period; 
    var pulseDelay = maxT/10; 
    var maxT = maxT + pulseDelay; 
    var xValues = [];
    var yValues = [];

    for (let t = 0; t < maxT; t=t+period/10) {
        // Short pulse for 1ms
        if (t < pulseDelay) {
            yValues.push(1);
        } else {
            // Sine wave with delay and duration
            const tWithDelay = t - (delayStart + pulseDelay);
            //console.log(tWithDelay)
            var y = 0 
            if(tWithDelay >= 0 && tWithDelay <= duration){
                console.log(tWithDelay);
                y = Math.sin(2 * Math.PI * frequency * tWithDelay)**2
            }
            yValues.push(y);
        }

        xValues.push(t);
    }

    return { x: xValues, y: yValues, type: 'scatter', mode: 'lines', name: 'Pulse and Sine' };
}

// Example usage:
const frequency = 1e3; // Set your desired frequency
const delayStart = 1e-3; // Set delay in seconds
const duration = 1e-3; // Set duration in seconds
const numPoints = 10000; // Set the number of points for the plot
const maxT = 10;
const pulseAndSineData = generatePulseAndSine(frequency, delayStart, duration, numPoints);

// Layout configuration for the plot
const layout = {
    title: 'Pulse and Sine',
    xaxis: { title: 'Time' },
    yaxis: { title: 'Amplitude', range: [-1.5, 1.5] }
};

// Create the plot
Plotly.newPlot('scatterPlot', [pulseAndSineData], layout);

function captureFormData() {
    // Get values from the first set of dropdown and text fields
    const polarity1 = document.getElementById('polarity1').value;
    const delayStart1 = document.getElementById('delayStart1').value;
    const currentDuration1 = document.getElementById('currentDuration1').value;

    // Get values from the second set of dropdown and text fields
    const polarity2 = document.getElementById('polarity2').value;
    const delayStart2 = document.getElementById('delayStart2').value;
    const currentDuration2 = document.getElementById('currentDuration2').value;

    // Get value from the frequency textbox
    const frequency = document.getElementById('frequency').value;

    // Create a JSON object with the captured data
    const formData = {
        sets: [
            {
                polarity: polarity1,
                delayStart: delayStart1,
                currentDuration: currentDuration1
            },
            {
                polarity: polarity2,
                delayStart: delayStart2,
                currentDuration: currentDuration2
            }
            // Add more sets as needed
        ],
        frequency: frequency
    };

    return formData;
}

function generateArray(start, end, numPoints) {
    const step = (end - start) / (numPoints - 1);
    return Array.from({ length: numPoints }, (_, index) => start + index * step);
}


function generateSine2(frequency, t,tstart,duration){
    var y = {};
    for(var i = 0; i<t.length;i++){
        var ti = t[i]
        if(ti>=tstart){
            y.push(Math.sin(2*Math.pi*f*(ti-tstart))**2);
        }
        else{
            y.push(0);
        }
    }
}

function setCurrentDuration(value1,value2) {
    // Set the value of the "Current Duration" textbox for the first set
    document.getElementById('currentDuration1').value = value1;

    // Set the value of the "Current Duration" textbox for the second set
    document.getElementById('currentDuration2').value = value2;
    // Add similar lines for more sets if needed
}

function roundToMultiple(number, multiple) {
    return Math.round(number / multiple) * multiple;
}


function analyze(){
   var formData = captureFormData();
   var period = 1/formData.frequency; 
   var roundedDuration1 = roundToMultiple(formData.sets[0].currentDuration,period*0.5);
   var roundedDuration2 = roundToMultiple(formData.sets[1].currentDuration,period*0.5);
    setCurrentDuration(roundedDuration1,roundedDuration2);
}

//serial communications 
let serialPort;

async function connectToArduino() {
    try {
        serialPort = await navigator.serial.requestPort();
        await serialPort.open({ baudRate: 9600 });
        console.log('Connected to Arduino');
        readDataFromArduino();
    } catch (error) {
        console.error('Error connecting to Arduino:', error);
    }
}

async function readDataFromArduino() {
    try {
        const reader = serialPort.readable.getReader();

        while (true) {
            const { value, done } = await reader.read();

            if (done) {
                console.log('Serial port closed');
                break;
            }

            // Process the received data
            processDataFromArduino(value);
        }
    } catch (error) {
        console.error('Error reading data from Arduino:', error);
    }
}

function processDataFromArduino(data) {
    // Convert the received data (Uint8Array) to a string
    const receivedText = new TextDecoder().decode(data);

    // Update the webpage with the received data
    updateWebpageWithArduinoData(receivedText);
}
var terminalText = "";

function updateWebpageWithArduinoData(data) {
    const receivedDataDiv = document.getElementById('receivedData');
    terminalText = terminalText+data;
    receivedDataDiv.textContent = terminalText;
    

    // Create a cursor element and append it to the end of the received data
    const cursorElement = document.createElement('span');
    cursorElement.className = 'cursor';
    receivedDataDiv.appendChild(cursorElement);

    // Scroll the received data to the end
    receivedDataDiv.scrollTop = receivedDataDiv.scrollHeight;
}

async function sendDataToArduino(data) {
    try {
        // Convert the string to ArrayBuffer
        const encoder = new TextEncoder();
        const dataArrayBuffer = encoder.encode(data);

        const writer = serialPort.writable.getWriter();
        await writer.write(dataArrayBuffer);
        await writer.releaseLock();
        console.log('Data sent to Arduino:', data);
    } catch (error) {
        console.error('Error sending data to Arduino:', error);
    }
}


function analyze() {
    var formData = captureFormData();
    var period = 1 / formData.frequency;
    var roundedDuration1 = roundToMultiple(formData.sets[0].currentDuration, period * 0.5);
    var roundedDuration2 = roundToMultiple(formData.sets[1].currentDuration, period * 0.5);
    setCurrentDuration(roundedDuration1, roundedDuration2);

    // Continue with your existing analyze logic
}

async function analyzeArduino(formData) {
    // Customize the data structure based on your Arduino code
    const dataToSend = JSON.stringify(formData);
    var formData = captureFormData();
    var period = 1 / formData.frequency;
    var roundedDuration1 = roundToMultiple(formData.sets[0].currentDuration, period * 0.5);
    var roundedDuration2 = roundToMultiple(formData.sets[1].currentDuration, period * 0.5);
    setCurrentDuration(roundedDuration1, roundedDuration2);
    var dataSend = `${formData.frequency}-${formData.sets[0].polarity == 'Positive'? 1:0}-${formData.sets[0].delayStart*1000}-${formData.sets[0].currentDuration*1000}-${formData.sets[1].polarity == 'Positive'? 1:0}-${formData.sets[1].delayStart*1000}-${formData.sets[1].currentDuration*1000}`;
    console.log(dataSend);
    try {
        // Ensure the connection is established before sending data
        if (!serialPort || !serialPort.readable) {
            console.error('Arduino connection not established.');
            return;
        }

        // Send data to Arduino
        await sendDataToArduino(dataSend);

        // Add any additional logic specific to analyzing Arduino data, if needed
    } catch (error) {
        console.error('Error during Arduino analysis:', error);
    }
}

