var url = "ws://192.168.4.1:1337/";
var output;
var button_up;
var button_down;
var canvas;
var context;

// This is called when the page finishes loading
function init() {

    // Assign page elements to variables
    button_up = document.getElementById("changeRPM");
    output = document.getElementById("output");
    canvas = document.getElementById("led");
    
    // Draw circle in canvas
    
    // Connect to WebSocket server
    wsConnect(url);
}

// Call this to connect to the WebSocket server
function wsConnect(url) {
    
    // Connect to WebSocket server
    websocket = new WebSocket(url);
    
    // Assign callbacks
    websocket.onopen = function(evt) { onOpen(evt) };
    websocket.onclose = function(evt) { onClose(evt) };
    websocket.onmessage = function(evt) { onMessage(evt) };
    websocket.onerror = function(evt) { onError(evt) };
}

// Called when a WebSocket connection is established with the server
function onOpen(evt) {

    // Log connection state
    console.log("Connected");
    writeToScreen("Connected");
    
    // Enable button
    button.disabled = false;
    
    // Get the current state of the LED
    doSend("getRPM");
}

// Called when the WebSocket connection is closed
function onClose(evt) {

    // Log disconnection state
    console.log("Disconnected");
    writeToScreen("Disconnected");
    
    // Disable button
    button.disabled = true;
    
    // Try to reconnect after a few seconds
    setTimeout(function() { wsConnect(url) }, 2000);
}

function fillArc(x,y,color) {
    context.arc(x,y,15,0,Math.Pi*2,false);
    context.fillStyle = color;
    context.fill();
}

function writeToScreen(message)
{
    document.getElementById("msg").innerHTML += message + "\n";
    document.getElementById("msg").scrollTop = document.getElementById("msg").scrollHeight;
}

// Called when a message is received from the server
function onMessage(evt) {

    // Print out our received message
    console.log("Received: " + evt.data);

    writeToScreen("MOTOR RPM: " + evt.data);
    
    // // Update circle graphic with LED state
    // switch(evt.data) {
    //     case "0":
    //         writeToScreen("MOTOR RPM: " + );
    //         break;
    //     default:
    //         break;
    // }
}

// Called when a WebSocket error occurs
function onError(evt) {
    console.log("ERROR: " + evt.data);
}

// Sends a message to the server (and prints it to the console)
function doSend(message) {
    console.log("Sending: " + message);
    websocket.send(message);
}

// Called whenever the HTML button is pressed
function onPress_up() {
    doSend("increaseRPM");
    doSend("getRPM");
}

function onPress_down() {
    doSend("decreaseRPM");
    doSend("getRPM");
}

function shutdown(){
    doSend("shutdown");
    doSend("getRPM");
}

// Call the init function as soon as the page loads
window.addEventListener("load", init, false);