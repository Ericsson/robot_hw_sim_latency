"use strict"

var imageRanges = [10,20,25,29,31]
var imageUrl = [
    'pics/5G_urllc_2.png',
    'pics/5G_eMBB.png',
    'pics/lte_private.jpg',
    'pics/lte.jpg',
    'pics/3G.jpg' 
    ]

var parameters = {
    "low-pistons": {
        type: "id",
        name: "Low priority pistons",
        value: null
    },
    "low-gears": {
        type: "id",
        name: "Low priority gears",
        value: null
    },
    "low-pulleys": {
        type: "id",
        name: "Low priority pulleys",
        value: null
    },
    "low-discs": {
        type: "id",
        name: "Low priority discs",
        value: null
    },
    "high-time": {
        type: "id",
        name: "Time when issued",
        value: null
    },
    "high-pistons": {
        type: "id",
        name: "High priority pistons",
        value: null
    },
    "high-gears": {
        type: "id",
        name: "High priority gears",
        value: null
    },
    "high-pulleys": {
        type: "id",
        name: "High priority pulleys",
        value: null
    },
    "high-discs": {
        type: "id",
        name: "High priority discs",
        value: null
    },
    "scenarios": {
        type: "name",
        name: "Selected scenario",
        value: null
    },
    "failures": {
        type: "idchecked",
        name: "Failures",
        value: null
    },
    "qoc": {
        type: "idchecked",
        name: "QoC",
        value: null
    },
    "latency": {
        type: "id",
        name: "Latency",
        value: null
    }
}

var websocket;

function onLoad(){


    websocket = new WebSocket("ws://"+window.location.hostname+":8765/")
    websocket.onclose = function (event) {
        console.log(event.reason)
    }
    websocket.onmessage = function (event) {
        let message = JSON.parse(event.data)
        let overlay = document.getElementById("overlay")
        let overlayContents = overlay.firstElementChild
        console.log("WebSocket message received:", message);
        if (message["messagetype"]=="END"){
            let para = pElemWithText("Click here to to exit", "red")
            overlayContents.appendChild(para)
            scrollToBottom(overlayContents)
            para.addEventListener("click", function( event ) {
              overlay.style.display = "none"
              deleteNodeContents(overlayContents)
            }, false);

        }
        else if(message["messagetype"]=="MESSAGE"){
            let para = pElemWithText(message["data"], message["style"])
            overlayContents.appendChild(para)
            scrollToBottom(overlayContents)
        }
            
    }

}

function scrollToBottom(div){
   div.scrollTop = div.scrollHeight - div.clientHeight;
}

function deleteNodeContents(node){
    var range = document.createRange();
    range.selectNodeContents(node);
    range.deleteContents();
}

function pElemWithText(text, style){
    var para = document.createElement("p");
    if (style != "normal"){
      para.classList.add(style);
    }
    var textn = document.createTextNode(text);
    para.appendChild(textn);
    return para
}

function onStart() {
    if (websocket.readyState != 1) {
        console.log("WS is not ready")
    }
    for (const key of Object.keys(parameters)) {
        let value = parameters[key];
        if (value.type == "id"){
            value.value = document.getElementById(key).value
        }else if (value.type == "idchecked"){
            value.value = document.getElementById(key).checked
        } else if (value.type == "name"){
            value.value = document.querySelector('input[name='+key+']:checked').value;
        }
        console.log(value.name+": "+value.value.toString())
    }
    console.log(parameters)
    websocket.send(JSON.stringify({"messagetype":"START","params":parameters}));
    document.getElementById("overlay").style.display = "block"

}

function onLatencyChange(event){
    document.getElementById("latencytext").innerText = event.srcElement.value + " ms"
    document.getElementById("latencyimg").src = imageUrl[imageRanges.findIndex(val => val > event.srcElement.value)];
}
