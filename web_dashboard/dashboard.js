let ros;
let statusInterval;
let wasEspConnected = false;
let wasGo2Connected = false;

// Services
let containerCloseService;
let containerOpenService;
let containerNextSampleService;

let drillHomeService;
let drillCleanService;
let drillStopCleaningService;
let drillRetrieveService;
let drillStopRetrievingService;
let drillDrillService;
let drillRetractService;

let go2Service;

// Drill Status
const DrillState = {
    0: { label: "Unknown" },
    1: { label: "Ready" },
    2: { label: "Homing" },
    3: { label: "Drilling" },
    4: { label: "Retracting" },
    5: { label: "Retrieving" },
    6: { label: "Cleaning" }
};

// Container Status
const ContainerState = {
    0: { label: "Unknown" },
    1: { label: "Closing" },
    2: { label: "Closed" },
    3: { label: "Opening" },
    4: { label: "Ready" },
    5: { label: "Revolving" },
    6: { label: "FULL" }
};



const bridgeIndicator = document.getElementById('bridge-status');
const agentIndicator = document.getElementById('agent-status');
const go2BridgeIndicator = document.getElementById('go2-status');
const agentMsg = document.getElementById('agent-msg');
const go2BridgeMsg = document.getElementById('go2-bridge-msg');
const retryBtn = document.getElementById('retry-agent-btn');
const go2RetryBtn = document.getElementById('retry-go2-bridge-btn');
const tareBtn = document.getElementById('tare-btn');

function connectToROS() {
    if (ros) { ros.close(); }
    if (statusInterval) { clearInterval(statusInterval); }

    const address = document.getElementById('rosbridge-address').value || 'ws://localhost:9090';
    ros = new ROSLIB.Ros({ url: address });

    ros.on('connection', () => {
        bridgeIndicator.className = 'status-circle status-green';
        retryBtn.disabled = false;
        go2RetryBtn.disabled = false;
        const buttons = document.querySelectorAll('.service-button');
        buttons.forEach(btn => {
            btn.disabled = false;
        });
        checkAgentNode(false);
        checkGo2BridgeNode(false);
        statusInterval = setInterval(() => {
            checkAgentNode(true);
            checkGo2BridgeNode(true);
        }, 5000); 
        console.log('Rosbridge established')
    });

    ros.on('error', () => { 
        handleDisconnect();
        console.error('Something went wrong during ros connection')
    });
    ros.on('close', () => { 
        handleDisconnect(); 
        console.log('Closed ros connection');
    });
}

function handleDisconnect() {
    bridgeIndicator.className = 'status-circle status-red';
    agentIndicator.className = 'status-circle';
    go2BridgeIndicator.className = 'status-circle';
    go2RetryBtn.disabled = true;
    retryBtn.disabled = true;
    const buttons = document.querySelectorAll('.service-button');
    buttons.forEach(btn => {
        btn.disabled = true;
    });
    agentMsg.innerText = "Bridge closed";
    go2BridgeMsg.innerText = "Bridge closed";
}

function checkAgentNode(autoCheck) {
    if (!ros || !ros.isConnected) return;
    
    if (!autoCheck) {
        agentIndicator.className = 'status-circle';
        agentMsg.innerText = "Searching...";
    }
    
    setTimeout(() => {
        ros.getNodes((nodes) => {
            const isRunning = nodes.includes('/esp32');
            if (isRunning) {
                agentIndicator.className = 'status-circle status-green';
                agentMsg.innerText = "ESP connected";
                if (!wasEspConnected) {
                    setupESPSubscriptions();
                    setupESPServices();
                    wasEspConnected = true;
                }
            } else {
                agentIndicator.className = 'status-circle status-red';
                agentMsg.innerText = "ESP not detected";
                wasEspConnected = false;
            }
        }, (err) => {
            agentIndicator.className = 'status-circle status-red';
        });
    }, 200); 
}

function checkGo2BridgeNode(autoCheck) {
    if (!ros || !ros.isConnected) return;

    if (!autoCheck) {
        go2BridgeIndicator.className = 'status-circle';
        go2BridgeMsg.innerText = "Searching...";
    }

    setTimeout(() => {
        ros.getNodes((nodes) => {
            const isRunning = nodes.includes('/go2_driver');
            if (isRunning) {
                go2BridgeIndicator.className = 'status-circle status-green';
                go2BridgeMsg.innerText = "Go2 Driver detected";
                if (!wasGo2Connected) {
                    setupGo2Services();
                    wasGo2Connected = true;
                }
            } else {
                go2BridgeIndicator.className = 'status-circle status-red';
                go2BridgeMsg.innerText = "Go2 Driver not detected";
                wasGo2Connected = false;
            }
        }, (err) => {
            agentIndicator.className = 'status-circle status-red';
        });
    }, 200); 
}

function setupGo2Services() {
    go2Service = new ROSLIB.Service({
        ros: ros,
        name: '/mode',
        serviceType: 'go2_interfaces/srv/Mode'
    });
}



function setupESPServices() {
    containerCloseService = new ROSLIB.Service({
        ros: ros,
        name: '/container/close',
        serviceType: 'std_srvs/srv/Trigger'
    });
    containerOpenService = new ROSLIB.Service({
        ros: ros,
        name: '/container/open',
        serviceType: 'std_srvs/srv/Trigger'
    });
    containerNextSampleService = new ROSLIB.Service({
        ros: ros,
        name: '/container/nextSample',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillHomeService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/home',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillCleanService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/clean',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillStopCleaningService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/stopCleaning',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillRetrieveService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/retrieve',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillStopRetrievingService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/stopRetrieving',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillDrillService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/drill',
        serviceType: 'std_srvs/srv/Trigger'
    });
    drillRetractService = new ROSLIB.Service({
        ros: ros,
        name: '/drill/retract',
        serviceType: 'std_srvs/srv/Trigger'
    });

}

function setupESPSubscriptions() {
    const weightTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/container/weight',
        messageType: 'std_msgs/msg/Float32'
    });

    const containerSystemStateTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/container/state',
        messageType: 'std_msgs/msg/Int8'
    });

    const drillSystemStateTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/drill/state',
        messageType: 'std_msgs/msg/Int8'
    });

    weightTopic.subscribe(function (message) {
        document.getElementById('container-sample-weight').innerText = message.data.toFixed(2);
    });

    containerSystemStateTopic.subscribe(function (message) {
        document.getElementById('container-system-status').innerText = ContainerState[message.data].label;
    });

    drillSystemStateTopic.subscribe(function (message) {
        document.getElementById('drill-system-status').innerText = DrillState[message.data].label;
    });
}



function go2Sit() {
    if (!go2Service || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }

    const request = new ROSLIB.ServiceRequest({
        mode: 'sit'
    });
    go2Service.callService(request, function(result) {
        if (!result.success) {
            console.error("Failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}

function go2Rise() {
    if (!go2Service || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }

    const request = new ROSLIB.ServiceRequest({
        mode: 'rise_sit'
    });
    go2Service.callService(request, function(result) {
        if (!result.success) {
            console.error("Failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}




function containerClose() {
    if (!containerCloseService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    containerCloseService.callService(request, function(result) {
        if (!result.success) {
            console.error("Closing failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}

function containerNextSample() {
    if (!containerNextSampleService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    containerNextSampleService.callService(request, function(result) {
        if (!result.success) {
            console.error("Next Sample failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}

function containerOpen() {
    if (!containerOpenService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    containerOpenService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}







function drillClean() {
    if (!drillCleanService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillCleanService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}


function drillDrill() {
    if (!drillDrillService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillDrillService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}


function drillHome() {
    if (!drillHomeService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillHomeService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}



function drillRetract() {
    if (!drillRetractService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillRetractService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}


function drillRetrieve() {
    if (!drillRetrieveService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillRetrieveService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}




function drillStopCleaning() {
    if (!drillStopCleaningService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillStopCleaningService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}



function stopRetrieving() {
    if (!drillStopRetrievingService || !ros.isConnected) {
        console.error("Error: Service or ROS not initialized");
        return;
    }
    const request = new ROSLIB.ServiceRequest({});
    drillStopRetrievingService.callService(request, function(result) {
        if (!result.success) {
            console.error("Service failed: " + result.message);
            return;
        }
        console.log(result.message);
    }, function(error) {
        console.error("Error calling service:", error);
    });
}