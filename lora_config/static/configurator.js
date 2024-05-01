let msgSocket = new WebSocket("ws://localhost:6543/message");       
msgSocket.onmessage = function(event) {
    let msgOut = document.getElementById("message_out");
    msgOut.innerHTML = msgOut.innerHTML + "<br>" + event.data;
};

document.addEventListener('DOMContentLoaded', (event) => {
    let bandwidthInput = document.getElementById("bandwidth_input");
    let spreadingFactorInput = document.getElementById("spreading_factor_input");
    let codingRateInput = document.getElementById("coding_rate_input");
    let txPowerInput = document.getElementById("tx_power_input");

    let updateBtn = document.getElementById("update_config");
    updateBtn.addEventListener("click", (event) => {
        try {
            let bValue = validator(bandwidthInput.value);
            let sfValue = validator(spreadingFactorInput.value);
            let crValue = validator(codingRateInput.value);
            let tpValue = validator(txPowerInput.value);
            let data = { bandwidth: bValue, tx_power: tpValue, spreading_factor: sfValue, coding_rate: crValue };
            fetch(`/config_params`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(data)
            });
        } catch (error) {
            console.error("Error sending command:", error);
        }
    });

    let connectBtn = document.getElementById("reconnect_button");
    connectBtn.addEventListener("click", (event) => {
        try {
            fetch(`/reconnect_node`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            });
        } catch (error) {
            console.error("Error sending command:", error);
        }
    });

    let durationInput = document.getElementById("duration_input");
    let packetSizeInput = document.getElementById("packet_size_input");
    let numPacketsInput = document.getElementById("num_packets_input");
    let dtBtn = document.getElementById("data_test_button");
    dtBtn.addEventListener("click", (event) => {
        try {
            let dValue = validator(durationInput.value);
            let psValue = validator(packetSizeInput.value);
            let npValue = validator(numPacketsInput.value);
            let data = { duration: dValue, packet_size: psValue, num_packets: npValue };
            fetch(`/data_rate_test`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(data)
            });
        } catch (error) {
            console.error("Error sending command:", error);
        }
    });
});

function validator(input) {
    if (input === ""){
        return "noupdate"
    }
    else {
        return input
    }
}