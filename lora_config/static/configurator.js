var msgSocket = new WebSocket("ws://localhost:6543/message");       
msgSocket.onmessage = function(event) {
    var msgOut = document.getElementById("message_out");
    msgOut.innerHTML = msgOut.innerHTML + "<br>" + event.data;
};

document.addEventListener('DOMContentLoaded', (event) => {
    var bandwidthInput = document.getElementById("bandwidth_input");
    var spreadingFactorInput = document.getElementById("spreading_factor_input");
    var codingRateInput = document.getElementById("coding_rate_input");
    var txPowerInput = document.getElementById("tx_power_input");

    var updateBtn = document.getElementById("update_config");
    updateBtn.addEventListener("click", (event) => {
        try {
            var bValue = validator(bandwidthInput.value);
            var sfValue = validator(spreadingFactorInput.value);
            var crValue = validator(codingRateInput.value);
            var tpValue = validator(txPowerInput.value);
            var data = { bandwidth: bValue, tx_power: tpValue, spreading_factor: sfValue, coding_rate: crValue };
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

    var connectBtn = document.getElementById("reconnect_button");
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
});

function validator(input) {
    if (input === ""){
        return "noupdate"
    }
    else {
        return input
    }
}