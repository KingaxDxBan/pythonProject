function updateSliderValue(sliderId, displayId) {
    var slider = document.getElementById(sliderId);
    var display = document.getElementById(displayId);
    display.innerText = slider.value;
}

function updateSimulation() {
    console.log("Update simulation function called.");
    var zadana = document.getElementById('zadana').value;
    var kp = document.getElementById('kp').value;
    var ti = document.getElementById('ti').value;
    var td = document.getElementById('td').value;

    $.ajax({
        type: 'POST',
        url: '/update_simulation',
        data: {
            zadana: zadana,
            kp: kp,
            ti: ti,
            td: td
        },
        success: function (response) {
            var plotContainer = document.getElementById('plot-container');
            Plotly.newPlot(plotContainer, JSON.parse(response));
        }
    });
}
