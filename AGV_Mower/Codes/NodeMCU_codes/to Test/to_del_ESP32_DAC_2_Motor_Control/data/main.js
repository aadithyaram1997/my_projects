function sendMovementData(direction) {
  console.log("sendMovementData " + direction);
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("currentDirection").innerHTML =
        "Current State " + this.responseText;
    }
  };
  xhttp.open("GET", "setDIR?Motionstate=" + direction, true);
  xhttp.send();
}
function sendPowerData() {
  console.log(
    "sendPowerData " + document.getElementById("powerSwitch").checked
  );
  var state = document.getElementById("powerSwitch").checked;
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      if (this.responseText == 1)
        document.getElementById("powerSwitch").checked = true;
      else if (this.responseText == 0)
        document.getElementById("powerSwitch").checked = false;
    }
  };
  xhttp.open("GET", "setPower?Powerstate=" + state, true);
  xhttp.send();
}
function sendSpeedData(speed) {
  console.log("sendSpeedData " + speed);
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("currentDirection").innerHTML =
        "Going " + this.responseText;
    }
  };
  xhttp.open("GET", "setSpeed?Speedstate=" + speed, true);
  xhttp.send();
}

// setInterval(function () {
//   // Call a function repetatively with 2 Second interval
//   getData();
// }, 2000); //2000mSeconds update rate

// function getData() {
//   var xhttp = new XMLHttpRequest();
//   xhttp.onreadystatechange = function () {
//     if (this.readyState == 4 && this.status == 200) {
//       document.getElementById("ADCValue").innerHTML = this.responseText;
//     }
//   };
//   xhttp.open("GET", "readADC", true);
//   xhttp.send();
// }
