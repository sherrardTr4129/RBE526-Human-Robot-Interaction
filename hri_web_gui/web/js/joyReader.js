/* Author: Trevor Sherrard
 * Course: Human Robot Interaction
 * Project: Autonomous Camera Control Optimization
 * Since: November 1st, 2020
 */

var Joy1 = new JoyStick('joy1');
var xyURL = "http://localhost:5000/xyJoyPost";
var xyData = {
	"x": 0,
	"y": 0
};

function sendXYData()
{
	$.ajax({type: 'POST',
		url: xyURL,
		data: JSON.stringify (xyData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ xyData.x=Joy1.GetX(); }, 50);
setInterval(function(){ xyData.y=Joy1.GetY(); }, 50);
setInterval(function(){ sendXYData() }, 200);

var Joy2 = new JoyStick('joy2');
var zURL = "http://localhost:5000/zJoyPost"
var zData = {
	"z": 0
}

function sendZData()
{
	$.ajax({type: 'POST',
		url: zURL,
		data: JSON.stringify (zData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

setInterval(function(){ zData.z=Joy2.GetY(); }, 50);
setInterval(function(){ sendZData() }, 200);

var Joy3 = new JoyStick('joy3');
var yawPitchURL = "http://localhost:5000/yawPitchJoyPost";
var yawPitchData = {
        "yaw": 0,
        "pitch": 0
};

function sendYawPitchData()
{
        $.ajax({type: 'POST',
                url: yawPitchURL,
                data: JSON.stringify (yawPitchData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ yawPitchData.pitch = Joy3.GetX(); }, 50);
setInterval(function(){ yawPitchData.yaw = Joy3.GetY(); }, 50);
setInterval(function(){ sendYawPitchData() }, 200);
setInterval(function(){ console.log(yawPitchData) }, 200);

var Joy4 = new JoyStick('joy4');
var rollURL = "http://localhost:5000/rollJoyPost"
var rollData = {
        "roll": 0
}

function sendRollData()
{
        $.ajax({type: 'POST',
                url: rollURL,
                data: JSON.stringify (rollData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ rollData.roll=Joy4.GetY(); }, 50);
setInterval(function(){ sendRollData() }, 200);

