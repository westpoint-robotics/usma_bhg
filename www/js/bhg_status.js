// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();

ros.connect('ws://10.42.0.1:9090');

// ------------------------
// RECORD Topic
// ------------------------

// Create the record publisher
var record_pub = new ROSLIB.Topic({
  ros: ros,
  name: '/record',
  messageType: 'std_msgs/Bool'
});

// Create an initial message
var record_msg = new ROSLIB.Message({
  data: false
});

// Publish the initial message
record_pub.publish(record_msg);

// Publish start function for button clicks
function start_recordFunction() {
  var record_msg = new ROSLIB.Message({
      data: true
  });
  record_pub.publish(record_msg);
}

// Publish stop function for button clicks
function stop_recordFunction() {
  var record_msg = new ROSLIB.Message({
      data: false
  });
  record_pub.publish(record_msg);
}

// ------------------------
// IMAGE display
// ------------------------

// Create the image subscriber for the flir camera
var imageTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/camera/image_color/compressed',
  messageType: 'sensor_msgs/CompressedImage'
});

// The flir image subscriber callback
imageTopic.subscribe(function(message) {
  var imagedata = message.data;
  document.getElementById('flir_image').src = "data:image/jpg;base64," + message.data;

});

// Create the image subscriber for the gobi camera
var imageTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/camera/gobi_image/compressed',
  messageType: 'sensor_msgs/CompressedImage'
});

// The gobi image subscriber callback
imageTopic.subscribe(function(message) {
  var imagedata = message.data;
  document.getElementById('gobi_image').src = "data:image/jpg;base64," + message.data;

});


// ------------------------
// Diagnostics
// ------------------------

// Create the diagnostics subscriber 
var diagnosticsTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/diagnostics',
  messageType: 'diagnostic_msgs/DiagnosticArray'
});

// The diagnostice subscriber callback
diagnosticsTopic.subscribe(function(message) {
  for (var status of message.status){
    var status_name = status.name;

    // Get status from FCU System
    if (status_name.includes("System")){
        for (var el of status.values){ 
            if (el.key.startsWith("CPU Load")){
              if (el.value > 90){
                html_color = "red";
              }
              else if (el.value > 80){
                html_color = "orange";              
              }
              else
              {
                html_color = "green";              
              }              
              htmlOut = `<span style="font-weight: bold;color:${html_color}"> ${el.value} <\span>`
              document.getElementById('fcu_cpuload').innerHTML = htmlOut;     
            }
            else if (el.key.startsWith("Drop rate")){
              if (el.value > 10){
                html_color = "red";
              }
              else if (el.value > 5){
                html_color = "orange";              
              }
              else
              {
                html_color = "green";              
              }             
              htmlOut = `<span style="font-weight: bold;color:${html_color}"> ${el.value} <\span>`
              document.getElementById('fcu_droprate').innerHTML = htmlOut; 
            }
        }
    };

    // Get status from the NUC
    if (status_name.includes("Package")){
        for (var el of status.values){
            //console.log(`${el.key} : ${el.value}`);             
            if (el.key.startsWith("Temperature (")){
              if (el.value > 90){
                html_color = "red";
              }
              else if (el.value > 80){
                html_color = "orange";              
              }
              else
              {
                html_color = "green";              
              } 
              //console.log(`${el.key} : ${el.value}`);
              htmlOut = `<span style="font-weight: bold;color:${html_color}"> ${el.value} <\span>`
              document.getElementById('nuc_temp').innerHTML = htmlOut;    
            }
        }
    };
  }

});


















