<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/css/bootstrap.min.css">
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/css/bootstrap-theme.min.css">
  <script src="https://ajax.googleapis.com/ajax/libs/jquery/1.11.3/jquery.min.js"></script>
  <script src="http://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/js/bootstrap.min.js"></script>

  <script src="https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
  <script src="https://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>

    <script type="text/javascript" type="text/javascript">



  // Connecting to ROS
  // -----------------

  var ros = new ROSLIB.Ros();

  ros.connect('ws://10.42.0.1:9090');
  
  // Publish the record topic
  // ------------------------
  var record_pub = new ROSLIB.Topic({
    ros : ros,
    name : '/record',
    messageType : 'std_msgs/Bool'  
  });
  
  var record_msg = new ROSLIB.Message({
    data : false
  });  
  
  record_pub.publish(record_msg);

  function start_recordFunction() {
    var record_msg = new ROSLIB.Message({
        data : true
    });
    record_pub.publish(record_msg);
  }

  function stop_recordFunction() {
    var record_msg = new ROSLIB.Message({
        data : false
    });
    record_pub.publish(record_msg);
  }
  
  
    var imageTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/image_color/compressed',
    messageType : 'sensor_msgs/CompressedImage'
  });
  
  imageTopic.subscribe(function(message)
{
   var imagedata = message.data;
  document.getElementById('flir_image').src = "data:image/jpg;base64," + message.data;

});
  
  
 </script>
</head>

<body>
    <div>
        <h1>Simple roslib Example</h1>
        <p>Check your Web Console for output.</p>

        <p><button type="button" id="start_record_btn" onclick="start_recordFunction()" >Start Recording</button>
        <button type="button" id="stop_record_btn"  onclick="stop_recordFunction()"> Stop Recording </button></p>
        <img id="flir_image" src="resources/flir.jpg" alt="FLIR Camera" width="304" height="228" style="border:1px solid black;"/>    
        <img id="gobi_image" src="resources/gobi.jpg" border="1" alt="Gobi Camera" width="304" height="228" style="border:1px solid black;"/>    
    <div>
</body>
</html>
