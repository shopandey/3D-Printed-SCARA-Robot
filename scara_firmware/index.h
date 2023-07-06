const char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<style type="text/css">
.button {
  background-color: #5E5E5E; /* Green */
  border: none;
  color: white;
  height:50px;
  width:150px;
  padding: 15px 32px;
  text-align: center;
  line-height: 10px;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin:2px auto;
  border-radius:5px;
}
</style>
<body style="background-color: #DFDFDF ">
<center>
<h1>SCARA Robot (I4.0)</h1>
  <table width="400" border="1" align="center" cellpadding="3" cellspacing="1" class="Table">
    <tr>
    <th rowspan="4" align="left" valign="top">
    <div>
    <button class="button" onclick="send('rst')">RESET ALL</button><BR>
    <span id="points">P1: 0,0 <BR> P2: 0,0 <BR> P3: 0,0 <BR> P4: 0,0 <BR> P5: 0,0 </span>
      </div>
  </th>
  <td>&nbsp;</td>
    <td align="center" valign="top">
      <div>
      <button class="button" onclick="send('yp5')">Y+ 50 mm</button><BR>
      <button class="button" onclick="send('yp4')">Y+ 10 mm</button><BR>
      <button class="button" onclick="send('yp3')">Y+ 5 mm</button><BR>
      <button class="button" onclick="send('yp2')">Y+ 1 mm</button><BR>
      <button class="button" onclick="send('yp1')">Y+ 0.1 mm</button><BR>
      </div>
    </td>
    <td>&nbsp;</td>
    <td>
      <div>
      <button class="button" onclick="send('mon')">F1</button><BR>
      <button class="button" onclick="send('mof')">F2</button><BR>
      <button class="button" onclick="send('feed')">F3</button><BR>
      <button class="button" onclick="send('xy0')">XY Zero</button><BR>
      <button class="button" onclick="send('z0')">Z Zero</button><BR>
      </div>
    </td>
    </tr>
    <tr>
  <td>
      <div>
      <button class="button" onclick="send('xn5')">X- 50 mm</button><BR>
      <button class="button" onclick="send('xn4')">X- 10 mm</button><BR>
      <button class="button" onclick="send('xn3')">X- 5 mm</button><BR>
      <button class="button" onclick="send('xn2')">X- 1 mm</button><BR>
      <button class="button" onclick="send('xn1')">X- 0.1 mm</button><BR>
      </div>
    </td>
    <td width="300px" align="center" valign="center">
      <div><h3>
      Mpos: <span id="mpos_val">0,0,0</span><br><br>
      </h3>
      </div>
    </td>
    <td>
      <div>
      <button class="button" onclick="send('xp5')">X+ 50 mm</button><BR>
      <button class="button" onclick="send('xp4')">X+ 10 mm</button><BR>
      <button class="button" onclick="send('xp3')">X+ 5 mm</button><BR>
      <button class="button" onclick="send('xp2')">X+ 1 mm</button><BR>
      <button class="button" onclick="send('xp1')">X+ 0.1 mm</button><BR>
      </div>
    </td>
    <td>
      <div>
      <button class="button" onclick="send('zp3')">Z+ 5mm</button><BR>
      <button class="button" onclick="send('zp2')">Z+ 1mm</button><BR>
      <button class="button" onclick="send('zp1')">Z+ 0.1mm</button><BR>
      <button class="button" onclick="send('zn1')">Z- 0.1mm</button><BR>
      <button class="button" onclick="send('zn2')">Z- 1mm</button><BR>
      <button class="button" onclick="send('zn3')">Z- 5mm</button><BR>
      </div>
    </td>
    </tr>
    <tr>
    <td>&nbsp;</td>
    <td align="center" valign="top">
      <div>
      <button class="button" onclick="send('yn1')">Y- 0.1 mm</button><BR>
      <button class="button" onclick="send('yn2')">Y- 1 mm</button><BR>
      <button class="button" onclick="send('yn3')">Y- 5 mm</button><BR>
      <button class="button" onclick="send('yn4')">Y- 10 mm</button><BR>
      <button class="button" onclick="send('yn5')">Y- 50 mm</button><BR>
      </div>
    </td>
    <td>&nbsp;</td>
    <td>
      <div>
      <button class="button" onclick="send('add')">ADD Point</button><BR>
      <button class="button" onclick="send('del')">DEL Point</button><BR>
      <button class="button" onclick="send('str')">Store Point</button><BR>
      <button class="button" onclick="send('run')">RUN</button><BR>
      </div>
    </td>
    </tr>
    <tr>
    <td colspan="3" align="left" valign="top"><h3> No Error </h3></td>
  <td>
    <div>
      
    </div>
  </td>
    </tr>
  </table>
  
 <br>
 
<script>
function send(led_sts) 
{
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("state").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "led_set?state="+led_sts, true);
  xhttp.send();
}
setInterval(function() 
{
  getMpos();
  getData();
}, 1000); 
function getMpos() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("mpos_val").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "mpos", true);
  xhttp.send();
}
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("points").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "points", true);
  xhttp.send();
}

</script>
</center>
</body>
</html>
)=====";
