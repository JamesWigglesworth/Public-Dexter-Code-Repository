/*

The purpose of this program (for the second time) is to allow you to move Dexter by 
applying direct pressure to the arm itself (Follow) and then note XYZ points (setPoint)
or join angles (setAngels) as waypoints, then (Record) those waypoints in a (Macro) 
for later (Play)back.

Things I learned:
1. When Dexter powers up, he is open loop. The encoders are NOT used.

2. Before the encoders can be used, the (Keep) mode must be entered because that 
initializes the tables in the FPGA from the configuration data. I've changed the 
program so it now goes into Keep mode automatically.

3. The .joint_angles(), .robot_status[Dexter.J1_ANGLE], etc... do NOT feedback the
actual angle of the joint from the robot. .robot_status[Dexter.J1_FORCE_CALC_ANGLE]
appears to do that.

4. The kinematic system sometimes (often) has issues getting from one XYZ to another
XYZ. It will claim that position is unreachable, even when it very much is. Joint 
angles are more reliable.



*/


persistent_get(
	"Directory", 
    function(val){ 
       if (val===undefined){
          out("building directory");
          var dirset = [];
          dirset.push(" ");
          persistent_set("Directory",dirset)
          }
       }
    )
//AddToDir("boxadd")



var db_fetch = undefined
Dexter.LINK2 = 321246
Dexter.LINK3 = 298982

var pidXYZ = 0x3e4ccccc
var pidRP = 0x3dcccccc
var PID_DELTATNOT = 16
var PID_DELTAT = 17
var PID_D = 18
var PID_I = 19
var PID_P = 20
var PID_ADDRESS = 21

var DIFF_FORCE_SPEED_FACTOR_ANGLE = 55
var DIFF_FORCE_SPEED_FACTOR_ROT = 56
var SPEED_FACTORA = 27
var SPEED_FACTORB = 28

var DEF_SPEED_FACTOR_A = 10
var DEF_SPEED_FACTOR_DIFF = 10

var gJobDone = 0
var gWindowVals = undefined
var ForceModeEnabled = false
var timeXYZ = []
function setFollowMe(){
    ForceModeEnabled = true
	var retCMD = []
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, DEF_SPEED_FACTOR_DIFF))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, DEF_SPEED_FACTOR_DIFF))
    retCMD.push(make_ins("w", PID_P, 0))
    retCMD.push(make_ins("w", PID_ADDRESS, 3))
    retCMD.push(make_ins("w", PID_ADDRESS, 4))
    retCMD.push(make_ins("w", PID_ADDRESS, 0))
    retCMD.push(make_ins("w", PID_ADDRESS, 1))
    retCMD.push(make_ins("w", PID_ADDRESS, 2))
    retCMD.push(make_ins("w", SPEED_FACTORA, DEF_SPEED_FACTOR_A))
    retCMD.push(make_ins("S", "J1Friction",5 ))
    retCMD.push(make_ins("S", "J2Friction",5 ))
    retCMD.push(make_ins("S", "J3Friction",5 ))
    retCMD.push(make_ins("S", "J4Friction",75 ))
    retCMD.push(make_ins("S", "J5Friction",75 ))
    retCMD.push(make_ins("w", 67, 0))
    retCMD.push(make_ins("w", 68, 0))
    retCMD.push(make_ins("w", 69, 0))
    retCMD.push(make_ins("w", 70, 0))
    retCMD.push(make_ins("w", 71, 0))
    retCMD.push(make_ins("w", 51, 200000))
    retCMD.push(make_ins("w", 54, 260000))

    retCMD.push(make_ins("w", 79, 50 ^ 200 ))
    retCMD.push(make_ins("w", 80, 50 ^ 200 ))
    retCMD.push(make_ins("w", 81, 50 ^ 200 ))
    retCMD.push(make_ins("w", 42, 12448))
    return retCMD  
}
function setForceProtect(){
	ForceModeEnabled = false
	var retCMD = []
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, 4))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 4))
    retCMD.push(make_ins("w", SPEED_FACTORA, 15))
    retCMD.push(make_ins("S", "J1Friction",2 ))
    retCMD.push(make_ins("S", "J2Friction",3 ))
    retCMD.push(make_ins("S", "J3Friction",9 ))
    retCMD.push(make_ins("S", "J4Friction",15 ))
    retCMD.push(make_ins("S", "J5Friction",15 ))
    retCMD.push(make_ins("w", PID_ADDRESS, 0))
    retCMD.push(make_ins("w", PID_P, pidXYZ))
   	retCMD.push(make_ins("w", PID_ADDRESS, 1))
  	retCMD.push(make_ins("w", PID_ADDRESS, 2))
  	retCMD.push(make_ins("w", PID_ADDRESS, 3))
  	retCMD.push(make_ins("w", PID_P, pidRP))
  	retCMD.push(make_ins("w", PID_ADDRESS, 4))
    retCMD.push(make_ins("w", 67, 9000))
    retCMD.push(make_ins("w", 68, 9000))
    retCMD.push(make_ins("w", 69, 9000))
    retCMD.push(make_ins("w", 70, 9000))
    retCMD.push(make_ins("w", 71, 9000))
    retCMD.push(make_ins("w", 78,1 ))
    retCMD.push(make_ins("w", 78,0 ))
    retCMD.push(make_ins("w", 42, 12448))
    return retCMD  
}

function setKeepPossition(){
    ForceModeEnabled = false
	var retCMD = []
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ANGLE, 0))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 0))
	retCMD.push(make_ins("w", PID_ADDRESS, 0))
    retCMD.push(make_ins("w", PID_P, pidXYZ))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 0))
    retCMD.push(make_ins("w", DIFF_FORCE_SPEED_FACTOR_ROT, 0))
   	retCMD.push(make_ins("w", PID_ADDRESS, 1))
  	retCMD.push(make_ins("w", PID_ADDRESS, 2))
  	retCMD.push(make_ins("w", PID_ADDRESS, 3))
  	retCMD.push(make_ins("w", PID_P, pidRP))
  	retCMD.push(make_ins("w", PID_ADDRESS, 4))
  	retCMD.push(make_ins("w", SPEED_FACTORA, 0))
    retCMD.push(make_ins("w", 42, 12960))
    return retCMD
}



function reduceArray(array){
	var rtVal = []
	for (var i = 0;i < array.length;i++){
    	for (var j = 0;j < array[i].length;j++){
        rtVal.push (array[i][j])
        }
    	
    }
	return rtVal
}


function RoundArray(my_array){
var x = 0;
var len = my_array.length
while(x < len){ 
    my_array[x] = Math.round(my_array[x]) 
    x++
}
return my_array
}


function updateXYZPoint(){
//debugger; my_array[x] = Math.round(my_array[x])
    var xyzPoint = Kin.J_angles_to_xyz([
    	Dexter.my_dex.robot_status[Dexter.J1_FORCE_CALC_ANGLE] + Dexter.my_dex.robot_status[Dexter.J1_DELTA] + Dexter.my_dex.robot_status[Dexter.J1_ANGLE] , 
        Dexter.my_dex.robot_status[Dexter.J2_FORCE_CALC_ANGLE] + Dexter.my_dex.robot_status[Dexter.J2_DELTA]+ Dexter.my_dex.robot_status[Dexter.J2_ANGLE], 
        Dexter.my_dex.robot_status[Dexter.J3_FORCE_CALC_ANGLE] + Dexter.my_dex.robot_status[Dexter.J3_DELTA] +  Dexter.my_dex.robot_status[Dexter.J3_ANGLE],
        Dexter.my_dex.robot_status[Dexter.J4_FORCE_CALC_ANGLE] + (Dexter.my_dex.robot_status[Dexter.J4_DELTA] / 16) + Dexter.my_dex.robot_status[Dexter.J4_ANGLE], 
        Dexter.my_dex.robot_status[Dexter.J5_FORCE_CALC_ANGLE] + (Dexter.my_dex.robot_status[Dexter.J5_DELTA] / 16), + Dexter.my_dex.robot_status[Dexter.J5_ANGLE]
        ])
    showXYZPoint(Math.round(xyzPoint[5][0]),Math.round(xyzPoint[5][1]),Math.round(xyzPoint[5][2]))
    //dex.set_in_window(gWindowVals.window_index, "J1_display", "innerHTML", xyzPoint[5][0]) //shows values changing
    //dex.set_in_window(gWindowVals.window_index, "J1_display", "innerHTML", Dexter.my_dex.robot_status[Dexter.J1_ANGLE]) //always shows 0
    //dex.set_in_window(gWindowVals.window_index, "J1_display", "innerHTML", Dexter.my_dex.joint_angles()[0]) //always shows 0
    //dex.set_in_window(gWindowVals.window_index, "J1_display", "innerHTML", Dexter.my_dex.robot_status[Dexter.J1_FORCE_CALC_ANGLE]) //shows angle!
	showJointAngles(
    	Dexter.my_dex.robot_status[Dexter.J1_FORCE_CALC_ANGLE],
    	Dexter.my_dex.robot_status[Dexter.J2_FORCE_CALC_ANGLE],
    	Dexter.my_dex.robot_status[Dexter.J3_FORCE_CALC_ANGLE],
    	Dexter.my_dex.robot_status[Dexter.J4_FORCE_CALC_ANGLE],
    	Dexter.my_dex.robot_status[Dexter.J5_FORCE_CALC_ANGLE]
        ) 
//can't update here if we want to be able to edit

    xyzPoint.push(Vector.normalize(Vector.subtract(xyzPoint[5], xyzPoint[4])))
    return xyzPoint
    }

function showXYZPoint(X,Y,Z) {
    dex.set_in_window(gWindowVals.window_index, "X_display",  "innerHTML", X)   
    dex.set_in_window(gWindowVals.window_index, "Y_display",  "innerHTML", Y)
    dex.set_in_window(gWindowVals.window_index, "Z_display",  "innerHTML", Z)
	}

function updateAngles(){ //record current angles
	//var joints = Dexter.my_dex.joint_angles() //always all zeros. 
    //joints.unshift(0) //leave room for some metadata
    var joints = new Array(6)
    joints[1]=Dexter.my_dex.robot_status[Dexter.J1_FORCE_CALC_ANGLE]
    joints[2]=Dexter.my_dex.robot_status[Dexter.J2_FORCE_CALC_ANGLE]
    joints[3]=Dexter.my_dex.robot_status[Dexter.J3_FORCE_CALC_ANGLE]
    joints[4]=Dexter.my_dex.robot_status[Dexter.J4_FORCE_CALC_ANGLE]
    joints[5]=Dexter.my_dex.robot_status[Dexter.J5_FORCE_CALC_ANGLE]
	return joints
	}


function showJointAngles(J1,J2,J3,J4,J5){
    dex.set_in_window(gWindowVals.window_index, "J1_display", "innerHTML", J1)
    dex.set_in_window(gWindowVals.window_index, "J2_display", "innerHTML", J2)
    dex.set_in_window(gWindowVals.window_index, "J3_display", "innerHTML", J3)
    dex.set_in_window(gWindowVals.window_index, "J4_display", "innerHTML", J4)
    dex.set_in_window(gWindowVals.window_index, "J5_display", "innerHTML", J5)
    }

var pointIdx = 0

function distance(pointA, pointB){
var rt = [Math.abs(pointA[5][0]-pointB[5][0]),Math.abs(pointA[5][1]-pointB[5][1]),Math.abs(pointA[5][2]-pointB[5][2])]
  return Math.sqrt(Math.pow(rt[0],2)+Math.pow(rt[1],2)+Math.pow(rt[2],2))
}

//gotoXYZpose([100, 100, 100000], [0, 0, -1])
function gotoXYZpose(xyz, pose){
	return make_ins("a",RoundArray(Kin.xyz_to_J_angles( xyz, pose , Dexter.RIGHT_UP_OUT)))
	}

function replayPointsitr(points, times){
  out("replay "+times+" times")
  var rt =[]
  for (var j=0;j<times;j++){
    out("points:"+points.length)
    for (var i = 0;i < points.length;i++){
      out(points[i].length)
      if(6 == points[i].length) { //must be angles. Note that [0] isn't used.
        out("#"+i+" J1:"+points[i][1]+" J2:"+points[i][2]+" J3:"+points[i][3]+" J4:"+points[i][4]+" J5:"+points[i][5]);
        rt.push(Dexter.move_all_joints(points[i][1],points[i][2],points[i][3],points[i][4],points[i][5]))
        }
      else {
        out("#"+i+" xyz:"+points[i][5]+" in,out,down:"+points[i][6])
        if (undefined != points[i][5] && undefined != points[i][6]) {
	        rt.push(gotoXYZpose(points[i][5], points[i][6]))
            }
        }
      //rt.push(function() {showJointAngles()}) //useless, que doesn't sync with movements.
      //rt.push(Robot.wait_until(2000)) //delay is not needed, each move starts after the last.
      }
    }
  return rt
  }


function AddToDir(MacroStr){
    //out(MacroStr)
    var closeStr = MacroStr
	dir = persistent_get("Directory")
    dir.push(closeStr)
    persistent_set("Directory",dir)
	}
//var dirset = []
//dirset.push("KBhello")
//persistent_set("Directory",dirset)

var iterations = undefined

function handleWindowUI(vals){ //vals contains name-value pairs for each
                         //html elt in show_window's content with a name.
	gWindowVals = vals 
    //out(vals)
    if(vals.clicked_button_value == "SetPoint" ){ // Clicked button value holds the name of the clicked button.
        timeXYZ[pointIdx] = updateXYZPoint();
        //out(timeXYZ[pointIdx]);
        pointIdx = pointIdx +1;
        out("Set point #"+pointIdx);
    	}
    else if(vals.clicked_button_value == "Update" ) { 
        out("Go to XYZ #"+vals.X+","+vals.Y+","+vals.Z);
        Job.j1.user_data.choicemade = function (){return gotoXYZpose([vals.X,vals.Y,vals.Z], [0,0,-1])}
        //gotoXYZpose works fine in testing, but doesn't work from update.
        //it either says the position isn't reachable, or simply doesn't move.
        //out("Set angle #1:"+vals.J1+" #2:"+vals.J2+" #3:"+vals.J3+" #4:"+vals.J4+" #5:"+vals.J5);
        //Job.j1.user_data.choicemade = function (){return Dexter.move_all_joints(vals.J1,vals.J2,vals.J3,vals.J4,vals.J5)}
        
    }
    else if(vals.clicked_button_value == "SetAngles" ) { 
        timeXYZ[pointIdx] = updateAngles();
        out(timeXYZ[pointIdx]);
        pointIdx = pointIdx +1;
        out("Set angle #"+pointIdx+" ");
    }
    else if(vals.clicked_button_value == "Follow" ) { 
    	Job.j1.user_data.choicemade = function (){return setFollowMe()}
        out("Set FollowMe mode")
    }
    else if(vals.clicked_button_value == "Keep" ){ 
        Job.j1.user_data.choicemade = function (){return setKeepPossition()}
        out("Set set Keep  mode")

    }
    else if(vals.clicked_button_value == "Protect" ) { 
        Job.j1.user_data.choicemade = function (){return setForceProtect()}
        out("Set set Protect mode")
    	}
    else if(vals.clicked_button_value == "Record" )	{
		if (0==timeXYZ.length) {
        	out("Error, no setpoints found");
            } 
        else {
	       out("Recording \'" + vals.macro_name +"\'")
         persistent_set(vals.macro_name,timeXYZ)
         //out("Testing recording")
         //persistent_get(vals.macro_name, function(val){ out("Got: ")})
         var MacroStr = vals.macro_name
         persistent_get("Directory",AddToDir(MacroStr))
         }
    }
    else if(vals.clicked_button_value == "Flush" ) { 
		//persistent_clear() //doesn't exist?
		//also clear all setpoints?
		timeXYZ=[]
        pointIdx=0
		out("cleared database and setpoints")
		if (undefined===persistent_get("Directory")){
          out("building directory");
          var dirset = [];
          dirset.push(" ");
          persistent_set("Directory",dirset)
          }
	    }
    else if(vals.clicked_button_value == "Dir" ) { 
       out("Known Macros: " + persistent_get("Directory"))
    }
    else if(vals.clicked_button_value == "Play" ){ 
        db_fetch = undefined
        iterations = vals.LI
        out(vals.macro_name)
        db_fetch=persistent_get(vals.macro_name)
        if ("" == db_fetch) {
        	out("Error, macro \'"+vals.macro_name+"\' not found");
            } 
        else {
        	out("Loaded \'"+vals.macro_name+"\' "+db_fetch.length+" steps");
            //why not load the setpoints back so we can add to an existing job?
            pointIdx=db_fetch.length
            timeXYZ=db_fetch
	        Job.j1.user_data.choicemade = function () { 
                  var rt = []
                  rt.push(function(){return replayPointsitr(db_fetch,iterations)})
                  return rt                                        
                  }
    		}
    	}
    else if (vals.clicked_button_value == "Done" ){   
        gJobDone = 1
        gWindowVals = undefined
		for (var i = 0;i < pointIdx-1;i++){
    		out(timeXYZ[i])
    	}
        out("outta here " )
        //Job.j1.user_data.choicemade = function (){return Dexter.move_all_joints(0, 0, 0, 0, 0)}
    }
}





show_window({content: `
 <input name="SetFollowMe" type="button" value="Follow"/>
 <input name="SetKeepPoint" type="button" value="Keep"/>
 <input name="SetForceProtect" type="button" value="Protect"/>
 Mode: <span id="mode_id">None</mode><br/><br/>
 <input name="update" type="button" value="Update"/><br/>
 <TABLE BORDER="0"><TR><TD WIDTH="100px">
 X: <span name="X_display" id="X_id">0</span><input name="X" size=5 value="0"><br/>
 Y: <span name="Y_display" id="Y_id">0</span><input name="Y" size=5 value="0"><br/>
 Z: <span name="Z_display" id="Z_id">0</span><input name="Z" size=5 value="0"><br/>
 </TD><TD WIDTH="200px">
 1 Base:  
 <span onClick="J1_id.innerHTML=parseInt(J1_id.innerHTML)-1000;">&lt;</span>
 <span name="J1_display" id="J1_id" size=5 value="0">0</span>
 <span onClick="J1_id.innerHTML=parseInt(J1_id.innerHTML)+1000;">&gt;</span>
 <br/>
 2 Decl: <span name="J2_display" id="J2_id" size=5 value="0">0</span><br/>
 3 Elbow: <span name="J3_display" id="J3_id" size=5 value="0">0</span><br/>
 4 Wrist: <span name="J4_display" id="J4_id" size=5 value="0">0</span><br/>
 5 Wrist: <span name="J5_display" id="J5_id" size=5 value="0">0</span><br/>
 </TD></TR></TABLE>
 <br/>
 <input name="Flush" type="button" value="Flush"/>
 <input name="SetPoint" type="button" accesskey="p" value="SetPoint"/> 
 <input name="SetAngles" type="button" accesskey="a" value="SetAngles"/> 
 <input name="Write Time" type="button" value="Record"/>
 <input name="Dir" type="button" value="Dir"/>
 <input name="PlayMacro" type="button" value="Play"/>
 <br/>
 Macro Name <input type="text" name="macro_name" value="test" id="mn_id" <br/><br/>
 Loop Iterations <input type="text" name="LI" value="1" id="li_id" <br/><br/>
 <input type="submit" value="Done"/>`, 
             callback: handleWindowUI})     


var J3AngleHI = 100000
var J3AngleLow = -100000

function updateXYZForce(){
var rt = []
var J3Angle = Dexter.my_dex.robot_status[Dexter.J3_FORCE_CALC_ANGLE] + 
              Dexter.my_dex.robot_status[Dexter.J3_DELTA] +  
              Dexter.my_dex.robot_status[Dexter.J3_ANGLE]
  if(J3Angle > J3AngleHI){rt.push(make_ins("S","J3Force", (J3AngleHI - J3Angle) * -.01))}
  if(J3Angle < J3AngleLow){rt.push(make_ins("S","J3Force", (J3AngleLow - J3Angle) * -.01))}
  if(J3Angle > J3AngleLow && J3Angle < J3AngleHI){rt.push(make_ins("S","J3Force", 0))}
  return rt
}
              
var showinterval=0              
             
function resolve_choice() {
	
	var na = []
	//showJointAngles() //totally locks everything up.
   	//na.push(function(){if (100 < showinterval++) {Dexter.my_dex.get_robot_status();showJointAngles();showinterval = 0}}) 
    //At intervals 100 or more it doesn't lock up, but still does nothing. Not getting actual joint angle from Dexter.
    na.push(make_ins("g")) //get status?
    na.push(function(){if (this.user_data.choicemade != undefined) {
    		var rtval = this.user_data.choicemade
            this.user_data.choicemade = undefined
            return rtval
    		}})
   
    na.push(function(){if (gWindowVals != undefined){updateXYZPoint()}}) //just for the display. Not actually getting data from Dexter.
    //na.push(function(){if (gWindowVals != undefined){return updateXYZForce()}})
    na.push(function(){if (0 == gJobDone) {return resolve_choice()}})
	return na
}

new Dexter({name: "my_dex", ip_address: "192.168.0.142", port: 50000, enable_heartbeat: false, simulate: false})
new Job({name: "j1", robot: Robot.my_dex, keep_history: false,
         do_list: [	
              make_ins("S", "J1BoundryHigh",648000),
        	  make_ins("S", "J1BoundryLow",-648000),
              make_ins("S", "J2BoundryLow",-300000),
              make_ins("S", "J2BoundryHigh",300000),
              make_ins("S", "J3BoundryLow",-530000),
              make_ins("S", "J3BoundryHigh",530000),
              make_ins("S", "J4BoundryLow",-340000),
              make_ins("S", "J4BoundryHigh",340000),
              make_ins("S", "J5BoundryLow",-648000),
              make_ins("S", "J5BoundryHigh",648000),
// Since Dexter moves so well without these changes, I'm commenting them out.
//                make_ins("S", "MaxSpeed",360000),
//                make_ins("S", "Acceleration",5), 
//                make_ins("S", "StartSpeed", 1000),
                Dexter.move_all_joints(0, 0, 0, 0, 0),
// if you don't go into KeepPossition mode, Follow mode will "freak out"
// because Keep triggers an initialization of the encoders with make_ins("w", 42, 12448)
// so /always/ go into KeepPossitoin.
                setKeepPossition(),
                function(){return resolve_choice},
//                make_ins("S", "MaxSpeed",240000),
//                make_ins("S", "Acceleration",1),
                Dexter.move_all_joints(0, 0, 0, 0, 0)
         ]})


Job.j1.start()
//Job.j1.inter_do_item_dur = 50 //Was 10. Actually, seems to work ok with the default value.
//Dexter.heartbeat_dur = 25


//DrawNum(123456, 23000, [0, 1])
